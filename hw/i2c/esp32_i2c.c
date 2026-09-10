#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/error-report.h"
#include "hw/i2c/esp32_i2c.h"
#include "hw/irq.h"
#include "hw/xtensa/esp32_clk.h"

#include "../softmmu/simuliface.h"
#include "../softmmu/vnext_b.h"

static Esp32I2CState *vnext_i2c_instances[2];

void esp32_i2c_vnext_bind(Esp32I2CState *s) {
    if (s && s->busIndex < 2) vnext_i2c_instances[s->busIndex] = s;
}

static void esp32_i2c_do_transaction(void *opaque);

/* E118-AUDIT (EVIDENCE.md, 2026-09-05): see the struct comment on vnextContinuationBacklogged in
 * esp32_i2c.h. Called from vnext_b.c's vnext_resume() sweep once lane 0 regains credit -- never a
 * poll loop of its own, and this continuation only ever runs with current_cpu==NULL, so retrying
 * it directly here never touches a CPU. */
void esp32_i2c_vnext_credit_available(void) {
    for (unsigned bus = 0; bus < 2; ++bus) {
        Esp32I2CState *s = vnext_i2c_instances[bus];
        if (!s || !s->vnextContinuationBacklogged) continue;
        s->vnextContinuationBacklogged = false;
        esp32_i2c_do_transaction(s);
    }
}

void esp32_i2c_vnext_complete(uint32_t bus, uint32_t status, uint32_t first_nack,
                              uint64_t stretch_ns, const uint8_t *rx, uint32_t rx_len) {
    if (bus >= 2 || !vnext_i2c_instances[bus]) return;
    Esp32I2CState *s = vnext_i2c_instances[bus];
    s->burstAddressAck = (status & 2u) != 0;
    s->burstFirstNack = first_nack;
    s->burstRxLen = rx_len > 32 ? 32 : rx_len;
    if (s->burstRxLen && rx) memcpy(s->burstRxBuf, rx, s->burstRxLen);
    const uint64_t duration = s->period_ns +
        (uint64_t)(s->burstWriteCmdCount + s->burstReadCmdCount) * (9 * s->period_ns) +
        (s->burstReadCmdCount ? 0 : s->period_ns) + stretch_ns;
    timer_mod(&s->event_timer, getQemu_ns() + duration);
}


static uint8_t fifo8_peek(Fifo8 *fifo)
{
    if (fifo->num == 0) abort();
    return fifo->data[fifo->head];
}

static void esp32_i2c_update_irq(Esp32I2CState * s)
{
    int irq_state = !!(s->int_raw_reg & s->int_ena_reg);
    qemu_set_irq(s->irq, irq_state);
}

static void esp32_i2c_abort_empty_tx(Esp32I2CState *s)
{
    timer_del(&s->event_timer);
    s->bytesTx = 0;
    s->ackSamplePending = false;
    s->sr_reg &= ~(1 << 4);             /* I2C_BUS_BUSY */
    s->int_raw_reg &= ~(1 << 6);        /* I2C_BYTE_TRANS */
    s->int_raw_reg |= (1 << 10) | (1 << 7); /* ACK_ERR + TRANS_COMPLETE */
    /* This is a transaction-terminal path, same class as a normal STOP: leaving
     * burstAddressValid set would let a later, unrelated command-list slice attempt a
     * continuation burst against this now-finished transaction's address. */
    s->burstAddressValid = false;
    if (s->lastCMD < ESP32_I2C_CMD_COUNT) {
        s->cmd_reg[s->lastCMD] =
            FIELD_DP32(s->cmd_reg[s->lastCMD], I2C_CMD, DONE, 1);
    }
    esp32_i2c_update_irq(s);
}

/* Plano de burst: varre cmd_reg a partir do comando logo apos o RSTART ja' consumido pelo
 * chamador, juntando (a) uma corrida contigua de WRITE (o primeiro byte dela e' o endereco+RW,
 * exatamente como o driver ESP-IDF gera: i2c_master_write_byte(endereco) + i2c_master_write(dados)
 * viram DOIS opcodes WRITE separados na lista, nao um so') e (b) uma corrida contigua de READ logo
 * em seguida (leitura de sensor apos repeated-start). So' e' seguro fast-pathear quando o proximo
 * comando depois dessas corridas for STOP ou END -- outro RSTART encadeado (ex: mais um segmento
 * de leitura com ACK_VAL diferente no meio) fica fora deste primeiro corte e cai no caminho
 * eletrico de sempre. Funcao pura: nao mexe em tx_fifo/rx_fifo/cmd_reg. */
typedef struct Esp32I2cBurstPlan {
    uint32_t writeCmdCount;
    uint32_t txBytes;
    uint32_t readCmdCount;
    uint32_t rxBytes;
    bool stop;
} Esp32I2cBurstPlan;

static bool esp32_i2c_plan_burst(Esp32I2CState *s, Esp32I2cBurstPlan *plan)
{
    memset(plan, 0, sizeof(*plan));
    uint32_t idx = s->lastCMD;

    while (idx < ESP32_I2C_CMD_COUNT) {
        uint32_t cmd = s->cmd_reg[idx];
        if (FIELD_EX32(cmd, I2C_CMD, OPCODE) != I2C_OPCODE_WRITE) break;
        uint32_t byteNum = cmd & 0xFF;
        if (plan->txBytes + byteNum > ESP32_I2C_FIFO_LENGTH) return false;
        /* O resultado do mailbox tem um primeiro NACK, não uma política ACK_EXP por comando.
         * Restrinja o fast path à forma normal do ESP-IDF (verificar ACK e esperar ACK); listas
         * especiais continuam no executor elétrico, que mantém semântica por opcode. */
        if (!FIELD_EX32(cmd, I2C_CMD, ACK_CHECK_EN) || FIELD_EX32(cmd, I2C_CMD, ACK_EXP)) return false;
        plan->txBytes += byteNum;
        plan->writeCmdCount++;
        idx++;
    }
    if (plan->writeCmdCount == 0 || plan->txBytes == 0) return false; /* precisa do endereco */

    while (idx < ESP32_I2C_CMD_COUNT) {
        uint32_t cmd = s->cmd_reg[idx];
        if (FIELD_EX32(cmd, I2C_CMD, OPCODE) != I2C_OPCODE_READ) break;
        uint32_t byteNum = cmd & 0xFF;
        if (plan->rxBytes + byteNum > ESP32_I2C_FIFO_LENGTH) return false;
        plan->rxBytes += byteNum;
        plan->readCmdCount++;
        idx++;
    }

    /* Precisa sobrar pelo menos um slot pro STOP/END que fecha a lista -- sem isto,
     * esp32_i2c_do_transaction() leria cmd_reg fora dos limites na retomada abaixo. */
    if (idx >= ESP32_I2C_CMD_COUNT) return false;
    {
        uint8_t opcode = FIELD_EX32(s->cmd_reg[idx], I2C_CMD, OPCODE);
        if (opcode != I2C_OPCODE_STOP && opcode != I2C_OPCODE_END && opcode != I2C_OPCODE_RSTART) return false;
        plan->stop = (opcode == I2C_OPCODE_STOP);
    }

    if (fifo8_num_used(&s->tx_fifo) < plan->txBytes) return false;
    if (plan->rxBytes > 0 && fifo8_num_free(&s->rx_fifo) < plan->rxBytes) return false;

    return true;
}

/* Tenta despachar RSTART + a corrida de WRITE/READ planejada acima como UM pedido de burst pro
 * Core (ver i2cBurstTransfer em simuliface.h) em vez de um timer/round-trip por byte. Retorna
 * false sem tocar em nenhum estado (FIFO/cmd_reg intactos) quando o plano nao se aplica ou o Core
 * recusa (`handled=false`, ex: nada conectado suporta o protocolo de burst) -- o chamador cai de
 * volta pro caminho eletrico byte-a-byte existente exatamente como se esta funcao nao existisse. */
static bool esp32_i2c_try_burst(Esp32I2CState *s)
{
    Esp32I2cBurstPlan plan;
    if (!esp32_i2c_plan_burst(s, &plan)) return false;

    uint8_t txbuf[ESP32_I2C_FIFO_LENGTH];
    /* Espie sem consumir. Se o Core recusar, o FIFO deve permanecer bit-a-bit idêntico; retirar e
     * recolocar no fim muda a ordem quando houver bytes residuais no ring. */
    for (uint32_t i = 0; i < plan.txBytes; ++i) {
        txbuf[i] = s->tx_fifo.data[(s->tx_fifo.head + i) % s->tx_fifo.capacity];
    }
    /* A direção codificada no byte de endereço precisa concordar com a lista de comandos.
     * Listas artesanais/inconsistentes devem conservar exatamente o comportamento elétrico. */
    if ((bool)(txbuf[0] & 1u) != (plan.rxBytes > 0)) return false;

    /* The ESP32 command engine reports END_DETECT rather than STOP, but the
     * Core-side device model needs a transaction boundary to commit a write
     * (notably SSD1306 command/data streams).  Keep the guest-visible opcode
     * unchanged while advertising that boundary in the mailbox. */
    const uint8_t terminalOpcode = FIELD_EX32(s->cmd_reg[s->lastCMD +
                                                          plan.writeCmdCount + plan.readCmdCount],
                                               I2C_CMD, OPCODE);
    const bool logicalStop = plan.stop || terminalOpcode == I2C_OPCODE_END;

    I2cBurstRequest req = {0};
    req.flags = 1u /* START */ | (plan.rxBytes ? 4u /* READ */ : 0u) | (logicalStop ? 2u : 0u);
    req.period_ns = s->period_ns;
    req.tx = txbuf;
    req.tx_len = plan.txBytes;
    req.rx_len = plan.rxBytes;

    if (vnext_b_active()) {
        if (!vnext_b_i2c_submit(s->busIndex, 1u | (plan.rxBytes ? 4u : 0u) |
                                (logicalStop ? 2u : 0u), s->period_ns, txbuf,
                                plan.txBytes, plan.rxBytes)) return false;
        s->burstAddressByte = txbuf[0];
        /* A write-only address probe is complete in this command list; it is
         * not a permission to reinterpret the next list as a continuation. */
        /* Even an address-only WRITE establishes the selected target for the
         * following END/WRITE slice.  ESP-IDF commonly publishes the address
         * and payload as separate command lists; clearing this latch here
         * forces the payload back through the byte-at-a-time path and leaves
         * the guest waiting forever for the probe sequence to complete. */
        s->burstAddressValid = (plan.txBytes > 0 || plan.rxBytes > 0);
        for (uint32_t i = 0; i < plan.txBytes; ++i) fifo8_pop(&s->tx_fifo);
        s->burstActive = true;
        s->burstPartial = false;
        s->burstStop = logicalStop;
        s->burstAddressAck = false;
        s->burstFirstNack = UINT32_MAX;
        s->burstRxLen = 0;
        s->burstWriteCmdCount = plan.writeCmdCount;
        s->burstReadCmdCount = plan.readCmdCount;
        return true;
    }

    I2cBurstResponse resp;
    if (!i2cBurstTransfer(s->busIndex, &req, &resp, s->burstRxBuf)) {
        return false;
    }

    s->burstAddressByte = txbuf[0];
    s->burstAddressValid = true;

    for (uint32_t i = 0; i < plan.txBytes; ++i) fifo8_pop(&s->tx_fifo);

    s->burstActive = true;
    s->burstPartial = false;
    s->burstStop = logicalStop;
    s->burstAddressAck = resp.address_ack;
    /* Simplificacao deliberada: ESP-IDF sempre gera ACK_CHECK_EN=1/ACK_EXP=0 pro byte de
     * endereco e pros bytes de dado de escrita -- nao ha' granularidade por-byte de ACK_EXP no
     * burst mesclado, entao usamos o ACK_CHECK_EN do PRIMEIRO comando WRITE (o do endereco) como
     * intencao pra toda a corrida, igual ao que qualquer firmware real observado gera. */
    s->burstFirstNack = resp.first_nack;
    s->burstRxLen = resp.rx_len;
    s->burstWriteCmdCount = plan.writeCmdCount;
    s->burstReadCmdCount = plan.readCmdCount;

    /* Reserva na linha do tempo virtual a mesma duracao que o caminho eletrico byte-a-byte teria
     * gasto: 3 periodos pro START (ver "6*period_ns/2" abaixo) + 10 periodos por byte transmitido
     * ou recebido (ver "20*period_ns/2" no WRITE/READ do caminho eletrico). Isso preserva o tempo
     * virtual visto pelo firmware (millis()/esp_timer_get_time() durante a transacao) mesmo sem os
     * round-trips por byte. */
    uint64_t time = s->period_ns +
                    (uint64_t)(plan.txBytes + plan.rxBytes) * (9 * s->period_ns) +
                    (plan.stop ? s->period_ns : 0) + resp.stretch_ns;
    time += getQemu_ns();
    timer_mod(&s->event_timer, time);
    return true;
}

/* The ESP32 HAL also emits a compact read-only command list: the device
 * address is placed in the TX FIFO and cmd[0] is READ, followed by END.  It
 * does not pass through the RSTART/WRITE planner above, but it has the same
 * causal BATCH semantics. */
static bool esp32_i2c_try_read_burst(Esp32I2CState *s)
{
    if (s->lastCMD >= ESP32_I2C_CMD_COUNT ||
        FIELD_EX32(s->cmd_reg[s->lastCMD], I2C_CMD, OPCODE) != I2C_OPCODE_READ) return false;
    const uint32_t rxBytes = s->cmd_reg[s->lastCMD] & 0xffu;
    if (!rxBytes || rxBytes > ESP32_I2C_FIFO_LENGTH || s->lastCMD + 1 >= ESP32_I2C_CMD_COUNT) return false;
    const uint32_t nextOpcode = FIELD_EX32(s->cmd_reg[s->lastCMD + 1], I2C_CMD, OPCODE);
    if (nextOpcode != I2C_OPCODE_STOP && !(nextOpcode == I2C_OPCODE_END && rxBytes == 1)) return false;
    uint8_t address = fifo8_num_used(&s->tx_fifo) ? fifo8_peek(&s->tx_fifo) :
                      (uint8_t)(s->burstAddressByte | 1u);
    if (!(address & 1u) || !vnext_b_active()) return false;
    if (!vnext_b_i2c_submit(s->busIndex, 1u | 2u | 4u, s->period_ns,
                            &address, 1, rxBytes)) return false;
    if (fifo8_num_used(&s->tx_fifo)) fifo8_pop(&s->tx_fifo);
    s->burstActive = true;
    s->burstPartial = false;
    s->burstStop = true;
    s->burstAddressByte = address;
    s->burstAddressAck = false;
    s->burstFirstNack = UINT32_MAX;
    s->burstRxLen = 0;
    s->burstWriteCmdCount = 0;
    s->burstReadCmdCount = 1;
    return true;
}

/* O driver ESP-IDF pode encerrar uma fatia da command-list com END, reencher o FIFO e iniciar a
 * próxima fatia com WRITE direto, sem novo RSTART. O endereço continua selecionado no barramento.
 * Inclua-o apenas como metadado no mailbox (START fica desligado), permitindo ao Core localizar o
 * mesmo alvo sem reiniciar a máquina de protocolo do dispositivo. */
static bool esp32_i2c_try_continuation_burst(Esp32I2CState *s)
{
    Esp32I2cBurstPlan plan;
    if (!s->burstAddressValid || !esp32_i2c_plan_burst(s, &plan)) {
        return false;
    }
    /* The VNEXT-B mailbox carries at most 32 bytes total, including the
     * address byte prepended below.  A 32-byte guest WRITE therefore needs a
     * single split (31 bytes, then the final byte).  Keep the command at the
     * same slot until the partial burst completes; this preserves the guest
     * DONE/END ordering while retaining the selected I2C target. */
    const bool partial = plan.txBytes + 1 > 32;
    if (partial && (plan.writeCmdCount != 1 || plan.readCmdCount != 0 || plan.txBytes != 32))
        return false;
    const uint32_t payloadBytes = partial ? 31u : plan.txBytes;

    uint8_t txbuf[ESP32_I2C_FIFO_LENGTH + 1];
    txbuf[0] = s->burstAddressByte;
    for (uint32_t i = 0; i < payloadBytes; ++i) {
        txbuf[i + 1] = s->tx_fifo.data[(s->tx_fifo.head + i) % s->tx_fifo.capacity];
    }
    const uint32_t terminalIndex = s->lastCMD + plan.writeCmdCount + plan.readCmdCount;
    const uint8_t terminalOpcode = FIELD_EX32(s->cmd_reg[terminalIndex], I2C_CMD, OPCODE);
    const bool logicalStop = !partial && (plan.stop || terminalOpcode == I2C_OPCODE_END);

    if (vnext_b_active()) {
        const bool submitted = vnext_b_i2c_submit(s->busIndex, (plan.rxBytes ? 4u : 0u) |
                                                   (logicalStop ? 2u : 0u), s->period_ns, txbuf,
                                                   payloadBytes + 1, plan.rxBytes);
        if (!submitted) return false;
        if (partial) {
            s->cmd_reg[s->lastCMD] = (s->cmd_reg[s->lastCMD] & ~0xffu) | 1u;
        }
    } else {
        I2cBurstRequest req = {0};
        req.flags = (plan.rxBytes ? 4u : 0u) | (logicalStop ? 2u : 0u);
        req.period_ns = s->period_ns;
        req.tx = txbuf;
        req.tx_len = payloadBytes + 1;
        req.rx_len = plan.rxBytes;
        I2cBurstResponse resp;
        if (!i2cBurstTransfer(s->busIndex, &req, &resp, s->burstRxBuf)) return false;
        s->burstAddressAck = resp.address_ack;
        s->burstFirstNack = resp.first_nack;
        s->burstRxLen = resp.rx_len;
        s->burstWriteCmdCount = plan.writeCmdCount;
        s->burstReadCmdCount = plan.readCmdCount;
        s->burstActive = true;
        timer_mod(&s->event_timer, getQemu_ns() +
                  (uint64_t)(plan.txBytes + plan.rxBytes) * (9 * s->period_ns) +
                  (logicalStop ? s->period_ns : 0) + resp.stretch_ns);
        for (uint32_t i = 0; i < payloadBytes; ++i) fifo8_pop(&s->tx_fifo);
        return true;
    }

    for (uint32_t i = 0; i < payloadBytes; ++i) fifo8_pop(&s->tx_fifo);
    if (partial) {
        /* Leave the command pending with exactly the one byte still in the
         * FIFO.  The next continuation will mark it DONE normally. */
        s->cmd_reg[s->lastCMD] = (s->cmd_reg[s->lastCMD] & ~0xffu) | 1u;
    }
    s->burstActive = true;
    s->burstPartial = partial;
    s->burstStop = logicalStop;
    s->burstAddressAck = false;
    s->burstFirstNack = UINT32_MAX;
    s->burstRxLen = 0;
    s->burstWriteCmdCount = partial ? 0u : plan.writeCmdCount;
    s->burstReadCmdCount = plan.readCmdCount;
    return true;
}

/* Opt-in gate for the per-operation I2C diagnostics below.  DECISION-003
 * requires per-operation I2C diagnostic output to be off by default; the two
 * ackERR sites had been reported as fixed but were still unconditional, and
 * emitted 22,930 lines in one 60 s 16-session run (E103).  Diagnostic-only:
 * no I2C, transport, watchdog or reset semantics depend on these writes. */
static bool esp32_i2c_trace_enabled(void)
{
    static gsize initialized;
    static bool enabled;

    if (g_once_init_enter(&initialized)) {
        const char *value = getenv("LASECSIMUL_VNEXT_TRACE");
        enabled = value && value[0] && strcmp(value, "0") != 0;
        g_once_init_leave(&initialized, 1);
    }
    return enabled;
}

static void esp32_i2c_trace_rejected_plan(Esp32I2CState *s)
{
    static unsigned reports;
    const char *enabled = getenv("LASECSIMUL_I2C_FASTPATH_TRACE");
    if (!enabled || !enabled[0] || !strcmp(enabled, "0") || reports++ >= 20) return;
    fprintf(stderr, "[LasecSimul][I2C fast-path] QEMU fallback fifo=%u lastCMD=%u cmds=",
            fifo8_num_used(&s->tx_fifo), s->lastCMD);
    for (uint32_t i = s->lastCMD; i < ESP32_I2C_CMD_COUNT; ++i) {
        fprintf(stderr, "%s%08x", i == s->lastCMD ? "" : ",", s->cmd_reg[i]);
    }
    fputc('\n', stderr);
}

static void esp32_i2c_finish_burst(Esp32I2CState *s)
{
    s->burstActive = false;

    const bool ackERR = !s->burstAddressAck || (s->burstFirstNack != UINT32_MAX);
    s->sr_reg &= ~1;
    s->sr_reg |= ackERR ? 1u : 0u; // I2C_ACK_REC: 0=ACK, 1=NACK do último resultado
    if (ackERR) {
        s->int_raw_reg |= 1 << 10;      // ACK_ERR
        if (esp32_i2c_trace_enabled()) {
            printf("Qemu: esp32_i2c_finish_burst ackERR\n"); fflush(stdout);
        }
    } else {
        s->int_raw_reg &= ~(1 << 10);
    }

    if (s->burstRxLen) fifo8_push_all(&s->rx_fifo, s->burstRxBuf, s->burstRxLen);

    const uint32_t consumed = s->burstWriteCmdCount + s->burstReadCmdCount;
    uint32_t idx = s->lastCMD;
    if (!s->burstPartial) {
        for (uint32_t i = 0; i < consumed && idx < ESP32_I2C_CMD_COUNT; ++i, ++idx) {
            s->cmd_reg[idx] = FIELD_DP32(s->cmd_reg[idx], I2C_CMD, DONE, 1);
        }
        s->lastCMD = idx;
    }

    if (s->burstPartial) {
        /* The first half of a 32-byte continuation was delivered.  Keep the
         * command pending; its low byte was reduced to the one remaining
         * payload byte by the submitter. */
        s->burstPartial = false;
    }

    /* Do not consume the terminal opcode here.  The electrical executor's
     * do_transaction() owns END_DETECT/STOP completion and the guest driver
     * distinguishes those interrupts.  Calling it below preserves that
     * contract for burst requests as well. */
    esp32_i2c_do_transaction(s);
}

static void esp32_i2c_do_transaction( void* opaque )
{
    Esp32I2CState* s = Esp32_I2C(opaque);

    uint32_t cmd = s->cmd_reg[s->lastCMD];
    s->lastOpcode = FIELD_EX32( cmd, I2C_CMD, OPCODE );

    //printf("Qemu: esp32_i2c_do_transaction %i\n", s->lastOpcode); fflush( stdout );

    uint64_t time = 0;

    if (s->lastOpcode == I2C_OPCODE_READ && esp32_i2c_try_read_burst(s)) return;

    switch( s->lastOpcode )
    {
    case I2C_OPCODE_RSTART:
    {
        /* A continued burst is valid only after THIS START was accepted by the burst path.
         * Keeping the address from a previous transaction lets a later FIFO slice switch from
         * already-scheduled electrical edges to direct delivery, reordering/duplicating bytes. */
        s->burstAddressValid = false;
        s->bytesTx = 0;
        s->sr_reg |= 1<<4;                // I2C_BUS_BUSY
        s->lastCMD++;

        // Tenta despachar RSTART + a corrida de WRITE/READ que segue como UM pedido de burst pro
        // Core (ver esp32_i2c_try_burst acima) em vez de um timer/round-trip por byte. So' cai
        // aqui dentro quando ha' suporte no outro lado E o plano se aplica (ate' 32 bytes, sem
        // RSTART encadeado no meio) -- qualquer outro caso segue o caminho eletrico de sempre,
        // inclusive dispositivos sem suporte ao protocolo de burst.
        if (esp32_i2c_try_burst(s)) {
            return; // pedido em voo; esp32_i2c_finish_burst() retoma via timer
        }
        esp32_i2c_trace_rejected_plan(s);

        // Espelha o proprio comando RSTART ANTES de avancar pro proximo -- e' o unico jeito do
        // lado Core (que gera SCL/SDA reais) distinguir "comeca uma transacao nova/repeated-START
        // agora" de "so' mais um byte do mesmo burst de escrita". Sem isto, RSTART era invisivel
        // na arena e o START real (inclusive um REPEATED START no meio de uma transacao, ex:
        // i2c_master_write_read_device()) nunca acontecia no barramento eletrico de verdade -- so'
        // no proprio QEMU, que sempre "via" a transacao completar.
        {
            const VnextPublishResult rstartResult =
                writeReg( s->iomem.addr+A_I2C_CMD, s->cmd_reg[s->lastCMD - 1] );
            if (rstartResult == VNEXT_WOULD_BLOCK) {
                /* E118-AUDIT: nothing else in this case has taken effect yet except lastCMD++
                 * above -- revert exactly that (idempotent to redo: burstAddressValid/bytesTx/
                 * sr_reg are all safely re-set to the same values on retry) and retry the whole
                 * RSTART step later, never falling through to WRITE on an unpublished mirror. */
                s->lastCMD--;
                s->vnextContinuationBacklogged = true;
                vnext_b_note_nonvcpu_backlog(0);
                return;
            }
        }
        s->ackSamplePending = true;
        /*
         * O motor eletrico do Core executa cinco meias-fases antes do primeiro bit. A sexta
         * meia-fase impede o proximo opcode de disputar o mesmo timestamp do START eletrico.
         */
        time = 6*s->period_ns/2;

        cmd = s->cmd_reg[s->lastCMD];
        s->lastOpcode = FIELD_EX32( cmd, I2C_CMD, OPCODE );
        cmd |= 1<<16; // Mark as start
    } // Fall through

    case I2C_OPCODE_WRITE:
    {
        if (s->bytesTx == 0 && !(cmd & (1u << 16))) {
            /* burstAddressValid is validated inside esp32_i2c_try_continuation_burst() itself --
             * a single canonical check, not duplicated here, so the two can't drift apart. */
            if (esp32_i2c_try_continuation_burst(s)) return;
            esp32_i2c_trace_rejected_plan(s);
        }
        if( s->bytesTx == 0 ) s->bytesTx = cmd & 0xFF; //FIELD_EX32( cmd, I2C_CMD, BYTE_NUM );
        if (fifo8_num_used(&s->tx_fifo) == 0) {
            error_report("esp32_i2c: timed write found an empty TX FIFO");
            esp32_i2c_abort_empty_tx(s);
            return;
        }
        uint8_t data = fifo8_peek( &s->tx_fifo );
        //printf("Qemu: esp32_i2c CMD write %i %i\n", s->bytesTx, data ); fflush( stdout );
        /* E118-AUDIT: tx_fifo is only ever PEEKED here, never popped -- the actual pop/decrement
         * happens later in esp32_i2c_event() when the timer armed below fires. So a WOULD_BLOCK
         * here needs no revert at all: bytesTx was already idempotently (re-)initialized above,
         * the byte is still sitting in tx_fifo untouched, and simply not arming the timer (by
         * returning before `time` is set, matching every other branch below) means the byte is
         * re-peeked and re-offered unchanged on retry. int_raw_reg/I2C_BYTE_TRANS must NOT be set
         * before confirming the mirror actually published, unlike the pre-E118-AUDIT code. */
        if (writeReg( s->iomem.addr+A_I2C_CMD, (cmd & ~0xFF) | data) == VNEXT_WOULD_BLOCK) {
            s->vnextContinuationBacklogged = true;
            vnext_b_note_nonvcpu_backlog(0);
            return;
        }
        s->int_raw_reg |= 1<<6;          // I2C_BYTE_TRANS
        /* Guarda de um meio-periodo: o ACK precisa estar assentado no Core antes do timer. */
        time += (20*s->period_ns)/2;
    }break;

    case I2C_OPCODE_READ:
    {
        // Espelha o comando READ (opcode + ACK_VAL + BYTE_NUM, todos em `cmd`) pro lado Core, que
        // e' quem de fato clockeia os 8 bits reais em SCL/SDA a partir do escravo endereçado e
        // decide ACK(continua lendo)/NACK(ultimo byte) conforme ACK_VAL -- mesmo papel que o WRITE
        // acima ja tinha pro sentido contrario. O byte realmente recebido so' fica disponivel
        // depois que o timer abaixo disparar esp32_i2c_event() (mesmo pacing de um WRITE: um byte
        // "no barramento" real dura o mesmo tempo em qualquer direcao).
        if( s->bytesTx == 0 ) s->bytesTx = cmd & 0xFF;
        /* E118-AUDIT: same reasoning as WRITE -- nothing else here is stateful before the timer
         * arms, so WOULD_BLOCK just needs to skip arming it and retry later. */
        if (writeReg( s->iomem.addr+A_I2C_CMD, cmd ) == VNEXT_WOULD_BLOCK) {
            s->vnextContinuationBacklogged = true;
            vnext_b_note_nonvcpu_backlog(0);
            return;
        }
        /* Mesma guarda do WRITE para byte/ACK eletrico. */
        time += (20*s->period_ns)/2;
    }break;

    case I2C_OPCODE_STOP:
    {
        //printf("Qemu: esp32_i2c CMD stop \n" ); fflush( stdout );
        if (writeReg( s->iomem.addr+A_I2C_CMD, cmd ) == VNEXT_WOULD_BLOCK) {
            s->vnextContinuationBacklogged = true;
            vnext_b_note_nonvcpu_backlog(0);
            return;
        }
        time = (3*s->period_ns)/2;
    }break;

    case I2C_OPCODE_END:
    {
        //printf("Qemu: esp32_i2c CMD end\n\n"); fflush( stdout );
        s->int_raw_reg |= 1<<3;          // END_DETECT
        esp32_i2c_update_irq(s);
    } break;
    default: break;                      // error_report("esp32_i2c: Invalid command %d opcode %d", i_cmd, opcode);
    }

    if( time )
    {
        time += getQemu_ns(); //qemu_clock_get_ns( QEMU_CLOCK_VIRTUAL );
        //printf("Qemu: i2c Event at %i %lu\n", s->number, time); fflush( stdout );
        timer_mod( &s->event_timer, time ); // Time to end byte transaction
    }
}

static void esp32_i2c_event( void* opaque ) // Timer event
{
    Esp32I2CState* s = Esp32_I2C(opaque);

    //printf("Qemu: esp32_i2c_event %i %lu\n", s->lastOpcode, getQemu_ps() ); fflush( stdout );

    if( s->burstActive )
    {
        esp32_i2c_finish_burst(s);
        return;
    }

    switch( s->lastOpcode )
    {
    case I2C_OPCODE_RSTART:
    {
    }break;

    case I2C_OPCODE_WRITE:
    {
        /* Sincronize o ACK eletrico apenas para o primeiro byte (endereco) depois de RSTART.
         * Dados do mesmo burst usam ACK assumido, evitando um round-trip Core/QEMU por byte. */
        uint8_t ackT = 0;
        if (s->ackSamplePending)
        {
            uint64_t status = readReg(s->iomem.addr + A_I2C_STATUS);
            ackT = (uint8_t)(((status & (1u << 4)) == 0) ? (status & 1u) : 0u);
            s->ackSamplePending = false;
        }
        if (fifo8_num_used(&s->tx_fifo) == 0) {
            error_report("esp32_i2c: stale write timer found an empty TX FIFO");
            esp32_i2c_abort_empty_tx(s);
            return;
        }
        fifo8_pop( &s->tx_fifo );
        s->bytesTx--;

        s->int_raw_reg &= ~(1<<6);            // Clear I2C_BYTE_TRANS

        s->sr_reg &= ~1;
        s->sr_reg |= ackT;                    // I2C_ACK_REC

        // ACK_ERR real: so' dispara quando o firmware pediu conferencia (ACK_CHECK_EN) E o ACK
        // observado de verdade no barramento (ackT) diverge do que o firmware esperava (ACK_EXP) --
        // antes disto, ackERR vinha so' dos bits que o proprio firmware escreveu (ACK_CHECK_EN/
        // ACK_EXP), nunca comparados contra o ACK real: um escravo que nunca respondesse (endereco
        // errado, dispositivo ausente) nunca gerava ACK_ERR nenhum.
        bool ackERR = false;
        uint32_t cmd = s->cmd_reg[s->lastCMD];

        if( cmd & 1<<8 )                      // ACK_CHECK_EN
        {
            bool ackExpectsNack = (cmd & 1<<9) != 0;  // ACK_EXP: 1 = firmware espera NACK deste byte
            ackERR = (ackT != 0) != ackExpectsNack;
        }

        if( ackERR ){
            s->int_raw_reg |= 1<<10;          // Set ACK_ERR

            if (esp32_i2c_trace_enabled()) {
                printf("Qemu: esp32_i2c_event ackERR\n"); fflush( stdout );
            }
            //return;
        }
        else s->int_raw_reg &= ~(1<<10);      // Clear ACK_ERR
    }break;

    case I2C_OPCODE_READ:
    {
        // Byte que o Core JA' recebeu eletricamente (8 bits clockados + ACK/NACK do MESTRE enviado
        // na janela certa, ver esp32_i2c_do_transaction acima) -- disponivel no mesmo canal privado
        // A_I2C_CMD que o WRITE usa pro sentido contrario.
        uint8_t data = (uint8_t)readReg( s->iomem.addr+A_I2C_CMD );
        if( fifo8_num_free(&s->rx_fifo) == 0 ) error_report("esp32_i2c: RX FIFO overflow");
        else fifo8_push( &s->rx_fifo, data );

        s->bytesTx--;
        s->int_raw_reg |= 1<<6;               // I2C_BYTE_TRANS
    }break;

    case I2C_OPCODE_STOP:
    {
        s->int_raw_reg |= 1<<7;            // TRANS_COMPLETE
        s->sr_reg      &= ~(1<<4);         // Clear I2C_BUS_BUSY
        s->burstAddressValid = false;
    }break;

    case I2C_OPCODE_END:
    {
        return;
    }
    default: return;                      // error_report("esp32_i2c: Invalid command %d opcode %d", i_cmd, opcode);
    }


    if( (s->lastOpcode == I2C_OPCODE_WRITE || s->lastOpcode == I2C_OPCODE_READ) && s->bytesTx )
    {
        ;
    }
    else{
        s->cmd_reg[s->lastCMD] = FIELD_DP32(s->cmd_reg[s->lastCMD], I2C_CMD, DONE, 1);
        s->lastCMD++;
    }

    esp32_i2c_do_transaction( s );
}

static void esp32_i2c_write_CTR( Esp32I2CState* s, uint16_t newCTR )
{
    if( s->ctr_reg == newCTR ) return;

    //printf("Qemu: esp32_i2c_write_CTR\n"); fflush( stdout );
    writeReg( s->iomem.addr+A_I2C_CTR, newCTR );

    if( newCTR & 1<<5 )               // bit 5: I2C_TRANS_START
    {
        s->lastCMD = 0;
        //printf("\nQemu: esp32_i2c_write_CTR TRANS %i\n", s->bytesTx ); fflush( stdout );
        esp32_i2c_do_transaction( s );

        newCTR &= ~(1<<5);            // Clear I2C_TRANS_START
    }
    s->ctr_reg = newCTR;
}

static uint32_t esp32_i2c_get_status_reg(Esp32I2CState* s)
{
    //uint32_t res = 0;
    //res = FIELD_DP32(res, I2C_STATUS, BUS_BUSY, s->trans_ongoing);

    uint32_t res = s->sr_reg;
    res = FIELD_DP32(res, I2C_STATUS, RXFIFO_CNT, fifo8_num_used(&s->rx_fifo));
    res = FIELD_DP32(res, I2C_STATUS, TXFIFO_CNT, fifo8_num_used(&s->tx_fifo));
    return res;
}

static void esp32_i2c_updt_frequency( Esp32I2CState* s )
{
    uint64_t fAPB = esp32_soc_get_apb_freq();
    uint64_t period_ns = s->low_period_reg+1 + s->high_period_reg+7;
    period_ns = period_ns*1000000000/fAPB;
    //uint32_t fSCL = fAPB / (s->low_period_reg + s->high_period_reg );

    if( s->period_ns == period_ns ) return;
    s->period_ns = period_ns;

    //printf("Qemu: esp32_i2c_updt_frequency: %i %lu, using %s %lu MHz\n", (s->low_period_reg + s->high_period_reg ), period_ns, "APB Clock", fAPB/1000000 );  fflush( stdout );
    writeReg( s->iomem.addr+A_I2C_LOW_PERIOD, period_ns );
}

static uint64_t esp32_i2c_read(void * opaque, hwaddr addr, unsigned int size)
{
    Esp32I2CState * s = Esp32_I2C(opaque);

    switch(addr) {
    case A_I2C_LOW_PERIOD:   return s->low_period_reg;
    case A_I2C_CTR:          return s->ctr_reg;
    case A_I2C_STATUS:       return esp32_i2c_get_status_reg(s);
    case A_I2C_TIMEOUT:      return s->timeout_reg;
    case A_I2C_FIFO_DATA: {
        if( fifo8_num_used(&s->rx_fifo) == 0) {
            /* Bounded, not silenced: 20,154 of these in one 60 s 16-session run
             * (E103) is a signal worth keeping, but an unbounded per-read report
             * floods the Core log buffer and evicts the reset records needed to
             * classify failures. */
            static unsigned empty_fifo_reports;
            if (esp32_i2c_trace_enabled() && empty_fifo_reports++ < 16)
                error_report("esp32_i2c: read I2C FIFO while it is empty");
            return 0xee;
        }
        uint8_t res = fifo8_pop(&s->rx_fifo);
        return res;
    }
    case A_I2C_INT_RAW:      return s->int_raw_reg;
    case A_I2C_INT_ENA:      return s->int_ena_reg;
    case A_I2C_INT_ST:       return s->int_raw_reg & s->int_ena_reg;
    case A_I2C_CMD ... (A_I2C_CMD + ESP32_I2C_CMD_COUNT * 4):
        return s->cmd_reg[(addr - A_I2C_CMD) / 4];
    case A_I2C_SDA_HOLD:     return s->sda_hold_reg;
    case A_I2C_SDA_SAMPLE:   return s->sda_sample_reg;
    case A_I2C_HIGH_PERIOD:  return s->high_period_reg;
    case A_I2C_START_HOLD:   return s->start_hold_reg;
    case A_I2C_RSTART_SETUP: return s->rstart_setup_reg;
    case A_I2C_STOP_HOLD:    return s->stop_hold_reg;
    case A_I2C_STOP_SETUP:   return s->stop_setup_reg;
    default:                 return 0;
    }
}

static void esp32_i2c_write(void * opaque, hwaddr addr, uint64_t value, unsigned int size)
{
    Esp32I2CState * s = Esp32_I2C(opaque);

    switch(addr) {
    case A_I2C_LOW_PERIOD:   s->low_period_reg   = value; esp32_i2c_updt_frequency( s ); break;
    case A_I2C_CTR:          esp32_i2c_write_CTR( s, value ); break;
    case A_I2C_TIMEOUT:      s->timeout_reg      = value; break;
    case A_I2C_FIFO_CONF:
        if( FIELD_EX32(value, I2C_FIFO_CONF, NONFIFO_EN) ) error_report("esp32_i2c: APB mode not implemented");
        if( FIELD_EX32(value, I2C_FIFO_CONF, RX_FIFO_RST)) fifo8_reset(&s->rx_fifo);
        if( FIELD_EX32(value, I2C_FIFO_CONF, TX_FIFO_RST)) {
            timer_del(&s->event_timer);
            fifo8_reset(&s->tx_fifo);
            s->bytesTx = 0;
            s->ackSamplePending = false;
            s->burstActive = false;
            s->burstPartial = false;
            s->burstStop = false;
            s->burstAddressValid = false;
            s->sr_reg &= ~(1 << 4);      /* I2C_BUS_BUSY */
            s->int_raw_reg &= ~(1 << 6); /* I2C_BYTE_TRANS */
        }
        break;
    case A_I2C_FIFO_DATA:
        if( fifo8_num_free(&s->tx_fifo) == 0) error_report("esp32_i2c: write to I2C TX FIFO while it is full");
        else {
            fifo8_push(&s->tx_fifo, value);
            //printf("fifo8_push %lu\n", value);
        }
        break;
    case A_I2C_INT_CLR: s->int_raw_reg &= ~value; esp32_i2c_update_irq(s); break;
    case A_I2C_INT_ENA: s->int_ena_reg  =  value; esp32_i2c_update_irq(s); break;
    case A_I2C_CMD ... (A_I2C_CMD + ESP32_I2C_CMD_COUNT * 4):
        s->cmd_reg[(addr - A_I2C_CMD) / 4] = value;
        break;
    case A_I2C_SDA_HOLD:     s->sda_hold_reg     = value; break;
    case A_I2C_SDA_SAMPLE:   s->sda_sample_reg   = value; break;
    case A_I2C_HIGH_PERIOD:  s->high_period_reg  = value; esp32_i2c_updt_frequency( s ); break;
    case A_I2C_START_HOLD:   s->start_hold_reg   = value; break;
    case A_I2C_RSTART_SETUP: s->rstart_setup_reg = value; break;
    case A_I2C_STOP_HOLD:    s->stop_hold_reg    = value; break;
    case A_I2C_STOP_SETUP:   s->stop_setup_reg   = value; break;
    default:  break;
    }
}

//static void esp32_i2c_do_transaction(Esp32I2CState* s)
//{
//    bool stop_or_end = false;
//
//    for( int i_cmd=0; i_cmd<ESP32_I2C_CMD_COUNT && !stop_or_end; ++i_cmd )
//    {
//        uint32_t cmd = s->cmd_reg[i_cmd];
//        char opcode = FIELD_EX32(cmd, I2C_CMD, OPCODE);
//        switch (opcode) {
//            case I2C_OPCODE_RSTART:
//                i2c_end_transfer(s->bus);
//                s->trans_ongoing = false;
//                break;
//            case I2C_OPCODE_WRITE: {
//                size_t length = FIELD_EX32(cmd, I2C_CMD, BYTE_NUM);
//                if( !s->trans_ongoing) {
//                    s->trans_ongoing = true;
//                    uint8_t data = fifo8_pop(&s->tx_fifo);
//                    uint8_t addr = data >> 1;
//                    uint8_t is_read = data & 0x1;
//                    if( i2c_start_transfer(s->bus, addr, is_read) != 0) {
//                        /* NACK */
//                        if( FIELD_EX32(cmd, I2C_CMD, ACK_CHECK_EN)
//                            && FIELD_EX32(cmd, I2C_CMD, ACK_EXP) == 0) {
//                            s->int_raw_reg = FIELD_DP32(s->int_raw_reg, I2C_INT_RAW, ACK_ERR, 1);
//                            stop_or_end = true;
//                        }
//                        s->trans_ongoing = false;
//                        break;
//                    }
//                    s->int_raw_reg = FIELD_DP32(s->int_raw_reg, I2C_INT_RAW, ACK_ERR, 0);
//                    length -= 1;
//                }
//                for( size_t nbytes = 0; nbytes < length; ++nbytes) {
//                    uint8_t data = fifo8_pop(&s->tx_fifo);
//                    i2c_send(s->bus, data);
//                }
//                break;
//            }
//            case I2C_OPCODE_READ: {
//                size_t length = FIELD_EX32(cmd, I2C_CMD, BYTE_NUM);
//                for( size_t nbytes = 0; nbytes < length; ++nbytes) {
//                    if( fifo8_num_free(&s->rx_fifo) == 0) {
//                        error_report("esp32_i2c: RX FIFO overflow");
//                    } else {
//                        uint8_t data = i2c_recv(s->bus);
//                        fifo8_push(&s->rx_fifo, data);
//                    }
//                }
//                break;
//            }
//            case I2C_OPCODE_STOP:
//                i2c_end_transfer(s->bus);
//                s->trans_ongoing = false;
//                s->int_raw_reg = FIELD_DP32(s->int_raw_reg, I2C_INT_RAW, TRANS_COMPLETE, 1);
//                stop_or_end = true;
//                break;
//            case I2C_OPCODE_END:
//                s->int_raw_reg = FIELD_DP32(s->int_raw_reg, I2C_INT_RAW, END_DETECT, 1);
//                stop_or_end = true;
//                break;
//            default:
//                error_report("esp32_i2c: Invalid command %d opcode %d", i_cmd, opcode);
//                break;
//        }
//        s->cmd_reg[i_cmd] = FIELD_DP32(s->cmd_reg[i_cmd], I2C_CMD, DONE, 1);
//    }
//    esp32_i2c_update_irq(s);
//}

static void esp32_i2c_reset(DeviceState * dev)
{
    Esp32I2CState * s = Esp32_I2C(dev);

    timer_del(&s->event_timer);
    fifo8_reset(&s->rx_fifo);
    fifo8_reset(&s->tx_fifo);
    s->period_ns = 0;
    s->ackSamplePending = false;
    /* E118-AUDIT: cancel any pending current_cpu==NULL continuation retry -- see the struct
     * comment on vnextContinuationBacklogged in esp32_i2c.h. No vCPU is ever parked for this
     * (unlike UART's tx_backlog_waiter[]), so there is nothing to wake, only to forget safely. */
    s->vnextContinuationBacklogged = false;
    s->burstActive = false;
    s->burstStop = false;
    s->burstAddressValid = false;
    s->trans_ongoing = false;
    s->ctr_reg = 0;
    s->timeout_reg = 0;
    s->int_ena_reg = 0;
    s->int_raw_reg = 0;
    s->sda_hold_reg = 0;
    s->sda_sample_reg = 0;
    s->high_period_reg = 0;
    s->low_period_reg = 0;
    s->start_hold_reg = 0;
    s->rstart_setup_reg = 0;
    s->stop_hold_reg = 0;
    s->stop_setup_reg = 0;
    memset(s->cmd_reg, 0, sizeof(s->cmd_reg));

    fifo8_reset(&s->tx_fifo);
    fifo8_reset(&s->rx_fifo);
}

static const MemoryRegionOps esp32_i2c_ops = {
    .read = esp32_i2c_read,
    .write = esp32_i2c_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_i2c_init(Object * obj)
{
    Esp32I2CState *s = Esp32_I2C(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_i2c_ops, s, TYPE_ESP32_I2C, ESP32_I2C_MEM_SIZE);
    /* E118-AUDIT-2 (EVIDENCE.md, 2026-09-05): softmmu/memory.c's generic per-device reentrancy
     * guard (mr->dev->mem_reentrancy_guard.engaged_in_io) is set true on entry to this device's
     * dispatch and only cleared by a NORMAL return from that dispatch (memory.c:573-575, a plain
     * post-call statement with no unwind-safety). vnext_b_gpio_write() -- reached from
     * esp32_i2c_write_CTR()'s writeReg() call, itself reached from THIS device's own .write
     * callback -- can respond to VNEXT_WOULD_BLOCK with cpu_stop_current() +
     * cpu_loop_exit_restore(), a siglongjmp back into cpu_exec() that skips every intervening
     * return, this cleanup included. Root-caused via the "Blocked re-entrant IO on MemoryRegion:
     * esp32.i2c" warning immediately preceding the E118-AUDIT gate's reset storm: the guard got
     * stuck true forever after the first such retry, permanently rejecting every later I2C access
     * (MEMTX_ACCESS_ERROR) for the rest of the process, which is what actually drove the guest's
     * own repeated watchdog-reset recovery cycle -- not a transport/backpressure timing issue.
     * Disabling the guard here is the same idiom already used by hw/intc/apic.c, hw/scsi/
     * lsi53c895a.c, hw/misc/bcm2835_property.c, hw/ppc/pnv_lpc.c and hw/intc/loongarch_ipi.c for
     * comparable cases. Safe specifically because vnext_b_gpio_write()'s retry path never drops
     * the BQL -- the actual hazard this guard defends against (a second vCPU dispatching into the
     * same device during a BQL-released window, e.g. the LEGACY arena's arenaTransactionBegin())
     * cannot occur through VNEXT_B's synchronous, BQL-held call chain. */
    s->iomem.disable_reentrancy_guard = true;
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    s->bus = i2c_init_bus(DEVICE(s), "i2c");

    fifo8_create( &s->tx_fifo, ESP32_I2C_FIFO_LENGTH );
    fifo8_create( &s->rx_fifo, ESP32_I2C_FIFO_LENGTH );

    timer_init_ns( &s->event_timer, QEMU_CLOCK_VIRTUAL, esp32_i2c_event, s );
}

static void esp32_i2c_class_init(ObjectClass* klass, void* data)
{
    DeviceClass * dc = DEVICE_CLASS(klass);
    dc->reset = esp32_i2c_reset;
}

static const TypeInfo esp32_i2c_type_info = {
    .name = TYPE_ESP32_I2C,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32I2CState),
    .instance_init = esp32_i2c_init,
    .class_init = esp32_i2c_class_init,
};

static void esp32_i2c_register_types(void)
{
    type_register_static(&esp32_i2c_type_info);
}

type_init(esp32_i2c_register_types)
