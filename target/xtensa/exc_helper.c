/*
 * Copyright (c) 2011 - 2019, Max Filippov, Open Source and Linux Lab.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Open Source and Linux Lab nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/main-loop.h"
#include "cpu.h"
#include "exec/helper-proto.h"
#include "qemu/host-utils.h"
#include "exec/exec-all.h"

/* ===== [XTENSA-EXC-TRACE] instrumentacao temporaria, ver .spec secao 32.5.9 =====
 * Achado ao vivo 2026-07-27 (.spec 32.5.7/32.5.8): a trap de acesso ilegal do DPORT
 * (esp32_cache_ill_read()) nunca dispara nas reproducoes do "Cache error", e a causa
 * PC_VALUE_ERROR_CAUSE (7 -- a que o ESP-IDF exibe como "Cache error", achado estatico de 32.5.4)
 * NUNCA e referenciada em nenhum lugar deste fork alem da propria definicao do enum em cpu.h --
 * ou seja, NADA no modelo de CPU levanta essa causa genuinamente. O rotulo "Cache error" tem que
 * vir de uma RELABELAGEM feita pelo proprio software do ESP-IDF (panic_soc_check_pseudo_cause(),
 * ja confirmado em 32.5.4) em cima de um evento REAL diferente. Esta instrumentacao captura esse
 * evento real -- a excecao xtensa genuina (sincrona) OU a interrupcao de alta prioridade (NMI) --
 * ANTES do firmware rodar seu proprio handler, a unica forma de ver a causa/PC verdadeiros.
 *
 * Dois pontos de captura, ambos filtrados pra excluir o que e rotineiro demais pra ser util (senao
 * o "primeiro evento" capturado seria um window overflow/underflow ou um tick de timer de nivel 1,
 * que acontecem constantemente e nada tem a ver com o bug):
 *   1. Excecoes SINCRONAS genuinas (EXC_KERNEL/EXC_USER/EXC_DOUBLE) em xtensa_cpu_do_interrupt(),
 *      ANTES de env->pc ser sobrescrito pelo vetor de excecao -- cobre qualquer causa real
 *      (ILLEGAL_INSTRUCTION, LOAD_STORE_ERROR, etc.), nao so as ja demonstradas como ausentes.
 *   2. Interrupcoes de ALTA PRIORIDADE (nivel > 1, especialmente nivel == nmi_level) dentro de
 *      handle_interrupt() -- candidato direto pra um watchdog (RTC/TIMG) disparando um NMI que o
 *      ESP-IDF trata como panic e rotula erroneamente como "Cache error".
 * Mesmo padrao de ring buffer + despejo throttled + captura permanente no primeiro evento
 * qualificado, ja validado em 32.5.7/32.5.8 (evita o Heisenbug de I/O por evento -- fprintf/fflush
 * por chamada muda timing o bastante pra mascarar corridas reais, ja documentado duas vezes nesta
 * investigacao). Sera revertido apos a causa raiz ser confirmada. */
#include "qemu/timer.h"
#include "exec/cpu-common.h"

#define XTENSA_EXC_TRACE_CAPACITY 256
#define XTENSA_EXC_TRACE_PATH "c:/tmp/lasecsimul_xtensa_exc_trace"
#define XTENSA_EXC_TRACE_FIRST_PATH "c:/tmp/lasecsimul_xtensa_exc_trace_FIRST"

typedef struct XtensaExcTraceEvent {
    uint64_t seq;
    int64_t virt_ns;
    int64_t host_ns;
    int cpu_index;
    uint32_t pc;
    uint32_t next_pc;
    uint32_t exccause;
    uint32_t excvaddr;
    uint32_t ps;
    uint32_t epc1;
    uint32_t epc2;
    int exception_index;
    int irq_level;
    hwaddr phys_addr;
    int phys_addr_valid;
    uint8_t insn_bytes[8];
    int insn_bytes_len;
    char cause_name[48];
    char access_type[16];
    char path[16];
    /* [XTENSA-PC-WATCH]/32.5.13: INTSET bruto (todas as linhas de CPU pendentes) no momento exato da
     * entrada -- permite confirmar se a linha 26 (compartilhada TG1 WDT / cache-IA neste build, achado
     * de 32.5.12) especificamente esta setada, e cruzar com o roteamento capturado em esp32_intc.c. */
    uint32_t intset;
} XtensaExcTraceEvent;

static XtensaExcTraceEvent xtensa_exc_trace_ring[XTENSA_EXC_TRACE_CAPACITY];
static uint64_t xtensa_exc_trace_seq;
static bool xtensa_exc_trace_first_captured;
static int64_t xtensa_exc_trace_last_dump_us;
#define XTENSA_EXC_TRACE_DUMP_THROTTLE_US 200000

static const char *xtensa_exc_trace_pid_path(const char *base)
{
    static char path[256];
    snprintf(path, sizeof(path), "%s.%" PRId64 ".log", base, (int64_t)getpid());
    return path;
}

/* Nomes reais do enum de causas em cpu.h (nao reinventa "numeros magicos" -- pedido explicito do
 * usuario nesta tarefa, mesmo criterio ja seguido pros bits MMU_IA_CLR). */
static const char *xtensa_exc_cause_name(uint32_t cause)
{
    switch (cause) {
    case ILLEGAL_INSTRUCTION_CAUSE: return "ILLEGAL_INSTRUCTION_CAUSE";
    case SYSCALL_CAUSE: return "SYSCALL_CAUSE";
    case INSTRUCTION_FETCH_ERROR_CAUSE: return "INSTRUCTION_FETCH_ERROR_CAUSE";
    case LOAD_STORE_ERROR_CAUSE: return "LOAD_STORE_ERROR_CAUSE";
    case LEVEL1_INTERRUPT_CAUSE: return "LEVEL1_INTERRUPT_CAUSE";
    case ALLOCA_CAUSE: return "ALLOCA_CAUSE";
    case INTEGER_DIVIDE_BY_ZERO_CAUSE: return "INTEGER_DIVIDE_BY_ZERO_CAUSE";
    case PC_VALUE_ERROR_CAUSE: return "PC_VALUE_ERROR_CAUSE";
    case PRIVILEGED_CAUSE: return "PRIVILEGED_CAUSE";
    case LOAD_STORE_ALIGNMENT_CAUSE: return "LOAD_STORE_ALIGNMENT_CAUSE";
    case INSTR_PIF_DATA_ERROR_CAUSE: return "INSTR_PIF_DATA_ERROR_CAUSE";
    case LOAD_STORE_PIF_DATA_ERROR_CAUSE: return "LOAD_STORE_PIF_DATA_ERROR_CAUSE";
    case INSTR_PIF_ADDR_ERROR_CAUSE: return "INSTR_PIF_ADDR_ERROR_CAUSE";
    case LOAD_STORE_PIF_ADDR_ERROR_CAUSE: return "LOAD_STORE_PIF_ADDR_ERROR_CAUSE";
    case INST_TLB_MISS_CAUSE: return "INST_TLB_MISS_CAUSE";
    case INST_TLB_MULTI_HIT_CAUSE: return "INST_TLB_MULTI_HIT_CAUSE";
    case INST_FETCH_PRIVILEGE_CAUSE: return "INST_FETCH_PRIVILEGE_CAUSE";
    case INST_FETCH_PROHIBITED_CAUSE: return "INST_FETCH_PROHIBITED_CAUSE";
    case LOAD_STORE_TLB_MISS_CAUSE: return "LOAD_STORE_TLB_MISS_CAUSE";
    case LOAD_STORE_TLB_MULTI_HIT_CAUSE: return "LOAD_STORE_TLB_MULTI_HIT_CAUSE";
    case LOAD_STORE_PRIVILEGE_CAUSE: return "LOAD_STORE_PRIVILEGE_CAUSE";
    case LOAD_PROHIBITED_CAUSE: return "LOAD_PROHIBITED_CAUSE";
    case STORE_PROHIBITED_CAUSE: return "STORE_PROHIBITED_CAUSE";
    default: return "UNKNOWN_CAUSE";
    }
}

/* Classificacao de tipo de acesso a partir da causa REAL (nomes/semantica do proprio enum, nao
 * inventados) -- fetch (busca de instrucao), load-store (dado) ou n/a (nao e falha de memoria). */
static const char *xtensa_exc_access_type(uint32_t cause)
{
    switch (cause) {
    case INSTRUCTION_FETCH_ERROR_CAUSE:
    case INSTR_PIF_DATA_ERROR_CAUSE:
    case INSTR_PIF_ADDR_ERROR_CAUSE:
    case INST_TLB_MISS_CAUSE:
    case INST_TLB_MULTI_HIT_CAUSE:
    case INST_FETCH_PRIVILEGE_CAUSE:
    case INST_FETCH_PROHIBITED_CAUSE:
        return "fetch";
    case LOAD_STORE_ERROR_CAUSE:
    case LOAD_STORE_PIF_DATA_ERROR_CAUSE:
    case LOAD_STORE_PIF_ADDR_ERROR_CAUSE:
    case LOAD_STORE_ALIGNMENT_CAUSE:
    case LOAD_STORE_TLB_MISS_CAUSE:
    case LOAD_STORE_TLB_MULTI_HIT_CAUSE:
    case LOAD_STORE_PRIVILEGE_CAUSE:
    case LOAD_PROHIBITED_CAUSE:
    case STORE_PROHIBITED_CAUSE:
        return "load-store";
    default:
        return "n/a";
    }
}

static void xtensa_exc_trace_format_event(FILE *f, const XtensaExcTraceEvent *ev)
{
    fprintf(f,
            "[%06" PRIu64 "] virt_ns=%" PRId64 " host_ns=%" PRId64 " path=%-10s cpu=%d"
            " pc=0x%08x next_pc=0x%08x exccause=%u(%s) excvaddr=0x%08x ps=0x%08x"
            " epc1=0x%08x epc2=0x%08x exception_index=%d irq_level=%d intset=0x%08x line26=%d"
            " phys_addr=%s0x%08" HWADDR_PRIx " access_type=%s insn_bytes=",
            ev->seq, ev->virt_ns, ev->host_ns, ev->path, ev->cpu_index,
            ev->pc, ev->next_pc, ev->exccause, ev->cause_name, ev->excvaddr, ev->ps,
            ev->epc1, ev->epc2, ev->exception_index, ev->irq_level, ev->intset,
            (ev->intset >> 26) & 1,
            ev->phys_addr_valid ? "" : "invalid:", ev->phys_addr, ev->access_type);
    for (int i = 0; i < ev->insn_bytes_len; ++i) {
        fprintf(f, "%02x", ev->insn_bytes[i]);
    }
    fprintf(f, "\n");
}

static void xtensa_exc_trace_dump(void)
{
    const int64_t now_us = g_get_monotonic_time();
    if (xtensa_exc_trace_last_dump_us != 0 &&
        now_us - xtensa_exc_trace_last_dump_us < XTENSA_EXC_TRACE_DUMP_THROTTLE_US) {
        return;
    }
    xtensa_exc_trace_last_dump_us = now_us;
    FILE *f = fopen(xtensa_exc_trace_pid_path(XTENSA_EXC_TRACE_PATH), "w");
    if (!f) {
        return;
    }
    const uint64_t total = xtensa_exc_trace_seq;
    const uint64_t count = total < XTENSA_EXC_TRACE_CAPACITY ? total : XTENSA_EXC_TRACE_CAPACITY;
    const uint64_t start = total - count;
    fprintf(f, "# [XTENSA-EXC-TRACE] %" PRIu64 " eventos totais (mostrando os ultimos %" PRIu64 ")\n",
            total, count);
    for (uint64_t seq = start; seq < total; ++seq) {
        xtensa_exc_trace_format_event(f, &xtensa_exc_trace_ring[seq % XTENSA_EXC_TRACE_CAPACITY]);
    }
    fclose(f);
}

/* Registra um evento e, se for o PRIMEIRO evento qualificado do processo inteiro, despeja
 * incondicionalmente e para sempre num arquivo separado que nunca e sobrescrito de novo --
 * garante que a causa raiz fique preservada mesmo que o processo depois gere muitos outros
 * eventos (cascata de resets) que dominariam um ring de tamanho fixo. */
static void xtensa_exc_trace_record(CPUXtensaState *env, const char *path, int exception_index,
                                     uint32_t next_pc, int irq_level, bool is_interrupt_class)
{
    CPUState *cs = env_cpu(env);
    XtensaExcTraceEvent *ev = &xtensa_exc_trace_ring[xtensa_exc_trace_seq % XTENSA_EXC_TRACE_CAPACITY];
    ev->seq = xtensa_exc_trace_seq++;
    ev->virt_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    ev->host_ns = get_clock();
    ev->cpu_index = cs->cpu_index;
    ev->pc = env->pc;
    ev->next_pc = next_pc;
    /* Para o ramo de interrupcao de alta prioridade (level > 1, nivel>1/NMI), o registrador
     * EXCCAUSE do hardware real NAO se aplica -- nao e uma excecao sincrona. Gravamos o campo
     * como o proprio nivel da interrupcao (nunca colide com os codigos de causa reais, que vao
     * so ate 29) para nao inventar um valor de EXCCAUSE que o hardware nunca produziria. */
    ev->exccause = is_interrupt_class ? (0xFFFF0000u | (uint32_t)irq_level) : env->sregs[EXCCAUSE];
    ev->excvaddr = env->sregs[EXCVADDR];
    ev->ps = env->sregs[PS];
    ev->epc1 = env->sregs[EPC1];
    ev->epc2 = env->config->ndepc ? env->sregs[DEPC] : 0;
    ev->exception_index = exception_index;
    ev->irq_level = irq_level;
    ev->intset = env->sregs[INTSET];
    snprintf(ev->path, sizeof(ev->path), "%s", path);
    if (is_interrupt_class) {
        snprintf(ev->cause_name, sizeof(ev->cause_name), "HIGH_PRIO_INTERRUPT_LEVEL_%d%s",
                 irq_level, irq_level == env->config->nmi_level ? "_NMI" : "");
        snprintf(ev->access_type, sizeof(ev->access_type), "interrupt");
    } else {
        snprintf(ev->cause_name, sizeof(ev->cause_name), "%s", xtensa_exc_cause_name(ev->exccause));
        snprintf(ev->access_type, sizeof(ev->access_type), "%s", xtensa_exc_access_type(ev->exccause));
    }

    ev->phys_addr_valid = 0;
    ev->phys_addr = (hwaddr)-1;
    hwaddr phys = cpu_get_phys_page_debug(cs, ev->pc);
    if (phys != (hwaddr)-1) {
        ev->phys_addr = phys;
        ev->phys_addr_valid = 1;
    }

    ev->insn_bytes_len = 0;
    {
        uint8_t buf[8];
        if (cpu_memory_rw_debug(cs, ev->pc, buf, sizeof(buf), 0) == 0) {
            memcpy(ev->insn_bytes, buf, sizeof(buf));
            ev->insn_bytes_len = sizeof(buf);
        }
    }

    xtensa_exc_trace_dump();

    if (!xtensa_exc_trace_first_captured) {
        xtensa_exc_trace_first_captured = true;
        FILE *f = fopen(xtensa_exc_trace_pid_path(XTENSA_EXC_TRACE_FIRST_PATH), "w");
        if (f) {
            fprintf(f, "# [XTENSA-EXC-TRACE] PRIMEIRA excecao/interrupcao de alta prioridade real "
                       "capturada neste processo -- ver .spec 32.5.9\n");
            xtensa_exc_trace_format_event(f, ev);
            fclose(f);
        }
    }
}
/* ===== fim [XTENSA-EXC-TRACE] ===== */

/* ===== [XTENSA-PC-WATCH] instrumentacao temporaria, ver .spec secao 32.5.10/32.5.11 =====
 * Escalada explicitamente autorizada pelo usuario apos 32.5.9/32.5.10 refutarem, em sequencia,
 * (a) qualquer excecao sincrona genuina de CPU precedendo o panic "Cache error" e (b) qualquer
 * leitura do bit illegal_access_status com valor setado -- em NENHUM dos ~85 boots reproduzidos
 * nesta investigacao (3 baterias de 30 ciclos). A correlacao por simbolos do ELF real do firmware
 * (via `nm`) aponta esp_cache_err_get_cpuid() (0x400e2170) e panic_soc_check_pseudo_cause()
 * (0x4016ec54, ja citado estaticamente desde 32.5.4) como os proximos pontos a observar -- a
 * hipotese e que esp_cache_err_get_cpuid() sempre retorna -1 (nem PRO nem APP com bit setado) e
 * ESP-IDF, ao ver cpuid==-1, cai num fallback que forca "Cache error"/EXCCAUSE=7 independente do
 * que realmente aconteceu.
 *
 * NAO e uma excecao -- nao chama cpu_loop_exit(), nao mexe em cs->exception_index nem env->pc.
 * E so uma observacao lateral (side-channel), disparada por um helper comum de TCG (nao um trap),
 * emitido em translate.c toda vez que a tradutora decodifica uma instrucao cujo PC bate com um dos
 * enderecos abaixo -- enderecos ESPECIFICOS deste build do firmware (obtidos via `nm` no ELF real
 * usado nesta bateria, nao genericos/hardcoded por suposicao). Sera revertido apos a causa raiz ser
 * confirmada (mesmo criterio das demais instrumentacoes desta investigacao). */
#define XTENSA_PC_WATCH_CACHE_ERR_GET_CPUID   0x400e2170u
#define XTENSA_PC_WATCH_PANIC_PSEUDO_CAUSE    0x4016ec54u
/* [XTENSA-PC-WATCH] adicionado em 32.5.14 -- escalada explicitamente autorizada ("pode fazer") apos
 * 32.5.13 identificar que a linha de CPU 26 (WDT do TIMER_GROUP1, confirmado por fonte==20) so vira
 * panic depois que o contador de tolerancia a livelock do workaround ECO3
 * (`_lx_intr_livelock_counter`/`_lx_intr_livelock_max`, `CONFIG_ESP32_ECO3_CACHE_LOCK_FIX`) se esgota.
 * `.handle_livelock_int` (endereco especifico deste build, achado via `nm`) roda em TODA entrada
 * genuina de WDT/cache-IA compartilhada, exigida ou nao a expirar depois -- observa-la mostra a
 * TRAJETORIA do contador ao longo do boot, correlacionavel por virt_ns com o handshake de cache ja
 * rastreado desde a rodada 2 (32.5.7/32.5.8), pra determinar se o contador sobe mais rapido durante
 * sequencias de disable/enable de cache (contencao real entre nucleos) ou independente delas
 * (artefato de agendamento do MTTCG sem correlacao com o estado do cache). */
#define XTENSA_PC_WATCH_HANDLE_LIVELOCK_INT   0x4008214cu
#define XTENSA_PC_WATCH_LIVELOCK_COUNTER_ADDR 0x3ffbdd40u
#define XTENSA_PC_WATCH_LIVELOCK_MAX_ADDR     0x3ffbdd44u
/* [XTENSA-PC-WATCH] adicionado em 32.5.15 -- 32.5.14 refutou a hipotese do contador de livelock
 * (`.handle_livelock_int` nunca roda) e achou um enigma novo: um ciclo que passou limpo mostrou os
 * dois nucleos chamando esp_cache_err_get_cpuid() a partir do MESMO ponto usado pelos ciclos que
 * falham -- mas pela leitura de panic_handler.c:166-199 (busy_wait()/arbitragem entre nucleos), isso
 * deveria SEMPRE terminar em panic_restart() (nunca retorna) pro nucleo com core_id==0. A contagem de
 * eventos de PC-watch nao e um sinal confiavel (dominada pelo throttle de despejo, ver 32.5.14) --
 * este watchpoint observa DIRETAMENTE a entrada em panic_restart(), o sinal mais direto e inequivoco
 * de "este nucleo esta comprometido a reiniciar" (a funcao e __attribute__((noreturn)) e so chama
 * esp_restart_noos[_dig]() em seguida -- nao ha caminho de volta depois dela). */
#define XTENSA_PC_WATCH_PANIC_RESTART          0x400e1b40u

#define XTENSA_PC_WATCH_CAPACITY 256
#define XTENSA_PC_WATCH_PATH "c:/tmp/lasecsimul_xtensa_pcwatch"

typedef struct XtensaPcWatchEvent {
    uint64_t seq;
    int64_t virt_ns;
    int64_t host_ns;
    int cpu_index;
    uint32_t wp_id;
    uint32_t pc;
    uint32_t a0; /* endereco de retorno -- de onde esta funcao foi chamada */
    uint32_t a2;
    uint32_t a3;
    uint32_t exccause;
    uint32_t excvaddr;
    uint32_t ps;
    /* Validos so quando wp_id==2 (.handle_livelock_int) -- lidos diretamente da memoria do guest
     * (enderecos fixos deste build, ver defines acima), nao de registradores de CPU. */
    uint32_t livelock_counter;
    uint32_t livelock_max;
} XtensaPcWatchEvent;

static XtensaPcWatchEvent xtensa_pc_watch_ring[XTENSA_PC_WATCH_CAPACITY];
static uint64_t xtensa_pc_watch_seq;

static const char *xtensa_pc_watch_name(uint32_t wp_id)
{
    switch (wp_id) {
    case 0: return "esp_cache_err_get_cpuid";
    case 1: return "panic_soc_check_pseudo_cause";
    case 2: return "handle_livelock_int";
    case 3: return "panic_restart";
    default: return "unknown";
    }
}

static void xtensa_pc_watch_write_file(void)
{
    FILE *f = fopen(xtensa_exc_trace_pid_path(XTENSA_PC_WATCH_PATH), "w");
    if (!f) {
        return;
    }
    const uint64_t total = xtensa_pc_watch_seq;
    const uint64_t count = total < XTENSA_PC_WATCH_CAPACITY ? total : XTENSA_PC_WATCH_CAPACITY;
    const uint64_t start = total - count;
    fprintf(f, "# [XTENSA-PC-WATCH] %" PRIu64 " entradas totais nas funcoes observadas (mostrando"
               " as ultimas %" PRIu64 ")\n", total, count);
    for (uint64_t seq = start; seq < total; ++seq) {
        const XtensaPcWatchEvent *ev = &xtensa_pc_watch_ring[seq % XTENSA_PC_WATCH_CAPACITY];
        fprintf(f,
                "[%06" PRIu64 "] virt_ns=%" PRId64 " host_ns=%" PRId64 " cpu=%d func=%s pc=0x%08x"
                " a0(ret)=0x%08x a2=0x%08x a3=0x%08x exccause=%u excvaddr=0x%08x ps=0x%08x"
                " livelock_counter=%u livelock_max=%u\n",
                ev->seq, ev->virt_ns, ev->host_ns, ev->cpu_index, xtensa_pc_watch_name(ev->wp_id),
                ev->pc, ev->a0, ev->a2, ev->a3, ev->exccause, ev->excvaddr, ev->ps,
                ev->livelock_counter, ev->livelock_max);
    }
    fclose(f);
}

/* [XTENSA-PC-WATCH] achado em 32.5.15: o harness (QemuProcessManager::reapProcess(), confirmado por
 * leitura direta) encerra o processo QEMU entre ciclos com TerminateProcess() -- um kill duro que
 * NUNCA roda atexit()/destrutores/qualquer cleanup do processo. Um despejo throttled (200ms de tempo
 * de parede real) corre o risco real de nunca chegar a refletir os ULTIMOS eventos antes desse kill
 * -- exatamente a janela mais critica pra esta investigacao (confirmar se panic_restart() roda antes
 * do processo morrer). Como os 4 watchpoints atuais empiricamente disparam no maximo ~5 vezes em todo
 * o boot (nunca a "torrente" de eventos por milissegundo que justificou o throttle no ring de
 * [XTENSA-EXC-TRACE]/[CACHE-TRACE] em rodadas anteriores), o throttle aqui NAO tem a mesma
 * justificativa de performance -- removido nesta rodada: despejo incondicional a cada evento,
 * garantindo que o arquivo em disco sempre reflita o estado mais recente do ring, custe o que custar
 * de TerminateProcess() interromper o processo a qualquer momento depois. */
static void xtensa_pc_watch_dump(void)
{
    xtensa_pc_watch_write_file();
}

void HELPER(trace_watched_pc)(CPUXtensaState *env, uint32_t pc, uint32_t wp_id)
{
    CPUState *cs = env_cpu(env);
    XtensaPcWatchEvent *ev = &xtensa_pc_watch_ring[xtensa_pc_watch_seq % XTENSA_PC_WATCH_CAPACITY];
    ev->seq = xtensa_pc_watch_seq++;
    ev->virt_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    ev->host_ns = get_clock();
    ev->cpu_index = cs->cpu_index;
    ev->wp_id = wp_id;
    ev->pc = pc;
    ev->a0 = env->regs[0];
    ev->a2 = env->regs[2];
    ev->a3 = env->regs[3];
    ev->exccause = env->sregs[EXCCAUSE];
    ev->excvaddr = env->sregs[EXCVADDR];
    ev->ps = env->sregs[PS];
    ev->livelock_counter = 0;
    ev->livelock_max = 0;
    if (wp_id == 2) {
        uint8_t buf[4];
        if (cpu_memory_rw_debug(cs, XTENSA_PC_WATCH_LIVELOCK_COUNTER_ADDR, buf, sizeof(buf), 0) == 0) {
            memcpy(&ev->livelock_counter, buf, sizeof(buf));
        }
        if (cpu_memory_rw_debug(cs, XTENSA_PC_WATCH_LIVELOCK_MAX_ADDR, buf, sizeof(buf), 0) == 0) {
            memcpy(&ev->livelock_max, buf, sizeof(buf));
        }
    }
    xtensa_pc_watch_dump();
}
/* ===== fim [XTENSA-PC-WATCH] ===== */

void HELPER(exception)(CPUXtensaState *env, uint32_t excp)
{
    CPUState *cs = env_cpu(env);

    cs->exception_index = excp;
    if (excp == EXCP_YIELD) {
        env->yield_needed = 0;
    }
    cpu_loop_exit(cs);
}

void HELPER(exception_cause)(CPUXtensaState *env, uint32_t pc, uint32_t cause)
{
    uint32_t vector;

    env->pc = pc;
    if (env->sregs[PS] & PS_EXCM) {
        if (env->config->ndepc) {
            env->sregs[DEPC] = pc;
        } else {
            env->sregs[EPC1] = pc;
        }
        vector = EXC_DOUBLE;
    } else {
        env->sregs[EPC1] = pc;
        vector = (env->sregs[PS] & PS_UM) ? EXC_USER : EXC_KERNEL;
    }

    env->sregs[EXCCAUSE] = cause;
    env->sregs[PS] |= PS_EXCM;

    HELPER(exception)(env, vector);
}

void HELPER(exception_cause_vaddr)(CPUXtensaState *env,
                                   uint32_t pc, uint32_t cause, uint32_t vaddr)
{
    env->sregs[EXCVADDR] = vaddr;
    HELPER(exception_cause)(env, pc, cause);
}

void debug_exception_env(CPUXtensaState *env, uint32_t cause)
{
    if (xtensa_get_cintlevel(env) < env->config->debug_level) {
        HELPER(debug_exception)(env, env->pc, cause);
    }
}

void HELPER(debug_exception)(CPUXtensaState *env, uint32_t pc, uint32_t cause)
{
    unsigned level = env->config->debug_level;

    env->pc = pc;
    env->sregs[DEBUGCAUSE] = cause;
    env->sregs[EPC1 + level - 1] = pc;
    env->sregs[EPS2 + level - 2] = env->sregs[PS];
    env->sregs[PS] = (env->sregs[PS] & ~PS_INTLEVEL) | PS_EXCM |
        (level << PS_INTLEVEL_SHIFT);
    HELPER(exception)(env, EXC_DEBUG);
}

#ifndef CONFIG_USER_ONLY

void HELPER(waiti)(CPUXtensaState *env, uint32_t pc, uint32_t intlevel)
{
    CPUState *cpu = env_cpu(env);

    env->pc = pc;
    env->sregs[PS] = (env->sregs[PS] & ~PS_INTLEVEL) |
        (intlevel << PS_INTLEVEL_SHIFT);

    qemu_mutex_lock_iothread();
    check_interrupts(env);
    qemu_mutex_unlock_iothread();

    if (env->pending_irq_level) {
        cpu_loop_exit(cpu);
        return;
    }

    cpu->halted = 1;
    HELPER(exception)(env, EXCP_HLT);
}

void HELPER(check_interrupts)(CPUXtensaState *env)
{
    qemu_mutex_lock_iothread();
    check_interrupts(env);
    qemu_mutex_unlock_iothread();
}

void HELPER(intset)(CPUXtensaState *env, uint32_t v)
{
    qatomic_or(&env->sregs[INTSET],
              v & env->config->inttype_mask[INTTYPE_SOFTWARE]);
}

static void intclear(CPUXtensaState *env, uint32_t v)
{
    qatomic_and(&env->sregs[INTSET], ~v);
}

void HELPER(intclear)(CPUXtensaState *env, uint32_t v)
{
    intclear(env, v & (env->config->inttype_mask[INTTYPE_SOFTWARE] |
                       env->config->inttype_mask[INTTYPE_EDGE]));
}

static uint32_t relocated_vector(CPUXtensaState *env, uint32_t vector)
{
    if (xtensa_option_enabled(env->config,
                              XTENSA_OPTION_RELOCATABLE_VECTOR)) {
        return vector - env->config->vecbase + env->sregs[VECBASE];
    } else {
        return vector;
    }
}

/*!
 * Handle penging IRQ.
 * For the high priority interrupt jump to the corresponding interrupt vector.
 * For the level-1 interrupt convert it to either user, kernel or double
 * exception with the 'level-1 interrupt' exception cause.
 */
static void handle_interrupt(CPUXtensaState *env)
{
    int level = env->pending_irq_level;

    if ((level > xtensa_get_cintlevel(env) &&
         level <= env->config->nlevel &&
         (env->config->level_mask[level] &
          env->sregs[INTSET] & env->sregs[INTENABLE])) ||
        level == env->config->nmi_level) {
        CPUState *cs = env_cpu(env);

        if (level > 1) {
            /* env->config->nlevel check should have ensured this */
            assert(level < sizeof(env->config->interrupt_vector));

            /* [XTENSA-EXC-TRACE] captura ANTES de qualquer campo (pc/ps) ser sobrescrito --
             * candidato direto a NMI de watchdog (RTC/TIMG) mal-rotulada como "Cache error"
             * pelo ESP-IDF; ver .spec 32.5.9. */
            xtensa_exc_trace_record(env, "handle_irq", cs->exception_index,
                                     relocated_vector(env, env->config->interrupt_vector[level]),
                                     level, true);

            env->sregs[EPC1 + level - 1] = env->pc;
            env->sregs[EPS2 + level - 2] = env->sregs[PS];
            env->sregs[PS] =
                (env->sregs[PS] & ~PS_INTLEVEL) | level | PS_EXCM;
            env->pc = relocated_vector(env,
                                       env->config->interrupt_vector[level]);
            if (level == env->config->nmi_level) {
                intclear(env, env->config->inttype_mask[INTTYPE_NMI]);
            }
        } else {
            env->sregs[EXCCAUSE] = LEVEL1_INTERRUPT_CAUSE;

            if (env->sregs[PS] & PS_EXCM) {
                if (env->config->ndepc) {
                    env->sregs[DEPC] = env->pc;
                } else {
                    env->sregs[EPC1] = env->pc;
                }
                cs->exception_index = EXC_DOUBLE;
            } else {
                env->sregs[EPC1] = env->pc;
                cs->exception_index =
                    (env->sregs[PS] & PS_UM) ? EXC_USER : EXC_KERNEL;
            }
            env->sregs[PS] |= PS_EXCM;
        }
    }
}

/* Called from cpu_handle_interrupt with BQL held */
void xtensa_cpu_do_interrupt(CPUState *cs)
{
    XtensaCPU *cpu = XTENSA_CPU(cs);
    CPUXtensaState *env = &cpu->env;

    if (cs->exception_index == EXC_IRQ) {
        qemu_log_mask(CPU_LOG_INT,
                      "%s(EXC_IRQ) level = %d, cintlevel = %d, "
                      "pc = %08x, a0 = %08x, ps = %08x, "
                      "intset = %08x, intenable = %08x, "
                      "ccount = %08x\n",
                      __func__, env->pending_irq_level,
                      xtensa_get_cintlevel(env),
                      env->pc, env->regs[0], env->sregs[PS],
                      env->sregs[INTSET], env->sregs[INTENABLE],
                      env->sregs[CCOUNT]);
        handle_interrupt(env);
    }

    switch (cs->exception_index) {
    case EXC_WINDOW_OVERFLOW4:
    case EXC_WINDOW_UNDERFLOW4:
    case EXC_WINDOW_OVERFLOW8:
    case EXC_WINDOW_UNDERFLOW8:
    case EXC_WINDOW_OVERFLOW12:
    case EXC_WINDOW_UNDERFLOW12:
    case EXC_KERNEL:
    case EXC_USER:
    case EXC_DOUBLE:
    case EXC_DEBUG:
        qemu_log_mask(CPU_LOG_INT, "%s(%d) "
                      "pc = %08x, a0 = %08x, ps = %08x, ccount = %08x\n",
                      __func__, cs->exception_index,
                      env->pc, env->regs[0], env->sregs[PS],
                      env->sregs[CCOUNT]);
        if (env->config->exception_vector[cs->exception_index]) {
            uint32_t vector;

            vector = env->config->exception_vector[cs->exception_index];
            /* [XTENSA-EXC-TRACE] so as tres excecoes SINCRONAS genuinas -- deliberadamente
             * excluindo EXC_WINDOW_OVERFLOW e EXC_WINDOW_UNDERFLOW (rotina de ABI de janela de
             * registradores, disparam o tempo todo e nao tem nada a ver com o bug) e EXC_DEBUG (irrelevante aqui).
             * Captura ANTES de env->pc ser sobrescrito pelo vetor -- unico jeito de ver a causa e o
             * PC reais antes do handler de excecao (e depois o panicHandler do ESP-IDF) rodar em
             * cima disso. Ver .spec 32.5.9. */
            /* LEVEL1_INTERRUPT_CAUSE aqui NAO e uma excecao sincrona genuina -- e qualquer
             * interrupcao de nivel 1 (UART, tick de timer, etc.) que handle_interrupt() disfarça
             * de EXC_KERNEL/EXC_USER. E extremamente frequente (milhares de eventos por batalha)
             * e afogou o ring de 256 entradas antes de qualquer excecao sincrona real sobreviver
             * ate o dump -- achado ao vivo nesta rodada, ver .spec 32.5.9. So registra causas
             * sincronas de verdade (ILLEGAL_INSTRUCTION, LOAD/STORE error, TLB, privilegio, etc.). */
            if ((cs->exception_index == EXC_KERNEL || cs->exception_index == EXC_USER ||
                 cs->exception_index == EXC_DOUBLE) &&
                env->sregs[EXCCAUSE] != LEVEL1_INTERRUPT_CAUSE) {
                xtensa_exc_trace_record(env, "do_interrupt", cs->exception_index,
                                         relocated_vector(env, vector), 0, false);
            }
            env->pc = relocated_vector(env, vector);
        } else {
            qemu_log_mask(CPU_LOG_INT,
                          "%s(pc = %08x) bad exception_index: %d\n",
                          __func__, env->pc, cs->exception_index);
        }
        break;

    case EXC_IRQ:
        break;

    default:
        qemu_log("%s(pc = %08x) unknown exception_index: %d\n",
                 __func__, env->pc, cs->exception_index);
        break;
    }
    check_interrupts(env);
}

bool xtensa_cpu_exec_interrupt(CPUState *cs, int interrupt_request)
{
    if (interrupt_request & CPU_INTERRUPT_HARD) {
        cs->exception_index = EXC_IRQ;
        xtensa_cpu_do_interrupt(cs);
        return true;
    }
    return false;
}

#endif /* !CONFIG_USER_ONLY */
