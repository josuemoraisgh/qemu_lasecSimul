/*
 * ESP32 "DPORT" device
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "hw/registerfields.h"
#include "hw/boards.h"
#include "hw/misc/esp32_reg.h"
#include "hw/misc/esp32_dport.h"

#include "hw/misc/esp32_flash_enc.h"
#include "hw/nvram/esp32_efuse.h"

/* ===== [CACHE-TRACE] instrumentação temporária, ver .spec secao 32.5.7 =====
 * Investigacao do "Cache error" (Guru Meditation) documentado em 32.5.1-32.5.6. Ring buffer em
 * memoria (sem I/O por evento no caminho quente -- cache_ill_read()/dport_write() disparam raro,
 * entao o fwrite por evento abaixo e barato) despejado por inteiro a cada evento num arquivo de
 * tamanho limitado (capacidade fixa do ring, nunca cresce sem limite -- ver achado de disco cheio
 * em 32.5.4). Sera revertido apos a causa raiz ser confirmada (criterio de aceitacao: "todas as
 * alteracoes experimentais foram removidas"). */
#include "qemu/timer.h"
#include "hw/core/cpu.h"

#define CACHE_TRACE_ENABLED 1
/* Achado ao vivo 2026-07-27: com 800 o ring dava só ~1 segundo virtual de historico antes de dar a
 * volta -- o loop de disable/enable de cache do ADC (ver abaixo) gera dezenas de eventos por
 * milissegundo virtual, afogando o evento raro (ill_read) bem antes do despejo seguinte. Subido
 * para 20000 (ainda um array de tamanho fixo, ~2,3MB -- não é um log ilimitado) só para ter
 * historico o bastante. Mitigado tambem por CACHE_TRACE_FIRST_PATH abaixo, que grava o primeiro
 * ill_read incondicionalmente e para sempre, independente do que aconteça com o ring depois. */
#define CACHE_TRACE_CAPACITY 20000
#define CACHE_TRACE_PATH "c:/tmp/lasecsimul_cache_trace.log"
/* Despejado uma unica vez, na primeira vez que illegal_access_status latcheia de verdade (nao em
 * hits com illegal_access_trap_en==false) -- preserva a janela critica mesmo que o ring geral
 * continue girando e sobrescrevendo depois. */
#define CACHE_TRACE_FIRST_PATH "c:/tmp/lasecsimul_cache_trace_FIRST.log"
/* Achado ao vivo 2026-07-27 (retomada pos-crash do editor): illegal_access_status NUNCA latcheia
 * nas reproducoes ao vivo (ver .spec 32.5.7) -- CACHE_TRACE_FIRST_PATH nunca dispara, entao nao
 * serve pra capturar a janela critica de uma falha real. Em vez disso, cada ciclo do harness sobe
 * um processo QEMU NOVO (fresh ring buffer) e o CACHE_TRACE_PATH fixo e sobrescrito a cada ciclo --
 * um ciclo que falha e seguido por um ciclo BOM (proximo processo) apaga os dados do ciclo ruim
 * antes de conseguirmos ler o arquivo. Corrigido de duas formas: (1) o path inclui o PID, entao
 * cada processo QEMU de uma bateria de N ciclos deixa seu proprio arquivo, nenhum se sobrescreve;
 * (2) captura permanente adicional disparada no primeiro reset "inesperado" (SW_CPU_RESET alem dos
 * dois resets de boot normais -- ver esp32_log_reset()/expected_app_cpu_boot_reset em esp32.c, MESMA
 * logica de contagem espelhada aqui) -- esse reset e a CONSEQUENCIA observavel do panic quando o
 * trap de acesso ilegal deste modelo nao e a causa (ja confirmado 3x), entao serve como gatilho
 * alternativo confiavel pra preservar a janela de eventos que levou ate ele. */
#define CACHE_TRACE_UNEXPECTED_RESET_PATH "c:/tmp/lasecsimul_cache_trace_UNEXPECTED_RESET"
static int cache_trace_reset_events_seen;
static bool cache_trace_unexpected_reset_captured;

typedef struct CacheTraceEvent {
    uint64_t seq;
    int64_t virt_ns;
    int64_t host_ns;
    char tag[32];
    int core;          /* nucleo "dono" do registro/regiao envolvida (nao necessariamente quem escreveu) */
    uint32_t pc;        /* PC do nucleo `core` acima, no momento do evento */
    int writer_core;    /* nucleo que efetivamente executou este acesso (current_cpu), -1 se desconhecido */
    uint32_t writer_pc;
    /* PC de AMBOS os nucleos, sempre preenchido independente do tipo de evento -- pedido explicito
     * pra verificar o handshake spi_flash_op_block_func/esp_ipc_isr completo (.spec 32.5.8): permite
     * ver o que o OUTRO nucleo estava fazendo em qualquer evento, nao so no dono do registro. */
    uint32_t pc0;
    uint32_t pc1;
    uint64_t vaddr;
    uint32_t size;
    /* [core][region] regiao: 0=drom0 1=iram0 2=dram1 */
    uint8_t cache_ena[2][3];
    uint8_t ill_status[2][3];
    int ill_irq_level;
    uint32_t reg_addr;
    uint32_t old_val;
    uint32_t new_val;
} CacheTraceEvent;

static CacheTraceEvent cache_trace_ring[CACHE_TRACE_CAPACITY];
static uint64_t cache_trace_seq;
static int cache_trace_ill_irq_level;
static bool cache_trace_first_ill_captured;

static void cache_trace_current_writer(int *writer_core, uint32_t *writer_pc)
{
    *writer_core = -1;
    *writer_pc = 0;
    CPUState *cs = current_cpu;
    if (!cs) {
        return;
    }
    for (int c = 0; c < ESP32_CPU_COUNT; ++c) {
        if (qemu_get_cpu(c) == cs) {
            *writer_core = c;
            break;
        }
    }
    CPUClass *cc = CPU_GET_CLASS(cs);
    if (cc && cc->get_pc) {
        *writer_pc = (uint32_t)cc->get_pc(cs);
    }
}

static void cache_trace_snapshot(Esp32DportState *s, uint8_t ena[2][3], uint8_t status[2][3])
{
    for (int c = 0; c < ESP32_CPU_COUNT; ++c) {
        ena[c][0] = s->cache_state[c].drom0.mem.enabled;
        ena[c][1] = s->cache_state[c].iram0.mem.enabled;
        ena[c][2] = s->cache_state[c].dram1.mem.enabled;
        status[c][0] = s->cache_state[c].drom0.illegal_access_status;
        status[c][1] = s->cache_state[c].iram0.illegal_access_status;
        status[c][2] = s->cache_state[c].dram1.illegal_access_status;
    }
}

/* Achado ao vivo 2026-07-27: despejar o arquivo inteiro a CADA evento travou uma bateria de 20
 * ciclos por mais de 240s (nenhum ciclo alem do 0 chegou a rodar) -- se esp32_cache_ill_read()
 * dispara em alta frequencia (ex.: um nucleo preso buscando instrucao repetidamente de uma regiao
 * desabilitada), reabrir/reescrever o arquivo a cada chamada vira o gargalo (fopen("w") + N linhas
 * de fprintf, possivelmente agravado por antivirus escaneando o arquivo reescrito repetidamente).
 * O ring buffer em MEMORIA continua atualizado a cada evento (barato, sem I/O); só o DESPEJO em
 * disco é limitado a no maximo uma vez a cada kDumpThrottleUs de tempo de parede real -- garante
 * uma taxa de escrita limitada mesmo sob uma rajada de eventos, sem perder resolucao no ring (o
 * conteudo em memoria no momento do proximo despejo permitido ainda reflete os eventos mais
 * recentes). */
static int64_t cache_trace_last_dump_us;
#define CACHE_TRACE_DUMP_THROTTLE_US 200000

/* Constroi "<base>.<pid>.log" -- ver comentario de CACHE_TRACE_UNEXPECTED_RESET_PATH acima sobre
 * por que o PID entra no nome. Achado ao vivo 2026-07-27: a primeira versao cacheava o buffer só
 * por PID (getpid() nao muda dentro do processo) -- mas essa funcao e chamada com bases DIFERENTES
 * (CACHE_TRACE_PATH pelo despejo regular, CACHE_TRACE_UNEXPECTED_RESET_PATH pela captura de reset
 * inesperado), e o cache so verificava se o PID batia, nao se `base` batia -- assim que qualquer
 * chamada com QUALQUER base preenchia o cache, TODAS as chamadas seguintes no mesmo processo
 * (mesmo com uma base diferente) recebiam de volta o path ERRADO (o da primeira base usada nesse
 * processo -- na pratica, sempre CACHE_TRACE_PATH, cujo despejo regular e chamado a cada evento
 * desde o primeiro). Resultado: nenhum arquivo CACHE_TRACE_UNEXPECTED_RESET_PATH era criado, seus
 * dados iam pro arquivo do despejo regular sem aviso. Corrigido formatando a string toda vez (custo
 * desprezivel -- so acontece nos poucos eventos que realmente despejam algo em disco, nunca no
 * caminho quente de gravar no ring em memoria). */
static const char *cache_trace_pid_path(const char *base)
{
    static char path[256];
    snprintf(path, sizeof(path), "%s.%" PRId64 ".log", base, (int64_t)getpid());
    return path;
}

static void cache_trace_format_event(FILE *f, const CacheTraceEvent *ev)
{
    fprintf(f,
            "[%06" PRIu64 "] virt_ns=%" PRId64 " host_ns=%" PRId64 " tag=%-24s core=%d pc=0x%08x"
            " writer_core=%d writer_pc=0x%08x pc0=0x%08x pc1=0x%08x"
            " vaddr=0x%08" PRIx64 " size=%u reg=0x%03x old=0x%08x new=0x%08x ill_irq=%d"
            " cache_ena[0]={drom0=%d iram0=%d dram1=%d} cache_ena[1]={drom0=%d iram0=%d dram1=%d}"
            " ill_status[0]={drom0=%d iram0=%d dram1=%d} ill_status[1]={drom0=%d iram0=%d dram1=%d}\n",
            ev->seq, ev->virt_ns, ev->host_ns, ev->tag, ev->core, ev->pc,
            ev->writer_core, ev->writer_pc, ev->pc0, ev->pc1, ev->vaddr, ev->size,
            ev->reg_addr, ev->old_val, ev->new_val, ev->ill_irq_level,
            ev->cache_ena[0][0], ev->cache_ena[0][1], ev->cache_ena[0][2],
            ev->cache_ena[1][0], ev->cache_ena[1][1], ev->cache_ena[1][2],
            ev->ill_status[0][0], ev->ill_status[0][1], ev->ill_status[0][2],
            ev->ill_status[1][0], ev->ill_status[1][1], ev->ill_status[1][2]);
}

static uint32_t cache_trace_pc_of(int core)
{
    if (core < 0 || core >= ESP32_CPU_COUNT) {
        return 0;
    }
    CPUState *cs = qemu_get_cpu(core);
    if (!cs) {
        return 0;
    }
    CPUClass *cc = CPU_GET_CLASS(cs);
    return (cc && cc->get_pc) ? (uint32_t)cc->get_pc(cs) : 0;
}

static void cache_trace_dump(void)
{
    const int64_t now_us = g_get_monotonic_time();
    if (cache_trace_last_dump_us != 0 &&
        now_us - cache_trace_last_dump_us < CACHE_TRACE_DUMP_THROTTLE_US) {
        return;
    }
    cache_trace_last_dump_us = now_us;

    FILE *f = fopen(cache_trace_pid_path(CACHE_TRACE_PATH), "w");
    if (!f) {
        return;
    }
    const uint64_t total = cache_trace_seq;
    const uint64_t count = total < CACHE_TRACE_CAPACITY ? total : CACHE_TRACE_CAPACITY;
    const uint64_t start = total - count;
    fprintf(f, "# [CACHE-TRACE] %" PRIu64 " eventos totais (mostrando os ultimos %" PRIu64 ")\n",
            total, count);
    for (uint64_t seq = start; seq < total; ++seq) {
        cache_trace_format_event(f, &cache_trace_ring[seq % CACHE_TRACE_CAPACITY]);
    }
    fclose(f);
}

static void cache_trace_record(Esp32DportState *s, const char *tag, int core,
                                uint64_t vaddr, uint32_t size,
                                uint32_t reg_addr, uint32_t old_val, uint32_t new_val)
{
    if (!CACHE_TRACE_ENABLED) {
        return;
    }
    CacheTraceEvent *ev = &cache_trace_ring[cache_trace_seq % CACHE_TRACE_CAPACITY];
    ev->seq = cache_trace_seq++;
    ev->virt_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    ev->host_ns = get_clock();
    snprintf(ev->tag, sizeof(ev->tag), "%s", tag);
    ev->core = core;
    /* hw/misc e codigo "common" (compilado sem headers target-especificos) -- usa o hook generico
     * CPUClass::get_pc() em vez de tocar CPUXtensaState diretamente (isso exigiria
     * target/xtensa/cpu.h, que a build recusa incluir fora do codigo per-target). pc0/pc1 sao
     * sempre preenchidos, independente de qual "core" for o dono do registro deste evento --
     * necessario pra verificar o handshake completo (quem estava fazendo o que nos dois nucleos a
     * cada transicao, nao so no dono do registro tocado). */
    ev->pc = cache_trace_pc_of(core);
    ev->pc0 = cache_trace_pc_of(0);
    ev->pc1 = cache_trace_pc_of(1);
    cache_trace_current_writer(&ev->writer_core, &ev->writer_pc);
    ev->vaddr = vaddr;
    ev->size = size;
    if (s) {
        cache_trace_snapshot(s, ev->cache_ena, ev->ill_status);
    } else {
        memset(ev->cache_ena, 0, sizeof(ev->cache_ena));
        memset(ev->ill_status, 0, sizeof(ev->ill_status));
    }
    ev->ill_irq_level = cache_trace_ill_irq_level;
    ev->reg_addr = reg_addr;
    ev->old_val = old_val;
    ev->new_val = new_val;
    cache_trace_dump();
}

/* Despeja uma JANELA de eventos ate o mais recente incondicionalmente, num arquivo proprio
 * (nomeado por PID + `reason`, nunca reescrito de novo -- ver comentario de
 * CACHE_TRACE_UNEXPECTED_RESET_PATH acima). Independe do tamanho do ring geral/da taxa de despejo
 * throttled: dispara sempre que o evento critico acontece, ainda que CACHE_TRACE_CAPACITY seja
 * pequeno demais pra cobrir todo o boot (o loop de disable/enable de cache do spi_flash, achado ao
 * vivo nesta sessao, gera demais eventos de rotina pra caber um historico completo num ring de
 * tamanho fixo modesto). */
#define CACHE_TRACE_WINDOW 500

static void cache_trace_capture_window(const char *path_base, const char *reason)
{
    if (cache_trace_seq == 0) {
        return;
    }
    FILE *f = fopen(cache_trace_pid_path(path_base), "w");
    if (!f) {
        return;
    }
    const uint64_t total = cache_trace_seq;
    const uint64_t window = total < CACHE_TRACE_WINDOW ? total : CACHE_TRACE_WINDOW;
    const uint64_t ring_count = total < CACHE_TRACE_CAPACITY ? total : CACHE_TRACE_CAPACITY;
    const uint64_t available = window < ring_count ? window : ring_count;
    const uint64_t start = total - available;
    fprintf(f, "# [CACHE-TRACE] %s -- ultimos %" PRIu64 " eventos ate ele (evento critico e o"
               " ultimo desta lista)\n", reason, available);
    for (uint64_t seq = start; seq < total; ++seq) {
        cache_trace_format_event(f, &cache_trace_ring[seq % CACHE_TRACE_CAPACITY]);
    }
    fclose(f);
}

static void cache_trace_capture_first_illegal_access(void)
{
    if (cache_trace_first_ill_captured) {
        return;
    }
    cache_trace_first_ill_captured = true;
    cache_trace_capture_window(CACHE_TRACE_FIRST_PATH,
        "primeiro acesso ilegal real (illegal_access_status latcheado)");
}

/* Usado por dispositivos que nao tem um Esp32DportState* a mao (ex.: esp32_crosscore_int.c) --
 * ver .spec 32.5.8, instrumentacao do handshake completo de esp_ipc_isr/spi_flash_op_block_func.
 * `val` carrega o payload especifico do chamador (ex.: nivel da interrupcao cross-core). */
void esp32_cache_trace_generic_event(const char *tag, int core, uint64_t vaddr, uint32_t val)
{
    cache_trace_record(NULL, tag, core, vaddr, 0, 0, 0, val);
}

void esp32_cache_trace_reset_event(Esp32DportState *s, const char *tag, int core, uint32_t cause)
{
    cache_trace_record(s, tag, core, 0, 0, 0xFFFFFFFF, cause, 0);

    /* Acompanha a MESMA logica de esp32_log_reset()/expected_app_cpu_boot_reset em esp32.c, mas
     * contando só os eventos que passam por ESTAS quatro GPIOs (o reset inicial de boot,
     * count=1/mask=0x0f, é disparado direto por qemu_system_reset_request() na inicializacao da
     * maquina, NUNCA por uma destas funcoes -- confirmado comparando com o log real de
     * esp32_log_reset em .spec 32.5.7/lasecsimul.spec: só o reset "app-cpu-startup" (count=2,
     * mask=APPCPU, cause=SW_CPU_RESET) passa por esp32_cpu_reset() em bateria limpa). Ou seja, só 1
     * evento "reset_cpu_sw"-family é esperado por ciclo normal -- qualquer um a partir do 2o é a
     * CONSEQUENCIA observavel de um panic (cascata documentada em .spec 32.5.1). */
    if (strcmp(tag, "reset_cpu_sw") == 0 || strcmp(tag, "reset_timg_cpu") == 0 ||
        strcmp(tag, "reset_dig") == 0 || strcmp(tag, "reset_timg_sys") == 0) {
        ++cache_trace_reset_events_seen;
        if (cache_trace_reset_events_seen > 1 && !cache_trace_unexpected_reset_captured) {
            cache_trace_unexpected_reset_captured = true;
            cache_trace_capture_window(CACHE_TRACE_UNEXPECTED_RESET_PATH,
                "primeiro reset inesperado (alem do 1 esperado -- app-cpu-startup)");
        }
    }
}
/* ===== fim [CACHE-TRACE] ===== */


#define ESP32_DPORT_SIZE        (DR_REG_DPORT_APB_BASE - DR_REG_DPORT_BASE)

#define MMU_RANGE_SIZE          (ESP32_CACHE_PAGES_PER_REGION * sizeof(uint32_t))
#define MMU_RANGE_LAST          (MMU_RANGE_SIZE - sizeof(uint32_t))

#define PRO_DROM0_MMU_FIRST     (DR_REG_FLASH_MMU_TABLE_PRO - DR_REG_DPORT_BASE)
#define PRO_DROM0_MMU_LAST      (PRO_DROM0_MMU_FIRST + MMU_RANGE_LAST)
#define PRO_IRAM0_MMU_FIRST     (DR_REG_FLASH_MMU_TABLE_PRO - DR_REG_DPORT_BASE + MMU_RANGE_SIZE)
#define PRO_IRAM0_MMU_LAST      (PRO_IRAM0_MMU_FIRST + MMU_RANGE_LAST)
#define APP_DROM0_MMU_FIRST     (DR_REG_FLASH_MMU_TABLE_APP - DR_REG_DPORT_BASE)
#define APP_DROM0_MMU_LAST      (APP_DROM0_MMU_FIRST + MMU_RANGE_LAST)
#define APP_IRAM0_MMU_FIRST     (DR_REG_FLASH_MMU_TABLE_APP - DR_REG_DPORT_BASE + MMU_RANGE_SIZE)
#define APP_IRAM0_MMU_LAST      (APP_IRAM0_MMU_FIRST + MMU_RANGE_LAST)
#define MMU_ENTRY_MASK          0x1ff

static void esp32_cache_state_update(Esp32CacheState* cs);
static void esp32_cache_data_sync(Esp32CacheRegionState* crs);
static void esp32_cache_invalidate_all_entries(Esp32CacheRegionState* crs);

static inline uint32_t get_mmu_entry(Esp32CacheRegionState* crs, hwaddr base, hwaddr addr)
{
    return crs->mmu_table[(addr - base)/sizeof(uint32_t)] & MMU_ENTRY_MASK;
}

static inline void set_mmu_entry(Esp32CacheRegionState* crs, hwaddr base, hwaddr addr, uint64_t val)
{
    uint32_t old_val = crs->mmu_table[(addr - base)/sizeof(uint32_t)];
    if (val != old_val) {
        crs->mmu_table[(addr - base)/sizeof(uint32_t)] = (val & MMU_ENTRY_MASK) | ESP32_CACHE_MMU_ENTRY_CHANGED;
    }
}

static uint64_t esp32_dport_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32DportState *s = ESP32_DPORT(opaque);
    uint64_t r = 0;
    switch (addr) {
    case A_DPORT_APPCPU_RESET:
        r = s->appcpu_reset_state;
        break;
    case A_DPORT_APPCPU_CLK:
        r = s->appcpu_clkgate_state;
        break;
    case A_DPORT_APPCPU_RUNSTALL:
        r = s->appcpu_stall_state;
        break;
    case A_DPORT_APPCPU_BOOT_ADDR:
        r = s->appcpu_boot_addr;
        break;
    case A_DPORT_CPU_PER_CONF:
        r = s->cpuperiod_sel;
        break;
    case A_DPORT_PRO_CACHE_CTRL:
        r = s->cache_state[0].cache_ctrl_reg;
        break;
    case A_DPORT_PRO_CACHE_CTRL1:
        r = s->cache_state[0].cache_ctrl1_reg;
        break;
    case A_DPORT_APP_CACHE_CTRL:
        r = s->cache_state[1].cache_ctrl_reg;
        break;
    case A_DPORT_APP_CACHE_CTRL1:
        r = s->cache_state[1].cache_ctrl1_reg;
        break;
    case A_DPORT_PRO_DCACHE_DBUG0:
    case A_DPORT_APP_DCACHE_DBUG0:
        /* in idle state */
        r = FIELD_DP32(0, DPORT_PRO_DCACHE_DBUG0, CACHE_STATE, 1);
        break;
    case A_DPORT_CACHE_IA_INT_EN:
        r = s->cache_ill_trap_en_reg;
        break;
    case A_DPORT_PRO_DCACHE_DBUG3:
        r = 0;
        r = FIELD_DP32(r, DPORT_PRO_DCACHE_DBUG3, IA_INT_DROM0, s->cache_state[0].drom0.illegal_access_status);
        r = FIELD_DP32(r, DPORT_PRO_DCACHE_DBUG3, IA_INT_IRAM0, s->cache_state[0].iram0.illegal_access_status);
        /* [CACHE-TRACE] achado de 32.5.9: nenhuma excecao sincrona genuina de CPU precede os panics
         * "Cache error" reproduzidos -- a hipotese em aberto e que o ESP-IDF entra no panic por uma
         * rota ordinaria (interrupcao de nivel 1 ou a de nivel 5 ja rastreada em target/xtensa/) e so
         * DEPOIS le este registrador (o real DPORT_PRO_DCACHE_DBUG3, bits IA_INT_*) e encontra estado
         * indevidamente marcado. So esta LEITURA nunca tinha sido instrumentada (so a escrita, desde
         * 32.5.6/32.5.7) -- registra incondicionalmente pra comparar, no proximo experimento, se essa
         * leitura acontece com o bit setado ANTES do texto "Cache error" aparecer no UART. */
        cache_trace_record(s, "read_pro_dcache_dbug3", 0, addr, size, addr, 0, (uint32_t)r);
        break;
    case A_DPORT_APP_DCACHE_DBUG3:
        r = 0;
        r = FIELD_DP32(r, DPORT_APP_DCACHE_DBUG3, IA_INT_DROM0, s->cache_state[1].drom0.illegal_access_status);
        r = FIELD_DP32(r, DPORT_APP_DCACHE_DBUG3, IA_INT_IRAM0, s->cache_state[1].iram0.illegal_access_status);
        cache_trace_record(s, "read_app_dcache_dbug3", 1, addr, size, addr, 0, (uint32_t)r);
        break;
    case PRO_DROM0_MMU_FIRST ... PRO_DROM0_MMU_LAST:
        r = get_mmu_entry(&s->cache_state[0].drom0, PRO_DROM0_MMU_FIRST, addr);
        break;
    case PRO_IRAM0_MMU_FIRST ... PRO_IRAM0_MMU_LAST:
        r = get_mmu_entry(&s->cache_state[0].iram0, PRO_IRAM0_MMU_FIRST, addr);
        break;
    case APP_DROM0_MMU_FIRST ... APP_DROM0_MMU_LAST:
        r = get_mmu_entry(&s->cache_state[1].drom0, APP_DROM0_MMU_FIRST, addr);
        break;
    case APP_IRAM0_MMU_FIRST ... APP_IRAM0_MMU_LAST:
        r = get_mmu_entry(&s->cache_state[1].iram0, APP_IRAM0_MMU_FIRST, addr);
        break;
    case A_DPORT_SLAVE_SPI_CONFIG:
        r = s->slave_spi_config_reg;
        break;
    /* [FIX] .spec 32.5.16 -- ver comentario completo em include/hw/misc/esp32_dport.h. Ate aqui este
     * fork sempre retornava 0 pra estes dois registradores (nenhum case existia, cai no default
     * r=0), fazendo xt_highint5 (highint_hdl.S real) SEMPRE concluir "nao e o watchdog confirmado" e
     * rotular incondicionalmente como PANIC_RSN_CACHEERR mesmo quando a causa real, ponta a ponta, e
     * uma expiracao genuina do watchdog do TIMER_GROUP1 (fonte 20) -- achado de 32.5.12/32.5.13/
     * 32.5.15. PRO e APP leem o MESMO estado bruto: a fonte de interrupcao ou esta pedindo atencao
     * ou nao, independente de qual nucleo pergunta (a diferenca real entre os dois registradores no
     * hardware e so o endereco de acesso, nao o dado). */
    case A_DPORT_PRO_INTR_STATUS_0:
    case A_DPORT_APP_INTR_STATUS_0:
        r = s->intmatrix_opaque ? esp32_intmatrix_get_raw_status_bits(s->intmatrix_opaque, 0, 32) : 0;
        break;
    }

    return r;
}

static void esp32_dport_write(void *opaque, hwaddr addr,
                              uint64_t value, unsigned int size)
{
    Esp32DportState *s = ESP32_DPORT(opaque);
    bool old_state;
    uint32_t old_val;
    switch (addr) {
    case A_DPORT_APPCPU_RESET:
        old_state = s->appcpu_reset_state;
        s->appcpu_reset_state = value & 1;
        if (old_state && !s->appcpu_reset_state) {
            s->appcpu_reset_pending = true;
            qemu_irq_pulse(s->appcpu_reset_req);
        }
        qemu_set_irq(s->appcpu_stall_req,
                     s->appcpu_stall_state || !s->appcpu_clkgate_state ||
                     s->appcpu_reset_state || s->appcpu_reset_pending);
        break;
    case A_DPORT_APPCPU_CLK:
        s->appcpu_clkgate_state = value & 1;
        qemu_set_irq(s->appcpu_stall_req,
                     s->appcpu_stall_state || !s->appcpu_clkgate_state ||
                     s->appcpu_reset_state || s->appcpu_reset_pending);
        break;
    case A_DPORT_APPCPU_RUNSTALL:
        /* [CACHE-TRACE] ver .spec 32.5.8 -- este e o "stall request" via hardware (xtensa_runstall,
         * PARADA de verdade do nucleo, distinto do handshake por software esp_ipc_isr). */
        cache_trace_record(s, "write_appcpu_runstall", 1, addr, size, addr,
                            s->appcpu_stall_state, (uint32_t)(value & 1));
        s->appcpu_stall_state = value & 1;
        qemu_set_irq(s->appcpu_stall_req,
                     s->appcpu_stall_state || !s->appcpu_clkgate_state ||
                     s->appcpu_reset_state || s->appcpu_reset_pending);
        break;
    case A_DPORT_APPCPU_BOOT_ADDR:
        s->appcpu_boot_addr = value;
        break;
    case A_DPORT_CPU_PER_CONF:
        s->cpuperiod_sel = value & R_DPORT_CPU_PER_CONF_CPUPERIOD_SEL_MASK;
        qemu_irq_pulse(s->clk_update_req);
        break;
    case A_DPORT_PRO_CACHE_CTRL:
        if (FIELD_EX32(value, DPORT_PRO_CACHE_CTRL, CACHE_FLUSH_ENA)) {
            value |= R_DPORT_PRO_CACHE_CTRL_CACHE_FLUSH_DONE_MASK;
            value &= ~R_DPORT_PRO_CACHE_CTRL_CACHE_FLUSH_ENA_MASK;
            esp32_cache_invalidate_all_entries(&s->cache_state[0].drom0);
            esp32_cache_data_sync(&s->cache_state[0].drom0);
            esp32_cache_invalidate_all_entries(&s->cache_state[0].iram0);
            esp32_cache_data_sync(&s->cache_state[0].iram0);
        }
        old_val = s->cache_state[0].cache_ctrl_reg;
        if (value != old_val) {
            cache_trace_record(s, "write_pro_cache_ctrl", 0, addr, size, addr, old_val, (uint32_t)value);
        }
        s->cache_state[0].cache_ctrl_reg = value;
        if (value != old_val) {
            esp32_cache_state_update(&s->cache_state[0]);
        }
        break;
    case A_DPORT_PRO_CACHE_CTRL1:
        old_val = s->cache_state[0].cache_ctrl1_reg;
        if (value != old_val) {
            cache_trace_record(s, "write_pro_cache_ctrl1", 0, addr, size, addr, old_val, (uint32_t)value);
        }
        s->cache_state[0].cache_ctrl1_reg = value;
        if (value != old_val) {
            esp32_cache_state_update(&s->cache_state[0]);
        }
        break;
    case A_DPORT_APP_CACHE_CTRL:
        if (FIELD_EX32(value, DPORT_APP_CACHE_CTRL, CACHE_FLUSH_ENA)) {
            value |= R_DPORT_APP_CACHE_CTRL_CACHE_FLUSH_DONE_MASK;
            value &= ~R_DPORT_APP_CACHE_CTRL_CACHE_FLUSH_ENA_MASK;
            esp32_cache_invalidate_all_entries(&s->cache_state[1].drom0);
            esp32_cache_data_sync(&s->cache_state[1].drom0);
            esp32_cache_invalidate_all_entries(&s->cache_state[1].iram0);
            esp32_cache_data_sync(&s->cache_state[1].iram0);
        }
        old_val = s->cache_state[1].cache_ctrl_reg;
        if (value != old_val) {
            cache_trace_record(s, "write_app_cache_ctrl", 1, addr, size, addr, old_val, (uint32_t)value);
        }
        s->cache_state[1].cache_ctrl_reg = value;
        if (value != old_val) {
            esp32_cache_state_update(&s->cache_state[1]);
        }
        break;
    case A_DPORT_APP_CACHE_CTRL1:
        old_val = s->cache_state[1].cache_ctrl1_reg;
        if (value != old_val) {
            cache_trace_record(s, "write_app_cache_ctrl1", 1, addr, size, addr, old_val, (uint32_t)value);
        }
        s->cache_state[1].cache_ctrl1_reg = value;
        if (value != old_val) {
            esp32_cache_state_update(&s->cache_state[1]);
        }
        break;
    case A_DPORT_CACHE_IA_INT_EN:
        cache_trace_record(s, "write_cache_ia_int_en", -1, addr, size, addr,
                            s->cache_ill_trap_en_reg, (uint32_t)value);
        s->cache_ill_trap_en_reg = value;
        s->cache_state[0].drom0.illegal_access_trap_en = (FIELD_EX32(value, DPORT_CACHE_IA_INT_EN, IA_INT_PRO_DROM0));
        s->cache_state[0].iram0.illegal_access_trap_en = (FIELD_EX32(value, DPORT_CACHE_IA_INT_EN, IA_INT_PRO_IRAM0));
        s->cache_state[1].drom0.illegal_access_trap_en = (FIELD_EX32(value, DPORT_CACHE_IA_INT_EN, IA_INT_APP_DROM0));
        s->cache_state[1].iram0.illegal_access_trap_en = (FIELD_EX32(value, DPORT_CACHE_IA_INT_EN, IA_INT_APP_IRAM0));
        s->cache_state[0].dram1.illegal_access_trap_en = (FIELD_EX32(value, DPORT_CACHE_IA_INT_EN, IA_INT_PRO_DRAM1));
        s->cache_state[1].dram1.illegal_access_trap_en = (FIELD_EX32(value, DPORT_CACHE_IA_INT_EN, IA_INT_APP_DRAM1));
        break;
    case PRO_DROM0_MMU_FIRST ... PRO_DROM0_MMU_LAST:
        set_mmu_entry(&s->cache_state[0].drom0, PRO_DROM0_MMU_FIRST, addr, value);
        break;
    case PRO_IRAM0_MMU_FIRST ... PRO_IRAM0_MMU_LAST:
        set_mmu_entry(&s->cache_state[0].iram0, PRO_IRAM0_MMU_FIRST, addr, value);
        break;
    case APP_DROM0_MMU_FIRST ... APP_DROM0_MMU_LAST:
        set_mmu_entry(&s->cache_state[1].drom0, APP_DROM0_MMU_FIRST, addr, value);
        break;
    case APP_IRAM0_MMU_FIRST ... APP_IRAM0_MMU_LAST:
        set_mmu_entry(&s->cache_state[1].iram0, APP_IRAM0_MMU_FIRST, addr, value);
        break;
    case A_DPORT_SLAVE_SPI_CONFIG:
        s->slave_spi_config_reg = value;
        qemu_set_irq(s->flash_enc_en_gpio, FIELD_EX32(value, DPORT_SLAVE_SPI_CONFIG, SLAVE_SPI_ENCRYPT_ENABLE));
        qemu_set_irq(s->flash_dec_en_gpio, FIELD_EX32(value, DPORT_SLAVE_SPI_CONFIG, SLAVE_SPI_DECRYPT_ENABLE));
        break;
    }
}

static const MemoryRegionOps esp32_dport_ops = {
    .read =  esp32_dport_read,
    .write = esp32_dport_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_cache_data_sync(Esp32CacheRegionState* crs)
{
    if (crs->cache->dport->flash_blk == NULL) {
        return;
    }

    Esp32FlashEncryptionState * flash_enc = esp32_flash_encryption_find();
    bool decrypt = (flash_enc != NULL && esp32_flash_decryption_enabled(flash_enc));

    uint8_t* cache_data = (uint8_t*) memory_region_get_ram_ptr(&crs->mem);
    for (int i = 0; i < ESP32_CACHE_PAGES_PER_REGION; ++i) {
        uint32_t* cache_page = (uint32_t*) (cache_data + i * ESP32_CACHE_PAGE_SIZE);
        uint32_t mmu_entry = crs->mmu_table[i];
        if (!(mmu_entry & ESP32_CACHE_MMU_ENTRY_CHANGED)) {
            continue;
        }
        mmu_entry &= MMU_ENTRY_MASK;
        if (mmu_entry & ESP32_CACHE_MMU_INVALID_VAL) {
            uint32_t fill_val = crs->illegal_access_retval;
            for (int word = 0; word < ESP32_CACHE_PAGE_SIZE / sizeof(uint32_t); ++word) {
                cache_page[word] = fill_val;
            }
        } else {
            uint32_t phys_addr = mmu_entry * ESP32_CACHE_PAGE_SIZE;
            blk_pread(crs->cache->dport->flash_blk, phys_addr, ESP32_CACHE_PAGE_SIZE, cache_page, 0);
            if (decrypt) {
                esp32_flash_decrypt_inplace(flash_enc, phys_addr, cache_page, ESP32_CACHE_PAGE_SIZE/4);
            }
        }
        crs->mmu_table[i] &= ~ESP32_CACHE_MMU_ENTRY_CHANGED;
    }
    memory_region_flush_rom_device(&crs->mem, 0, ESP32_CACHE_REGION_SIZE);
}

static void esp32_cache_invalidate_all_entries(Esp32CacheRegionState* crs)
{
    for (int i = 0; i < ESP32_CACHE_PAGES_PER_REGION; ++i) {
        crs->mmu_table[i] |= ESP32_CACHE_MMU_ENTRY_CHANGED;
    }
}

static void esp32_cache_state_update(Esp32CacheState* cs)
{
    bool cache_enabled = FIELD_EX32(cs->cache_ctrl_reg, DPORT_PRO_CACHE_CTRL, CACHE_ENA) != 0;

    bool drom0_enabled = cache_enabled &&
        FIELD_EX32(cs->cache_ctrl1_reg, DPORT_PRO_CACHE_CTRL1, MASK_DROM0) == 0;
    if (!cs->drom0.mem.enabled && drom0_enabled) {
        esp32_cache_data_sync(&cs->drom0);
    }
    memory_region_set_enabled(&cs->drom0.mem, drom0_enabled);

    bool iram0_enabled = cache_enabled &&
        FIELD_EX32(cs->cache_ctrl1_reg, DPORT_PRO_CACHE_CTRL1, MASK_IRAM0) == 0;
    if (!cs->iram0.mem.enabled && iram0_enabled) {
        esp32_cache_data_sync(&cs->iram0);
    }
    memory_region_set_enabled(&cs->iram0.mem, iram0_enabled);

    if (cs->dport->has_psram) {
        bool dram1_enabled = cache_enabled &&
            FIELD_EX32(cs->cache_ctrl1_reg, DPORT_PRO_CACHE_CTRL1, MASK_DRAM1) == 0;
        memory_region_set_enabled(&cs->dram1.mem, dram1_enabled);
    }
}

static void esp32_cache_region_reset(Esp32CacheRegionState *crs)
{
    for (int i = 0; i < ESP32_CACHE_PAGES_PER_REGION; ++i) {
        crs->mmu_table[i] = ESP32_CACHE_MMU_ENTRY_CHANGED;
    }
    crs->illegal_access_trap_en = false;
}

static void esp32_cache_reset(Esp32CacheState *cs)
{
    esp32_cache_region_reset(&cs->drom0);
    esp32_cache_region_reset(&cs->iram0);
}

static uint64_t esp32_cache_ill_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32CacheRegionState *crs = (Esp32CacheRegionState*) opaque;
    uint32_t ill_data[] = { crs->illegal_access_retval, crs->illegal_access_retval };
    uint32_t result;
    memcpy(&result, ((uint8_t*) ill_data) + (addr % 4), size);
    /* [CACHE-TRACE] registra TODO acesso ao trap, mesmo quando illegal_access_trap_en==false (esse
     * caso tambem importa: mostra que o guest acessou uma regiao com cache desabilitado ANTES de
     * habilitar a IRQ correspondente -- ver .spec 32.5.7). */
    {
        const char *region_name = crs->type == ESP32_ICACHE_FLASH ? "iram0"
                                 : crs->type == ESP32_DCACHE_PSRAM ? "dram1" : "drom0";
        char tag[32];
        snprintf(tag, sizeof(tag), "ill_read_%s%s", region_name,
                 crs->illegal_access_trap_en ? "" : "_notrapen");
        cache_trace_record(crs->cache->dport, tag, crs->cache->core_id,
                            crs->base + addr, size, 0, 0, 0);
    }
    if (crs->illegal_access_trap_en) {
        crs->illegal_access_status = true;
        cache_trace_ill_irq_level = 1;
        qemu_irq_raise(crs->cache->dport->cache_ill_irq);
        cache_trace_capture_first_illegal_access();
    }
    return result;
}

static void esp32_cache_ill_write(void *opaque, hwaddr addr,
                                  uint64_t value, unsigned int size)
{
}

static bool esp32_cache_ill_accepts(void *opaque, hwaddr addr,
                             unsigned size, bool is_write,
                             MemTxAttrs attrs)
{
    return !is_write;
}


void esp32_dport_clear_ill_trap_state(Esp32DportState* s)
{
    cache_trace_record(s, "clear_ill_trap_state", -1, 0, 0, 0, 0, 0);
    s->cache_state[0].drom0.illegal_access_status = false;
    s->cache_state[1].drom0.illegal_access_status = false;
    s->cache_state[0].iram0.illegal_access_status = false;
    s->cache_state[1].iram0.illegal_access_status = false;
    s->cache_state[0].dram1.illegal_access_status = false;
    s->cache_state[1].dram1.illegal_access_status = false;
    cache_trace_ill_irq_level = 0;
    qemu_irq_lower(s->cache_ill_irq);
}

static const MemoryRegionOps esp32_cache_ops = {
    .write = NULL,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.accepts = esp32_cache_ill_accepts,
};


static const MemoryRegionOps esp32_cache_ill_trap_ops = {
    .read = esp32_cache_ill_read,
    .write = esp32_cache_ill_write,
};

static void esp32_dport_reset(DeviceState *dev)
{
    Esp32DportState *s = ESP32_DPORT(dev);

    s->appcpu_boot_addr = 0;
    s->appcpu_clkgate_state = false;
    s->appcpu_reset_state = true;
    s->appcpu_reset_pending = false;
    s->appcpu_stall_state = false;
    s->cache_ill_trap_en_reg = 0;
    esp32_cache_reset(&s->cache_state[0]);
    esp32_cache_reset(&s->cache_state[1]);
    qemu_irq_lower(s->appcpu_stall_req);
}

static void esp32_dport_realize(DeviceState *dev, Error **errp)
{
    Esp32DportState *s = ESP32_DPORT(dev);
    MachineState *ms = MACHINE(qdev_get_machine());

    s->cpu_count = ms->smp.cpus;
}

static void esp32_cache_init_region(Esp32CacheState *cs,
                                    Esp32CacheRegionState *crs,
                                    Esp32CacheRegionType type,
                                    const char* name, hwaddr base,
                                    uint32_t illegal_access_retval)
{
    char desc[16];
    crs->cache = cs;
    crs->type = type;
    crs->base = base;
    crs->illegal_access_retval = illegal_access_retval;
    snprintf(desc, sizeof(desc), "cpu%d-%s", cs->core_id, name);
    if (type == ESP32_DCACHE_PSRAM) {
        memory_region_init_ram(&crs->mem, OBJECT(cs->dport), desc,
                               ESP32_CACHE_REGION_SIZE, &error_abort);
    } else {
        memory_region_init_rom_device(&crs->mem, OBJECT(cs->dport),
                                    &esp32_cache_ops, crs,
                                    desc, ESP32_CACHE_REGION_SIZE, &error_abort);
    }

    snprintf(desc, sizeof(desc), "cpu%d-%s-ill", cs->core_id, name);
    memory_region_init_io(&crs->illegal_access_trap_mem, OBJECT(cs->dport),
                          &esp32_cache_ill_trap_ops, crs,
                          desc, ESP32_CACHE_REGION_SIZE);
}

static void esp32_dport_init(Object *obj)
{
    Esp32DportState *s = ESP32_DPORT(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_dport_ops, s,
                          TYPE_ESP32_DPORT, ESP32_DPORT_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);

    for (int i = 0; i < ESP32_CPU_COUNT; ++i) {
        Esp32CacheState* cs = &s->cache_state[i];
        cs->core_id = i;
        cs->dport = s;
        esp32_cache_init_region(cs, &cs->drom0, ESP32_DCACHE_FLASH, "drom0",
                                0x3F400000, 0xbaadbaad);
        esp32_cache_init_region(cs, &cs->iram0, ESP32_ICACHE_FLASH, "iram0",
                                0x40000000, 0x00000000);
        esp32_cache_init_region(cs, &cs->dram1, ESP32_DCACHE_PSRAM, "dram1",
                                0x3F800000, 0xbaadbaad);
    }

    qdev_init_gpio_out_named(DEVICE(sbd), &s->appcpu_stall_req, ESP32_DPORT_APPCPU_STALL_GPIO, 1);
    qdev_init_gpio_out_named(DEVICE(sbd), &s->appcpu_reset_req, ESP32_DPORT_APPCPU_RESET_GPIO, 1);
    qdev_init_gpio_out_named(DEVICE(sbd), &s->clk_update_req, ESP32_DPORT_CLK_UPDATE_GPIO, 1);
    qdev_init_gpio_out_named(DEVICE(sbd), &s->cache_ill_irq, ESP32_DPORT_CACHE_ILL_IRQ_GPIO, 1);
    qdev_init_gpio_out_named(DEVICE(sbd), &s->flash_enc_en_gpio, ESP32_DPORT_FLASH_ENC_EN_GPIO, 1);
    qdev_init_gpio_out_named(DEVICE(sbd), &s->flash_dec_en_gpio, ESP32_DPORT_FLASH_DEC_EN_GPIO, 1);
}

static Property esp32_dport_properties[] = {
    DEFINE_PROP_DRIVE("flash", Esp32DportState, flash_blk),
    DEFINE_PROP_BOOL("has_psram", Esp32DportState, has_psram, false),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32_dport_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = esp32_dport_reset;
    dc->realize = esp32_dport_realize;
    device_class_set_props(dc, esp32_dport_properties);
}

static const TypeInfo esp32_dport_info = {
    .name = TYPE_ESP32_DPORT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32DportState),
    .instance_init = esp32_dport_init,
    .class_init = esp32_dport_class_init
};

static void esp32_dport_register_types(void)
{
    type_register_static(&esp32_dport_info);
}

type_init(esp32_dport_register_types)
