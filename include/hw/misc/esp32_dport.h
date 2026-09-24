#pragma once

#include "hw/hw.h"
#include "hw/registerfields.h"
#include "hw/sysbus.h"
#include "hw/misc/esp32_reg.h"
#include "sysemu/block-backend.h"
#include "hw/misc/esp32_flash_enc.h"
#include "hw/misc/esp32_cache_race_stall.h"

typedef struct Esp32DportState Esp32DportState;
typedef struct Esp32CacheState Esp32CacheState;

#define TYPE_ESP32_DPORT "misc.esp32.dport"
#define ESP32_DPORT(obj) OBJECT_CHECK(Esp32DportState, (obj), TYPE_ESP32_DPORT)

/* E125/E129 (EVIDENCE.md, 2026-09-05): bits shared by Esp32DportState::
 * appcpu_cache_externally_disabled_mask AND appcpu_cache_wait_mask -- same bits
 * esp32_cache_race_stall_gain()/_release()/esp32_cache_access_should_wait()/_wait_release()
 * (hw/misc/esp32_cache_race_stall.h) use; aliased here rather than redefined so there is exactly
 * one definition to keep in sync. */
#define ESP32_CACHE_RACE_STALL_DROM0 ESP32_CACHE_RACE_STALL_DROM0_BIT
#define ESP32_CACHE_RACE_STALL_IRAM0 ESP32_CACHE_RACE_STALL_IRAM0_BIT

#define ESP32_CACHE_PAGE_SIZE           0x10000
#define ESP32_CACHE_PAGES_PER_REGION    64
#define ESP32_CACHE_REGION_SIZE         (ESP32_CACHE_PAGE_SIZE * ESP32_CACHE_PAGES_PER_REGION)
#define ESP32_CACHE_MMU_INVALID_VAL     0x100
#define ESP32_CACHE_MMU_ENTRY_CHANGED   0x200     /* not a hardware flag; used here to check if the page data needs to be updated */
#define ESP32_CACHE_MAX_PHYS_PAGES      0x100

typedef enum Esp32CacheRegionType {
    ESP32_DCACHE_FLASH,
    ESP32_ICACHE_FLASH,
    ESP32_DCACHE_PSRAM,
} Esp32CacheRegionType;

typedef struct Esp32CacheRegionState {
    Esp32CacheState* cache;
    MemoryRegion mem;
    MemoryRegion illegal_access_trap_mem;
    Esp32CacheRegionType type;
    hwaddr base;
    uint32_t illegal_access_retval;
    bool illegal_access_trap_en;
    bool illegal_access_status;
    uint16_t mmu_table[ESP32_CACHE_PAGES_PER_REGION];
} Esp32CacheRegionState;

typedef struct Esp32CacheState {
    Esp32DportState* dport;
    int core_id;

    uint32_t cache_ctrl_reg;
    uint32_t cache_ctrl1_reg;
    /* Using only the first 4MB range.
     * TODO: add memory regions for other ports: iram1, irom0
     */
    Esp32CacheRegionState iram0;
    Esp32CacheRegionState drom0;
    Esp32CacheRegionState dram1;  /* PSRAM */
} Esp32CacheState;

typedef struct Esp32DportState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    bool has_psram;
    int cpu_count;
    Esp32CacheState cache_state[ESP32_CPU_COUNT];
    BlockBackend *flash_blk;
    /* E120 (EVIDENCE.md, 2026-09-05): the realized m25p80 (or subtype) DeviceState* wired up by
     * esp32_dport_set_flash_device() -- see that function and hw/misc/esp32_dport.c's
     * esp32_cache_data_sync() for why this replaced reading flash_blk directly via blk_pread()
     * from inside this device's own MMIO dispatch. NULL in any configuration without a flash
     * chip, exactly like flash_blk. Lifetime: parented onto the SPI bus of the same
     * always-alive, embedded-for-the-machine's-lifetime SoC this DPORT instance itself belongs
     * to (hw/xtensa/esp32.c) -- never unparented or finalized before the whole machine is, so no
     * additional refcounting is taken here, matching every other cross-device pointer this
     * struct already stores the same way (flash_blk above, intmatrix_opaque below). */
    DeviceState *flash_dev;
    qemu_irq appcpu_stall_req;
    qemu_irq appcpu_reset_req;
    qemu_irq clk_update_req;
    qemu_irq cache_ill_irq;
    qemu_irq flash_enc_en_gpio;
    qemu_irq flash_dec_en_gpio;

    bool appcpu_reset_state;
    bool appcpu_reset_pending;
    bool appcpu_stall_state;
    bool appcpu_clkgate_state;
    /* E124/E125/E129 (EVIDENCE.md, 2026-09-05): tracks, per APP-owned cache region, whether a
     * DIFFERENT core (not APP CPU itself) is the one currently holding it disabled -- see
     * ESP32_CACHE_RACE_STALL_DROM0/IRAM0 below. See esp32_cache_state_update()'s own doc-comment
     * for the exact race this exists for (real ESP-IDF's own spi_flash_disable_interrupts_caches_
     * and_other_cpu() early-boot fast path assumes "APP CPU is either in reset or spinning inside
     * call_start_cpu1, which is IRAM-only" whenever xTaskGetSchedulerState()==
     * taskSCHEDULER_NOT_STARTED -- an assumption this fork's MTTCG timing can violate once APP CPU
     * has passed s_resume_cores but the scheduler still hasn't started).
     *
     * E125 replaced a single bool (E124's original shape) with this per-region mask specifically
     * because the bool's own release condition ("both regions fully enabled") could never fire,
     * and APP CPU would stall forever, whenever DROM0 or IRAM0 was ALREADY legitimately masked
     * (via CACHE_CTRL1's own MASK_DROM0/MASK_IRAM0 bits) independent of this stall's own cause --
     * a region that was never accessible in the first place must never gate the release. Each bit
     * here is set ONLY on a true enabled-by-this-region's-own-history -> disabled-by-a-different-
     * core transition (esp32_cache_state_update() compares old vs. new `mem.enabled`, not just
     * the new value), and cleared the instant THAT SAME region is observed enabled again,
     * regardless of who re-enabled it.
     *
     * E129: renamed from E124/E125's own `appcpu_cache_race_stall_mask` and reduced to PURE
     * bookkeeping -- proven (EVIDENCE.md E128) that turning this mask directly into a blanket
     * `xtensa_runstall()` of APP CPU (E124/E125's original design, via the now-removed
     * `appcpu_cache_race_stall` bool) deadlocks APP CPU permanently if the bit happens to be set
     * while APP CPU is mid-interrupt-return inside unrelated, IRAM-resident code that never
     * touches the disabled region. This mask no longer feeds `esp32_dport_update_appcpu_stall()`/
     * `appcpu_stall_req` at all -- see `appcpu_cache_wait_mask` below for what actually suspends
     * APP CPU now (only the specific access that lands on a disabled region, via
     * `esp32_cache_ill_read()`'s own `cpu_stop_current()`/`cpu_loop_exit_restore()`, not the whole
     * CPU). */
    uint32_t appcpu_cache_externally_disabled_mask;
    /* E129: set only when APP CPU's OWN load/fetch actually lands on a region currently in
     * appcpu_cache_externally_disabled_mask (hw/misc/esp32_dport.c's esp32_cache_ill_read()) --
     * i.e. APP CPU is suspended (cpu_stop_current(), not xtensa_runstall()) waiting specifically
     * for THAT region, not merely because it exists in a disabled state APP CPU may never actually
     * touch. Cleared, and APP CPU resumed via cpu_resume(), the instant the specific region it is
     * waiting on is re-enabled (esp32_cache_wait_release(), include/hw/misc/
     * esp32_cache_race_stall.h) -- deliberately a SEPARATE mask from the one above so a region
     * APP CPU never tried to touch can never spuriously suspend it, and so a still-pending wait on
     * a sibling region is never released early. */
    uint32_t appcpu_cache_wait_mask;
    uint32_t appcpu_boot_addr;
    uint32_t cpuperiod_sel;
    uint32_t cache_ill_trap_en_reg;
    uint32_t slave_spi_config_reg;
    /* Peripheral clock/reset gates (PERIP_CLK_EN, PERIP_RST_EN, WIFI_CLK_EN, CORE_RST_EN).
     * The model does not gate anything with them, but ESP-IDF reads them back: IDF 5.x
     * esp_phy_enable() asserts that the modem clock bits it just set in WIFI_CLK_EN stuck. */
    uint32_t perip_clk_en_reg;
    uint32_t perip_rst_en_reg;
    uint32_t wifi_clk_en_reg;
    uint32_t core_rst_en_reg;

    /* Adicionado em 32.5.16 -- ponteiro opaco pro Esp32IntMatrixState irmao (mesmo pai
     * Esp32SocState, ligado em esp32.c logo apos os dois serem inicializados). Usado so pra ler o
     * estado bruto ("raw") das fontes de interrupcao via esp32_intmatrix_get_raw_status_bits(),
     * exposto atraves de DPORT_PRO/APP_INTR_STATUS_0_REG -- ver esp32_dport.c. Opaco (void*, nao
     * Esp32IntMatrixState*) porque este header nao inclui hw/xtensa/esp32_intc.h. */
    void *intmatrix_opaque;
} Esp32DportState;

void esp32_dport_clear_ill_trap_state(Esp32DportState* s);
void esp32_dport_clear_appcpu_cache_wait_on_reset(Esp32DportState *s);

/* E120 (EVIDENCE.md, 2026-09-05): links this DPORT instance to its machine's realized flash chip
 * (hw/xtensa/esp32.c's esp32_machine_init_spi_flash(), called after DPORT itself is realized, so
 * this cannot happen any earlier than a dedicated post-realize setter call). Must be called
 * before any vCPU starts executing -- machine init always completes before the first cpu_exec(),
 * so calling this from esp32_machine_init() right after the flash chip is realized satisfies
 * that trivially. `flash_dev` must be a realized "m25p80-generic" (or subtype) device; passing
 * NULL is valid and matches "no flash chip configured" (see flash_blk's own NULL handling). */
void esp32_dport_set_flash_device(Esp32DportState* s, DeviceState *flash_dev);

/* Implementada em hw/xtensa/esp32_intc.c (ver .spec 32.5.16). Declarada aqui (nao em
 * hw/xtensa/esp32_intc.h) porque esp32_dport.c e codigo "common" e esp32_intc.h inclui
 * target/xtensa/cpu.h (que a build recusa fora de codigo per-target) -- mesma restricao ja
 * documentada em 32.5.9 pro CPUClass::get_pc(). `opaque` deve ser o ponteiro
 * Esp32DportState::intmatrix_opaque; retorna os bits [start_bit, start_bit+count) do estado bruto
 * (`irq_raw[]`) da matriz de interrupcao, empacotados a partir do bit 0 do valor retornado. */
uint32_t esp32_intmatrix_get_raw_status_bits(void *opaque, int start_bit, int count);

#define ESP32_DPORT_APPCPU_STALL_GPIO   "appcpu-stall"
#define ESP32_DPORT_APPCPU_RESET_GPIO   "appcpu-reset"
#define ESP32_DPORT_CLK_UPDATE_GPIO     "clk-update"
#define ESP32_DPORT_CACHE_ILL_IRQ_GPIO  "cache-ill-irq"
#define ESP32_DPORT_FLASH_ENC_EN_GPIO   "flash-enc-en"
#define ESP32_DPORT_FLASH_DEC_EN_GPIO   "flash-dec-en"


REG32(DPORT_APPCPU_RESET, 0x2c)
REG32(DPORT_APPCPU_CLK, 0x30)
REG32(DPORT_APPCPU_RUNSTALL, 0x34)
REG32(DPORT_APPCPU_BOOT_ADDR, 0x38)

REG32(DPORT_CPU_PER_CONF, 0x3c)
    FIELD(DPORT_CPU_PER_CONF, CPUPERIOD_SEL, 0, 2)

REG32(DPORT_PRO_CACHE_CTRL, 0x40)
    FIELD(DPORT_PRO_CACHE_CTRL, CACHE_FLUSH_DONE, 5, 1)
    FIELD(DPORT_PRO_CACHE_CTRL, CACHE_FLUSH_ENA, 4, 1)
    FIELD(DPORT_PRO_CACHE_CTRL, CACHE_ENA, 3, 1)

REG32(DPORT_PRO_CACHE_CTRL1, 0x44)
    FIELD(DPORT_PRO_CACHE_CTRL1, MMU_IA_CLR, 13, 1)
    FIELD(DPORT_PRO_CACHE_CTRL1, MASK_OPSDRAM, 5, 1)
    FIELD(DPORT_PRO_CACHE_CTRL1, MASK_DROM0, 4, 1)
    FIELD(DPORT_PRO_CACHE_CTRL1, MASK_DRAM1, 3, 1)
    FIELD(DPORT_PRO_CACHE_CTRL1, MASK_IROM0, 2, 1)
    FIELD(DPORT_PRO_CACHE_CTRL1, MASK_IRAM1, 1, 1)
    FIELD(DPORT_PRO_CACHE_CTRL1, MASK_IRAM0, 0, 1)

REG32(DPORT_APP_CACHE_CTRL, 0x58)
    FIELD(DPORT_APP_CACHE_CTRL, CACHE_FLUSH_DONE, 5, 1)
    FIELD(DPORT_APP_CACHE_CTRL, CACHE_FLUSH_ENA, 4, 1)
    FIELD(DPORT_APP_CACHE_CTRL, CACHE_ENA, 3, 1)

REG32(DPORT_APP_CACHE_CTRL1, 0x5C)
    FIELD(DPORT_APP_CACHE_CTRL1, MMU_IA_CLR, 13, 1)
    FIELD(DPORT_APP_CACHE_CTRL1, MASK_OPSDRAM, 5, 1)
    FIELD(DPORT_APP_CACHE_CTRL1, MASK_DROM0, 4, 1)
    FIELD(DPORT_APP_CACHE_CTRL1, MASK_DRAM1, 3, 1)
    FIELD(DPORT_APP_CACHE_CTRL1, MASK_IROM0, 2, 1)
    FIELD(DPORT_APP_CACHE_CTRL1, MASK_IRAM1, 1, 1)
    FIELD(DPORT_APP_CACHE_CTRL1, MASK_IRAM0, 0, 1)

REG32(DPORT_PERIP_CLK_EN, 0xC0)
REG32(DPORT_PERIP_RST_EN, 0xC4)
REG32(DPORT_SLAVE_SPI_CONFIG, 0xC8)
REG32(DPORT_WIFI_CLK_EN, 0xCC)
REG32(DPORT_CORE_RST_EN, 0xD0)
    FIELD(DPORT_SLAVE_SPI_CONFIG, SLAVE_SPI_ENCRYPT_ENABLE, 8, 1)
    FIELD(DPORT_SLAVE_SPI_CONFIG, SLAVE_SPI_DECRYPT_ENABLE, 12, 1)

REG32(DPORT_CPU_INTR_FROM_CPU_0, 0xdc)
REG32(DPORT_CPU_INTR_FROM_CPU_1, 0xe0)
REG32(DPORT_CPU_INTR_FROM_CPU_2, 0xe4)
REG32(DPORT_CPU_INTR_FROM_CPU_3, 0xe8)

/* Adicionados na rodada de 2026-07-27 (.spec 32.5.12/32.5.16) -- offsets reais confirmados contra
 * components/soc/esp32/register/soc/dport_reg.h do ESP-IDF v5.5.4 real (DPORT_PRO_INTR_STATUS_0_REG/
 * DPORT_APP_INTR_STATUS_0_REG). Bit N reflete o estado bruto ("raw", antes de roteamento) da fonte de
 * interrupcao N da matriz (o mesmo N de periph_interrupt_t em soc/interrupts.h -- ex.: bit 20 =
 * ETS_TG1_WDT_LEVEL_INTR_SOURCE, bit 68 nao cabe aqui pois so os primeiros 32 bits existem neste
 * registrador). Faltando neste fork ate 32.5.15 -- xt_highint5 (highint_hdl.S real) le esse bit pra
 * desambiguar entre o watchdog do TIMER_GROUP1 e o acesso ilegal ao cache (que compartilham a mesma
 * linha de interrupcao de CPU neste build) -- sem ele, a leitura sempre volta zero e a desambiguacao
 * sempre erra, rotulando um watchdog genuino como "Cache error". */
REG32(DPORT_PRO_INTR_STATUS_0, 0xec)
REG32(DPORT_APP_INTR_STATUS_0, 0xf8)

REG32(DPORT_PRO_MAC_INTR_MAP, 0x104)
REG32(DPORT_APP_MAC_INTR_MAP, 0x218)

REG32(DPORT_PRO_DCACHE_DBUG0, 0x3f0)
    FIELD(DPORT_PRO_DCACHE_DBUG0, CACHE_STATE, 7, 12)

REG32(DPORT_PRO_DCACHE_DBUG3, 0x3FC)
    FIELD(DPORT_PRO_DCACHE_DBUG3, IA_INT_OPPOSITE, 9, 1)
    FIELD(DPORT_PRO_DCACHE_DBUG3, IA_INT_DRAM1, 10, 1)
    FIELD(DPORT_PRO_DCACHE_DBUG3, IA_INT_IROM0, 11, 1)
    FIELD(DPORT_PRO_DCACHE_DBUG3, IA_INT_IRAM1, 12, 1)
    FIELD(DPORT_PRO_DCACHE_DBUG3, IA_INT_IRAM0, 13, 1)
    FIELD(DPORT_PRO_DCACHE_DBUG3, IA_INT_DROM0, 14, 1)

REG32(DPORT_APP_DCACHE_DBUG0, 0x418)
    FIELD(DPORT_APP_DCACHE_DBUG0, CACHE_STATE, 7, 12)

REG32(DPORT_APP_DCACHE_DBUG3, 0x424)
    FIELD(DPORT_APP_DCACHE_DBUG3, IA_INT_OPPOSITE, 9, 1)
    FIELD(DPORT_APP_DCACHE_DBUG3, IA_INT_DRAM1, 10, 1)
    FIELD(DPORT_APP_DCACHE_DBUG3, IA_INT_IROM0, 11, 1)
    FIELD(DPORT_APP_DCACHE_DBUG3, IA_INT_IRAM1, 12, 1)
    FIELD(DPORT_APP_DCACHE_DBUG3, IA_INT_IRAM0, 13, 1)
    FIELD(DPORT_APP_DCACHE_DBUG3, IA_INT_DROM0, 14, 1)

REG32(DPORT_CACHE_IA_INT_EN, 0x5A0)
    FIELD(DPORT_CACHE_IA_INT_EN, IA_INT_PRO_OPPOSITE, 19, 1)
    FIELD(DPORT_CACHE_IA_INT_EN, IA_INT_PRO_DRAM1, 18, 1)
    FIELD(DPORT_CACHE_IA_INT_EN, IA_INT_PRO_IROM0, 17, 1)
    FIELD(DPORT_CACHE_IA_INT_EN, IA_INT_PRO_IRAM1, 16, 1)
    FIELD(DPORT_CACHE_IA_INT_EN, IA_INT_PRO_IRAM0, 15, 1)
    FIELD(DPORT_CACHE_IA_INT_EN, IA_INT_PRO_DROM0, 14, 1)
    FIELD(DPORT_CACHE_IA_INT_EN, IA_INT_APP_OPPOSITE, 5, 1)
    FIELD(DPORT_CACHE_IA_INT_EN, IA_INT_APP_DRAM1, 4, 1)
    FIELD(DPORT_CACHE_IA_INT_EN, IA_INT_APP_IROM0, 3, 1)
    FIELD(DPORT_CACHE_IA_INT_EN, IA_INT_APP_IRAM1, 2, 1)
    FIELD(DPORT_CACHE_IA_INT_EN, IA_INT_APP_IRAM0, 1, 1)
    FIELD(DPORT_CACHE_IA_INT_EN, IA_INT_APP_DROM0, 0, 1)


#define ESP32_DPORT_PRO_INTMATRIX_BASE    A_DPORT_PRO_MAC_INTR_MAP
#define ESP32_DPORT_APP_INTMATRIX_BASE    A_DPORT_APP_MAC_INTR_MAP
#define ESP32_DPORT_CROSSCORE_INT_BASE    A_DPORT_CPU_INTR_FROM_CPU_0
