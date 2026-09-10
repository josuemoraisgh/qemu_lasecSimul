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
#include "qemu/timer.h"
#include "hw/core/cpu.h"
#include "hw/misc/esp32_flash_enc.h"
#include "hw/nvram/esp32_efuse.h"
#include "hw/block/flash.h"
/* E129 (EVIDENCE.md, 2026-09-05): the per-access cache-wait interlock in esp32_cache_ill_read()
 * needs the same cpu_stop_current()/cpu_loop_exit_restore() idiom softmmu/vnext_b.c already uses
 * for VNEXT_B backpressure replay -- but this file is compiled into libcommon.fa (target-
 * independent) and cannot include exec/exec-all.h directly (it pulls in the target's own cpu.h,
 * not available here -- confirmed by a failed build attempt, not assumed). vnext_b.c IS compiled
 * per-target (softmmu/meson.build) and already provides every other cpu_loop_exit_restore() call
 * site in this fork for exactly that reason, so the actual suspend call is a thin wrapper declared
 * there: vnext_cache_wait_suspend_current_cpu(). cpu_resume() itself needs no such wrapper -- it is
 * declared by hw/core/cpu.h (included above), which esp32_dport.c already includes and which has
 * no target-specific dependency. */
#include "../softmmu/vnext_b.h"

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
        break;
    case A_DPORT_APP_DCACHE_DBUG3:
        r = 0;
        r = FIELD_DP32(r, DPORT_APP_DCACHE_DBUG3, IA_INT_DROM0, s->cache_state[1].drom0.illegal_access_status);
        r = FIELD_DP32(r, DPORT_APP_DCACHE_DBUG3, IA_INT_IRAM0, s->cache_state[1].iram0.illegal_access_status);
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

/* E124 (EVIDENCE.md, 2026-09-05): single place computing appcpu_stall_req's level from every
 * reason APP CPU's whole execution might need to be held via xtensa_runstall() -- reset/clkgate/
 * RUNSTALL writes.
 *
 * E129: the cache-race reason that used to participate here (appcpu_cache_race_stall) is GONE --
 * proven (EVIDENCE.md E128) to deadlock APP CPU when this blanket runstall happened to engage
 * while APP CPU was mid-interrupt-return inside unrelated IRAM-resident code. That reason now
 * lives entirely on the orthogonal cpu_stop_current()/cpu_resume() axis
 * (esp32_cache_ill_read()/esp32_cache_state_update(), driven by appcpu_cache_wait_mask) and
 * deliberately never reaches this function or appcpu_stall_req at all -- only an access that
 * actually lands on a disabled region suspends, not the whole CPU. */
/* True when APP CPU is held by any of the orthogonal, non-cache-wait device reasons this same
 * OR-condition already tests in esp32_dport_update_appcpu_stall() below (RUNSTALL, clockgate,
 * reset). Used by esp32_dport_maybe_resume_appcpu_cache_wait() so a cache-wait release never
 * calls cpu_resume() while one of these orthogonal reasons is still holding the CPU. */
static bool esp32_dport_appcpu_has_non_cache_stop(Esp32DportState *s)
{
    return s->appcpu_stall_state || !s->appcpu_clkgate_state ||
           s->appcpu_reset_state || s->appcpu_reset_pending;
}

static void esp32_dport_maybe_resume_appcpu_cache_wait(Esp32DportState *s)
{
    if (s->appcpu_cache_wait_mask != 0 || esp32_dport_appcpu_has_non_cache_stop(s)) {
        return;
    }
    CPUState *appcpu = qemu_get_cpu(1);
    if (appcpu) {
        cpu_resume(appcpu);
    }
}

static void esp32_dport_update_appcpu_stall(Esp32DportState *s)
{
    qemu_set_irq(s->appcpu_stall_req,
                 s->appcpu_stall_state || !s->appcpu_clkgate_state ||
                 s->appcpu_reset_state || s->appcpu_reset_pending);
    esp32_dport_maybe_resume_appcpu_cache_wait(s);
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
        esp32_dport_update_appcpu_stall(s);
        break;
    case A_DPORT_APPCPU_CLK:
        s->appcpu_clkgate_state = value & 1;
        esp32_dport_update_appcpu_stall(s);
        break;
    case A_DPORT_APPCPU_RUNSTALL:
        s->appcpu_stall_state = value & 1;
        esp32_dport_update_appcpu_stall(s);
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
        }
        s->cache_state[0].cache_ctrl_reg = value;
        if (value != old_val) {
            esp32_cache_state_update(&s->cache_state[0]);
        }
        break;
    case A_DPORT_PRO_CACHE_CTRL1:
        old_val = s->cache_state[0].cache_ctrl1_reg;
        if (value != old_val) {
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
        }
        s->cache_state[1].cache_ctrl_reg = value;
        if (value != old_val) {
            esp32_cache_state_update(&s->cache_state[1]);
        }
        break;
    case A_DPORT_APP_CACHE_CTRL1:
        old_val = s->cache_state[1].cache_ctrl1_reg;
        if (value != old_val) {
        }
        s->cache_state[1].cache_ctrl1_reg = value;
        if (value != old_val) {
            esp32_cache_state_update(&s->cache_state[1]);
        }
        break;
    case A_DPORT_CACHE_IA_INT_EN:
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
    Esp32DportState *dport = crs->cache->dport;
    if (dport->flash_blk == NULL) {
        /* No flash chip configured at all -- pre-existing, legitimate "no flash" semantics
         * (e.g. a unit test or config with no -drive). Nothing to sync; unchanged from before
         * E120. */
        return;
    }
    if (dport->flash_dev == NULL) {
        /* E120 (EVIDENCE.md, 2026-09-05): flash_blk is set (a flash chip WAS requested) but
         * flash_dev never got linked -- a genuine machine-init wiring bug
         * (esp32_dport_set_flash_device() not called after esp32_machine_init_spi_flash()), not
         * "no flash configured". Fail loudly and deterministically rather than silently reading
         * stale/zeroed cache pages or falling back to the blk_pread()-from-MMIO-dispatch hazard
         * this fix exists to remove. */
        error_report("esp32_dport: cache sync requested but no flash device is linked "
                     "(flash_blk is set, flash_dev is NULL) -- esp32_dport_set_flash_device() "
                     "was not called during machine init");
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
            /* E120 (EVIDENCE.md, 2026-09-05): was blk_pread() -- a synchronous block-layer read
             * that releases the BQL (AIO_WAIT_WHILE) from inside this device's own active MMIO
             * dispatch, letting a second, genuinely concurrent access into the same still-
             * "active" MemoryRegion get rejected with "Blocked re-entrant IO" (root-caused in
             * E119; up to ESP32_CACHE_PAGES_PER_REGION calls per cache-enable event, each capable
             * of stalling for as long as host disk I/O takes under real N=8 contention).
             * m25p80_read_array() instead copies straight out of the flash chip's own coherent,
             * RAM-backed storage[] -- the exact same array the guest's own program/erase commands
             * already mutate synchronously and immediately -- so this is plain memory access: it
             * never touches BlockBackend and cannot release the BQL. Its old return value (an
             * int, silently ignored here) is now checked; a failed copy leaves this page's
             * ESP32_CACHE_MMU_ENTRY_CHANGED bit SET (see below) instead of being marked synced. */
            Error *local_err = NULL;
            if (!m25p80_read_array(dport->flash_dev, phys_addr, ESP32_CACHE_PAGE_SIZE,
                                   cache_page, &local_err)) {
                error_report_err(local_err);
                continue;
            }
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

/* E124 (EVIDENCE.md, 2026-09-05): root cause proven by direct reproduction with
 * LASECSIMUL_CACHE_TRACE -- CPU1 (APP), on its first real generation right after passing
 * call_start_cpu1()'s `while (!s_resume_cores)` wait, read g_startup_fn[1] (DROM0 vaddr
 * 0x3f4049f0) at the EXACT virtual instant CPU0 (PRO), executing spi_flash_mmap_init() ->
 * spi_flash_disable_cache()/spi_flash_restore_cache() (components/spi_flash/cache_utils.c),
 * had ju st cleared APP_CACHE_CTRL's CACHE_ENA bit as part of its own rapid, repeated
 * disable/restore cycling -- landing the read while cache_ena[1].drom0==0, which this model
 * correctly (not buggily) routes to illegal_access_trap_mem, returning illegal_access_retval
 * (0xBAADBAAD) and raising the cache-IA interrupt exactly as designed.
 *
 * The real ESP-IDF invariant being violated is documented in cache_utils.c itself:
 * spi_flash_disable_interrupts_caches_and_other_cpu()'s xTaskGetSchedulerState()==
 * taskSCHEDULER_NOT_STARTED fast path disables the OTHER cpu's cache WITHOUT any esp_ipc_call-
 * based cross-core synchronization, explicitly because "APP CPU is either in reset or spinning
 * inside call_start_cpu1, which is in IRAM" -- i.e. real ESP-IDF assumes the other core cannot
 * be concurrently touching DROM0 during this unsynchronized window. That assumption holds up to
 * the s_resume_cores wait; once APP CPU has passed it (during ESP_SYSTEM_INIT_STAGE_SECONDARY,
 * which both cores execute concurrently, still before vTaskStartScheduler() so the fast path
 * above still applies) it no longer holds, and nothing in this fork's emulation reproduced the
 * one property real hardware silently relies on here: this exact disable/restore sequence is a
 * handful of back-to-back instructions on real silicon, so a same-instant collision with the
 * other core's own memory access is not a practically reachable window -- under this fork's
 * MTTCG (each vCPU a genuinely concurrent host thread, synchronized only at BQL-held MMIO
 * boundaries), the SAME sequence spans a real, host-scheduler-dependent slice of wall/virtual
 * time in which the other vCPU's thread can and does execute independently.
 *
 * Fix, matching the "impedir CPU1 de executar enquanto DROM esta desabilitado" category
 * explicitly allowed for this task: extend the SAME cross-core stall this fork already has
 * wired for APP CPU (appcpu_stall_req -- today driven by RUNSTALL/RESET/CLK writes only) to
 * also hold APP CPU whenever a DIFFERENT core disables APP's own drom0 or iram0 cache region,
 * releasing it the instant both are enabled again. This does not touch PRO's cache path (no
 * equivalent DPORT-level stall exists for PRO on real hardware either), does not change what
 * counts as an illegal access (a disabled-region read from APP CPU ITSELF, or from PRO's own
 * cache while PRO runs, still traps exactly as before), and does not serialize MTTCG globally --
 * only the one core whose OWN cache another core just pulled out from under it is held, for
 * exactly as long as that specific condition holds. */
/* E124/E125 (EVIDENCE.md, 2026-09-05): root cause proven by direct reproduction with
 * LASECSIMUL_CACHE_TRACE -- CPU1 (APP), on its first real generation right after passing
 * call_start_cpu1()'s `while (!s_resume_cores)` wait, read g_startup_fn[1] (DROM0 vaddr
 * 0x3f4049f0) at the EXACT virtual instant CPU0 (PRO), executing spi_flash_mmap_init() ->
 * spi_flash_disable_cache()/spi_flash_restore_cache() (components/spi_flash/cache_utils.c),
 * had just cleared APP_CACHE_CTRL's CACHE_ENA bit as part of its own rapid, repeated
 * disable/restore cycling -- landing the read while cache_ena[1].drom0==0, which this model
 * correctly (not buggily) routes to illegal_access_trap_mem, returning illegal_access_retval
 * (0xBAADBAAD) and raising the cache-IA interrupt exactly as designed.
 *
 * The real ESP-IDF invariant being violated is documented in cache_utils.c itself:
 * spi_flash_disable_interrupts_caches_and_other_cpu()'s xTaskGetSchedulerState()==
 * taskSCHEDULER_NOT_STARTED fast path disables the OTHER cpu's cache WITHOUT any esp_ipc_call-
 * based cross-core synchronization, explicitly because "APP CPU is either in reset or spinning
 * inside call_start_cpu1, which is in IRAM" -- i.e. real ESP-IDF assumes the other core cannot
 * be concurrently touching DROM0 during this unsynchronized window. That assumption holds up to
 * the s_resume_cores wait; once APP CPU has passed it (during ESP_SYSTEM_INIT_STAGE_SECONDARY,
 * which both cores execute concurrently, still before vTaskStartScheduler() so the fast path
 * above still applies) it no longer holds, and nothing in this fork's emulation reproduced the
 * one property real hardware silently relies on here: this exact disable/restore sequence is a
 * handful of back-to-back instructions on real silicon, so a same-instant collision with the
 * other core's own memory access is not a practically reachable window -- under this fork's
 * MTTCG (each vCPU a genuinely concurrent host thread, synchronized only at BQL-held MMIO
 * boundaries), the SAME sequence spans a real, host-scheduler-dependent slice of wall/virtual
 * time in which the other vCPU's thread can and does execute independently.
 *
 * Fix, matching the "impedir CPU1 de executar enquanto DROM esta desabilitado" category
 * explicitly allowed for this task: extend the SAME cross-core stall this fork already has
 * wired for APP CPU (appcpu_stall_req -- today driven by RUNSTALL/RESET/CLK writes only) to
 * also hold APP CPU whenever a DIFFERENT core disables APP's own drom0 or iram0 cache region,
 * releasing it the instant both are enabled again. This does not touch PRO's cache path (no
 * equivalent DPORT-level stall exists for PRO on real hardware either), does not change what
 * counts as an illegal access (a disabled-region read from APP CPU ITSELF, or from PRO's own
 * cache while PRO runs, still traps exactly as before), and does not serialize MTTCG globally --
 * only the one core whose OWN cache another core just pulled out from under it is held, for
 * exactly as long as that specific condition holds.
 *
 * E125 review closed three gaps found auditing E124's own first cut:
 *
 * (1) STALL-BEFORE-DISABLE ORDERING. E124 disabled the MemoryRegions (memory_region_set_enabled)
 *     and only afterwards computed/applied the stall -- a real, if narrow, ordering gap. Proven
 *     during E125's own review that the BQL does NOT close it in general: this device's write
 *     handler holds the BQL for its whole duration, and any access that must take the SLOW path
 *     (device dispatch -- e.g. the very first, never-cached access this bug's own proven scenario
 *     is built on) is genuinely serialized behind it, so for THAT specific access pattern the old
 *     ordering was not exploitable. But cross-CPU TLB invalidation in this codebase
 *     (accel/tcg/cputlb.c, tlb_flush() -> async_run_on_cpu() whenever the target is not the
 *     calling thread) is asynchronous, not synchronous with memory_region_set_enabled() -- a vCPU
 *     already holding a cached FAST-PATH TLB entry for the same page does not need the BQL at all
 *     and is not blocked by it, so it could in principle race through the old disable-then-stall
 *     window on a page it had touched before. Reordered below to remove any dependence on which
 *     access pattern applies: the stall is now requested BEFORE either MemoryRegion is disabled,
 *     and only released AFTER the corresponding region is re-enabled (and, for a first
 *     transition back to enabled, re-synced).
 * (2) OVER-BROAD RELEASE CONDITION. E124's release test was "drom0_enabled && iram0_enabled"
 *     globally -- if either region was ALREADY legitimately masked (CACHE_CTRL1's own
 *     MASK_DROM0/MASK_IRAM0, independent of this stall's cause) before the race-triggering
 *     disable, that region could never read back as "enabled", so the stall could never release:
 *     a real, provable permanent-stall bug. Replaced with a per-region bitmask
 *     (appcpu_cache_race_stall_mask) that only ever gains a bit on an observed TRUE->FALSE
 *     transition of that SPECIFIC region's own `mem.enabled`, caused by a different core, and
 *     only ever loses that bit when that SAME region is next observed enabled -- a sibling region
 *     that was never enabled in the first place can never appear in the mask and can therefore
 *     never block release.
 * (3) RESET LIFETIME. esp32_dport_reset() lowered appcpu_stall_req but never cleared
 *     appcpu_cache_race_stall/_mask -- see esp32_dport_reset()'s own updated body.
 *
 * E129 review (EVIDENCE.md E128/E129, 2026-09-05): the above is E125's own historical account,
 * kept as-is -- all three of its fixes were real and remain in effect (the ordering in this
 * function, the per-region mask, the reset cleanup). What E125 did NOT catch is a fourth, deeper
 * problem E128 proved by direct causal instrumentation: turning the per-region mask directly into
 * a blanket xtensa_runstall() of ALL of APP CPU's execution (regardless of what APP CPU is
 * currently doing) deadlocks it permanently if the mask happens to gain a bit while APP CPU is
 * mid-interrupt-return inside unrelated, IRAM-resident code (spi_flash_op_block_func()'s own
 * legitimate busy-wait) that never touches the disabled region at all. The mask itself -- renamed
 * appcpu_cache_externally_disabled_mask -- is UNCHANGED and still correct pure bookkeeping; what
 * changed is that it no longer drives xtensa_runstall()/appcpu_stall_req by itself any more. See
 * esp32_cache_ill_read() (this file) and include/hw/misc/esp32_cache_race_stall.h for the
 * per-access interlock that replaced it: only an access that actually LANDS on a disabled region
 * suspends (via cpu_stop_current(), not xtensa_runstall()), tracked in the separate
 * appcpu_cache_wait_mask. */
static void esp32_cache_state_update(Esp32CacheState* cs)
{
    bool cache_enabled = FIELD_EX32(cs->cache_ctrl_reg, DPORT_PRO_CACHE_CTRL, CACHE_ENA) != 0;
    bool drom0_enabled = cache_enabled &&
        FIELD_EX32(cs->cache_ctrl1_reg, DPORT_PRO_CACHE_CTRL1, MASK_DROM0) == 0;
    bool iram0_enabled = cache_enabled &&
        FIELD_EX32(cs->cache_ctrl1_reg, DPORT_PRO_CACHE_CTRL1, MASK_IRAM0) == 0;

    int writer_core = -1;
    if (cs->core_id == 1 && current_cpu) {
        for (int c = 0; c < ESP32_CPU_COUNT; ++c) {
            if (qemu_get_cpu(c) == current_cpu) {
                writer_core = c;
                break;
            }
        }
    }

    /* Phase A -- E129 (EVIDENCE.md, 2026-09-05): record that a DIFFERENT core is about to disable
     * one of APP's own regions, BEFORE touching either MemoryRegion -- but this is now PURE
     * bookkeeping (appcpu_cache_externally_disabled_mask), not a stall request. Nothing here calls
     * esp32_dport_update_appcpu_stall() or touches xtensa_runstall() any more -- see this field's
     * own doc-comment in include/hw/misc/esp32_dport.h for why (E128's proven deadlock). Whether
     * APP CPU actually needs to be suspended is now decided per-access, in esp32_cache_ill_read()
     * below, not here. */
    if (cs->core_id == 1) {
        Esp32DportState *dport = cs->dport;
        dport->appcpu_cache_externally_disabled_mask |= esp32_cache_race_stall_gain(
            writer_core, cs->drom0.mem.enabled, drom0_enabled, cs->iram0.mem.enabled, iram0_enabled);
    }

    /* Phase B -- apply the actual region state (APP CPU is never blanket-held for this reason any
     * more; only an access that actually lands on the disabled region suspends, in
     * esp32_cache_ill_read()). */
    if (!cs->drom0.mem.enabled && drom0_enabled) {
        esp32_cache_data_sync(&cs->drom0);
    }
    memory_region_set_enabled(&cs->drom0.mem, drom0_enabled);

    if (!cs->iram0.mem.enabled && iram0_enabled) {
        esp32_cache_data_sync(&cs->iram0);
    }
    memory_region_set_enabled(&cs->iram0.mem, iram0_enabled);

    if (cs->dport->has_psram) {
        bool dram1_enabled = cache_enabled &&
            FIELD_EX32(cs->cache_ctrl1_reg, DPORT_PRO_CACHE_CTRL1, MASK_DRAM1) == 0;
        memory_region_set_enabled(&cs->dram1.mem, dram1_enabled);
    }

    /* Phase C -- release per-region, only after that SAME region is confirmed enabled again
     * (never "both regions enabled" globally -- see gap (2) above), and only after Phase B's own
     * sync/enable for it has already happened. Decision in esp32_cache_race_stall_release(), same
     * pure header as Phase A's gain.
     *
     * E129: also releases appcpu_cache_wait_mask for the same bits -- if APP CPU is currently
     * suspended (via esp32_cache_ill_read()'s own cpu_stop_current(), not xtensa_runstall())
     * specifically waiting on a region THIS write just re-enabled, esp32_cache_wait_release()
     * reports true only when that was the LAST cache-wait reason. E131-A tightens the final wake:
     * the region release may coincide with an independent effective stop (RUNSTALL, clockgate, or
     * reset-pending), so the wait is cleared and recorded immediately, but cpu_resume() is deferred
     * until those orthogonal device reasons are gone. */
    if (cs->core_id == 1) {
        Esp32DportState *dport = cs->dport;
        uint32_t release_bits = esp32_cache_race_stall_release(drom0_enabled, iram0_enabled);
        dport->appcpu_cache_externally_disabled_mask &= ~release_bits;
        bool was_waiting = dport->appcpu_cache_wait_mask != 0;
        bool fully_released = esp32_cache_wait_release(&dport->appcpu_cache_wait_mask, release_bits);
        if (fully_released) {
            esp32_dport_maybe_resume_appcpu_cache_wait(dport);
        } else if (was_waiting && release_bits != 0) {
            /* A release happened but did not clear the last pending reason (e.g. DROM0 came back
             * while IRAM0 is still disabled) -- still an observed release, just not a wake. */
        }
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

    if (crs->type != ESP32_DCACHE_PSRAM) {
        Esp32CacheState *cs = crs->cache;
        Esp32DportState *dport = cs->dport;
        int accessing_core = -1;

        if (current_cpu) {
            for (int c = 0; c < ESP32_CPU_COUNT; ++c) {
                if (qemu_get_cpu(c) == current_cpu) {
                    accessing_core = c;
                    break;
                }
            }
        }

        uint32_t region_bit = (crs == &cs->drom0) ? ESP32_CACHE_RACE_STALL_DROM0
                             : (crs == &cs->iram0) ? ESP32_CACHE_RACE_STALL_IRAM0 : 0;
        bool should_wait = esp32_cache_access_should_wait(accessing_core, cs->core_id, region_bit,
                                                          dport->appcpu_cache_externally_disabled_mask);
        if (should_wait) {
            dport->appcpu_cache_wait_mask |= region_bit;
            vnext_cache_wait_suspend_current_cpu();
        }
    }

    uint32_t ill_data[] = { crs->illegal_access_retval, crs->illegal_access_retval };
    uint32_t result;
    memcpy(&result, ((uint8_t*) ill_data) + (addr % 4), size);
    if (crs->illegal_access_trap_en) {
        crs->illegal_access_status = true;
        qemu_irq_raise(crs->cache->dport->cache_ill_irq);
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
    s->cache_state[0].drom0.illegal_access_status = false;
    s->cache_state[1].drom0.illegal_access_status = false;
    s->cache_state[0].iram0.illegal_access_status = false;
    s->cache_state[1].iram0.illegal_access_status = false;
    s->cache_state[0].dram1.illegal_access_status = false;
    s->cache_state[1].dram1.illegal_access_status = false;
    qemu_irq_lower(s->cache_ill_irq);
}

void esp32_dport_clear_appcpu_cache_wait_on_reset(Esp32DportState *s)
{
    if (!s) {
        return;
    }

    /*
     * E131-A: a per-CPU reset is a generation boundary for APP CPU's cache-wait episode. The
     * faulting instruction has been discarded by cpu_reset(), so a later cache-region re-enable
     * must not resume/replay an instruction from the previous generation, and the next APP CPU
     * boot must not inherit a stale wait bit. Full DPORT reset already clears these fields in
     * esp32_dport_reset(); this helper covers SW_CPU_RESET_REGISTER/MWDT_CPU_STAGE resets that do
     * not reset the whole DPORT device.
     */
    s->appcpu_cache_wait_mask = 0;
}

void esp32_dport_set_flash_device(Esp32DportState* s, DeviceState *flash_dev)
{
    s->flash_dev = flash_dev;
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
    /* E125 (EVIDENCE.md, 2026-09-05): esp32_cache_reset() below only ever touched the MMU
     * CHANGED flags and illegal_access_trap_en -- it left cache_ctrl_reg/cache_ctrl1_reg (the raw
     * registers), each region's MemoryRegion.enabled, and illegal_access_status exactly as they
     * were before this reset, for both cores. On real hardware, POR/reset leaves the cache
     * subsystem disabled (CACHE_ENA=0) until firmware explicitly re-enables it -- this model must
     * match that, both for its own sake and because esp32_cache_state_update()'s new
     * appcpu_cache_externally_disabled_mask logic (E124/E125/E129) detects transitions by comparing this SAME
     * `mem.enabled` against the value a write is about to produce; leaving it stale here would
     * feed a wrong "old" state into the very first post-reset write's transition check.
     *
     * Deliberately NOT done by calling esp32_cache_state_update() from here: that function also
     * runs the new stall-mask logic, which reads `current_cpu` -- meaningless/misleading at reset
     * time (this device can be reset from the monitor, machine init, or a guest-triggered SoC
     * reset processed on whichever vCPU requested it), and could spuriously add a bit that then
     * needs unwinding. Instead: set the registers to their POR value (0) and drive the
     * MemoryRegions/status directly to match, then reset the stall mask/bool as their own,
     * independent, unconditional step below -- no dependency on write-handler logic at all. */
    for (int core = 0; core < ESP32_CPU_COUNT; ++core) {
        s->cache_state[core].cache_ctrl_reg = 0;
        s->cache_state[core].cache_ctrl1_reg = 0;
        memory_region_set_enabled(&s->cache_state[core].drom0.mem, false);
        memory_region_set_enabled(&s->cache_state[core].iram0.mem, false);
        if (s->has_psram) {
            memory_region_set_enabled(&s->cache_state[core].dram1.mem, false);
        }
    }
    esp32_cache_reset(&s->cache_state[0]);
    esp32_cache_reset(&s->cache_state[1]);
    s->cache_state[0].drom0.illegal_access_status = false;
    s->cache_state[0].iram0.illegal_access_status = false;
    s->cache_state[0].dram1.illegal_access_status = false;
    s->cache_state[1].drom0.illegal_access_status = false;
    s->cache_state[1].iram0.illegal_access_status = false;
    s->cache_state[1].dram1.illegal_access_status = false;
    /* E125: clear BOTH the mask and its cached derived bool -- a stale true here would hold APP
     * CPU stalled, for an indeterminate extra duration, the next time it goes through a normal
     * post-reset bring-up (reset_state/clkgate already force the stall true immediately after
     * THIS reset regardless, via esp32_dport_update_appcpu_stall()'s other OR terms -- the bug
     * this specifically prevents is the flag surviving to interfere with a LATER, otherwise-
     * healthy bring-up in the SAME process, e.g. after a CACHEERR-triggered reboot).
     *
     * E129: appcpu_cache_race_stall_mask/_race_stall (bool) are gone -- see esp32_dport.h. Reset
     * both surviving masks the same way: unconditionally, independent of any write-handler logic.
     * appcpu_cache_wait_mask in particular must never survive a reset -- a stale bit here would
     * leave esp32_cache_state_update()'s Phase C waiting forever for a release that already
     * happened before this reset, since nothing calls cpu_resume() just because a reset occurred
     * (APP CPU is reset separately, via appcpu_reset_state/qemu_irq_lower(appcpu_stall_req) below
     * and the CPU reset machinery itself -- it does not need an additional wakeup here, but the
     * bookkeeping must not carry a stale "still waiting" bit into whatever runs next). */
    s->appcpu_cache_externally_disabled_mask = 0;
    s->appcpu_cache_wait_mask = 0;
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
    /* E129 (EVIDENCE.md, 2026-09-05, extends DECISION-014): esp32_cache_ill_read()'s own
     * cpu_stop_current()/cpu_loop_exit_restore() retry (the interlock replacing E124/E125's
     * blanket runstall) is a siglongjmp that skips softmmu/memory.c's normal-return clear of
     * mem_reentrancy_guard.engaged_in_io, exactly like every other VNEXT_B retry site DECISION-014
     * already covers -- without this, the guard would wedge `true` for this MemoryRegion after the
     * first compensated access, silently rejecting every later one (real or genuinely illegal)
     * with "Blocked re-entrant IO". Safe here for the same reason DECISION-014 gives for its own
     * seven devices: this retry path never releases the BQL (memory_region_dispatch_read()'s own
     * QEMU_IOTHREAD_LOCK_GUARD() holds it for the callback's entire duration), so the cross-vCPU
     * race the guard exists to catch cannot occur through it. */
    crs->illegal_access_trap_mem.disable_reentrancy_guard = true;
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

