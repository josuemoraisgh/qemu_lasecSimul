#pragma once

#include <stdint.h>
#include <stdbool.h>

/*
 * E125 (LasecSimul orchestrator/.ai/EVIDENCE.md, entries E124/E125): pure, dependency-free
 * extraction of the appcpu_cache_race_stall_mask decision logic at the heart of E124's fix for
 * the CACHEERR race (CPU1/APP reading DROM0 while CPU0/PRO holds its cache disabled -- see
 * hw/misc/esp32_dport.c's esp32_cache_state_update() for the full account and citations).
 * Deliberately standalone (no QEMU headers) so the two decisions themselves -- "does this write
 * newly require holding APP CPU" and "does this write let go of a previously-held reason" -- are
 * unit-testable (tests/unit/test-esp32-dport-cache-race-stall.c) without instantiating a real
 * Esp32DportState, CPUState, or MemoryRegion.
 *
 * Split into two functions, not one, because esp32_cache_state_update() must call them at two
 * DIFFERENT points relative to the actual MemoryRegion enable/disable it performs in between
 * (E125's own Fase 1 finding): the "gain" (stall newly required) must be applied BEFORE the
 * region is disabled, and the "release" (stall no longer required) must only be applied AFTER
 * the region is re-enabled and re-synced. A single combined function computed once could not
 * express that ordering.
 *
 * esp32_dport.h's own ESP32_CACHE_RACE_STALL_DROM0/IRAM0 are aliases of the bits defined here
 * (this header has no QEMU dependency -- only <stdint.h>/<stdbool.h> -- so esp32_dport.h includes
 * it and defines its own names in terms of these, rather than the reverse, keeping exactly one
 * definition to stay in sync).
 */
#define ESP32_CACHE_RACE_STALL_DROM0_BIT (1u << 0)
#define ESP32_CACHE_RACE_STALL_IRAM0_BIT (1u << 1)

/*
 * Only a DIFFERENT core (writer_core == 0, i.e. PRO/CPU0) pulling APP's (CPU1's) own cache out
 * from under it is the proven hazard this stall exists for. APP CPU disabling its own cache
 * (writer_core == 1) must never be treated as a hazard -- it could then never run the code that
 * re-enables it, a guaranteed permanent self-stall. An unknown writer (writer_core == -1, e.g. a
 * device reset applying its own POR defaults with no live vCPU context) must also never gain a
 * bit here -- esp32_dport_reset() clears the whole mask directly and unconditionally instead,
 * precisely so this function never has to guess what an unattributed writer meant.
 *
 * old_*_enabled must be the region's OWN enabled state strictly BEFORE this write is applied;
 * new_*_enabled is the value this write is ABOUT to produce. Only a true old=enabled ->
 * new=disabled transition, for a region that genuinely toggled, ever sets a bit -- a region
 * that was already disabled (e.g. permanently masked via CACHE_CTRL1 for reasons unrelated to
 * this race) never contributes a bit no matter how many further writes toggle CACHE_ENA, because
 * old_*_enabled is already false for it every time.
 */
static inline uint32_t esp32_cache_race_stall_gain(int writer_core,
                                                    bool old_drom0_enabled, bool new_drom0_enabled,
                                                    bool old_iram0_enabled, bool new_iram0_enabled)
{
    uint32_t gained = 0;
    if (writer_core == 0) {
        if (old_drom0_enabled && !new_drom0_enabled) {
            gained |= ESP32_CACHE_RACE_STALL_DROM0_BIT;
        }
        if (old_iram0_enabled && !new_iram0_enabled) {
            gained |= ESP32_CACHE_RACE_STALL_IRAM0_BIT;
        }
    }
    return gained;
}

/*
 * Release is unconditional on the writer -- it does not matter WHO re-enabled a region, only
 * that it is enabled now. This is the per-region replacement for E124's original "both regions
 * fully enabled" global test, which could never fire (a permanent stall) whenever a sibling
 * region was legitimately masked independent of this race for its own, unrelated reasons.
 */
static inline uint32_t esp32_cache_race_stall_release(bool new_drom0_enabled, bool new_iram0_enabled)
{
    uint32_t released = 0;
    if (new_drom0_enabled) {
        released |= ESP32_CACHE_RACE_STALL_DROM0_BIT;
    }
    if (new_iram0_enabled) {
        released |= ESP32_CACHE_RACE_STALL_IRAM0_BIT;
    }
    return released;
}

/*
 * E129 (EVIDENCE.md, 2026-09-05): closes the deadlock E128 proved in E124/E125's own use of the
 * two functions above. Those functions are UNCHANGED and still correct -- they compute
 * `Esp32DportState::appcpu_cache_externally_disabled_mask` (renamed from E124/E125's
 * `appcpu_cache_race_stall_mask`), pure bookkeeping of "which of APP's own regions is currently
 * disabled by a DIFFERENT core", updated exactly as before (gain before the MemoryRegion changes,
 * release after). What E124/E125 got wrong was turning that bookkeeping DIRECTLY into a blanket
 * `xtensa_runstall()` of all of APP CPU's execution the instant a bit was set -- proven (E128) to
 * deadlock APP CPU permanently if the bit happens to be set while APP CPU is mid-interrupt-return
 * inside unrelated, `IRAM_ATTR` code (`spi_flash_op_block_func()`'s own busy-wait) that never
 * touches the disabled region at all and needs no protection.
 *
 * The fix: `externally_disabled_mask` no longer drives any CPU-wide stall by itself. A SECOND,
 * separate mask, `appcpu_cache_wait_mask`, is set only when APP CPU's OWN load or instruction
 * fetch actually LANDS on a region currently in `externally_disabled_mask` -- checked from inside
 * `hw/misc/esp32_dport.c`'s `esp32_cache_ill_read()` (the illegal-access-trap MemoryRegion's own
 * read callback, already the single choke point every access to a disabled DROM0/IRAM0 region
 * passes through, real CACHEERR or not). Only THAT specific access is suspended
 * (`cpu_stop_current()` + `cpu_loop_exit_restore()`, the same idiom already proven safe throughout
 * this fork for VNEXT_B backpressure replay -- never releases the BQL, so it coexists safely with
 * reset/clkgate/RUNSTALL, which remain on the orthogonal `env.runstall`/`appcpu_stall_req` axis
 * this mechanism no longer touches at all) -- APP CPU's own unrelated IRAM/DRAM execution is never
 * touched, so it cannot be caught stalled mid-interrupt-return again. `esp32_cache_access_should_
 * wait()` is the pure predicate for "should THIS access be deferred rather than trapped as
 * illegal"; `esp32_cache_wait_release()` is the pure state transition applied when the region a
 * pending access was waiting on becomes available again (mirrors `_release()` above but tracks
 * `appcpu_cache_wait_mask`, not `externally_disabled_mask`, and reports whether this was the LAST
 * pending reason -- the caller only calls `cpu_resume()` when it was, so a still-pending wait on a
 * sibling region is never released early).
 */

/*
 * @accessing_core: the CPU whose load/fetch just landed on the illegal-access-trap MemoryRegion
 *   (-1 if unattributed, e.g. no live vCPU context).
 * @region_owner_core_id: which core's OWN cache region this specific access hit (Esp32CacheState::
 *   core_id -- 0 or 1; this trap region is instantiated once per core, so this is always known).
 * @region_bit: ESP32_CACHE_RACE_STALL_DROM0_BIT/IRAM0_BIT for whichever region was hit, or 0 for a
 *   region this compensation was never scoped to (e.g. DRAM1/PSRAM).
 * @externally_disabled_mask: the region owner's current Esp32DportState::
 *   appcpu_cache_externally_disabled_mask.
 *
 * Only APP CPU (core 1) touching ITS OWN region, while a bit for that exact region is set (i.e. a
 * DIFFERENT core is the one currently holding it disabled), is compensable. PRO CPU reading APP's
 * region (or any other combination) is not this race and must fall through to the existing
 * genuine-illegal-access handling unchanged -- this predicate returning false is what preserves
 * "acesso genuinamente ilegal... continua produzindo CACHEERR normalmente".
 */
static inline bool esp32_cache_access_should_wait(int accessing_core, int region_owner_core_id,
                                                   uint32_t region_bit,
                                                   uint32_t externally_disabled_mask)
{
    return region_owner_core_id == 1 && accessing_core == 1 && region_bit != 0 &&
           (externally_disabled_mask & region_bit) != 0;
}

/*
 * Applies a release (the same `release_bits` esp32_cache_race_stall_release() just computed) to
 * the wait mask specifically. Returns true only when this release clears the LAST bit APP CPU was
 * actually waiting on (was non-zero, now zero) -- exactly the instant `cpu_resume()` must be
 * called; a still-pending wait on a sibling region (e.g. DROM0 released while APP CPU is waiting
 * on IRAM0) must not wake it early, so this returns false in that case even though `release_bits`
 * itself was non-zero.
 */
static inline bool esp32_cache_wait_release(uint32_t *wait_mask, uint32_t release_bits)
{
    bool was_waiting = *wait_mask != 0;
    *wait_mask &= ~release_bits;
    return was_waiting && *wait_mask == 0;
}
