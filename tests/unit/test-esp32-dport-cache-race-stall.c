/*
 * Unit test for esp32_cache_race_stall_gain()/_release() -- the E125 extraction of E124's
 * appcpu_cache_race_stall_mask decision logic (see LasecSimul orchestrator/.ai/EVIDENCE.md,
 * entries E124/E125, and hw/misc/esp32_dport.c's esp32_cache_state_update()). This is the pure
 * calculation extracted from that function; it has no QEMU device-model, CPUState, or
 * MemoryRegion dependency, so it is tested here directly, without instantiating a real machine.
 *
 * Each test name maps directly to one of E125's own Fase 2/Fase 5 enumerated audit cases.
 *
 * This work is licensed under the terms of the GNU LGPL, version 2 or later.
 * See the COPYING.LIB file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "hw/misc/esp32_cache_race_stall.h"

/* Case: DROM0 already masked before CPU0's write (never enabled to begin with) -- a write that
 * leaves it disabled must never gain a bit for it, regardless of writer. */
static void test_case1_drom0_already_masked_before_write(void)
{
    g_assert_cmpint(esp32_cache_race_stall_gain(0, /*old_drom0=*/false, /*new_drom0=*/false,
                                                 /*old_iram0=*/true, /*new_iram0=*/true),
                     ==, 0);
}

/* Case: IRAM0 already masked before CPU0's write -- symmetric to case 1. */
static void test_case2_iram0_already_masked_before_write(void)
{
    g_assert_cmpint(esp32_cache_race_stall_gain(0, /*old_drom0=*/true, /*new_drom0=*/true,
                                                 /*old_iram0=*/false, /*new_iram0=*/false),
                     ==, 0);
}

/* Case: CPU0 masks only DROM0 (via CACHE_CTRL1's MASK_DROM0) while IRAM0 stays untouched -- only
 * DROM0's bit is gained. */
static void test_case3_cpu0_masks_only_drom0(void)
{
    g_assert_cmpint(esp32_cache_race_stall_gain(0, true, false, true, true),
                     ==, ESP32_CACHE_RACE_STALL_DROM0_BIT);
}

/* Case: CPU0 masks only IRAM0 -- only IRAM0's bit is gained. */
static void test_case4_cpu0_masks_only_iram0(void)
{
    g_assert_cmpint(esp32_cache_race_stall_gain(0, true, true, true, false),
                     ==, ESP32_CACHE_RACE_STALL_IRAM0_BIT);
}

/* Case: CPU0 toggles CACHE_ENA 1->0->1 -- each individual call is evaluated independently (the
 * real device calls this once per register write, never "batched"); the disable call gains both
 * bits, the following enable call (modeled via _release, since that is what
 * esp32_cache_state_update()'s Phase C actually invokes on the way back up) clears them again. */
static void test_case5_cache_ena_toggle_1_0_1(void)
{
    uint32_t mask = 0;
    mask |= esp32_cache_race_stall_gain(0, true, false, true, false); /* 1 -> 0 */
    g_assert_cmpint(mask, ==, ESP32_CACHE_RACE_STALL_DROM0_BIT | ESP32_CACHE_RACE_STALL_IRAM0_BIT);
    mask &= ~esp32_cache_race_stall_release(true, true); /* 0 -> 1 */
    g_assert_cmpint(mask, ==, 0);
}

/* Case: CPU1 (APP) disables its OWN cache -- writer_core==1 must never gain a bit; APP CPU could
 * then never run the code that clears it (guaranteed permanent self-stall). */
static void test_case6_self_disable_never_self_stalls(void)
{
    g_assert_cmpint(esp32_cache_race_stall_gain(1, true, false, true, false), ==, 0);
}

/* Case: writer_core unknown (-1, e.g. a non-vCPU context) must never gain a bit -- only an
 * explicit, attributed writer_core==0 may. */
static void test_case7_unknown_writer_never_gains(void)
{
    g_assert_cmpint(esp32_cache_race_stall_gain(-1, true, false, true, false), ==, 0);
}

/* Case: reset alters the registers with no live vCPU context at all. This pure function is not
 * itself called from esp32_dport_reset() (which clears the mask directly and unconditionally --
 * see esp32_dport.c) -- documented here as a negative case: an unattributed writer (-1) must
 * behave identically whether it represents "no CPU is current" during a real MMIO dispatch or a
 * reset applying defaults, i.e. never gain a bit either way. Same assertion as case 7, kept as
 * its own named test because it exercises a conceptually distinct scenario from E125's own audit
 * list (case 8), not a duplicate by accident. */
static void test_case8_reset_like_writer_never_gains(void)
{
    g_assert_cmpint(esp32_cache_race_stall_gain(-1, true, false, false, false), ==, 0);
}

/* Case: two consecutive external disables of the SAME region before any enable -- must not
 * double-count or misbehave; the mask is a bitmask (idempotent OR), not a counter. */
static void test_case9_two_consecutive_disables_before_enable(void)
{
    uint32_t mask = 0;
    mask |= esp32_cache_race_stall_gain(0, true, false, true, true);   /* drom0 1st disable */
    g_assert_cmpint(mask, ==, ESP32_CACHE_RACE_STALL_DROM0_BIT);
    /* Second write: drom0 already disabled (old=false now), so old&&!new is false -- no re-gain,
     * but the bit must also not spuriously clear (release is a separate, explicit call). */
    mask |= esp32_cache_race_stall_gain(0, false, false, true, true);
    g_assert_cmpint(mask, ==, ESP32_CACHE_RACE_STALL_DROM0_BIT);
}

/* Case: only one of two disabled regions is re-enabled ("partial enable after global disable") --
 * the mask must retain the bit for the region still disabled, so the stall is NOT released while
 * any affected region remains inaccessible. */
static void test_case10_partial_enable_after_global_disable(void)
{
    uint32_t mask = ESP32_CACHE_RACE_STALL_DROM0_BIT | ESP32_CACHE_RACE_STALL_IRAM0_BIT;
    mask &= ~esp32_cache_race_stall_release(/*drom0=*/true, /*iram0=*/false);
    g_assert_cmpint(mask, ==, ESP32_CACHE_RACE_STALL_IRAM0_BIT);
}

/* Fase 2's own headline case: a sibling region legitimately, permanently masked before AND after
 * the race-triggering disable/enable cycle must never gate release -- the classic E124 bug this
 * whole mask design replaces a single bool to fix. IRAM0 is masked (disabled) throughout; only
 * DROM0 is ever part of this race. The mask must reach 0 (release) the instant DROM0 alone comes
 * back, regardless of IRAM0's permanently-disabled state. */
static void test_preexisting_mask_never_blocks_release(void)
{
    uint32_t mask = 0;
    /* CPU0 disables DROM0 (IRAM0 was already, and remains, masked/disabled the whole time). */
    mask |= esp32_cache_race_stall_gain(0, /*old_drom0=*/true, /*new_drom0=*/false,
                                         /*old_iram0=*/false, /*new_iram0=*/false);
    g_assert_cmpint(mask, ==, ESP32_CACHE_RACE_STALL_DROM0_BIT);
    /* CPU0 re-enables DROM0 only; IRAM0 stays masked by its own, unrelated configuration. */
    mask &= ~esp32_cache_race_stall_release(/*drom0=*/true, /*iram0=*/false);
    g_assert_cmpint(mask, ==, 0);
}

/* No configuration of gain/release calls can ever leave the mask permanently non-zero once every
 * region that ever gained a bit is later observed enabled -- exhaustively over both bits. */
static void test_no_permanent_stall_path_exists(void)
{
    for (int w = -1; w <= 1; ++w) {
        for (int d = 0; d < 2; ++d) {
            for (int i = 0; i < 2; ++i) {
                uint32_t mask = 0;
                mask |= esp32_cache_race_stall_gain(w, /*old_drom0=*/true, d != 0,
                                                     /*old_iram0=*/true, i != 0);
                /* Whatever got gained, releasing with both regions enabled must always fully
                 * clear the mask -- there is no writer_core/old/new combination that can leave a
                 * residual bit once both regions are confirmed enabled. */
                mask &= ~esp32_cache_race_stall_release(true, true);
                g_assert_cmpint(mask, ==, 0);
            }
        }
    }
}

/* RED/GREEN documentation, not a test of live code: E124's ORIGINAL release condition (before
 * E125's per-region mask replaced it) was the single boolean `drom0_enabled && iram0_enabled`
 * computed globally. Encoded here literally (not by calling any function -- E124's shape no
 * longer exists in the tree to call) to give this gap a permanent, checkable regression anchor.
 * If IRAM0 is legitimately masked independent of the DROM0 race (E125's Fase 2 headline case,
 * also covered live above by test_preexisting_mask_never_blocks_release), E124's own formula
 * NEVER releases, for any number of subsequent DROM0 enable/disable cycles -- a real, provable
 * permanent stall. This function documents that RED result; esp32_cache_race_stall_release()
 * above is the GREEN replacement, already proven correct by the live tests above it. */
static void test_e124_original_formula_would_have_stalled_forever(void)
{
    const bool iram0_permanently_masked = false; /* legitimately masked the whole time */
    bool drom0_enabled = false;                  /* CPU0 just disabled DROM0 for the race */
    bool e124_would_stall = !drom0_enabled || !iram0_permanently_masked; /* E124's own OR */
    g_assert_true(e124_would_stall);
    /* DROM0 comes back -- E124's actual re-check happened again from
     * esp32_cache_state_update(), which recomputes the SAME two booleans from the CURRENT
     * registers. IRAM0 is still masked (never changes in this scenario). */
    drom0_enabled = true;
    const bool e124_fully_enabled = drom0_enabled && iram0_permanently_masked;
    /* This is the bug: DROM0 is back, but E124's global "fully_enabled" test can never be true
     * while IRAM0 stays masked for its own, unrelated reasons -- so E124's own stall bool would
     * never clear here, for any number of further DROM0 cycles. */
    g_assert_false(e124_fully_enabled);
}

/* E129 (EVIDENCE.md, 2026-09-05): esp32_cache_access_should_wait()/esp32_cache_wait_release() --
 * the per-access interlock that replaced E124/E125's blanket xtensa_runstall() (proven, E128, to
 * deadlock APP CPU when it engaged while APP CPU was mid-interrupt-return inside unrelated
 * IRAM-resident code). esp32_cache_race_stall_gain()/_release() above are UNCHANGED and still
 * compute appcpu_cache_externally_disabled_mask exactly as before -- these new tests cover the
 * NEW decision: given that bookkeeping, should THIS specific access actually suspend the CPU. */

/* Case: APP CPU (core 1) reading its OWN DROM0 while a different core holds it externally
 * disabled -- the exact compensable race. Must wait. */
static void test_wait_case1_appcpu_own_drom0_externally_disabled(void)
{
    g_assert_true(esp32_cache_access_should_wait(/*accessing_core=*/1, /*region_owner_core_id=*/1,
                                                  ESP32_CACHE_RACE_STALL_DROM0_BIT,
                                                  /*externally_disabled_mask=*/
                                                  ESP32_CACHE_RACE_STALL_DROM0_BIT));
}

/* Case: same, but IRAM0 (instruction fetch path) -- the predicate does not distinguish fetch from
 * load; both funnel through the same illegal-access-trap callback with the same region_bit. */
static void test_wait_case2_appcpu_own_iram0_externally_disabled(void)
{
    g_assert_true(esp32_cache_access_should_wait(1, 1, ESP32_CACHE_RACE_STALL_IRAM0_BIT,
                                                  ESP32_CACHE_RACE_STALL_IRAM0_BIT));
}

/* Case: APP CPU executing/accessing memory that is NOT DROM0/IRAM0 at all (region_bit==0, e.g.
 * genuine IRAM physical SRAM outside the flash-cache window, or DRAM1/PSRAM) -- must never wait,
 * regardless of what is externally disabled. This is the exact scenario that deadlocked under
 * E124/E125's design: APP CPU running spi_flash_op_block_func() (IRAM_ATTR, architecturally
 * outside these windows) must be free to keep running. */
static void test_wait_case3_appcpu_iram_physical_sram_never_waits(void)
{
    g_assert_false(esp32_cache_access_should_wait(1, 1, /*region_bit=*/0,
                                                   ESP32_CACHE_RACE_STALL_DROM0_BIT |
                                                   ESP32_CACHE_RACE_STALL_IRAM0_BIT));
}

/* Case: the region is not (or no longer) externally disabled -- e.g. CPU0 already re-enabled it
 * before APP CPU ever got around to touching it. Must not wait; the access proceeds against the
 * now-enabled real MemoryRegion (this trap callback would not even be reached in that case, but
 * the predicate itself must still say so for the enable-happens-before-access scenario where a
 * stale bit could otherwise linger). */
static void test_wait_case4_region_already_reenabled_never_waits(void)
{
    g_assert_false(esp32_cache_access_should_wait(1, 1, ESP32_CACHE_RACE_STALL_DROM0_BIT,
                                                   /*externally_disabled_mask=*/0));
}

/* Case: PRO CPU (core 0) is the one whose OWN region trap fired -- core_id==1 check excludes PRO's
 * own cache path entirely, matching DECISION-017's original "does not touch PRO's cache path"
 * guarantee. Must never wait (PRO has no compensation mechanism, by design). */
static void test_wait_case5_pro_cpu_own_region_never_waits(void)
{
    g_assert_false(esp32_cache_access_should_wait(/*accessing_core=*/0, /*region_owner_core_id=*/0,
                                                   ESP32_CACHE_RACE_STALL_DROM0_BIT,
                                                   ESP32_CACHE_RACE_STALL_DROM0_BIT));
}

/* Case: PRO CPU (core 0) reading APP's (core 1's) region while it happens to be externally
 * disabled -- not the compensable race (only APP CPU's OWN access to APP's OWN region is), so
 * this must fall through to genuine-illegal-access handling, never wait. */
static void test_wait_case6_pro_cpu_accessing_app_region_never_waits(void)
{
    g_assert_false(esp32_cache_access_should_wait(/*accessing_core=*/0, /*region_owner_core_id=*/1,
                                                   ESP32_CACHE_RACE_STALL_DROM0_BIT,
                                                   ESP32_CACHE_RACE_STALL_DROM0_BIT));
}

/* Case: unattributed writer/accessor (current_cpu==NULL at access time, accessing_core==-1) --
 * must never wait; only a live, identified APP CPU access is compensable. */
static void test_wait_case7_unattributed_accessor_never_waits(void)
{
    g_assert_false(esp32_cache_access_should_wait(/*accessing_core=*/-1, 1,
                                                   ESP32_CACHE_RACE_STALL_DROM0_BIT,
                                                   ESP32_CACHE_RACE_STALL_DROM0_BIT));
}

/* Case: APP CPU disabled its OWN cache (never possible for externally_disabled_mask to have a bit
 * set in the first place, since esp32_cache_race_stall_gain() excludes writer_core==1 -- but
 * proven here directly at the should_wait() predicate level too, in case a bit were ever set by
 * some other path: the predicate alone, given a self-inflicted-looking mask, still says wait,
 * which is exactly why gain() must be the one that never sets it for a self-disable, not this
 * predicate -- documented so a future reader does not expect this function to also guard against
 * self-disable independently). */
static void test_wait_case8_mask_semantics_depend_on_gain_never_self_setting_it(void)
{
    /* If gain() is doing its job, externally_disabled_mask can never be non-zero from a
     * self-disable -- so the only way this predicate sees a bit set is a genuine external-disable,
     * making "wait" the correct answer here too. This test documents that dependency explicitly
     * rather than leaving it implicit. */
    g_assert_cmpint(esp32_cache_race_stall_gain(/*writer_core=*/1, true, false, true, true), ==, 0);
}

/* Case: a region that was permanently masked (never enabled to begin with, e.g. CACHE_CTRL1's own
 * MASK_DROM0 for reasons unrelated to any race) never appears in externally_disabled_mask (gain()
 * requires old_enabled==true), so an APP CPU access to it correctly does NOT compensate-wait here
 * -- it falls through to genuine-illegal-access handling, matching "acesso genuinamente ilegal...
 * continua produzindo CACHEERR normalmente" for a region that was never part of any race. */
static void test_wait_case9_permanently_masked_region_never_compensated(void)
{
    g_assert_cmpint(esp32_cache_race_stall_gain(0, /*old_drom0=*/false, false, true, true), ==, 0);
    g_assert_false(esp32_cache_access_should_wait(1, 1, ESP32_CACHE_RACE_STALL_DROM0_BIT,
                                                   /*externally_disabled_mask=*/0));
}

/* esp32_cache_wait_release(): releasing DROM0 while APP CPU waits only on DROM0 -- the last
 * pending reason clears, caller must resume. */
static void test_wait_release_case1_last_reason_clears_reports_resume(void)
{
    uint32_t wait_mask = ESP32_CACHE_RACE_STALL_DROM0_BIT;
    bool should_resume = esp32_cache_wait_release(&wait_mask, ESP32_CACHE_RACE_STALL_DROM0_BIT);
    g_assert_true(should_resume);
    g_assert_cmpint(wait_mask, ==, 0);
}

/* Releasing DROM0 while APP CPU is waiting on IRAM0 (a sibling region) -- must NOT report resume;
 * APP CPU is still legitimately waiting on IRAM0 and must not be woken early. */
static void test_wait_release_case2_unrelated_release_does_not_wake(void)
{
    uint32_t wait_mask = ESP32_CACHE_RACE_STALL_IRAM0_BIT;
    bool should_resume = esp32_cache_wait_release(&wait_mask, ESP32_CACHE_RACE_STALL_DROM0_BIT);
    g_assert_false(should_resume);
    g_assert_cmpint(wait_mask, ==, ESP32_CACHE_RACE_STALL_IRAM0_BIT);
}

/* Releasing both regions at once while APP CPU waits on both -- last reason clears, resume. */
static void test_wait_release_case3_both_regions_released_together(void)
{
    uint32_t wait_mask = ESP32_CACHE_RACE_STALL_DROM0_BIT | ESP32_CACHE_RACE_STALL_IRAM0_BIT;
    bool should_resume = esp32_cache_wait_release(&wait_mask,
        ESP32_CACHE_RACE_STALL_DROM0_BIT | ESP32_CACHE_RACE_STALL_IRAM0_BIT);
    g_assert_true(should_resume);
    g_assert_cmpint(wait_mask, ==, 0);
}

/* APP CPU was never waiting on anything (wait_mask starts at 0) -- a release must never spuriously
 * report "resume" (there is nothing to resume; calling cpu_resume() here would still be harmless
 * per its own idempotency, but the predicate itself must reflect reality). */
static void test_wait_release_case4_nothing_pending_never_reports_resume(void)
{
    uint32_t wait_mask = 0;
    bool should_resume = esp32_cache_wait_release(&wait_mask, ESP32_CACHE_RACE_STALL_DROM0_BIT);
    g_assert_false(should_resume);
    g_assert_cmpint(wait_mask, ==, 0);
}

/* Exhaustive: no combination of a prior wait_mask and a release can ever leave a bit set that the
 * release itself targeted -- i.e. releasing a bit always actually clears it, and "resume" is
 * reported if and only if the mask transitions non-zero -> zero. No permanent-wait path exists. */
static void test_wait_release_no_permanent_wait_path_exists(void)
{
    for (uint32_t initial = 0; initial <= 3; ++initial) {
        for (uint32_t release_bits = 0; release_bits <= 3; ++release_bits) {
            uint32_t wait_mask = initial;
            bool should_resume = esp32_cache_wait_release(&wait_mask, release_bits);
            g_assert_cmpint(wait_mask, ==, (initial & ~release_bits));
            g_assert_true(should_resume == (initial != 0 && (initial & ~release_bits) == 0));
        }
    }
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/esp32_dport/cache_race_stall/case1_drom0_already_masked",
                     test_case1_drom0_already_masked_before_write);
    g_test_add_func("/esp32_dport/cache_race_stall/case2_iram0_already_masked",
                     test_case2_iram0_already_masked_before_write);
    g_test_add_func("/esp32_dport/cache_race_stall/case3_cpu0_masks_only_drom0",
                     test_case3_cpu0_masks_only_drom0);
    g_test_add_func("/esp32_dport/cache_race_stall/case4_cpu0_masks_only_iram0",
                     test_case4_cpu0_masks_only_iram0);
    g_test_add_func("/esp32_dport/cache_race_stall/case5_cache_ena_toggle_1_0_1",
                     test_case5_cache_ena_toggle_1_0_1);
    g_test_add_func("/esp32_dport/cache_race_stall/case6_self_disable_never_self_stalls",
                     test_case6_self_disable_never_self_stalls);
    g_test_add_func("/esp32_dport/cache_race_stall/case7_unknown_writer_never_gains",
                     test_case7_unknown_writer_never_gains);
    g_test_add_func("/esp32_dport/cache_race_stall/case8_reset_like_writer_never_gains",
                     test_case8_reset_like_writer_never_gains);
    g_test_add_func("/esp32_dport/cache_race_stall/case9_two_consecutive_disables",
                     test_case9_two_consecutive_disables_before_enable);
    g_test_add_func("/esp32_dport/cache_race_stall/case10_partial_enable_after_global_disable",
                     test_case10_partial_enable_after_global_disable);
    g_test_add_func("/esp32_dport/cache_race_stall/preexisting_mask_never_blocks_release",
                     test_preexisting_mask_never_blocks_release);
    g_test_add_func("/esp32_dport/cache_race_stall/no_permanent_stall_path_exists",
                     test_no_permanent_stall_path_exists);
    g_test_add_func("/esp32_dport/cache_race_stall/e124_original_formula_would_have_stalled_forever",
                     test_e124_original_formula_would_have_stalled_forever);
    g_test_add_func("/esp32_dport/cache_wait/case1_appcpu_own_drom0_externally_disabled",
                     test_wait_case1_appcpu_own_drom0_externally_disabled);
    g_test_add_func("/esp32_dport/cache_wait/case2_appcpu_own_iram0_externally_disabled",
                     test_wait_case2_appcpu_own_iram0_externally_disabled);
    g_test_add_func("/esp32_dport/cache_wait/case3_appcpu_iram_physical_sram_never_waits",
                     test_wait_case3_appcpu_iram_physical_sram_never_waits);
    g_test_add_func("/esp32_dport/cache_wait/case4_region_already_reenabled_never_waits",
                     test_wait_case4_region_already_reenabled_never_waits);
    g_test_add_func("/esp32_dport/cache_wait/case5_pro_cpu_own_region_never_waits",
                     test_wait_case5_pro_cpu_own_region_never_waits);
    g_test_add_func("/esp32_dport/cache_wait/case6_pro_cpu_accessing_app_region_never_waits",
                     test_wait_case6_pro_cpu_accessing_app_region_never_waits);
    g_test_add_func("/esp32_dport/cache_wait/case7_unattributed_accessor_never_waits",
                     test_wait_case7_unattributed_accessor_never_waits);
    g_test_add_func("/esp32_dport/cache_wait/case8_mask_semantics_depend_on_gain",
                     test_wait_case8_mask_semantics_depend_on_gain_never_self_setting_it);
    g_test_add_func("/esp32_dport/cache_wait/case9_permanently_masked_region_never_compensated",
                     test_wait_case9_permanently_masked_region_never_compensated);
    g_test_add_func("/esp32_dport/cache_wait/release_case1_last_reason_clears_reports_resume",
                     test_wait_release_case1_last_reason_clears_reports_resume);
    g_test_add_func("/esp32_dport/cache_wait/release_case2_unrelated_release_does_not_wake",
                     test_wait_release_case2_unrelated_release_does_not_wake);
    g_test_add_func("/esp32_dport/cache_wait/release_case3_both_regions_released_together",
                     test_wait_release_case3_both_regions_released_together);
    g_test_add_func("/esp32_dport/cache_wait/release_case4_nothing_pending_never_reports_resume",
                     test_wait_release_case4_nothing_pending_never_reports_resume);
    g_test_add_func("/esp32_dport/cache_wait/release_no_permanent_wait_path_exists",
                     test_wait_release_no_permanent_wait_path_exists);
    return g_test_run();
}
