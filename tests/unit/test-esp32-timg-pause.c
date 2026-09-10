/*
 * Unit test for esp32_timg_transport_pause_compute_delta() -- the E114 fix for the
 * SW_CPU_RESET_REGISTER storm (see the LasecSimul orchestrator's orchestrator/.ai/EVIDENCE.md,
 * entry E114). This is the pure calculation function extracted from
 * hw/timer/esp32_timg.c's esp32_timg_transport_pause_apply(); it has no QEMU device-model
 * dependencies, so it is tested here directly, without needing a real-QEMU regression run.
 *
 * This work is licensed under the terms of the GNU LGPL, version 2 or later.
 * See the COPYING.LIB file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "hw/timer/esp32_timg_pause_math.h"

static void test_pause_start_before_ns_base(void)
{
    /* The exact E114 reproduction (LASECSIMUL_WDT_CAUSAL_TRACE ring, entries 284-287): a FEED
     * (ns_base=1800265600) landed after the pause opened (pause_start=1800241700), and apply()
     * ran later (now=1800293900). The old, buggy formula computed
     * ns_base + (now - pause_start) = 1800265600 + 52200 = 1800317700, which is > now -- the bug.
     * The fixed formula must land exactly on `now`, never past it. */
    uint64_t delta = esp32_timg_transport_pause_compute_delta(1800241700, 1800265600, 1800293900);
    g_assert_cmpuint(delta, ==, 28300);
    g_assert_cmpuint(1800265600 + delta, ==, 1800293900);
}

static void test_ns_base_equals_now(void)
{
    uint64_t delta = esp32_timg_transport_pause_compute_delta(100, 500, 500);
    g_assert_cmpuint(delta, ==, 0);
    g_assert_cmpuint(500 + delta, ==, 500);
}

static void test_ns_base_after_now(void)
{
    /* Pathological/out-of-domain input: ns_base already ahead of now (should never happen at this
     * function's call site after the fix -- ns_base <= now is a system-wide invariant every other
     * writer maintains). The pure function must still be safe: it must not advance ns_base any
     * further into the future than it already, wrongly, is. */
    uint64_t delta = esp32_timg_transport_pause_compute_delta(100, 600, 500);
    g_assert_cmpuint(delta, ==, 0);
    g_assert_cmpuint(600 + delta, ==, 600);
}

static void test_pause_entirely_after_anchor(void)
{
    /* Ordinary, non-buggy case: no intervening feed, ns_base is older than pause_start. The whole
     * pause interval is legitimately uncompensated. */
    uint64_t delta = esp32_timg_transport_pause_compute_delta(1000, 100, 1500);
    g_assert_cmpuint(delta, ==, 500);
    g_assert_cmpuint(100 + delta, ==, 600);
}

static void test_feed_during_pause(void)
{
    /* General form of pause_start < ns_base < now (a feed landed strictly inside the open pause
     * window). Only the post-feed portion [ns_base, now) should be compensated. */
    uint64_t delta = esp32_timg_transport_pause_compute_delta(1000, 1200, 2000);
    g_assert_cmpuint(delta, ==, 800);
    g_assert_cmpuint(1200 + delta, ==, 2000);
}

static void test_repeated_compensation_does_not_duplicate(void)
{
    /* Mirrors esp32_timg_transport_pause_apply()'s own call pattern: after applying, the caller
     * advances transport_pause_start_virtual_ns to `now` (see the call site) before any second
     * apply. Two sequential calls across a split interval must compensate exactly the same total
     * as one call across the whole interval -- never more (no double-compensation, the original
     * bug's exact failure mode). */
    uint64_t ns_base = 1000;
    uint64_t pause_start = 1000;

    uint64_t delta1 = esp32_timg_transport_pause_compute_delta(pause_start, ns_base, 1500);
    ns_base += delta1;
    pause_start = 1500;

    uint64_t delta2 = esp32_timg_transport_pause_compute_delta(pause_start, ns_base, 2000);
    ns_base += delta2;

    g_assert_cmpuint(delta1 + delta2, ==, 1000);
    g_assert_cmpuint(ns_base, ==, 2000);
}

static void test_never_advances_past_now_property(void)
{
    /* The invariant that actually prevents esp32_timg_wdt_get_count()'s unsigned underflow
     * downstream: for any well-formed input (current_ns_base <= now_ns, the system-wide invariant
     * every other writer of ns_base maintains after this fix), the compensated ns_base must never
     * exceed now_ns. This is what "no underflow in get_count()" reduces to -- get_count()'s own
     * ns_now <= ws->ns_base check would never trigger on the very next call if this invariant
     * holds, since the next call's ns_now is >= the now_ns used here. */
    static const uint64_t cases[][3] = {
        /* {pause_start_ns, current_ns_base, now_ns} -- all satisfy current_ns_base <= now_ns */
        {0, 0, 1},
        {5, 5, 5},
        {5, 10, 100},
        {100, 5, 100},
        {100, 5, 1000},
        {1800241700, 1800265600, 1800293900},
    };
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        uint64_t pause_start = cases[i][0];
        uint64_t ns_base = cases[i][1];
        uint64_t now = cases[i][2];
        uint64_t delta = esp32_timg_transport_pause_compute_delta(pause_start, ns_base, now);
        g_assert_cmpuint(ns_base + delta, <=, now);
    }
}

/*
 * Not covered here: the union-of-lanes bookkeeping (DECISION-011,
 * esp32_timg_transport_pause()/transport_pause_active_count) that decides *which* pause_start_ns
 * reaches this function -- that is stateful, atomics-based logic with QEMU device-model
 * dependencies, a different kind of test than a pure calculation. By the time
 * esp32_timg_transport_pause_apply() calls this function, the union logic has already reduced
 * however many overlapping per-CPU pauses were open to a single pause_start_ns (the earliest
 * still-active one) -- from this function's perspective, "multiple lanes" and "one lane" are the
 * same input shape.
 */

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/esp32_timg/pause_compensation/pause_start_before_ns_base",
                     test_pause_start_before_ns_base);
    g_test_add_func("/esp32_timg/pause_compensation/ns_base_equals_now",
                     test_ns_base_equals_now);
    g_test_add_func("/esp32_timg/pause_compensation/ns_base_after_now",
                     test_ns_base_after_now);
    g_test_add_func("/esp32_timg/pause_compensation/pause_entirely_after_anchor",
                     test_pause_entirely_after_anchor);
    g_test_add_func("/esp32_timg/pause_compensation/feed_during_pause",
                     test_feed_during_pause);
    g_test_add_func("/esp32_timg/pause_compensation/repeated_compensation_does_not_duplicate",
                     test_repeated_compensation_does_not_duplicate);
    g_test_add_func("/esp32_timg/pause_compensation/never_advances_past_now_property",
                     test_never_advances_past_now_property);
    return g_test_run();
}
