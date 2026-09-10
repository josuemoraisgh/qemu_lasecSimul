#include "qemu/osdep.h"
#include "hw/timer/esp32_timg_pause_math.h"
#include "hw/timer/esp32_timg_wdt_scale_math.h"

static uint64_t abs_delta_u64(uint64_t a, uint64_t b)
{
    return a > b ? a - b : b - a;
}

static void test_e134_exact_reanchor_keeps_scaled_budget(void)
{
    const uint64_t feed_ns_base = 3122490100ULL;
    const uint64_t config_ns_base = 3426994200ULL;
    const uint32_t stage_timeout = 600;
    const uint32_t prescale = 40000;
    const uint32_t apb_hz = 80000000;
    const uint32_t scale = 100;

    const uint64_t literal_tick_ns =
        esp32_timg_wdt_ticks_to_ns(1, prescale, apb_hz);
    g_assert_cmpuint(literal_tick_ns, ==, 500000);

    Esp32TimgWdtDeadlineCalc original =
        esp32_timg_wdt_compute_stage_deadline(feed_ns_base, 0,
                                              stage_timeout, prescale,
                                              apb_hz, scale);
    g_assert_cmpuint(original.effective_timeout_ticks, ==, 60000);
    g_assert_cmpuint(original.remaining_ticks, ==, 60000);
    g_assert_cmpuint(original.remaining_ns, ==, 30000000000ULL);
    g_assert_cmpuint(original.deadline_ns, ==, 33122490100ULL);

    const uint64_t literal_elapsed_ticks =
        (config_ns_base - feed_ns_base) / literal_tick_ns;
    g_assert_cmpuint(literal_elapsed_ticks, ==, 609);

    Esp32TimgWdtDeadlineCalc reanchored =
        esp32_timg_wdt_compute_stage_deadline(config_ns_base,
                                              literal_elapsed_ticks,
                                              stage_timeout, prescale,
                                              apb_hz, scale);

    g_assert_cmpuint(reanchored.effective_timeout_ticks, ==, 60000);
    g_assert_cmpuint(reanchored.remaining_ticks, ==, 59391);
    g_assert_cmpuint(reanchored.remaining_ns, ==, 29695500000ULL);
    g_assert_cmpuint(reanchored.deadline_ns, ==, 33122494200ULL);
    g_assert_cmpuint(abs_delta_u64(reanchored.deadline_ns,
                                   original.deadline_ns), <, literal_tick_ns);

    const uint64_t legacy_deadline =
        esp32_timg_wdt_compute_stage_deadline_ns_legacy_prefixed(config_ns_base,
                                                                 literal_elapsed_ticks,
                                                                 stage_timeout,
                                                                 prescale,
                                                                 apb_hz,
                                                                 scale);
    g_assert_cmpuint(legacy_deadline, ==, 3426994200ULL);
}

static void test_scale_zero_and_one_are_literal(void)
{
    const uint64_t scale_zero =
        esp32_timg_wdt_compute_stage_deadline_ns(1000, 5, 10, 1, 1000000, 0);
    const uint64_t scale_one =
        esp32_timg_wdt_compute_stage_deadline_ns(1000, 5, 10, 1, 1000000, 1);
    const uint64_t legacy_one =
        esp32_timg_wdt_compute_stage_deadline_ns_legacy_prefixed(1000, 5, 10, 1,
                                                                 1000000, 1);

    g_assert_cmpuint(scale_zero, ==, scale_one);
    g_assert_cmpuint(scale_one, ==, legacy_one);
    g_assert_cmpuint(scale_one, ==, 6000);

    const uint64_t literal_expired =
        esp32_timg_wdt_compute_stage_deadline_ns(3426994200ULL, 609,
                                                 600, 40000, 80000000, 1);
    g_assert_cmpuint(literal_expired, ==, 3426994200ULL);
}

static void test_no_reconfig_matches_scaled_feed_budget(void)
{
    Esp32TimgWdtDeadlineCalc calc =
        esp32_timg_wdt_compute_stage_deadline(5000000, 0, 600, 40000,
                                              80000000, 100);
    g_assert_cmpuint(calc.remaining_ticks, ==, 60000);
    g_assert_cmpuint(calc.remaining_ns, ==, 30000000000ULL);
    g_assert_cmpuint(calc.deadline_ns, ==, 30005000000ULL);
}

static void test_neutral_reconfig_preserves_deadline_with_sub_tick_error(void)
{
    const uint32_t raw_timeout = 600;
    const uint32_t prescale = 40000;
    const uint32_t apb_hz = 80000000;
    const uint32_t scale = 100;
    const uint64_t feed_anchor = 3122490100ULL;
    const uint64_t reanchor_ns = 3426994200ULL;
    const uint64_t tick_ns = esp32_timg_wdt_ticks_to_ns(1, prescale, apb_hz);
    const uint64_t elapsed_ticks = (reanchor_ns - feed_anchor) / tick_ns;

    const uint64_t original_deadline =
        esp32_timg_wdt_compute_stage_deadline_ns(feed_anchor, 0, raw_timeout,
                                                 prescale, apb_hz, scale);
    const uint64_t rearmed_deadline =
        esp32_timg_wdt_compute_stage_deadline_ns(reanchor_ns, elapsed_ticks,
                                                 raw_timeout, prescale,
                                                 apb_hz, scale);
    const uint64_t anticipated_deadline =
        rearmed_deadline < original_deadline ? rearmed_deadline : original_deadline;

    g_assert_cmpuint(abs_delta_u64(rearmed_deadline, original_deadline), <, tick_ns);
    g_assert_cmpuint(anticipated_deadline, ==, original_deadline);
}

static void test_repeated_reanchors_do_not_accumulate_tick_error_on_tick_boundaries(void)
{
    const uint32_t raw_timeout = 1000;
    const uint32_t prescale = 40000;
    const uint32_t apb_hz = 80000000;
    const uint32_t scale = 100;
    const uint64_t tick_ns = esp32_timg_wdt_ticks_to_ns(1, prescale, apb_hz);
    const uint64_t anchor = 1000000000ULL;
    const uint64_t total_elapsed_ticks = 5000;
    const uint64_t original_deadline =
        esp32_timg_wdt_compute_stage_deadline_ns(anchor, 0, raw_timeout,
                                                 prescale, apb_hz, scale);

    const unsigned splits[] = { 1, 2, 10, 100 };
    for (size_t i = 0; i < G_N_ELEMENTS(splits); ++i) {
        uint64_t count_base = 0;
        uint64_t ns_base = anchor;
        const uint64_t ticks_per_split = total_elapsed_ticks / splits[i];
        for (unsigned step = 0; step < splits[i]; ++step) {
            const uint64_t now = ns_base + ticks_per_split * tick_ns;
            count_base += (now - ns_base) / tick_ns;
            ns_base = now;
        }
        g_assert_cmpuint(count_base, ==, total_elapsed_ticks);
        const uint64_t rearmed_deadline =
            esp32_timg_wdt_compute_stage_deadline_ns(ns_base, count_base,
                                                     raw_timeout, prescale,
                                                     apb_hz, scale);
        g_assert_cmpuint(abs_delta_u64(rearmed_deadline, original_deadline),
                         <, tick_ns);
    }
}

static void test_feed_resets_to_full_scaled_budget(void)
{
    Esp32TimgWdtDeadlineCalc calc =
        esp32_timg_wdt_compute_stage_deadline(9000000000ULL, 0, 600, 40000,
                                              80000000, 100);
    g_assert_cmpuint(calc.remaining_ticks, ==, 60000);
    g_assert_cmpuint(calc.deadline_ns, ==, 39000000000ULL);
}

static void test_real_timeout_change_can_expire_immediately(void)
{
    Esp32TimgWdtDeadlineCalc shortened =
        esp32_timg_wdt_compute_stage_deadline(2000000, 700, 6, 40000,
                                              80000000, 100);
    g_assert_cmpuint(shortened.effective_timeout_ticks, ==, 600);
    g_assert_cmpuint(shortened.remaining_ticks, ==, 0);
    g_assert_cmpuint(shortened.deadline_ns, ==, 2000000);

    Esp32TimgWdtDeadlineCalc lengthened =
        esp32_timg_wdt_compute_stage_deadline(2000000, 700, 20, 40000,
                                              80000000, 100);
    g_assert_cmpuint(lengthened.effective_timeout_ticks, ==, 2000);
    g_assert_cmpuint(lengthened.remaining_ticks, ==, 1300);
}

static void test_prescaler_change_preserves_existing_literal_count_semantics(void)
{
    /*
     * E134 deliberately does not redefine real WDTCONFIG1/prescaler-change
     * semantics. The existing path materializes the old-prescaler elapsed
     * count, then arms with the new prescaler. This helper preserves that
     * interpretation: current_literal_count stays literal ticks, while the
     * remaining ticks are converted with the prescaler supplied to arm().
     */
    Esp32TimgWdtDeadlineCalc calc =
        esp32_timg_wdt_compute_stage_deadline(1000000, 100, 600, 80000,
                                              80000000, 100);
    g_assert_cmpuint(calc.remaining_ticks, ==, 59900);
    g_assert_cmpuint(calc.remaining_ns, ==, 59900000000ULL);
}

static void test_stage_transition_uses_same_scaled_policy(void)
{
    for (uint32_t raw_timeout = 1; raw_timeout <= 4; ++raw_timeout) {
        Esp32TimgWdtDeadlineCalc calc =
            esp32_timg_wdt_compute_stage_deadline(0, 0, raw_timeout, 40000,
                                                  80000000, 100);
        g_assert_cmpuint(calc.effective_timeout_ticks, ==, raw_timeout * 100);
        g_assert_cmpuint(calc.remaining_ticks, ==, raw_timeout * 100);
    }
}

static void test_enable_disable_math_edges(void)
{
    Esp32TimgWdtDeadlineCalc enable_from_stage0 =
        esp32_timg_wdt_compute_stage_deadline(1234, 0, 600, 40000,
                                              80000000, 100);
    g_assert_cmpuint(enable_from_stage0.remaining_ticks, ==, 60000);

    Esp32TimgWdtDeadlineCalc disabled_rearm_would_be_now_if_called =
        esp32_timg_wdt_compute_stage_deadline(1234, 60000, 600, 40000,
                                              80000000, 100);
    g_assert_cmpuint(disabled_rearm_would_be_now_if_called.deadline_ns, ==, 1234);
}

static void test_overflow_and_int64_deadline_limit(void)
{
    Esp32TimgWdtDeadlineCalc calc =
        esp32_timg_wdt_compute_stage_deadline(INT64_MAX - 10, 0,
                                              UINT32_MAX, UINT32_MAX,
                                              UINT32_MAX, 100);
    g_assert_cmpuint(calc.deadline_ns, ==, ESP32_TIMG_WDT_TIMER_DEADLINE_MAX_NS);

    const uint64_t zero_timeout =
        esp32_timg_wdt_compute_stage_deadline_ns(777, 0, 0, 0, 80000000, 100);
    g_assert_cmpuint(zero_timeout, ==, 777);

    const uint64_t prescale_zero =
        esp32_timg_wdt_ticks_to_ns(10, 0, 1000000);
    g_assert_cmpuint(prescale_zero, ==, 10000);
}

static void test_e114_pause_compensation_invariant_still_holds(void)
{
    uint64_t ns_base = 1800265600ULL;
    uint64_t delta =
        esp32_timg_transport_pause_compute_delta(1800241700ULL, ns_base,
                                                 1800293900ULL);
    ns_base += delta;
    g_assert_cmpuint(ns_base, ==, 1800293900ULL);
    g_assert_cmpuint(ns_base, <=, 1800293900ULL);
}

static void test_legitimate_later_expiry_boundary(void)
{
    Esp32TimgWdtDeadlineCalc before =
        esp32_timg_wdt_compute_stage_deadline(0, 59999, 600, 40000,
                                              80000000, 100);
    Esp32TimgWdtDeadlineCalc at =
        esp32_timg_wdt_compute_stage_deadline(0, 60000, 600, 40000,
                                              80000000, 100);

    g_assert_cmpuint(before.remaining_ticks, ==, 1);
    g_assert_cmpuint(before.remaining_ns, ==, 500000);
    g_assert_cmpuint(before.deadline_ns, ==, 500000);
    g_assert_cmpuint(at.remaining_ticks, ==, 0);
    g_assert_cmpuint(at.deadline_ns, ==, 0);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/esp32_timg/wdt_scale/e134_exact_reanchor_keeps_scaled_budget",
                    test_e134_exact_reanchor_keeps_scaled_budget);
    g_test_add_func("/esp32_timg/wdt_scale/scale_zero_and_one_are_literal",
                    test_scale_zero_and_one_are_literal);
    g_test_add_func("/esp32_timg/wdt_scale/no_reconfig_matches_scaled_feed_budget",
                    test_no_reconfig_matches_scaled_feed_budget);
    g_test_add_func("/esp32_timg/wdt_scale/neutral_reconfig_preserves_deadline",
                    test_neutral_reconfig_preserves_deadline_with_sub_tick_error);
    g_test_add_func("/esp32_timg/wdt_scale/repeated_reanchors",
                    test_repeated_reanchors_do_not_accumulate_tick_error_on_tick_boundaries);
    g_test_add_func("/esp32_timg/wdt_scale/feed_resets_to_full_scaled_budget",
                    test_feed_resets_to_full_scaled_budget);
    g_test_add_func("/esp32_timg/wdt_scale/real_timeout_change_can_expire_immediately",
                    test_real_timeout_change_can_expire_immediately);
    g_test_add_func("/esp32_timg/wdt_scale/prescaler_change_preserves_existing_literal_count",
                    test_prescaler_change_preserves_existing_literal_count_semantics);
    g_test_add_func("/esp32_timg/wdt_scale/stage_transition_uses_same_policy",
                    test_stage_transition_uses_same_scaled_policy);
    g_test_add_func("/esp32_timg/wdt_scale/enable_disable_edges",
                    test_enable_disable_math_edges);
    g_test_add_func("/esp32_timg/wdt_scale/overflow_and_int64_limit",
                    test_overflow_and_int64_deadline_limit);
    g_test_add_func("/esp32_timg/wdt_scale/e114_pause_compensation_invariant",
                    test_e114_pause_compensation_invariant_still_holds);
    g_test_add_func("/esp32_timg/wdt_scale/legitimate_later_expiry_boundary",
                    test_legitimate_later_expiry_boundary);
    return g_test_run();
}
