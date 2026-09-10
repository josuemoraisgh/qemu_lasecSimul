#pragma once

#include <limits.h>
#include <stdint.h>

#define ESP32_TIMG_WDT_TIMER_DEADLINE_MAX_NS ((uint64_t)INT64_MAX)

typedef struct Esp32TimgWdtDeadlineCalc {
    uint64_t effective_timeout_ticks;
    uint64_t remaining_ticks;
    uint64_t remaining_ns;
    uint64_t deadline_ns;
} Esp32TimgWdtDeadlineCalc;

static inline uint32_t esp32_timg_wdt_nonzero_prescale(uint32_t prescale)
{
    return prescale ? prescale : 1;
}

static inline uint64_t esp32_timg_wdt_saturating_add_u64(uint64_t a, uint64_t b)
{
    return UINT64_MAX - a < b ? UINT64_MAX : a + b;
}

static inline uint64_t esp32_timg_wdt_saturating_mul_u64(uint64_t a, uint64_t b)
{
#if defined(__SIZEOF_INT128__)
    __uint128_t product = (__uint128_t)a * (__uint128_t)b;
    return product > UINT64_MAX ? UINT64_MAX : (uint64_t)product;
#else
    return a != 0 && b > UINT64_MAX / a ? UINT64_MAX : a * b;
#endif
}

static inline uint64_t esp32_timg_wdt_clamp_timer_deadline_ns(uint64_t value)
{
    return value > ESP32_TIMG_WDT_TIMER_DEADLINE_MAX_NS
        ? ESP32_TIMG_WDT_TIMER_DEADLINE_MAX_NS : value;
}

static inline uint64_t esp32_timg_wdt_ticks_to_ns(uint64_t ticks,
                                                  uint32_t prescale,
                                                  uint32_t apb_freq_hz)
{
    uint64_t apb_mhz = apb_freq_hz / 1000000u;
    if (apb_mhz == 0) {
        return UINT64_MAX;
    }
#if defined(__SIZEOF_INT128__)
    __uint128_t numerator =
        (__uint128_t)ticks * 1000u * esp32_timg_wdt_nonzero_prescale(prescale);
    __uint128_t quotient = numerator / apb_mhz;
    return quotient > UINT64_MAX ? UINT64_MAX : (uint64_t)quotient;
#else
    uint64_t factor = esp32_timg_wdt_saturating_mul_u64(1000u,
        esp32_timg_wdt_nonzero_prescale(prescale));
    uint64_t numerator = esp32_timg_wdt_saturating_mul_u64(ticks, factor);
    return numerator / apb_mhz;
#endif
}

static inline uint64_t esp32_timg_wdt_effective_stage_timeout_ticks(uint32_t raw_stage_timeout,
                                                                    uint32_t wdt_time_scale)
{
    uint32_t scale = wdt_time_scale ? wdt_time_scale : 1;
    return esp32_timg_wdt_saturating_mul_u64(raw_stage_timeout, scale);
}

static inline uint64_t esp32_timg_wdt_remaining_ticks(uint64_t effective_stage_timeout,
                                                      uint64_t current_literal_count)
{
    return effective_stage_timeout > current_literal_count
        ? effective_stage_timeout - current_literal_count : 0;
}

static inline Esp32TimgWdtDeadlineCalc esp32_timg_wdt_compute_stage_deadline(uint64_t ns_now,
                                                                             uint64_t current_literal_count,
                                                                             uint32_t raw_stage_timeout,
                                                                             uint32_t prescale,
                                                                             uint32_t apb_freq_hz,
                                                                             uint32_t wdt_time_scale)
{
    Esp32TimgWdtDeadlineCalc calc;
    calc.effective_timeout_ticks =
        esp32_timg_wdt_effective_stage_timeout_ticks(raw_stage_timeout,
                                                     wdt_time_scale);
    calc.remaining_ticks =
        esp32_timg_wdt_remaining_ticks(calc.effective_timeout_ticks,
                                       current_literal_count);
    calc.remaining_ns =
        esp32_timg_wdt_ticks_to_ns(calc.remaining_ticks, prescale,
                                   apb_freq_hz);
    calc.deadline_ns = esp32_timg_wdt_clamp_timer_deadline_ns(
        esp32_timg_wdt_saturating_add_u64(ns_now, calc.remaining_ns));
    return calc;
}

static inline uint64_t esp32_timg_wdt_compute_stage_deadline_ns(uint64_t ns_now,
                                                                uint64_t current_literal_count,
                                                                uint32_t raw_stage_timeout,
                                                                uint32_t prescale,
                                                                uint32_t apb_freq_hz,
                                                                uint32_t wdt_time_scale)
{
    Esp32TimgWdtDeadlineCalc calc =
        esp32_timg_wdt_compute_stage_deadline(ns_now, current_literal_count,
                                              raw_stage_timeout, prescale,
                                              apb_freq_hz, wdt_time_scale);
    return calc.deadline_ns;
}

static inline uint64_t esp32_timg_wdt_compute_stage_deadline_ns_legacy_prefixed(uint64_t ns_now,
                                                                                uint64_t current_literal_count,
                                                                                uint32_t raw_stage_timeout,
                                                                                uint32_t prescale,
                                                                                uint32_t apb_freq_hz,
                                                                                uint32_t wdt_time_scale)
{
    uint64_t count_to_timeout = raw_stage_timeout > current_literal_count
        ? raw_stage_timeout - current_literal_count : 0;
    uint64_t ns_to_timeout =
        esp32_timg_wdt_ticks_to_ns(count_to_timeout, prescale, apb_freq_hz);
    if (wdt_time_scale > 1) {
        ns_to_timeout =
            esp32_timg_wdt_saturating_mul_u64(ns_to_timeout, wdt_time_scale);
    }
    return esp32_timg_wdt_clamp_timer_deadline_ns(
        esp32_timg_wdt_saturating_add_u64(ns_now, ns_to_timeout));
}
