#pragma once

#include <stdint.h>

/*
 * E114 (LasecSimul orchestrator/.ai/EVIDENCE.md, 2026-09-04): pure, dependency-free extraction of
 * the transport-pause compensation delta -- the exact formula whose bug caused the
 * SW_CPU_RESET_REGISTER storm, and whose fix closed it. Deliberately standalone (no QEMU headers
 * beyond <stdint.h>) so it is unit-testable (tests/unit/test-esp32-timg-pause.c) without pulling
 * in the device model. hw/timer/esp32_timg.c's esp32_timg_transport_pause_apply() calls this
 * directly -- there is no parallel/duplicated copy of this arithmetic anywhere else.
 *
 * The bug: esp32_timg_wdt_feed() resets ws->ns_base to its own timestamp unconditionally, with no
 * knowledge of an open transport pause. If a feed lands between the pause opening
 * (pause_start_ns) and this compensation running (now_ns), naively adding the *entire*
 * (now_ns - pause_start_ns) duration on top of that already-advanced ns_base double-counts the
 * portion of the pause before the feed (the feed already "forgave" everything up to its own
 * timestamp) and pushes ns_base past now_ns itself -- the next esp32_timg_wdt_get_count() call
 * then underflows (unsigned ns_now - ns_base), computing a near-UINT64_MAX count and collapsing
 * the watchdog's next timeout to ~0ns.
 *
 * The fix: compensate only the portion of [pause_start_ns, now_ns) not already covered by
 * whichever of pause_start_ns/current_ns_base is later -- this never returns a delta that would
 * push ns_base past now_ns, given the precondition current_ns_base <= now_ns (which every caller
 * in this fork maintains as a system-wide invariant after this fix; the caller must also ensure
 * now_ns > pause_start_ns before calling -- esp32_timg_transport_pause_apply() already early-
 * returns otherwise).
 */
static inline uint64_t esp32_timg_transport_pause_compute_delta(uint64_t pause_start_ns,
                                                                  uint64_t current_ns_base,
                                                                  uint64_t now_ns)
{
    uint64_t effective_start = pause_start_ns > current_ns_base ? pause_start_ns : current_ns_base;
    return now_ns > effective_start ? now_ns - effective_start : 0;
}
