#pragma once

#include <stdint.h>

/*
 * E118-AUDIT (LasecSimul orchestrator/.ai/EVIDENCE.md, entry E118-AUDIT, 2026-09-05): pure,
 * dependency-free extraction of the ring-occupancy classification at the heart of E118's
 * full/would-block/fatal contract (softmmu/vnext_b.c's vnext_b_gpio_write()/vnext_b_i2c_submit()/
 * vnext_publish_request()/vnext_write()). Deliberately standalone (no QEMU headers beyond
 * <stdint.h>) so the arithmetic itself is unit-testable (tests/unit/test-vnext-b-classify.c)
 * without needing a real, live VNEXT_B shared-memory ring -- E118 explicitly declined to write a
 * test that deliberately corrupts a real ring's write_seq to exercise the FATAL branch, judging
 * that riskier than its value over reviewing the (small, direct) branch itself. This header closes
 * that gap for the classification math specifically, without touching the live-ring risk at all.
 *
 * The contract (see EVIDENCE.md E118/E118-AUDIT for the full account of why occupancy == depth
 * must never be fatal):
 *   occupancy <  depth : room to publish now.
 *   occupancy == depth : ring genuinely full -- an ordinary wait condition, never fatal.
 *   occupancy >  depth : a real ring-invariant violation (corruption) -- should be unreachable
 *                        once every producer honors WOULD_BLOCK correctly; session-fatal.
 *
 * write_seq/read_seq are the ring's monotonically increasing 64-bit sequence counters
 * (lasec_at_ring_header on the Core side, VnextLane's meta[0]/meta[1] on the QEMU side) -- never
 * expected to wrap within a session's lifetime, and read_seq is never expected to exceed
 * write_seq. This function does not assume either: `write_seq - read_seq` is unsigned arithmetic,
 * so read_seq > write_seq underflows to a huge value, which correctly classifies as FATAL (a
 * genuine invariant violation, not a crash) rather than silently misclassifying it as
 * PUBLISHED_ALLOWED.
 */
typedef enum {
    VNEXT_RING_PUBLISHED_ALLOWED = 0,
    VNEXT_RING_WOULD_BLOCK = 1,
    VNEXT_RING_FATAL = 2,
} VnextRingClassification;

static inline VnextRingClassification vnext_ring_classify(uint64_t write_seq, uint64_t read_seq,
                                                            uint32_t depth)
{
    const uint64_t occupancy = write_seq - read_seq;
    if (occupancy > depth) {
        return VNEXT_RING_FATAL;
    }
    if (occupancy == depth) {
        return VNEXT_RING_WOULD_BLOCK;
    }
    return VNEXT_RING_PUBLISHED_ALLOWED;
}
