/*
 * Unit test for vnext_ring_classify() -- the E118-AUDIT extraction of the full/would-block/fatal
 * ring-occupancy contract (see the LasecSimul orchestrator's orchestrator/.ai/EVIDENCE.md, entries
 * E118 and E118-AUDIT). This is the pure calculation extracted from softmmu/vnext_b.c's producer
 * functions; it has no QEMU device-model or shared-memory dependencies, so it is tested here
 * directly, without needing a real, live VNEXT_B ring.
 *
 * This work is licensed under the terms of the GNU LGPL, version 2 or later.
 * See the COPYING.LIB file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "softmmu/vnext_b_classify.h"

static void test_empty_ring(void)
{
    /* write == read: nothing published yet, always room (any depth >= 1). */
    g_assert_cmpint(vnext_ring_classify(0, 0, 8), ==, VNEXT_RING_PUBLISHED_ALLOWED);
    g_assert_cmpint(vnext_ring_classify(100, 100, 8), ==, VNEXT_RING_PUBLISHED_ALLOWED);
}

static void test_depth_minus_one(void)
{
    /* occupancy == depth - 1: exactly one credit remains -- still allowed to publish. */
    g_assert_cmpint(vnext_ring_classify(7, 0, 8), ==, VNEXT_RING_PUBLISHED_ALLOWED);
    g_assert_cmpint(vnext_ring_classify(107, 100, 8), ==, VNEXT_RING_PUBLISHED_ALLOWED);
}

static void test_depth_exact(void)
{
    /* occupancy == depth: genuinely full. This is the entire point of E118 -- must be
     * WOULD_BLOCK, never FATAL, at every depth. */
    g_assert_cmpint(vnext_ring_classify(8, 0, 8), ==, VNEXT_RING_WOULD_BLOCK);
    g_assert_cmpint(vnext_ring_classify(108, 100, 8), ==, VNEXT_RING_WOULD_BLOCK);
    g_assert_cmpint(vnext_ring_classify(2, 0, 2), ==, VNEXT_RING_WOULD_BLOCK);
}

static void test_depth_plus_one(void)
{
    /* occupancy == depth + 1: one past capacity -- a real invariant violation, FATAL. */
    g_assert_cmpint(vnext_ring_classify(9, 0, 8), ==, VNEXT_RING_FATAL);
    g_assert_cmpint(vnext_ring_classify(109, 100, 8), ==, VNEXT_RING_FATAL);
}

static void test_wraparound_representative(void)
{
    /* write_seq/read_seq are monotonic uint64_t counters that in practice never wrap within a
     * session's lifetime, but the classification must still be well-defined near the boundary:
     * only the DIFFERENCE matters, not the absolute magnitude. A representative "large but not
     * yet wrapped" pair, far from either endpoint's own overflow, must classify identically to
     * the small-number cases above. */
    const uint64_t big = (UINT64_C(1) << 40);
    g_assert_cmpint(vnext_ring_classify(big + 7, big, 8), ==, VNEXT_RING_PUBLISHED_ALLOWED);
    g_assert_cmpint(vnext_ring_classify(big + 8, big, 8), ==, VNEXT_RING_WOULD_BLOCK);
    g_assert_cmpint(vnext_ring_classify(big + 9, big, 8), ==, VNEXT_RING_FATAL);
}

static void test_large_values(void)
{
    /* Largest configured lane depth (VNEXT_DEPTH in softmmu/vnext_b.c) and a write_seq far into a
     * long-running session's lifetime. */
    const uint32_t maxDepth = 1024;
    const uint64_t farAlongWrite = UINT64_C(1000000000);
    g_assert_cmpint(vnext_ring_classify(farAlongWrite, farAlongWrite - (maxDepth - 1), maxDepth),
                     ==, VNEXT_RING_PUBLISHED_ALLOWED);
    g_assert_cmpint(vnext_ring_classify(farAlongWrite, farAlongWrite - maxDepth, maxDepth),
                     ==, VNEXT_RING_WOULD_BLOCK);
    g_assert_cmpint(vnext_ring_classify(farAlongWrite, farAlongWrite - maxDepth - 1, maxDepth),
                     ==, VNEXT_RING_FATAL);
}

static void test_read_exceeds_write_is_fatal_not_misclassified(void)
{
    /* read_seq > write_seq should never happen (a consumer cannot consume something not yet
     * published) -- if it ever does, the unsigned subtraction underflows to a huge value, which
     * must classify as FATAL (a genuine, detected invariant violation), never silently as
     * PUBLISHED_ALLOWED or WOULD_BLOCK. */
    g_assert_cmpint(vnext_ring_classify(0, 1, 8), ==, VNEXT_RING_FATAL);
    g_assert_cmpint(vnext_ring_classify(100, 105, 8), ==, VNEXT_RING_FATAL);
}

static void test_depth_zero_boundary(void)
{
    /* depth == 0 is not a configuration the real system ever validates into use (vnext_valid()
     * rejects any lane depth < 2) -- documented here only to show the pure function still behaves
     * predictably (never crashes, never misclassifies) at this out-of-domain boundary, not that
     * it is a supported production input. */
    g_assert_cmpint(vnext_ring_classify(0, 0, 0), ==, VNEXT_RING_WOULD_BLOCK);
    g_assert_cmpint(vnext_ring_classify(1, 0, 0), ==, VNEXT_RING_FATAL);
}

static void test_depth_one_boundary(void)
{
    /* Smallest depth actually reachable via LASECSIMUL_VNEXT_B_LANE_DEPTH validation in practice
     * (depth >= 2 is enforced in vnext_valid(), but the pure function itself imposes no such
     * floor) -- depth=1 is a single-slot ring, exercised here for completeness. */
    g_assert_cmpint(vnext_ring_classify(0, 0, 1), ==, VNEXT_RING_PUBLISHED_ALLOWED);
    g_assert_cmpint(vnext_ring_classify(1, 0, 1), ==, VNEXT_RING_WOULD_BLOCK);
    g_assert_cmpint(vnext_ring_classify(2, 0, 1), ==, VNEXT_RING_FATAL);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/vnext_b/classify/empty_ring", test_empty_ring);
    g_test_add_func("/vnext_b/classify/depth_minus_one", test_depth_minus_one);
    g_test_add_func("/vnext_b/classify/depth_exact", test_depth_exact);
    g_test_add_func("/vnext_b/classify/depth_plus_one", test_depth_plus_one);
    g_test_add_func("/vnext_b/classify/wraparound_representative", test_wraparound_representative);
    g_test_add_func("/vnext_b/classify/large_values", test_large_values);
    g_test_add_func("/vnext_b/classify/read_exceeds_write_is_fatal_not_misclassified",
                     test_read_exceeds_write_is_fatal_not_misclassified);
    g_test_add_func("/vnext_b/classify/depth_zero_boundary", test_depth_zero_boundary);
    g_test_add_func("/vnext_b/classify/depth_one_boundary", test_depth_one_boundary);
    return g_test_run();
}
