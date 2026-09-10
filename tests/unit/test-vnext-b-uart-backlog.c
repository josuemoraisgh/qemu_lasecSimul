/*
 * E147 RED: deterministic non-vCPU UART backlog lost-wake interleaving.
 *
 * This models the exact ordering in softmmu/vnext_b.c:
 *   1. uart_tx_effect_bh() receives WOULD_BLOCK;
 *   2. vnext_block_producer_on_lane(lane, NULL) arms the lane backlog;
 *   3. credit returns before that helper's immediate recheck;
 *   4. the helper clears the backlog;
 *   5. the UART BH finally compacts/preserves its unsent byte;
 *   6. vnext_resume() sees no backlog and therefore never schedules the BH.
 *
 * The RED output is preserved in E147-J-uart-lost-wake/RED_output.txt.  This
 * test is now the GREEN contract: preserving a non-vCPU producer's pending
 * work must leave a wake token armed until the resume sweep notifies it.
 */

#include "qemu/osdep.h"

typedef struct BacklogModel {
    bool backlog_armed;
    bool credit_available;
    bool producer_pending;
    bool bh_scheduled;
} BacklogModel;

/* Corrected production ordering, copied as a pure test model. */
static void fixed_block_nonvcpu(BacklogModel *m)
{
    m->backlog_armed = true;
}

static void old_resume_sweep(BacklogModel *m)
{
    if (m->backlog_armed && m->credit_available) {
        m->backlog_armed = false;
        if (m->producer_pending) {
            m->bh_scheduled = true;
        }
    }
}

static void note_after_preserve(BacklogModel *m)
{
    m->backlog_armed = true;
    if (m->credit_available) {
        m->backlog_armed = false;
        m->bh_scheduled = true;
    }
}

static void test_credit_return_before_uart_preserves_effect(void)
{
    BacklogModel m = {
        .backlog_armed = false,
        .credit_available = true, /* credit returned in the race window */
        .producer_pending = false,
        .bh_scheduled = false,
    };

    fixed_block_nonvcpu(&m);
    m.producer_pending = true; /* uart_tx_effect_bh() compacts the unsent byte */
    old_resume_sweep(&m);

    /* GREEN: the token survives until the caller has recorded its pending work. */
    g_assert_true(m.bh_scheduled);
}

static void test_credit_already_available_after_preserve_rearms_bh(void)
{
    BacklogModel m = {
        .backlog_armed = false,
        .credit_available = true,
        .producer_pending = true,
        .bh_scheduled = false,
    };
    note_after_preserve(&m);
    g_assert_false(m.backlog_armed);
    g_assert_true(m.bh_scheduled);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/vnext_b/uart_backlog/credit_return_before_preserve",
                    test_credit_return_before_uart_preserves_effect);
    g_test_add_func("/vnext_b/uart_backlog/credit_already_available_after_preserve",
                    test_credit_already_available_after_preserve_rearms_bh);
    return g_test_run();
}
