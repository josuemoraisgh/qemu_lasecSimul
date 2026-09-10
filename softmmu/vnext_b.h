#pragma once

/* E118 (EVIDENCE.md, 2026-09-05): explicit publish outcome for every VNEXT_B ring producer.
 * Replaces the old binary "published or fatal" shape, which conflated an ordinary full ring
 * (occupancy == depth, meant to be a wait condition) with a genuine invariant violation
 * (occupancy > depth, meant to be unrecoverable) -- see EVIDENCE.md E118 for the full account of
 * why that conflation caused a real INVALID_BOOT.
 *
 *   VNEXT_PUBLISHED    -- accepted into the ring; write_seq advanced.
 *   VNEXT_WOULD_BLOCK  -- ring genuinely full (occupancy == depth). Not fatal. A real-vCPU caller
 *                         (current_cpu != NULL) never observes this value: vnext_b_gpio_write()
 *                         blocks that vCPU and replays the whole guest instruction once credit
 *                         returns (see cpu_loop_exit_restore() in its implementation), so from a
 *                         guest MMIO write handler's point of view the call simply does not return
 *                         until it can report PUBLISHED. A non-vCPU caller (current_cpu == NULL --
 *                         a bottom half or timer callback, e.g. ESP32 UART's TX BH) DOES observe
 *                         this value and must keep its own unsent payload and retry later; it must
 *                         never busy-loop, never stop any CPU, and never treat this as an error.
 *   VNEXT_FATAL        -- occupancy > depth: a real ring-invariant violation (should be
 *                         unreachable once every producer honors WOULD_BLOCK correctly). Session-
 *                         terminating, exactly as before.
 */
typedef enum {
    VNEXT_PUBLISHED = 0,
    VNEXT_WOULD_BLOCK = 1,
    VNEXT_FATAL = 2,
} VnextPublishResult;

int vnext_b_main(int argc, char **argv);
bool vnext_b_active(void);
VnextPublishResult vnext_b_gpio_write(uint64_t address, uint64_t value);
uint64_t vnext_b_gpio_read(uint64_t address);
uint64_t vnext_b_register_read(uint64_t address);
bool vnext_b_i2c_submit(uint32_t bus, uint32_t flags, uint64_t period_ns,
                        const uint8_t *tx, uint32_t tx_len, uint32_t rx_len);
uint64_t vnext_b_diag_lane0_exhaustion_count(void);
uint64_t vnext_b_diag_lane0_last_exhaustion_virtual_ns(void);
/* True once lane 0 has credit again after a non-vCPU producer observed VNEXT_WOULD_BLOCK there
 * (LASECSIMUL_VNEXT_B_LANE_DEPTH-friendly: works at any configured depth). A backlogged producer
 * (e.g. ESP32 UART's TX BH) may poll this from its own retry entry point in addition to being
 * notified via esp32_uart_vnext_credit_available(); it is cheap (one load, no lock). */
bool vnext_b_lane_has_credit(uint32_t lane);
void vnext_b_note_nonvcpu_backlog(uint32_t lane);

/* E129 (EVIDENCE.md, 2026-09-05): hw/misc/esp32_dport.c's esp32_cache_ill_read() needs
 * cpu_stop_current()/cpu_loop_exit_restore() (the same VNEXT_B backpressure-replay idiom this file
 * already uses) to suspend only the specific load/fetch that lands on an externally-disabled
 * DROM0/IRAM0 region, without stalling the rest of APP CPU's execution (see EVIDENCE.md E128 for
 * the deadlock this replaces). esp32_dport.c is compiled into libcommon.fa (target-independent),
 * which cannot include exec/exec-all.h (pulls in the target's own cpu.h) -- this file is compiled
 * per-target already (softmmu/meson.build) and provides every other cpu_loop_exit_restore() call
 * site in this fork for exactly that reason. Never returns -- the caller's own state (mask bits,
 * diagnostics) must already be fully updated before calling this. (Not annotated G_NORETURN here:
 * that macro's own fallback definition, glib-compat.h, is not reliably in scope at every one of
 * this header's inclusion points -- confirmed by a failed build attempt via softmmu/simuliface.c.
 * The definition in vnext_b.c carries the annotation instead, where exec/exec-all.h -- already
 * required there for cpu_loop_exit_restore() itself -- guarantees it.) */
void vnext_cache_wait_suspend_current_cpu(void);
