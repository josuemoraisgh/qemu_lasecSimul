/*
 * E118-AUDIT-2/B11 (EVIDENCE.md, 2026-09-05): diagnostic-only, opt-in concurrency probe built for
 * the DECISION-014 Phase 1 audit. Measures, across ALL device MemoryRegions (guarded and
 * unguarded alike -- unlike the reentrancy guard itself, this does not consult
 * mr->disable_reentrancy_guard), the maximum number of threads ever simultaneously inside a
 * device's .read/.write dispatch (softmmu/memory.c's access_with_adjusted_size()). This is a
 * GLOBAL bound, not per-device: it is strictly stronger than "per-DeviceState" concurrency (two
 * different devices dispatching at once would already violate it), which is exactly the property
 * VNEXT_B's BQL-held-throughout retry path is expected to guarantee. A result of 1 proves the
 * weaker per-device property as a corollary.
 *
 * Gated by LASECSIMUL_REENTRANCY_CONCURRENCY_PROBE (off by default, matching every other
 * diagnostic gate in this fork). Zero cost when disabled beyond one g_once_init-cached bool check
 * per dispatch.
 *
 * Pairing across cpu_loop_exit_restore(): that siglongjmp skips access_with_adjusted_size()'s own
 * normal-return cleanup (the same mechanism DECISION-014 itself is about), so any code path that
 * calls cpu_loop_exit_restore() from within a device's own dispatch MUST call
 * reentrancy_probe_exit() immediately before the siglongjmp, or this probe would leak an entry per
 * retry and falsely report concurrency that never happened. As of this writing there are exactly
 * two such sites for the 7 DECISION-014 devices: softmmu/vnext_b.c's vnext_b_gpio_write() (ring
 * WOULD_BLOCK) and hw/char/esp32_uart.c's uart_write() (TX backlog full) -- both call
 * reentrancy_probe_exit() right before cpu_loop_exit_restore(). Any future retry site added to one
 * of these devices must do the same.
 */
#ifndef LASECSIMUL_REENTRANCY_PROBE_H
#define LASECSIMUL_REENTRANCY_PROBE_H

#include "qemu/osdep.h"

void reentrancy_probe_enter(void);
void reentrancy_probe_exit(void);
uint32_t reentrancy_probe_get_max(void);

/* B11 diagnostic (2026-09-05, EVIDENCE.md): temporary, opt-in "who set this guard" tracker used
 * to root-cause the misc.esp32.dport stuck-guard finding. Only called from softmmu/memory.c
 * itself; `struct MemoryRegion *` (rather than the `MemoryRegion` typedef) keeps this declaration
 * compatible without requiring exec/memory.h here -- typedef'd and untyped struct pointers to the
 * same tag are the same type in C. */
void reentrancy_probe_last_entry_record(struct MemoryRegion *mr, hwaddr addr);
void reentrancy_probe_last_entry_dump(struct MemoryRegion *mr);

#endif
