/*
 * ESP32 UART emulation
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 *
 * The QEMU model of nRF51 UART by Julia Suvorova was used as a template.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "sysemu/sysemu.h"
#include "sysemu/cpus.h"
#include "hw/core/cpu.h"
#include "chardev/char-fe.h"
#include "hw/registerfields.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "hw/char/esp32_uart.h"

/* E118-AUDIT (EVIDENCE.md, 2026-09-05): same ad-hoc extern declaration vnext_b.c already uses --
 * no header exposes this without pulling in exec/exec-all.h's much larger dependency surface into
 * common device code. */
G_NORETURN void cpu_loop_exit_restore(CPUState *cpu, uintptr_t pc);

static ESP32UARTState *vnext_uart_instances[3];
static unsigned vnext_uart_instance_count;

/* E147-G (EVIDENCE.md, 2026-09-10): bounded, opt-in (LASECSIMUL_SETTLE_PROVENANCE=1 -- same gate
 * vnext_b.c's own provenance counters use; a plain local getenv() check here rather than a
 * cross-file dependency, to keep this device model's own instrumentation self-contained). Proves
 * or refutes H147-G's lost-wake hypothesis directly on the real UART config-publish path:
 * CLKDIV/CONF0/byte publish attempts and outcomes, and the FINAL state of tx_effect_count/
 * pending_config at process exit -- a non-zero pending count at exit is the smoking gun (work
 * that was queued but never delivered because nothing ever rescheduled the BH again). */
static uint64_t vnext_uart_clkdiv_attempts, vnext_uart_clkdiv_published;
static uint64_t vnext_uart_conf0_attempts, vnext_uart_conf0_published;
static uint64_t vnext_uart_byte_attempts, vnext_uart_byte_published;
static uint64_t vnext_uart_would_block_total;
static uint64_t vnext_uart_config_would_block;
static uint64_t vnext_uart_byte_would_block;
static uint64_t vnext_uart_bh_entries;
static uint64_t vnext_uart_credit_available_calls;
static uint64_t vnext_uart_backlog_full_stops;
static uint64_t vnext_uart_backlog_wakes;
static bool vnext_uart_provenance_checked, vnext_uart_provenance_enabled_cached;

static bool vnext_uart_is_power_of_two(uint64_t value) {
    return value != 0 && (value & (value - 1)) == 0;
}

static void vnext_uart_provenance_print_summary(void);

static bool vnext_uart_provenance_enabled(void) {
    if (!vnext_uart_provenance_checked) {
        vnext_uart_provenance_checked = true;
        const char *v = getenv("LASECSIMUL_SETTLE_PROVENANCE");
        vnext_uart_provenance_enabled_cached = v && *v && v[0] != '0';
        if (vnext_uart_provenance_enabled_cached) atexit(vnext_uart_provenance_print_summary);
    }
    return vnext_uart_provenance_enabled_cached;
}

static void vnext_uart_provenance_print_summary(void) {
    fprintf(stderr, "[SETTLE_PROVENANCE_UART] clkdivAttempts=%llu clkdivPublished=%llu "
            "conf0Attempts=%llu conf0Published=%llu byteAttempts=%llu bytePublished=%llu "
            "wouldBlockTotal=%llu configWouldBlock=%llu byteWouldBlock=%llu "
            "bhEntries=%llu creditAvailableCalls=%llu backlogFullStops=%llu backlogWakes=%llu\n",
            (unsigned long long)vnext_uart_clkdiv_attempts, (unsigned long long)vnext_uart_clkdiv_published,
            (unsigned long long)vnext_uart_conf0_attempts, (unsigned long long)vnext_uart_conf0_published,
            (unsigned long long)vnext_uart_byte_attempts, (unsigned long long)vnext_uart_byte_published,
            (unsigned long long)vnext_uart_would_block_total,
            (unsigned long long)vnext_uart_config_would_block,
            (unsigned long long)vnext_uart_byte_would_block,
            (unsigned long long)vnext_uart_bh_entries,
            (unsigned long long)vnext_uart_credit_available_calls,
            (unsigned long long)vnext_uart_backlog_full_stops,
            (unsigned long long)vnext_uart_backlog_wakes);
    for (unsigned i = 0; i < vnext_uart_instance_count; ++i) {
        ESP32UARTState *s = vnext_uart_instances[i];
        if (!s) continue;
        fprintf(stderr, "[SETTLE_PROVENANCE_UART] instance=%u tx_effect_count(final)=%u "
                "pending.clkdivDirty=%d pending.conf0StateDirty=%d\n",
                i, s->tx_effect_count, (int)s->pending_config.clkdivDirty,
                (int)s->pending_config.conf0StateDirty);
    }
}
#include "hw/xtensa/esp32_clk.h"
#include "qemu/main-loop.h"
#include "trace.h"

#include "../softmmu/simuliface.h"
#include "../softmmu/vnext_b.h"


//static gboolean uart_transmit(void *do_not_use, GIOCondition cond, void *opaque);
void uart_receive(void *opaque, const uint8_t *buf, int size);
int uart_can_receive(void *opaque);


void esp32_uart_update_irq(ESP32UARTState *s)
{
    bool irq = false;

    uint32_t tx_empty_raw = (fifo8_num_used(&s->tx_fifo) <= s->tx_empty_threshold);
    uint32_t rx_full_raw = (fifo8_num_used(&s->rx_fifo) >= s->rx_full_threshold);
    uint32_t tx_done_raw = (fifo8_num_used(&s->tx_fifo) != 0);
    uint32_t rxfifo_tout_raw = (s->rxfifo_tout) ? 1 : 0;

    uint32_t int_raw = s->reg[R_UART_INT_RAW];
    int_raw = FIELD_DP32(int_raw, UART_INT_RAW, RXFIFO_FULL, rx_full_raw);
    int_raw = FIELD_DP32(int_raw, UART_INT_RAW, TXFIFO_EMPTY, tx_empty_raw);
    int_raw = FIELD_DP32(int_raw, UART_INT_RAW, TX_DONE, tx_done_raw);
    int_raw = FIELD_DP32(int_raw, UART_INT_RAW, RXFIFO_TOUT, rxfifo_tout_raw);
    s->reg[R_UART_INT_RAW] = int_raw;

    uint32_t int_st = s->reg[R_UART_INT_RAW] & s->reg[R_UART_INT_ENA];
    irq = int_st != 0;
    s->reg[R_UART_INT_ST] = int_st;

    qemu_set_irq(s->irq, irq);
}


void esp32_uart_set_rx_timeout(ESP32UARTState *s)
{
    if (s->rx_tout_ena) {
        int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        int64_t rx_timeout_ns = now + s->rx_tout_thres * NANOSECONDS_PER_SECOND / s->baud_rate;
        /* If throttling is done, make sure timeout doesn't happen before more data
         * is allowed to come. Offset it by 1ms.
         */
        if (rx_timeout_ns <= s->throttle_timer.expire_time) {
            rx_timeout_ns = s->throttle_timer.expire_time + 10000000;
        }
        timer_mod_ns(&s->rx_timeout_timer, rx_timeout_ns);
    } else {
        timer_del(&s->rx_timeout_timer);
        s->rxfifo_tout = false;
    }
}


/* [FIX] TG0WDT_SYS_RESET investigation (2026-08-27/28) -- writeReg() used to be a Core-backed
 * synchronous round-trip called from here. Before this fix, the FIRST byte of a fresh TX burst
 * called this function synchronously from within uart_write()'s A_UART_FIFO case -- i.e. from
 * inside an active esp_soc.uart MemoryRegion dispatch. arenaTransactionBegin()'s BQL-release
 * window (needed to acquire m_arenaOrderLock without violating the arena->BQL lock order, see
 * softmmu/simuliface.c) then let the OTHER vCPU dispatch into the SAME MemoryRegion (observed: a
 * UART_STATUS poll) and get rejected -- "Blocked re-entrant IO", a genuine dropped guest register
 * access, source-confirmed to freeze the guest's virtual-time progress afterward.
 *
 * (2026-08-28) Core notification moved out of this function entirely, into tx_effect_bh (see
 * below), which drains it back-to-back and independently of frame_time_ns -- this function is now
 * LOCAL pacing only (tx_in_flight/tx_timer govern guest-visible FIFO occupancy/UART_STATUS/IRQ,
 * nothing Core-facing). */
static void uart_send_next(void* opaque)
{
    ESP32UARTState *s = ESP32_UART(opaque);
    s->tx_in_flight = true;
    timer_mod_ns( &s->tx_timer, getQemu_ns()+s->frame_time_ns);
}

static void uart_config_summary_clear(UartConfigSummary *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
}

/* Sends, at most, one Core notification per dirty register (CLKDIV, CONF0) for one config
 * summary snapshot -- never more than that regardless of how many guest writes were coalesced
 * into it. TXFIFO_RST/RXFIFO_RST are OR-accumulated separately from the raw CONF0 value (see
 * uart_write()'s A_UART_CONF0 case) and re-applied onto whatever CONF0 value is sent, so a clear
 * pulse from an earlier write in the same segment is never lost even if a later write in the same
 * segment doesn't have that bit set.
 *
 * The caller supplies a detached snapshot (2026-08-28 fix, see uart_tx_effect_bh()): writeReg()
 * internally releases the BQL for a bounded window (arenaTransactionBegin() dropping it to
 * acquire the arena order lock, softmmu/simuliface.c), during which a concurrent uart_write() on
 * the other vCPU can run to completion. A caller that read live pending fields, called writeReg(),
 * and only cleared them afterward could have a concurrently-merged contribution wiped out by that
 * trailing clear without ever sending it. Operating on a value copy (already fully detached from
 * shared state by the time any writeReg() here can release the BQL) closes that window -- see each
 * call site for how the copy/clear ordering is arranged. */
/* Preserve partial progress across credit stalls. Replaying accepted configuration
 * can consume every newly returned slot and starve the pending byte indefinitely;
 * replaying CONF0 reset pulses can also erase bytes already accepted by Core.
 * Clear each pending operation only after its publish succeeds. */
static VnextPublishResult uart_apply_config_summary(ESP32UARTState *s, UartConfigSummary *remaining)
{
    UartConfigSummary cfg = *remaining;
    const bool trace = vnext_uart_provenance_enabled();
    if (cfg.clkdivDirty) {
        if (trace) vnext_uart_clkdiv_attempts++;
        const VnextPublishResult r = writeReg( s->iomem.addr+A_UART_CLKDIV, cfg.clkdivValue );
        if (r != VNEXT_PUBLISHED) {
            if (trace && r == VNEXT_WOULD_BLOCK) { vnext_uart_would_block_total++; vnext_uart_config_would_block++; }
            return r;
        }
        if (trace) vnext_uart_clkdiv_published++;
        remaining->clkdivDirty = false;
    }
    if (cfg.conf0StateDirty || cfg.txFifoReset || cfg.rxFifoReset) {
        uint32_t value = cfg.conf0StateValue;
        if (cfg.txFifoReset) value |= (1u << 18);
        if (cfg.rxFifoReset) value |= (1u << 17);
        if (trace) vnext_uart_conf0_attempts++;
        const VnextPublishResult r = writeReg( s->iomem.addr+A_UART_CONF0, value );
        if (r != VNEXT_PUBLISHED) {
            if (trace && r == VNEXT_WOULD_BLOCK) { vnext_uart_would_block_total++; vnext_uart_config_would_block++; }
            return r;
        }
        if (trace) vnext_uart_conf0_published++;
        remaining->conf0StateDirty = false;
        remaining->txFifoReset = false;
        remaining->rxFifoReset = false;
    }
    return VNEXT_PUBLISHED;
}

/* Drains tx_effects/pending_config back-to-back, in guest write order, decoupled from
 * tx_timer/frame_time_ns on purpose (see UartConfigSummary comment in esp32_uart.h): Core's own
 * TX bit-clock needs zero pacing from QEMU, and pacing this delivery by frame_time_ns would let
 * Core finish transmitting an earlier byte -- and start the next one -- before a trailing
 * TXFIFO_RST meant to catch it had even been applied, silently reintroducing the ordering bug
 * this design closes.
 *
 * Only ever invoked by aio_bh_poll() under BQL (same convention as this device's existing
 * timers); uart_write() only ever runs under BQL too -- but each writeReg() call below releases
 * the BQL internally for a bounded window (see uart_apply_config_summary() comment), so a
 * concurrent uart_write() CAN interleave between (and even within) the writeReg() calls made
 * here. Per-event ev->configBefore is immune to this (a value-copy taken at push time, and the
 * fixed-size, never-reallocated tx_effects array means a concurrent append at a higher index
 * can't disturb an index already claimed by this loop) -- but the trailing pending_config is
 * live, shared state, so it is copied and cleared as two adjacent plain statements (no writeReg()
 * between them, hence no BQL release in between) before being applied, exactly mirroring how the
 * per-event snapshot in uart_write()'s A_UART_FIFO case already avoids the same race. */
/* E118-AUDIT (EVIDENCE.md, 2026-09-05): wakes any real vCPU parked in uart_write()'s A_UART_FIFO
 * case waiting for backlog space (tx_backlog_waiter[], NOT VNEXT_B ring credit -- see that call
 * site). Called only when at least one backlog slot was actually freed this pass. Each waiter is
 * resumed exactly once and cleared immediately, so a slower-draining BH pass cannot double-resume
 * the same CPUState. Safe to call with zero waiters (both slots NULL): a no-op. */
static void uart_wake_backlog_waiters(ESP32UARTState *s)
{
    for (unsigned i = 0; i < 2; ++i) {
        CPUState *waiter = s->tx_backlog_waiter[i];
        if (!waiter) continue;
        s->tx_backlog_waiter[i] = NULL;
        if (vnext_uart_provenance_enabled()) vnext_uart_backlog_wakes++;
        cpu_resume(waiter);
    }
}


/* E118 (EVIDENCE.md, 2026-09-05): stops at the first VNEXT_WOULD_BLOCK instead of treating "ring
 * full" as fatal or silently dropping the rest of the batch. Unsent tx_effects are compacted to
 * the front and left in place -- this BH does NOT reschedule itself (that would busy-loop with no
 * credit); vnext_b.c's vnext_resume() sweep calls esp32_uart_vnext_credit_available() once lane 0
 * regains credit, which reschedules exactly the peripherals that actually have a backlog. This
 * function never touches any CPU and never blocks on the BQL waiting for Core -- current_cpu is
 * NULL here (main-loop BH), so there is no vCPU to block in the first place (the ONE exception is
 * uart_wake_backlog_waiters() below, which only ever calls cpu_resume() -- never cpu_stop -- on a
 * vCPU that is not this thread). */
static void uart_tx_effect_bh(void *opaque)
{
    ESP32UARTState *s = ESP32_UART(opaque);
    const bool trace = vnext_uart_provenance_enabled();
    if (trace) {
        vnext_uart_bh_entries++;
        if (vnext_uart_is_power_of_two(vnext_uart_bh_entries)) vnext_uart_provenance_print_summary();
    }
    unsigned sent = 0;
    for (; sent < s->tx_effect_count; sent++) {
        UartTxEffect *ev = &s->tx_effects[sent];
        UartConfigSummary remainingConfig = ev->configBefore;
        const VnextPublishResult configResult = uart_apply_config_summary(s, &remainingConfig);
        ev->configBefore = remainingConfig;
        if (configResult == VNEXT_FATAL) return;
        if (configResult == VNEXT_WOULD_BLOCK) break;
        if (trace) vnext_uart_byte_attempts++;
        const VnextPublishResult byteResult = writeReg( s->iomem.addr, ev->byte );
        if (byteResult == VNEXT_FATAL) return;
        if (byteResult == VNEXT_WOULD_BLOCK) {
            if (trace) { vnext_uart_would_block_total++; vnext_uart_byte_would_block++; }
            break;
        }
        if (trace) vnext_uart_byte_published++;
    }
    if (sent < s->tx_effect_count) {
        const unsigned remaining = s->tx_effect_count - sent;
        memmove(s->tx_effects, s->tx_effects + sent, remaining * sizeof(UartTxEffect));
        s->tx_effect_count = remaining;
        /* E118-AUDIT: even a partial drain (sent > 0) freed backlog slots at the front -- wake
         * any waiter now so a blocked vCPU does not sit parked until the backlog fully clears. */
        if (sent > 0) uart_wake_backlog_waiters(s);
        vnext_b_note_nonvcpu_backlog(0);
        return;
    }
    s->tx_effect_count = 0;
    uart_wake_backlog_waiters(s);

    UartConfigSummary trailing = s->pending_config;
    uart_config_summary_clear(&s->pending_config);
    const VnextPublishResult trailingResult = uart_apply_config_summary(s, &trailing);
    if (trailingResult != VNEXT_PUBLISHED) {
        /* The byte backlog drained cleanly but the trailing config summary itself blocked --
         * put it back so the next drain (triggered by the credit-available notify) applies it
         * instead of losing a CLKDIV/CONF0 change. */
        s->pending_config = trailing;
        if (trailingResult == VNEXT_WOULD_BLOCK) vnext_b_note_nonvcpu_backlog(0);
    }
}

/* E118 (EVIDENCE.md, 2026-09-05): called from vnext_b.c once lane 0 regains credit after this (or
 * any) UART instance observed VNEXT_WOULD_BLOCK there. Reschedules exactly the instances that
 * actually have something queued (a backlogged tx_effect_count, or a pending trailing config with
 * nothing else to carry it) -- never all of them unconditionally, and never a bare poll/retry loop
 * of its own; qemu_bh_schedule() itself is a single, cheap "run once on the next main-loop pass"
 * request, not a busy-loop. */
static void esp32_uart_vnext_credit_available_impl(void);

void esp32_uart_vnext_credit_available(void)
{
    if (vnext_uart_provenance_enabled()) vnext_uart_credit_available_calls++;
    esp32_uart_vnext_credit_available_impl();
}

static void esp32_uart_vnext_credit_available_impl(void)
{
    for (unsigned i = 0; i < vnext_uart_instance_count; ++i) {
        ESP32UARTState *s = vnext_uart_instances[i];
        if (!s || !s->tx_effect_bh) continue;
        if (s->tx_effect_count > 0 || s->pending_config.clkdivDirty ||
            s->pending_config.conf0StateDirty || s->pending_config.txFifoReset ||
            s->pending_config.rxFifoReset) {
            qemu_bh_schedule(s->tx_effect_bh);
        }
    }
}

static uint64_t uart_read(void *opaque, hwaddr addr, unsigned int size)
{
    ESP32UARTState *s = ESP32_UART(opaque);
    uint64_t r = 0;

    switch (addr) {
    case A_UART_FIFO:
        if (fifo8_num_used(&s->rx_fifo) == 0) {
            r = 0xEE;
            error_report("esp_uart: read UART FIFO while it is empty");
        } else {
            r = fifo8_pop(&s->rx_fifo);
            esp32_uart_update_irq(s);
            qemu_chr_fe_accept_input(&s->chr);
        }
        break;

    case A_UART_STATUS:
        r = FIELD_DP32(r, UART_STATUS, RXFIFO_CNT, fifo8_num_used(&s->rx_fifo));
        r = FIELD_DP32(r, UART_STATUS, TXFIFO_CNT, fifo8_num_used(&s->tx_fifo));
        break;

    case A_UART_LOWPULSE:
    case A_UART_HIGHPULSE:
        r = 337;  /* FIXME: this should depend on the APB frequency */
        break;
    case A_UART_MEM_CONF:
        r = FIELD_DP32(r, UART_MEM_CONF, RX_SIZE, (unsigned char)(UART_FIFO_LENGTH/128));
        r = FIELD_DP32(r, UART_MEM_CONF, TX_SIZE,  (unsigned char)(UART_FIFO_LENGTH/128));
        break;
    case A_UART_MEM_RX_STATUS: {
        uint32_t fifo_size = fifo8_num_used(&s->rx_fifo);
        /* The software only cares about the differene between WR_ADDR and RD_ADDR;
         * to keep things simpler, set RD_ADDR to 0 and WR_ADDR to the number of bytes
         * in the FIFO. 128 is a special case — write and read pointers should be
         * the same in this case.
         */
        r = FIELD_DP32(0, UART_MEM_RX_STATUS, WR_ADDR, (fifo_size == 128) ? 0 : fifo_size);
        }
        break;
    case A_UART_DATE:
        r = 0x15122500;
        break;
    default:
        r = s->reg[addr / 4];
        break;
    }

    return r;
}

//static unsigned int uart_calc_baud(ESP32UARTState *s)
//{
//    unsigned clkdiv = (FIELD_EX32(s->reg[R_UART_CLKDIV], UART_CLKDIV, CLKDIV) << 4) +
//                          FIELD_EX32(s->reg[R_UART_CLKDIV], UART_CLKDIV, CLKDIV_FRAG);
//    unsigned baud_rate = 115200;
//    if (clkdiv != 0) {
//        /* FIXME: this should depend on the APB frequency */
//        if(FIELD_EX32(s->reg[R_UART_CONF0], UART_CONF0, TICK_REF_ALWAYS_ON)){
//            baud_rate = (unsigned) ((80000000ULL << 4) / clkdiv);
//        }
//        else{
//            baud_rate = (unsigned) ((1000000ULL << 4) / clkdiv); //REF_TICK
//        }
//    }
//    return baud_rate;
//}

static void updateBaud( ESP32UARTState *s )
{
    uint32_t baud_rate = 115273;
    uint32_t freq = s->use_apb ? esp32_soc_get_apb_freq() : 1000000;
    if( freq == 0 ) freq = 40000001;
    if( s->clkdiv ) baud_rate = (freq << 4) / s->clkdiv;

    uint32_t bitTime = 1e9/baud_rate;
    s->frame_time_ns = bitTime*10;  /// TODO: this depends on frame size
    if( s->baud_rate != baud_rate ){
        s->baud_rate = baud_rate;
        //printf("Qemu: baudrate %i %i %i %lu\n", baud_rate, s->clkdiv, freq, s->frame_time_ns ); fflush( stdout );
        /* [FIX] deferred -- see uart_apply_config_summary()/uart_tx_effect_bh(); do not
         * writeReg() synchronously from inside a guest MMIO dispatch (CLKDIV write, or a CONF0
         * write that toggles use_apb -- both call into this function). */
        s->pending_config.clkdivDirty = true;
        s->pending_config.clkdivValue = bitTime;
        qemu_bh_schedule( s->tx_effect_bh );
    }
}


/* Diagnostic-only UART0 TX mirror. In esp32-simul the UART is owned by the Core arena, so a
 * standalone QEMU run (no Core attached) has no visible console. Setting
 * LASECSIMUL_UART0_MIRROR=<file> appends every byte accepted into UART0's TX FIFO to that file.
 * Unset (the default) this is a single cached pointer test per byte and changes nothing. */
static void uart_mirror_tx(ESP32UARTState *s, uint8_t byte)
{
    static FILE *mirror;
    static bool resolved;
    if (s->iomem.addr != 0x3ff40000) return;   /* ESP32 UART0 */
    if (!resolved) {
        const char *path = getenv("LASECSIMUL_UART0_MIRROR");
        resolved = true;
        mirror = (path && *path) ? fopen(path, "ab") : NULL;
    }
    if (mirror) {
        fputc(byte, mirror);
        if (byte == 0x0a) fflush(mirror);
    }
}

static void uart_write(void *opaque, hwaddr addr, uint64_t value, unsigned int size )
{
    ESP32UARTState *s = ESP32_UART(opaque);

    switch (addr) {
    case A_UART_FIFO:
        if (fifo8_num_free(&s->tx_fifo) == 0) {
            error_report("esp_uart: write to UART FIFO while it is full");
        } else if (s->tx_effect_count >= UART_FIFO_LENGTH) {
            /* E118-AUDIT (EVIDENCE.md, 2026-09-05): the backlog (tx_effects[]/tx_effect_count) can
             * now genuinely stay at capacity while tx_fifo keeps draining independently at its own
             * baud-rate pace (see the struct comment on tx_backlog_waiter[] in esp32_uart.h) --
             * tx_fifo having room is no longer proof the backlog does too. Block the calling vCPU
             * BEFORE fifo8_push() (nothing guest-visible has happened yet for this byte) and replay
             * the whole write once tx_effect_bh() frees a backlog slot, instead of accepting the
             * byte into tx_fifo and silently never recording its effect. This is backlog capacity,
             * NOT VNEXT_B ring credit -- a different resource; see uart_tx_effect_bh(). */
            CPUState *cpu = current_cpu;
            if (cpu && cpu->cpu_index < 2) {
                if (vnext_uart_provenance_enabled()) vnext_uart_backlog_full_stops++;
                s->tx_backlog_waiter[cpu->cpu_index] = cpu;
                cpu_stop_current();
                cpu_loop_exit_restore(cpu, cpu->mem_io_pc);
            }
            /* No real vCPU to block (should not happen for a genuine guest MMIO write) -- fall
             * back to the pre-existing, honest error report rather than silently dropping. */
            error_report("esp_uart: TX backlog full with no vCPU to block (current_cpu=NULL)");
        } else {
            //printf("Qemu: uart_write, %lu\n", value ); fflush( stdout );
            fifo8_push( &s->tx_fifo, value );
            uart_mirror_tx( s, (uint8_t) value );
            /* [FIX] ordered Core-notification path -- snapshot whatever config accumulated since
             * the last accepted byte (or since reset) as THIS byte's configBefore, then reset the
             * pending summary for the next segment. Bounded at UART_FIFO_LENGTH via the backlog
             * capacity check above (E118-AUDIT) -- no longer identical to tx_fifo's own gate, see
             * that check's comment. */
            {
                UartTxEffect *ev = &s->tx_effects[s->tx_effect_count++];
                ev->byte = (uint8_t) value;
                ev->configBefore = s->pending_config;
                uart_config_summary_clear( &s->pending_config );
            }
            qemu_bh_schedule( s->tx_effect_bh );
            /* [FIX] TG0WDT_SYS_RESET investigation -- do NOT call uart_send_next()
             * (Core-backed writeReg()) synchronously from inside this MMIO dispatch; see the
             * comment on uart_send_next() above. Only arm the existing tx_timer for "now" (fires
             * on the next main-loop timer pass, outside this dispatch) if the TX pipeline is
             * genuinely idle -- neither a byte in flight nor a start already scheduled -- so
             * repeated guest writes while a start is pending don't re-arm redundantly. This timer
             * is LOCAL pacing only now (FIFO pop/UART_STATUS/IRQ) -- unrelated to Core
             * notification, which tx_effect_bh handles independently, back-to-back. */
            if( !s->tx_in_flight && !timer_pending(&s->tx_timer) ) {
                timer_mod_ns( &s->tx_timer, getQemu_ns() );
            }
            //uart_transmit(NULL, G_IO_OUT, s);
        }
        break;

    case A_UART_INT_CLR:
        s->reg[R_UART_INT_ST] &= ~((uint32_t) value);
        s->reg[addr / 4] = value;
        if (value & R_UART_INT_CLR_RXFIFO_TOUT_MASK) {
            s->rxfifo_tout = false;
        }
        break;

    case A_UART_INT_ENA: s->reg[addr / 4] = value; break;
    case A_UART_CLKDIV: {
        s->reg[addr / 4] = value;

        uint32_t clkFra = (value & 0x00F00000) >> 20;
        uint32_t clkInt = (value & 0x000FFFFF) << 4 ;
        uint32_t clkdiv = clkInt | clkFra;
        if( s->clkdiv != clkdiv ){
            s->clkdiv = clkdiv;
            updateBaud( s );
        }
        //unsigned clkdiv = (FIELD_EX32( value, UART_CLKDIV, CLKDIV) << 4)
        //                 + FIELD_EX32( value, UART_CLKDIV, CLKDIV_FRAG);
        break;
    }
    case A_UART_AUTOBAUD:
        /* If autobaud is enabled, pretend that sufficient number of edges on the RXD line
         * have been received instantly. Autobaud is only used in the ROM bootloader,
         * and it doesn't care if the result is ready immediately.
         */
        if( FIELD_EX32(value, UART_AUTOBAUD, EN) ) s->reg[R_UART_RXD_CNT] = 0x3FF;
        else                                       s->reg[R_UART_RXD_CNT] = 0;
        s->reg[addr / 4] = value;
        break;

    case A_UART_INT_RAW:
    case A_UART_INT_ST:
    case A_UART_STATUS: /* no-op */ break;
    case A_UART_CONF0:
        s->reg[addr / 4] = value;
        uint8_t use_apb = (value & 1<<27)? 1 : 0;
        if( s->use_apb != use_apb ){
            s->use_apb = use_apb;
            updateBaud( s );
        }
        //printf("Qemu: CONF0 %lu\n", value ); fflush( stdout );
        /* [FIX] deferred -- see uart_apply_config_summary()/uart_tx_effect_bh(). State bits
         * (dataBits/stopBits) last-write-wins via conf0StateValue; TXFIFO_RST/RXFIFO_RST
         * OR-accumulate separately so a clear pulse from an earlier write in this segment is
         * never lost even if this write's own value doesn't carry that bit. */
        s->pending_config.conf0StateDirty = true;
        s->pending_config.conf0StateValue = value;
        if( value & (1u << 18) ) s->pending_config.txFifoReset = true;
        if( value & (1u << 17) ) s->pending_config.rxFifoReset = true;
        qemu_bh_schedule( s->tx_effect_bh );
        break;
    case A_UART_CONF1:
        s->reg[addr / 4] = value;
        s->tx_empty_threshold = FIELD_EX32(s->reg[R_UART_CONF1], UART_CONF1, TXFIFO_EMPTY_THRD);
        s->rx_full_threshold = FIELD_EX32(s->reg[R_UART_CONF1], UART_CONF1, RXFIFO_FULL_THRD);
        /* On the ESP32, rx_tout_thres is in units of (bit_time * 8).
         * Note this is different on later chips.
         */
        s->rx_tout_thres = 8 * FIELD_EX32(s->reg[R_UART_CONF1], UART_CONF1, TOUT_THRD);
        s->rx_tout_ena = FIELD_EX32(s->reg[R_UART_CONF1], UART_CONF1, TOUT_EN) != 0;
        esp32_uart_set_rx_timeout(s);
        esp32_uart_update_irq(s);
        break;

    default:
        if (addr > sizeof(s->reg)) {
            error_report("esp_uart: write to addr=0x%x out of bounds\n", (uint32_t) addr);
        } else {
            s->reg[addr / 4] = value;
        }
        break;
    }
    esp32_uart_update_irq(s);
}

void uart_receive(void *opaque, const uint8_t *buf, int size)
{
    ESP32UARTState *s = ESP32_UART(opaque);

    if (size == 0) {
        return;
    }

    /* If we can receive anything: cancel any pending RX timeout timer,
     * and clear the receive timeout flag.
     */
    if (fifo8_num_free(&s->rx_fifo) > 0) {
        timer_del(&s->rx_timeout_timer);
        s->rxfifo_tout = false;
    }

    /* Move the data into the FIFO */
    for (int i = 0; i < size && fifo8_num_free(&s->rx_fifo) > 0; i++) {
        fifo8_push(&s->rx_fifo, buf[i]);
    }

    /* Receive throttling: some applications (in particular the ESP32 ROM bootloader)
     * may work incorrectly if the data comes in much faster than what UART baud rate
     * would allow. This code adds a delay every UART_FIFO_LENGTH bytes, to make the
     * average data rate match the configured baud rate.
     * This doesn't need to be very precise, so only add the delay if the FIFO is full
     * (which most likely means that more data will come).
     */
    if (fifo8_is_full(&s->rx_fifo)) {
        s->throttle_rx = true;
        const int bits_per_symbol = 10;
        int64_t throttle_time_ns = (int64_t) UART_FIFO_LENGTH * bits_per_symbol * NANOSECONDS_PER_SECOND / s->baud_rate;
        timer_mod_ns(&s->throttle_timer,
                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                     throttle_time_ns);
    }
    printf("uart_receive");
    esp32_uart_set_rx_timeout(s);
    esp32_uart_update_irq(s);
}

void esp32_uart_vnext_rx_byte(uint32_t uart_index, uint8_t byte)
{
    if (uart_index >= 3 || !vnext_uart_instances[uart_index]) return;
    ESP32UARTState *s = vnext_uart_instances[uart_index];
    if (fifo8_num_free(&s->rx_fifo) == 0) return;
    fifo8_push(&s->rx_fifo, byte);
    esp32_uart_set_rx_timeout(s);
    esp32_uart_update_irq(s);
}

int uart_can_receive(void *opaque)
{
    ESP32UARTState *s = ESP32_UART(opaque);
    if (s->throttle_rx) {
        return 0;
    }
    return fifo8_num_free(&s->rx_fifo);
}

static void uart_event(void *opaque, QEMUChrEvent event)
{
    /* TODO: handle UART break */
}

static void uart_tx_timer_cb(void* opaque)
{
    ESP32UARTState *s = ESP32_UART(opaque);

    /* [FIX] TG0WDT_SYS_RESET investigation -- this callback now services two distinct firings of
     * the same tx_timer: a FINISH (a byte uart_send_next() started frame_time_ns ago -- pop it,
     * exactly as before) and a START (armed directly by uart_write(), see A_UART_FIFO case above,
     * for the first byte of a fresh burst -- nothing to pop yet, tx_in_flight is still false). */
    if( s->tx_in_flight ) {
        fifo8_pop( &s->tx_fifo );
        s->tx_in_flight = false;
    }
    if( fifo8_num_used( &s->tx_fifo ) ) uart_send_next( s );
    //printf("uart_tx_timer %lu\n", getQemu_ps() );fflush( stdout );
    esp32_uart_update_irq(s);
}

static void uart_throttle_timer_cb(void* opaque)
{
    ESP32UARTState *s = ESP32_UART(opaque);
    s->throttle_rx = false;
    qemu_chr_fe_accept_input(&s->chr);
}

static void uart_rx_timeout_timer_cb(void* opaque)
{
    ESP32UARTState *s = ESP32_UART(opaque);
    s->rxfifo_tout = true;
    esp32_uart_update_irq(s);
}

static void esp32_uart_reset(DeviceState *dev)
{
    ESP32UARTState *s = ESP32_UART(dev);

    memset(s->reg, 0, sizeof(s->reg));
    s->reg[R_UART_RXD_CNT] = 0;
    s->reg[R_UART_INT_ST] = 0;
    s->reg[R_UART_INT_RAW] = 0;
    s->reg[R_UART_INT_ENA] = 0;
    s->reg[R_UART_AUTOBAUD] = 0;
    /* Default baud rate divider after reset */
    s->reg[R_UART_CLKDIV] = FIELD_DP32(0, UART_CLKDIV, CLKDIV, 0x2B6);
    //s->reg[R_UART_CONF0] = FIELD_DP32(0 , UART_CONF0, TICK_REF_ALWAYS_ON, 1);
    //s->reg[R_UART_CONF0] = FIELD_DP32(s->reg[R_UART_CONF0] , UART_CONF0, STOP_BIT_NUM, 1);
    //s->reg[R_UART_CONF0] = FIELD_DP32(s->reg[R_UART_CONF0] , UART_CONF0, BIT_NUM, 3);

    s->use_apb = 1;
    s->clkdiv = 11104;
    s->baud_rate = 115273;
    fifo8_reset(&s->tx_fifo);
    fifo8_reset(&s->rx_fifo);
    if (s->tx_watch_handle) {
        g_source_remove(s->tx_watch_handle);
        s->tx_watch_handle = 0;
    }
    /* [FIX] achado 2026-07-27 -- causa raiz do crash pre-existente e nao relacionado
     * "ERROR:../util/fifo8.c:62:fifo8_pop: assertion failed: (fifo->num > 0)" ja documentado desde
     * .spec 32.5.8/32.5.12/32.5.16 como "bug generico do QEMU, nao deste fork" -- na verdade E deste
     * fork: se um byte esta em transito (tx_timer armado) no momento de um reset, o timer
     * permanecia agendado (nunca cancelado aqui, ao contrario de throttle_timer alguns bytes
     * abaixo) e disparava DEPOIS do reset, chamando uart_tx_timer_cb() -> fifo8_pop(&s->tx_fifo) na
     * fifo que fifo8_reset() acabou de esvaziar -- violando a invariante que fifo8_pop() assume.
     * Corrigido cancelando tx_timer aqui tambem, mesmo padrao ja usado pra throttle_timer. */
    timer_del(&s->tx_timer);
    s->tx_in_flight = false;
    /* [FIX] deferred Core-notification state -- cancel any pending drain and discard whatever
     * was accumulated; matches the tx_timer/tx_in_flight reset immediately above. */
    qemu_bh_cancel(s->tx_effect_bh);
    uart_config_summary_clear(&s->pending_config);
    s->tx_effect_count = 0;
    /* E118-AUDIT (EVIDENCE.md, 2026-09-05): a vCPU can be parked in uart_write() waiting for
     * backlog space (see tx_backlog_waiter[] in esp32_uart.h) at the exact moment of a reset.
     * Discarding the backlog without waking it would leave that vCPU stopped forever -- resume it
     * now (the reset above already guarantees the retry will find space) rather than merely
     * forgetting the pointer. This reset discards only this device generation's own pending
     * events; the resumed vCPU replays its write against the post-reset state, not stale data. */
    uart_wake_backlog_waiters(s);
    timer_del(&s->throttle_timer);
    s->throttle_rx = false;
    s->rx_tout_ena = false;
    s->tx_empty_threshold = 0;
    s->rx_full_threshold = 0;
    s->rx_tout_thres = 0;
    qemu_irq_lower(s->irq);
}

extern GMainContext *g_main_context_default_l;

//static int uart_num = 0;
static void esp32_uart_realize(DeviceState *dev, Error **errp)
{
    //ESP32UARTState *s = ESP32_UART(dev);
    //s->id = uart_num++;
    //qemu_chr_fe_set_handlers(&s->chr, uart_can_receive, uart_receive,
    //                         uart_event, NULL, s, g_main_context_default_l, true);
}


static void esp32_uart_init(Object *obj)
{
    ESP32UARTState *s = ESP32_UART(obj);
    if (vnext_uart_instance_count < 3)
        vnext_uart_instances[vnext_uart_instance_count++] = s;
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    ESP32UARTClass *class = ESP32_UART_GET_CLASS(obj);

    s->uart_ops = (MemoryRegionOps) {
        .read =  class->uart_read,
        .write = class->uart_write,
        .endianness = DEVICE_LITTLE_ENDIAN,
    };

    memory_region_init_io(&s->iomem, obj, &s->uart_ops, s,
                          TYPE_ESP32_UART, UART_REG_CNT * sizeof(uint32_t));
    /* E118-AUDIT-2 (EVIDENCE.md, 2026-09-05): the E118-AUDIT backlog fix in uart_write()'s
     * A_UART_FIFO case calls cpu_stop_current() + cpu_loop_exit_restore() DIRECTLY from within
     * THIS device's own .write callback when the backlog is full. That siglongjmp skips
     * softmmu/memory.c's normal-return clear of mem_reentrancy_guard.engaged_in_io (set on entry
     * to this dispatch), permanently wedging every later access to this UART with "Blocked
     * re-entrant IO" -- root-caused via that exact warning preceding the E118-AUDIT gate's storm.
     * Same reasoning as hw/i2c/esp32_i2c.c's matching comment: safe to disable because this retry
     * path never drops the BQL. */
    s->iomem.disable_reentrancy_guard = true;
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
    fifo8_create(&s->tx_fifo, UART_FIFO_LENGTH);
    fifo8_create(&s->rx_fifo, UART_FIFO_LENGTH);
    timer_init_ns(&s->throttle_timer, QEMU_CLOCK_VIRTUAL, uart_throttle_timer_cb, s);
    timer_init_ns(&s->rx_timeout_timer, QEMU_CLOCK_VIRTUAL, uart_rx_timeout_timer_cb, s);
    timer_init_ns( &s->tx_timer, QEMU_CLOCK_VIRTUAL, uart_tx_timer_cb, s);
    s->tx_effect_bh = qemu_bh_new( uart_tx_effect_bh, s );
}


static Property esp32_uart_properties[] = {
    DEFINE_PROP_CHR("chardev", ESP32UARTState, chr),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32_uart_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ESP32UARTClass *class = ESP32_UART_CLASS(klass);

    /* Populate the virtual attributes and methods here (if any) */
    class->uart_write = uart_write;
    class->uart_read = uart_read;

    dc->reset = esp32_uart_reset;
    dc->realize = esp32_uart_realize;
    device_class_set_props(dc, esp32_uart_properties);
}

static const TypeInfo esp32_uart_info = {
    .name = TYPE_ESP32_UART,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ESP32UARTState),
    .instance_init = esp32_uart_init,
    .class_init = esp32_uart_class_init,
    .class_size = sizeof(ESP32UARTClass)
};

static void esp32_uart_register_types(void)
{
    type_register_static(&esp32_uart_info);
}

type_init(esp32_uart_register_types)
