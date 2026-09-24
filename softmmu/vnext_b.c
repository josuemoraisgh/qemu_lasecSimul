#include "qemu/osdep.h"
#include "exec/memory.h"
#include "exec/address-spaces.h"
#include "qemu/atomic.h"
#include "qemu/main-loop.h"
#include "qemu/timer.h"
#include "sysemu/cpus.h"
#include "sysemu/sysemu.h"
#include "sysemu/runstate.h"
#include "hw/core/cpu.h"
#include "hw/char/esp32_uart.h"
#include "hw/i2c/esp32_i2c.h"
#include "hw/timer/esp32_timg.h"
#include "softmmu/vnext_b_classify.h"
#include "vnext_b.h"

G_NORETURN void cpu_loop_exit_restore(CPUState *cpu, uintptr_t pc);

#ifdef _WIN32
#include <windows.h>

#define VNEXT_MAGIC UINT64_C(0x4C415443564E4251)
#define VNEXT_EVENT_BYTES 64u
#define VNEXT_LANES 16u
#define VNEXT_DEPTH 1024u

typedef struct VnextEvent { uint64_t timestamp, sequence; uint32_t kind, flags, endpoint, bytes; uint8_t payload[VNEXT_EVENT_BYTES]; } VnextEvent;
typedef struct VnextRegion { uint64_t offset, bytes; uint32_t stride, count; } VnextRegion;
typedef struct VnextEndpoint { uint32_t id, kind; uint64_t q2c, c2a, snapshot; uint32_t depth, stride, limit, reserved; } VnextEndpoint;
typedef struct VnextLane { VnextRegion metadata, events; uint32_t depth, stride; } VnextLane;
typedef struct VnextSnapshot { uint64_t publishSeq; uint8_t data[64]; } VnextSnapshot;
typedef struct VnextResponse { uint64_t requestSeq, responseSeq; int32_t status; uint32_t bytes; uint8_t payload[64]; } VnextResponse;
typedef struct VnextControl {
    uint64_t magic; uint32_t major, minor; uint64_t bytes, execution;
    uint32_t laneCount, responseCount, descriptorCount, descriptorCapacity;
    uint64_t descriptorOffset, q2cOffset, c2aOffset, snapshotOffset;
    uint64_t coreProgress, artifactProgress; uint32_t coreState, artifactState, coreFatal, artifactFatal;
    uint64_t coreFatalSeq, artifactFatalSeq, capabilities; uint32_t endpointCount, snapshotCount;
    uint64_t laneDescriptorOffset, responseOffset, snapshotDescriptorOffset, c2aDescriptorOffset;
    /* ABI v1.1 (see core/include/lasecsimul/artifact_transport_abi.h's
     * lasec_at_control_page::artifact_virtual_time_ns doc-comment -- this field must stay
     * byte-layout-identical to that one, appended last so no existing offset shifts). */
    uint64_t artifactVirtualTimeNs;
} VnextControl;

static uint8_t *vnext_base;
static VnextControl *vnext_control;
static HANDLE vnext_mapping, vnext_core_event, vnext_artifact_event;
static MemoryRegion vnext_mmio;

/* E147-D (EVIDENCE.md, 2026-09-10): bounded, opt-in (LASECSIMUL_SETTLE_PROVENANCE=1, same env
 * var the Core-side SettleProvenance.hpp instrumentation uses; off by default, zero cost when
 * unset) causal counters for vnext_resume() -- the Core-side instrumentation already proved
 * real lane-ring traffic (GPIO/I2C/register-read events) totals only hundreds across a 15s run
 * while the artifact-event wake callback fires millions of times; this settles whether
 * vnext_resume() itself (registered on vnext_core_event via qemu_add_wait_object) is invoked
 * anywhere near that often, and whether its own unconditional `if (vnext_vm_started)
 * SetEvent(vnext_artifact_event)` (no gating on whether the lane-credit sweep below it found
 * anything to do) is what dominates. Aggregated only -- never a line per call. */
static uint64_t vnext_resume_entry_count;
static uint64_t vnext_resume_unconditional_signal_count;
static uint64_t vnext_resume_found_real_work_count;
static bool vnext_provenance_enabled_cached;
static bool vnext_provenance_checked;

/* E147-G (EVIDENCE.md, 2026-09-10): H147-G -- vnext_block_producer_on_lane(lane, NULL)'s
 * immediate recheck of vnext_lane_credit() can observe credit that returned in the window
 * between the caller's WOULD_BLOCK classification and this call, and clears
 * vnext_lane_producer_backlog[lane] right there -- BEFORE the caller (e.g. uart_tx_effect_bh())
 * has actually preserved its own pending work. If that happens, vnext_resume()'s sweep sees no
 * backlog for this lane on its next pass and never calls vnext_notify_lane_backlog_cleared(),
 * so the peripheral's own BH -- which does not reschedule itself by design -- never gets called
 * again, and the pending item (a CLKDIV/CONF0 config or a TX byte) is stuck forever. Counted
 * here, not guessed. */
static uint64_t vnext_nonvcpu_block_calls;
static uint64_t vnext_nonvcpu_backlog_armed;
static uint64_t vnext_nonvcpu_immediate_credit_race;
static uint64_t vnext_last_lane0_credit;
static uint64_t vnext_lane0_credit_nonzero_sweeps;
static uint64_t vnext_i2c_submit_count;
static uint8_t vnext_i2c_first_address;
static uint8_t vnext_i2c_last_address;
static uint64_t vnext_i2c_response_count;
static uint64_t vnext_i2c_ack_count;
static CPUState *vnext_cpus[VNEXT_LANES];
static bool vnext_blocked[VNEXT_LANES];
static bool vnext_lane_producer_backlog[VNEXT_LANES];
static uint32_t vnext_lane_credit(uint32_t lane);
static void vnext_notify_lane_backlog_cleared(uint32_t lane);

static void vnext_provenance_print_summary(void) {
    fprintf(stderr, "[SETTLE_PROVENANCE_QEMU] vnextResumeEntries=%llu "
            "vnextResumeUnconditionalSignal=%llu vnextResumeFoundRealWork=%llu "
            "nonvcpuBlockCalls=%llu nonvcpuBacklogArmed=%llu nonvcpuImmediateCreditRace=%llu "
            "lastLane0Credit=%llu lane0CreditNonzeroSweeps=%llu\n",
            (unsigned long long)vnext_resume_entry_count,
            (unsigned long long)vnext_resume_unconditional_signal_count,
            (unsigned long long)vnext_resume_found_real_work_count,
            (unsigned long long)vnext_nonvcpu_block_calls,
            (unsigned long long)vnext_nonvcpu_backlog_armed,
            (unsigned long long)vnext_nonvcpu_immediate_credit_race,
            (unsigned long long)vnext_last_lane0_credit,
            (unsigned long long)vnext_lane0_credit_nonzero_sweeps);
    fprintf(stderr, "[SETTLE_PROVENANCE_QEMU] i2cSubmitCount=%llu i2cFirstAddress=0x%02x i2cLastAddress=0x%02x\n",
            (unsigned long long)vnext_i2c_submit_count,
            vnext_i2c_first_address, vnext_i2c_last_address);
    fprintf(stderr, "[SETTLE_PROVENANCE_QEMU] i2cResponseCount=%llu i2cAckCount=%llu\n",
            (unsigned long long)vnext_i2c_response_count,
            (unsigned long long)vnext_i2c_ack_count);
    if (vnext_control && vnext_control->laneCount > 0) {
        fprintf(stderr, "[SETTLE_PROVENANCE_QEMU] lane0Backlog=%d lane0Blocked=%d lane0CreditNow=%u "
                "cpu0Stopped=%d cpu1Stopped=%d cpu0Pc=0x%08" PRIx64 " cpu1Pc=0x%08" PRIx64 "\n",
                vnext_lane_producer_backlog[0] ? 1 : 0,
                vnext_blocked[0] ? 1 : 0,
                vnext_lane_credit(0),
                vnext_cpus[0] ? cpu_is_stopped(vnext_cpus[0]) : -1,
                vnext_cpus[1] ? cpu_is_stopped(vnext_cpus[1]) : -1,
                vnext_cpus[0] && CPU_GET_CLASS(vnext_cpus[0])->get_pc ?
                    CPU_GET_CLASS(vnext_cpus[0])->get_pc(vnext_cpus[0]) : 0,
                vnext_cpus[1] && CPU_GET_CLASS(vnext_cpus[1])->get_pc ?
                    CPU_GET_CLASS(vnext_cpus[1])->get_pc(vnext_cpus[1]) : 0);
    }
}

static bool vnext_is_power_of_two(uint64_t value) {
    return value != 0 && (value & (value - 1)) == 0;
}

static bool vnext_provenance_enabled(void) {
    if (!vnext_provenance_checked) {
        vnext_provenance_checked = true;
        const char *v = getenv("LASECSIMUL_SETTLE_PROVENANCE");
        vnext_provenance_enabled_cached = v && *v && v[0] != '0';
        /* Registered once, lazily, on first real use -- prints once at process exit regardless
         * of how vnext_b_main() itself returns/is torn down. No-op (never registered) when the
         * env var is unset, matching the Core-side SettleProvenance.hpp contract exactly. */
        if (vnext_provenance_enabled_cached) {
            atexit(vnext_provenance_print_summary);
        }
    }
    return vnext_provenance_enabled_cached;
}
/* R3c heartbeat (PLAN_MTTCG_VNEXT_B_CAUSALITY.md, per the user's own detailed spec): a
 * QEMU_CLOCK_VIRTUAL-driven watermark, independent of lane/endpoint activity, so
 * McuComponent::pacingPositionNs() has proof that virtual time elapsed even when the guest is
 * legitimately silent (WFI, no GPIO/I2C). Deliberately NOT a lane event: never touches
 * vnext_blocked[]/vnext_request_*[]/any ring, never causes an electrical effect, never
 * participates in the causal arbiter's ordering. */
static QEMUTimer *vnext_heartbeat_timer;
#define VNEXT_HEARTBEAT_INTERVAL_NS (1000LL * 1000 * 1000) /* 1s virtual: ample margin under any
                                                              * realistic pacing stall threshold
                                                              * (e.g. 10s in
                                                              * McuSchedulerPacingSyncRealQemuTest.cpp)
                                                              * without adding per-tick log volume
                                                              * (E099) or excessive doorbell churn. */
static bool vnext_transport_ready;
static bool vnext_start_guest_on_core_ready;
static bool vnext_vm_started;
static bool vnext_heartbeat_initialized;
static bool vnext_start_paused;
/* E118 (EVIDENCE.md, 2026-09-05): set when a non-vCPU producer (current_cpu == NULL -- a BH or
 * timer callback) observes VNEXT_WOULD_BLOCK on this lane. Distinct from vnext_blocked[], which
 * means "a real vCPU is parked waiting for credit and needs cpu_resume()". Cleared, and the
 * producer notified, by vnext_resume()'s sweep once credit returns -- never touches any CPU. */
static unsigned vnext_self_test_writes;
static bool vnext_self_test_lane1;
static bool vnext_self_test_snapshot;
static bool vnext_self_test_gpio;
static bool vnext_self_test_requests;
static void vnext_block_producer_on_lane(uint32_t lane, CPUState *cpu);
static uint64_t vnext_request_seq[VNEXT_LANES];
static bool vnext_request_waiting[VNEXT_LANES];
static bool vnext_request_ready[VNEXT_LANES];
static bool vnext_i2c_waiting[VNEXT_LANES];
static uint64_t vnext_i2c_seq[VNEXT_LANES];
static uint32_t vnext_i2c_bus[VNEXT_LANES];
static uint8_t vnext_i2c_rx[2][32];
static uint32_t vnext_i2c_rx_len[2], vnext_i2c_rx_pos[2];
static uint64_t vnext_i2c_status[2];
static uint64_t vnext_i2c_cmd[2];

static unsigned vnext_self_test_init_delay_ms(void)
{
    const char *value = getenv("LASECSIMUL_VNEXT_B_SELF_TEST_INIT_DELAY_MS");
    char *end = NULL;
    unsigned long delay;

    if (!value || !*value) {
        return 0;
    }
    delay = strtoul(value, &end, 10);
    return end && !*end && delay <= 10000 ? (unsigned)delay : 0;
}
static void vnext_self_test_publish(CPUState *cpu, uint64_t value);
static uint64_t vnext_read(void *opaque, hwaddr addr, unsigned size);

/* Endpoint adapter callback: the transport owns only the generic response bytes;
 * the ESP32 peripheral owns their interpretation and timing. Declared in
 * include/hw/i2c/esp32_i2c.h (E118-AUDIT: now included above for
 * esp32_i2c_vnext_credit_available() too) -- no ad-hoc redeclaration needed here anymore. */

bool vnext_b_i2c_submit(uint32_t bus, uint32_t flags, uint64_t period_ns,
                        const uint8_t *tx, uint32_t tx_len, uint32_t rx_len) {
    if (!vnext_b_active() || bus > 1 || !tx || !tx_len || tx_len > 32 || rx_len > 32) return false;
    CPUState *cpu = current_cpu;
    const uint32_t lane = cpu ? cpu->cpu_index : 0;
    if (lane >= vnext_control->laneCount || vnext_i2c_waiting[lane]) return false;
    VnextLane *d = (VnextLane *)(vnext_base + vnext_control->laneDescriptorOffset) + lane;
    uint64_t *meta = (uint64_t *)(vnext_base + d->metadata.offset);
    const uint64_t write = qatomic_load_acquire(&meta[0]);
    const uint64_t read = qatomic_load_acquire(&meta[1]);
    /* E118-AUDIT (EVIDENCE.md, 2026-09-05): classification extracted to a pure, unit-tested
     * helper (include/softmmu/vnext_b_classify.h, tests/unit/test-vnext-b-classify.c) so this
     * arithmetic has coverage independent of a live ring. */
    const VnextRingClassification classification = vnext_ring_classify(write, read, d->depth);
    if (classification == VNEXT_RING_FATAL) {
        /* E118: genuine corruption, distinct from ordinary full (occupancy == depth) below. */
        qatomic_store_release(&vnext_control->artifactFatal, 3);
        qatomic_store_release(&vnext_control->artifactState, 6);
        SetEvent(vnext_artifact_event);
        return false;
    }
    if (classification == VNEXT_RING_WOULD_BLOCK) {
        /* E118: ordinary backpressure, never fatal. The caller (esp32_i2c.c) already falls back
         * to the electrical writeReg() path on a false return without having mutated any
         * guest-visible state yet -- that path goes through vnext_b_gpio_write(), which blocks a
         * real vCPU and retries automatically, or backlogs a non-vCPU producer without loss. */
        return false;
    }
    const uint32_t op_bytes = 14u + tx_len;
    if (12u + op_bytes > VNEXT_EVENT_BYTES) return false;
    const uint64_t seq = ++vnext_request_seq[lane];
    if (vnext_provenance_enabled()) {
        if (vnext_i2c_submit_count == 0 && tx_len) vnext_i2c_first_address = tx[0];
        if (tx_len) vnext_i2c_last_address = tx[0];
        vnext_i2c_submit_count++;
    }
    vnext_i2c_seq[lane] = seq;
    vnext_i2c_bus[lane] = bus;
    vnext_i2c_status[bus] = 1u << 4; /* BUS_BUSY until causal completion */
    VnextResponse *slot = (VnextResponse *)(vnext_base + vnext_control->responseOffset) + lane;
    qatomic_store_release(&slot->requestSeq, seq);
    VnextEvent *event = (VnextEvent *)(vnext_base + d->events.offset) + (write & (d->depth - 1));
    memset(event, 0, sizeof(*event));
    event->timestamp = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    event->kind = 11; event->endpoint = 0; event->bytes = 12u + op_bytes; event->sequence = write;
    event->payload[0] = 1; /* generic BATCH count */
    memcpy(event->payload + 4, &seq, sizeof(uint16_t));
    event->payload[6] = 0; event->payload[7] = 0;
    memcpy(event->payload + 8, &op_bytes, sizeof(uint16_t));
    event->payload[12] = (uint8_t)bus;
    event->payload[13] = (uint8_t)flags;
    memcpy(event->payload + 14, &tx_len, sizeof(uint16_t));
    memcpy(event->payload + 16, &rx_len, sizeof(uint16_t));
    memcpy(event->payload + 18, &period_ns, sizeof(period_ns));
    memcpy(event->payload + 26, tx, tx_len);
    qatomic_store_release(&meta[0], write + 1);
    vnext_i2c_waiting[lane] = true;
    SetEvent(vnext_artifact_event);
    if (vnext_lane_credit(lane) == 0) {
        /* E118: cpu may be NULL here (a non-vCPU producer -- see vnext_b_gpio_write()'s comment
         * on the same distinction); vnext_block_producer_on_lane() already handles both cases
         * without touching a CPU that was never actually running on this thread. */
        vnext_block_producer_on_lane(lane, cpu);
    }
    return true;
}

static void vnext_publish_request(uint32_t lane, uint64_t address) {
    VnextLane *d = (VnextLane *)(vnext_base + vnext_control->laneDescriptorOffset) + lane;
    uint64_t *meta = (uint64_t *)(vnext_base + d->metadata.offset);
    const uint64_t write = qatomic_load_acquire(&meta[0]);
    const uint64_t read = qatomic_load_acquire(&meta[1]);
    const VnextRingClassification classification = vnext_ring_classify(write, read, d->depth);
    if (classification == VNEXT_RING_FATAL) {
        /* E118: genuine corruption, distinct from ordinary full (occupancy == depth) below. */
        qatomic_store_release(&vnext_control->artifactFatal, 2);
        qatomic_store_release(&vnext_control->artifactState, 6);
        SetEvent(vnext_artifact_event);
        return;
    }
    if (classification == VNEXT_RING_WOULD_BLOCK) {
        /* E118: ordinary backpressure, never fatal. vnext_publish_request()'s only caller
         * (vnext_read(), always with a real vCPU) unconditionally calls cpu_stop_current() +
         * cpu_loop_exit_restore() right after this returns regardless of outcome -- the guest's
         * read instruction is replayed once resumed, and this call is retried from scratch, so
         * there is nothing further to do here beyond not publishing and not marking fatal. */
        return;
    }
    uint64_t seq = ++vnext_request_seq[lane];
    VnextResponse *slot = (VnextResponse *)(vnext_base + vnext_control->responseOffset) + lane;
    qatomic_store_release(&slot->requestSeq, seq);
    VnextEvent *event = (VnextEvent *)(vnext_base + d->events.offset) + (write & (d->depth - 1));
    memset(event, 0, sizeof(*event));
    event->timestamp = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    event->kind = 3; event->endpoint = 0; event->bytes = sizeof(seq) + sizeof(address);
    memcpy(event->payload, &seq, sizeof(seq));
    memcpy(event->payload + sizeof(seq), &address, sizeof(address));
    event->sequence = write;
    qatomic_store_release(&meta[0], write + 1);
    vnext_request_waiting[lane] = true;
    {
        static uint32_t progress_requests[VNEXT_LANES];
        const char *progress = getenv("LASECSIMUL_VNEXT_PROGRESS_TRACE");
        if (progress && *progress && *progress != '0' && progress_requests[lane] < 64) {
            fprintf(stderr, "[VNEXT_PROGRESS] publish_request lane=%u addr=0x%" PRIx64 " seq=%" PRIu64 "\n",
                    lane, address, write);
            progress_requests[lane]++;
        }
    }
    SetEvent(vnext_artifact_event);
}

bool vnext_b_active(void) { return vnext_transport_ready; }

bool vnext_b_lane_has_credit(uint32_t lane) {
    if (!vnext_control || lane >= vnext_control->laneCount || lane >= VNEXT_LANES) return false;
    return vnext_lane_credit(lane) != 0;
}

/* E129 (EVIDENCE.md, 2026-09-05): declared in vnext_b.h for hw/misc/esp32_dport.c's
 * esp32_cache_ill_read() -- this file is compiled per-target and owns the
 * cpu_loop_exit_restore() dependency. */
G_NORETURN void vnext_cache_wait_suspend_current_cpu(void)
{
    CPUState *cpu = current_cpu;
    uintptr_t restore_pc = cpu ? cpu->mem_io_pc : 0;
    cpu_stop_current();
    cpu_loop_exit_restore(cpu, restore_pc);
}

/* E118 (EVIDENCE.md, 2026-09-05): the one place a producer becomes "waiting for lane credit".
 * `cpu` is the REAL calling vCPU (never a fake/borrowed CPUState) or NULL for a BH/timer
 * producer -- callers that need the cpu!=NULL case to actually retry the publish (rather than
 * just resume execution past it) do NOT call this helper; see vnext_b_gpio_write()'s
 * occupancy==depth branch, which uses cpu_loop_exit_restore() instead. This helper is for the
 * "already published, buffer just became exactly full" case (real vCPU: proactive pause/resume,
 * matching DECISION-011's timing intent) and for any BH/timer producer's WOULD_BLOCK (never
 * touches a CPU, only marks the backlog flag vnext_resume() later clears). */
static void vnext_block_producer_on_lane(uint32_t lane, CPUState *cpu) {
    esp32_timg_transport_pause(lane, true);
    if (cpu) {
        vnext_blocked[lane] = true;
        cpu_stop_current();
        if (vnext_lane_credit(lane) != 0) {
            vnext_blocked[lane] = false;
            esp32_timg_transport_pause(lane, false);
            cpu_resume(cpu);
        }
        return;
    }
    if (vnext_provenance_enabled()) vnext_nonvcpu_block_calls++;
    /* The caller arms the retry token after preserving its pending operation.  In
     * particular, do not clear it here: the caller has not yet recorded the
     * unsent byte/continuation, so doing so would recreate the lost-wake race. */
}

void vnext_b_note_nonvcpu_backlog(uint32_t lane)
{
    if (!vnext_b_active() || lane >= VNEXT_LANES) return;
    vnext_lane_producer_backlog[lane] = true;
    if (vnext_provenance_enabled()) vnext_nonvcpu_backlog_armed++;
    /* The producer has now preserved its pending operation.  If credit already
     * returned before this point, do not wait for another core-event edge: clear
     * the token and schedule the retry immediately. */
    if (vnext_lane_credit(lane) != 0) {
        vnext_lane_producer_backlog[lane] = false;
        esp32_timg_transport_pause(lane, false);
        vnext_notify_lane_backlog_cleared(lane);
    }
}

/* E118 (EVIDENCE.md, 2026-09-05): dispatches to every peripheral that might have a non-vCPU
 * backlog on this lane. Lane 0 is the fixed landing lane for any current_cpu==NULL producer
 * (UART's TX BH, and rarely a mid-transaction I2C timer continuation), so more than one producer
 * can legitimately be waiting on the same lane; each hook below is a safe no-op when that
 * specific peripheral has nothing queued. */
static void vnext_notify_lane_backlog_cleared(uint32_t lane) {
    if (lane == 0) {
        esp32_uart_vnext_credit_available();
        /* E118-AUDIT (EVIDENCE.md, 2026-09-05): I2C's own current_cpu==NULL continuation (reached
         * via esp32_i2c_event()'s timer, not synchronously from a guest write) also defaults to
         * lane 0 and can be independently backlogged from UART -- see esp32_i2c.h's
         * vnextContinuationBacklogged. Both hooks are safe no-ops when their peripheral has
         * nothing queued. */
        esp32_i2c_vnext_credit_available();
    }
}


VnextPublishResult vnext_b_gpio_write(uint64_t address, uint64_t value) {
    if (!vnext_b_active()) return VNEXT_PUBLISHED;
    CPUState *cpu = current_cpu;
    /* UART TX effects are drained by a main-loop BH, where current_cpu is
     * intentionally NULL.  They still belong to the fixed producer lane 0;
     * use that lane's live CPU as the stop/resume owner instead of treating
     * NULL as a resumable CPU. */
    const uint32_t lane = cpu ? cpu->cpu_index : 0;
    /* vnext_control->laneCount is already validated <= VNEXT_LANES at mapping-accept time
     * (vnext_validate_layout()), making this transitively safe -- but that invariant crosses
     * functions and runtime shared-memory data, which GCC's -Warray-bounds cannot prove. The
     * redundant, always-true-in-practice `lane >= VNEXT_LANES` check below makes the bound
     * provable locally (E117: this warning newly appeared here after nearby diagnostic additions
     * changed inlining, not a behavior change -- fixed rather than ignored, per "compile without
     * new warnings"). */
    if (lane >= vnext_control->laneCount || lane >= VNEXT_LANES) return VNEXT_PUBLISHED;
    VnextLane *d = (VnextLane *)(vnext_base + vnext_control->laneDescriptorOffset) + lane;
    uint64_t *meta = (uint64_t *)(vnext_base + d->metadata.offset);
    const uint64_t write = qatomic_load_acquire(&meta[0]);
    const uint64_t read = qatomic_load_acquire(&meta[1]);
    const VnextRingClassification classification = vnext_ring_classify(write, read, d->depth);

    if (classification == VNEXT_RING_FATAL) {
        /* E118: a genuine ring-invariant violation (corruption) -- distinct from ordinary
         * backpressure (occupancy == depth, handled below as WOULD_BLOCK). Should be unreachable
         * now that every producer honors WOULD_BLOCK correctly; kept as a hard safety net. See
         * EVIDENCE.md E118's full/would-block/fatal contract. */
        qatomic_store_release(&vnext_control->artifactFatal, 3);
        qatomic_store_release(&vnext_control->artifactState, 6);
        if (cpu) cpu_stop_current();
        return VNEXT_FATAL;
    }

    if (classification == VNEXT_RING_WOULD_BLOCK) {
        /* Ring genuinely full: ordinary backpressure. Never fatal (E118) -- distinguishing this
         * from corruption above is the entire point of the fix. */
        if (cpu) {
            /* Real vCPU: nothing has been published yet, so there is nothing to lose or reorder
             * by not completing this write. Block only this vCPU and replay the whole guest
             * instruction once resumed (same idiom as vnext_read()'s existing
             * cpu_loop_exit_restore() use) -- from the guest's perspective the write simply has
             * not happened until it can succeed. */
            esp32_timg_transport_pause(lane, true);
            vnext_blocked[lane] = true;
            cpu_stop_current();
            cpu_loop_exit_restore(cpu, cpu->mem_io_pc);
        }
        /* Non-vCPU producer (a BH or timer callback, e.g. ESP32 UART's TX BH): never stop a CPU,
         * never busy-loop. Mark the backlog; vnext_resume()'s sweep notifies the producer once
         * credit returns. The caller must keep whatever it could not publish and must not have
         * changed any guest-visible state for it yet. */
        vnext_block_producer_on_lane(lane, NULL);
        return VNEXT_WOULD_BLOCK;
    }

    /* occupancy < depth: room to publish now. */
    VnextEvent *event = (VnextEvent *)(vnext_base + d->events.offset) + (write & (d->depth - 1));
    memset(event, 0, sizeof(*event));
    event->timestamp = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    event->kind = 10; event->endpoint = 0; event->bytes = sizeof(address) + sizeof(value);
    memcpy(event->payload, &address, sizeof(address));
    memcpy(event->payload + sizeof(address), &value, sizeof(value));
    event->sequence = write;
    qatomic_store_release(&meta[0], write + 1);
    SetEvent(vnext_artifact_event);
    if (vnext_lane_credit(lane) == 0) {
        /* Just became exactly full: pre-emptively engage the MWDT pause window now (matching
         * DECISION-011's timing intent) rather than waiting for the next publish attempt to
         * discover WOULD_BLOCK on its own. This event already published successfully -- nothing
         * here can lose or reorder it. */
        vnext_block_producer_on_lane(lane, cpu);
    }
    return VNEXT_PUBLISHED;
}

uint64_t vnext_b_gpio_read(uint64_t address) {
    if (!vnext_b_active() || (address != UINT64_C(0x3FF4403C) && address != UINT64_C(0x3FF44040))) return 0;
    VnextSnapshot *s = (VnextSnapshot *)(vnext_base + vnext_control->snapshotOffset);
    uint64_t value = 0;
    memcpy(&value, s->data + (address == UINT64_C(0x3FF44040) ? sizeof(uint64_t) : 0), sizeof(value));
    return value;
}

static bool vnext_i2c_address(uint64_t address, unsigned *bus, bool *status) {
    if (address >= UINT64_C(0x3FF53000) && address < UINT64_C(0x3FF53100)) {
        *bus = 0; *status = address == UINT64_C(0x3FF53008); return true;
    }
    if (address >= UINT64_C(0x3FF67000) && address < UINT64_C(0x3FF67100)) {
        *bus = 1; *status = address == UINT64_C(0x3FF67008); return true;
    }
    return false;
}

uint64_t vnext_b_register_read(uint64_t address) {
    if (!vnext_b_active()) return 0;
    unsigned bus = 0; bool status = false;
    if (vnext_i2c_address(address, &bus, &status)) {
        /* esp32_i2c_event() reads these registers from a virtual-clock timer callback,
         * not from a guest MMIO callback. Never attempt cpu_loop_exit_restore() here;
         * Core results arrive asynchronously through C2A and are consumed by vnext_resume. */
        if (!status && (address == UINT64_C(0x3FF5301C) || address == UINT64_C(0x3FF6701C)) &&
            vnext_i2c_rx_pos[bus] < vnext_i2c_rx_len[bus])
            return vnext_i2c_rx[bus][vnext_i2c_rx_pos[bus]++];
        return status ? vnext_i2c_status[bus] : vnext_i2c_cmd[bus];
    }
    return vnext_b_gpio_read(address);
}

static void vnext_check_responses(void) {
    for (uint32_t lane = 0; lane < vnext_control->laneCount; ++lane) {
        if (!vnext_request_waiting[lane]) continue;
        VnextResponse *slot = (VnextResponse *)(vnext_base + vnext_control->responseOffset) + lane;
        if (qatomic_load_acquire(&slot->requestSeq) != vnext_request_seq[lane] ||
            qatomic_load_acquire(&slot->responseSeq) != vnext_request_seq[lane]) continue;
        vnext_request_waiting[lane] = false;
        vnext_request_ready[lane] = true;
        qatomic_fetch_add(&vnext_control->artifactProgress, 1);
    }
}

static void vnext_check_i2c_responses(void) {
    for (uint32_t lane = 0; lane < vnext_control->laneCount; ++lane) {
        if (!vnext_i2c_waiting[lane]) continue;
        VnextResponse *slot = (VnextResponse *)(vnext_base + vnext_control->responseOffset) + lane;
        const uint64_t seq = vnext_i2c_seq[lane];
        if (qatomic_load_acquire(&slot->requestSeq) != seq ||
            qatomic_load_acquire(&slot->responseSeq) != seq || slot->bytes < 20) continue;
        uint32_t status = 0, first_nack = UINT32_MAX, rx_len = 0;
        uint64_t stretch = 0;
        memcpy(&status, slot->payload, 4);
        memcpy(&first_nack, slot->payload + 4, 4);
        memcpy(&stretch, slot->payload + 8, 8);
        memcpy(&rx_len, slot->payload + 16, 4);
        if (rx_len > 32 || 20u + rx_len > slot->bytes) rx_len = 0;
        const uint32_t bus = vnext_i2c_bus[lane];
        vnext_i2c_rx_len[bus] = rx_len;
        vnext_i2c_rx_pos[bus] = 0;
        if (rx_len) memcpy(vnext_i2c_rx[bus], slot->payload + 20, rx_len);
        vnext_i2c_status[bus] = ((status & 2u) && first_nack == UINT32_MAX ? 0u : 1u) | (1u << 7);
        if (vnext_provenance_enabled()) {
            vnext_i2c_response_count++;
            if (status & 2u) vnext_i2c_ack_count++;
        }
        esp32_i2c_vnext_complete(bus, status, first_nack, stretch, slot->payload + 20, rx_len);
        vnext_i2c_waiting[lane] = false;
        qatomic_store_release(&slot->requestSeq, 0);
    }
}

/* Synthetic MMIO test endpoint only.  The historical assertion that
 * 0x60000000 was unused by production firmware was incorrect: ESP-IDF
 * writes the UART0 AHB FIFO there. Registration requires an explicit
 * diagnostic opt-in; normal SELF_TEST publications do not need this region. */
static uint64_t vnext_read(void *opaque, hwaddr addr, unsigned size) {
    (void)opaque; (void)addr; (void)size;
    CPUState *cpu = current_cpu;
    const uint32_t lane = cpu ? cpu->cpu_index : 0;
    if (!cpu || lane >= vnext_control->laneCount) return 0;
    VnextResponse *slot = (VnextResponse *)(vnext_base + vnext_control->responseOffset) + lane;
    if (!vnext_request_ready[lane]) {
        if (!vnext_request_waiting[lane]) {
            vnext_publish_request(lane, 0);
        }
        cpu_stop_current();
        // The I/O instruction is re-entered after the owning vCPU is resumed. No value is
        // returned on the waiting pass and no Core wait occurs under the BQL.
        cpu_loop_exit_restore(cpu, cpu->mem_io_pc);
    }
    if (!vnext_request_ready[lane] ||
        qatomic_load_acquire(&slot->requestSeq) != vnext_request_seq[lane] ||
        qatomic_load_acquire(&slot->responseSeq) != vnext_request_seq[lane]) {
        cpu_stop_current();
        cpu_loop_exit_restore(cpu, cpu->mem_io_pc);
    }
    uint64_t value = 0;
    if (slot->bytes >= sizeof(value)) memcpy(&value, slot->payload, sizeof(value));
    vnext_request_ready[lane] = false;
    qatomic_store_release(&slot->requestSeq, 0);
    return value;
}

static void vnext_publish_snapshot(uint64_t value) {
    VnextSnapshot *s = (VnextSnapshot *)(vnext_base + vnext_control->snapshotOffset);
    const uint64_t seq = qatomic_load_acquire(&s->publishSeq);
    qatomic_store_release(&s->publishSeq, seq | 1);
    for (unsigned i = 0; i < 8; ++i) {
        const uint64_t word = value + i;
        qatomic_store_release((uint64_t *)(s->data + i * sizeof(uint64_t)), word);
    }
    qatomic_store_release(&s->publishSeq, (seq | 1) + 1);
    SetEvent(vnext_artifact_event);
}

static void vnext_consume_c2a(void) {
    VnextControl *c = vnext_control;
    VnextRegion *descriptor = (VnextRegion *)(vnext_base + c->c2aDescriptorOffset);
    VnextRegion *metadata = descriptor;
    VnextRegion *events = descriptor + 1;
    uint64_t *meta = (uint64_t *)(vnext_base + metadata->offset);
    const uint64_t write = qatomic_load_acquire(&meta[0]);
    uint64_t read = qatomic_load_acquire(&meta[1]);
    while (read != write) {
        VnextEvent *event = (VnextEvent *)(vnext_base + events->offset) + (read & (events->count - 1));
        /* Generic C2A byte delivery: payload is deliberately opaque to the
         * transport; the UART model owns its interpretation and FIFO state. */
        if (event->kind == 2 && event->bytes >= 1)
            esp32_uart_vnext_rx_byte(0, event->payload[0]);
        if (event->kind == 10 && event->bytes >= sizeof(uint64_t) * 2) {
            uint64_t address = 0, value = 0;
            memcpy(&address, event->payload, sizeof(address));
            memcpy(&value, event->payload + sizeof(address), sizeof(value));
            unsigned bus = 0; bool status = false;
            if (vnext_i2c_address(address, &bus, &status)) {
                if (status) vnext_i2c_status[bus] = value;
                else vnext_i2c_cmd[bus] = value;
            }
        }
        qatomic_fetch_add(&vnext_control->artifactProgress, 1);
        read++;
    }
    qatomic_store_release(&meta[1], read);
}

static bool vnext_valid(void) {
    if (!vnext_control || vnext_control->magic != VNEXT_MAGIC || vnext_control->major != 1 ||
        !vnext_control->execution || !vnext_control->laneCount || vnext_control->laneCount > VNEXT_LANES ||
        vnext_control->endpointCount != 1 || vnext_control->snapshotCount > 1 ||
        vnext_control->descriptorCapacity < 1) return false;
    if (vnext_control->descriptorOffset + sizeof(VnextEndpoint) > vnext_control->bytes ||
        vnext_control->laneDescriptorOffset + vnext_control->laneCount * sizeof(VnextLane) > vnext_control->bytes ||
        (vnext_control->snapshotCount != 0 && vnext_control->snapshotOffset + sizeof(VnextSnapshot) > vnext_control->bytes)) return false;
    VnextLane *lanes = (VnextLane *)(vnext_base + vnext_control->laneDescriptorOffset);
    for (uint32_t i = 0; i < vnext_control->laneCount; ++i) {
        if (lanes[i].depth < 2 || lanes[i].depth > VNEXT_DEPTH || (lanes[i].depth & (lanes[i].depth - 1)) ||
            lanes[i].stride != sizeof(VnextEvent) || lanes[i].events.stride != sizeof(VnextEvent) ||
            lanes[i].events.count != lanes[i].depth || lanes[i].events.bytes != (uint64_t)lanes[i].depth * sizeof(VnextEvent) ||
            lanes[i].events.offset + lanes[i].events.bytes > vnext_control->bytes) return false;
    }
    return true;
}

static uint32_t vnext_lane_credit(uint32_t lane) {
    VnextLane *d = (VnextLane *)(vnext_base + vnext_control->laneDescriptorOffset) + lane;
    uint64_t *meta = (uint64_t *)(vnext_base + d->metadata.offset);
    return d->depth - (uint32_t)(qatomic_load_acquire(&meta[0]) - qatomic_load_acquire(&meta[1]));
}

/* R3c heartbeat callback: publishes the current QEMU_CLOCK_VIRTUAL instant as a monotonic
 * watermark and re-arms itself. Deliberately minimal -- no per-tick log line (E099), no lane
 * event, no electrical/dispatch side effect. Signals the SAME artifact doorbell every other
 * publish already uses; a manual-reset Windows Event coalesces repeated SetEvent() calls for
 * free, so this needs no separate "coalesced" bookkeeping of its own. */
static void vnext_heartbeat_cb(void *opaque)
{
    (void)opaque;
    if (!vnext_vm_started || !vnext_heartbeat_timer) {
        return;
    }
    const uint64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    qatomic_store_release(&vnext_control->artifactVirtualTimeNs, now);
    SetEvent(vnext_artifact_event);
    timer_mod(vnext_heartbeat_timer, now + VNEXT_HEARTBEAT_INTERVAL_NS);
}

static void vnext_resume(void *opaque) {
    (void)opaque;
    if (vnext_provenance_enabled()) {
        vnext_resume_entry_count++;
        /* E147-D: the QEMU child is routinely torn down by a forceful terminate (bounded
         * diagnostic runs, defensive teardown timeouts) rather than a clean exit -- atexit()
         * never runs in that case. Emitting on power-of-two entry counts guarantees at least a
         * geometric trail of snapshots survives any kill, instead of losing every count. */
        if (vnext_is_power_of_two(vnext_resume_entry_count)) {
            vnext_provenance_print_summary();
        }
    }
    ResetEvent(vnext_core_event);
    /* E118 (EVIDENCE.md, 2026-09-05): was `artifactState >= 2`, which is also true for the FAILED
     * states (5, 6) -- meaning a fatal artifact was still treated as "running enough" to keep
     * publishing self-test GPIO writes and (had it not already happened) start the VM. FAILED is
     * numerically greater than READY/RUNNING but is not healthier than either; require exactly
     * READY(2) or RUNNING(3) instead of "at least READY". */
    const uint32_t artifact_state_now = qatomic_load_acquire(&vnext_control->artifactState);
    if (qatomic_load_acquire(&vnext_control->coreState) == 3 &&
        (artifact_state_now == 2 || artifact_state_now == 3)) {
        const bool first_core_running = artifact_state_now == 2;
        if (first_core_running) {
            qatomic_store_release(&vnext_control->artifactState, 3);
        }
        if (vnext_self_test_gpio) {
            static const uint64_t addresses[] = {
                UINT64_C(0x3FF44004), UINT64_C(0x3FF44008),
                UINT64_C(0x3FF4400C), UINT64_C(0x3FF44020),
            };
            static const uint32_t values[] = {
                UINT32_C(0xB0010001), UINT32_C(0xB0010002),
                UINT32_C(0xB0010003), UINT32_C(0xB0010004),
            };
            vnext_self_test_gpio = false;
            for (size_t index = 0; index < G_N_ELEMENTS(addresses); ++index) {
                vnext_b_gpio_write(addresses[index], values[index]);
            }
        }
        if (vnext_start_guest_on_core_ready && !vnext_vm_started) {
            vnext_vm_started = true;
            vm_start();
            if (!vnext_heartbeat_initialized) {
                vnext_heartbeat_initialized = true;
                vnext_heartbeat_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                                     vnext_heartbeat_cb, NULL);
                vnext_heartbeat_cb(NULL);
            }
        }
        if (first_core_running) {
            SetEvent(vnext_artifact_event);
        }
    }
    vnext_check_responses();
    vnext_check_i2c_responses();
    vnext_consume_c2a();
    if (vnext_vm_started) {
        /* Do not re-signal the artifact doorbell merely because this callback ran.
         * Only a real publish, response, heartbeat, or credit transition should wake Core;
         * unconditional signaling here creates a self-sustaining wake/settle storm when there
         * is no new artifact work. */
        while (vnext_self_test_writes && vnext_cpus[0]) {
            vnext_self_test_writes--;
            fprintf(stderr, "[VNEXT_B_SELFTEST] lane0 publish remaining=%u\n", vnext_self_test_writes);
            vnext_self_test_publish(vnext_cpus[0], vnext_self_test_writes);
        }
        if (vnext_self_test_lane1 && vnext_cpus[1]) {
            vnext_self_test_lane1 = false;
            fprintf(stderr, "[VNEXT_B_SELFTEST] lane1 progress\n");
            vnext_self_test_publish(vnext_cpus[1], UINT64_C(0x1001));
        }
        if (vnext_self_test_snapshot) {
            vnext_self_test_snapshot = false;
            vnext_publish_snapshot(UINT64_C(0x5000));
        }
        if (vnext_self_test_requests && !vnext_request_waiting[0] && !vnext_request_ready[0]) {
            vnext_self_test_requests = false;
            vnext_publish_request(0, 0);
        }
    }
    for (uint32_t i = 0; i < vnext_control->laneCount; ++i) {
        if (!vnext_blocked[i] && !vnext_lane_producer_backlog[i]) continue;
        const uint32_t credit = vnext_lane_credit(i);
        if (i == 0) {
            vnext_last_lane0_credit = credit;
            if (credit != 0) vnext_lane0_credit_nonzero_sweeps++;
        }
        if (credit == 0) continue;
        if (vnext_provenance_enabled()) {
            vnext_resume_found_real_work_count++;
        }
        if (vnext_blocked[i]) {
            vnext_blocked[i] = false;
            esp32_timg_transport_pause(i, false);
            if (vnext_cpus[i]) cpu_resume(vnext_cpus[i]);
        }
        /* E118: a non-vCPU producer may ALSO be backlogged on this same lane (e.g. the UART TX
         * BH and CPU0's own I2C submissions both default to lane 0) -- checked independently of
         * the vCPU-resume branch above (not an "else if"), since vnext_lane_producer_backlog[]
         * tracks a disjoint set of waiters that never touched a CPU and must be woken even when
         * vnext_blocked[i] was never true for this lane. */
        if (vnext_lane_producer_backlog[i]) {
            vnext_lane_producer_backlog[i] = false;
            esp32_timg_transport_pause(i, false);
            vnext_notify_lane_backlog_cleared(i);
        }
    }
    /* A lane can also regain credit without vnext_blocked[i] or vnext_lane_producer_backlog[i]
     * ever having been true for it in THIS sweep -- e.g. credit returned between two sweeps with
     * no intervening publish attempt to notice. The loop above only fires the notify when this
     * exact sweep observed the transition; genuinely backlogged producers are covered because
     * vnext_lane_producer_backlog[i] stays true (and is re-checked every sweep) until credit is
     * observed and cleared here. */
}

/* NOT wired for LASECSIMUL_MWDT_ACCOUNTING (Fase B) -- see the comment on
 * vnext_read() above; this is the same unreachable-in-production region. */
static void vnext_write(void *opaque, hwaddr addr, uint64_t value, unsigned size) {
    (void)opaque; (void)addr; (void)size;
    CPUState *cpu = current_cpu;
    const uint32_t lane = cpu ? cpu->cpu_index : 0;
    if (lane >= vnext_control->laneCount) return;
    VnextLane *d = (VnextLane *)(vnext_base + vnext_control->laneDescriptorOffset) + lane;
    uint64_t *meta = (uint64_t *)(vnext_base + d->metadata.offset);
    const uint64_t write = qatomic_load_acquire(&meta[0]);
    const uint64_t read = qatomic_load_acquire(&meta[1]);
    const VnextRingClassification classification = vnext_ring_classify(write, read, d->depth);
    if (classification == VNEXT_RING_FATAL) {
        /* E118: genuine corruption, distinct from ordinary full (occupancy == depth) below. */
        fprintf(stderr, "Qemu: VNEXT_B normal FULL invariant violated lane=%u write=%" PRIu64 " read=%" PRIu64 "\n", lane, write, read);
        qatomic_store_release(&vnext_control->artifactFatal, 1);
        qatomic_store_release(&vnext_control->artifactState, 5);
        SetEvent(vnext_artifact_event);
        if (cpu) cpu_stop_current();
        return;
    }
    if (classification == VNEXT_RING_WOULD_BLOCK) {
        /* E118: ordinary backpressure, never fatal. This synthetic-region handler is unreachable
         * in production (see the comment above); left as a silent drop rather than fatal, matching
         * the real producers' contract. */
        return;
    }
    VnextEvent *event = (VnextEvent *)(vnext_base + d->events.offset) + (write & (d->depth - 1));
    memset(event, 0, sizeof(*event)); event->timestamp = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    event->kind = 1; event->endpoint = 0; event->bytes = sizeof(value); memcpy(event->payload, &value, sizeof(value));
    event->sequence = write; qatomic_store_release(&meta[0], write + 1);
    SetEvent(vnext_artifact_event);
    if (vnext_lane_credit(lane) == 0) {
        /* Diagnostics must not be charged to the guest watchdog budget.
         * esp32_timg_transport_pause(lane, true) opens the compensated window
         * and the matching (lane, false) credits the elapsed interval back to
         * the MWDT count base, so anything emitted *before* the window opens
         * is uncompensated host time that the guest watchdog deadline pays
         * for.  These two writes used to sit outside it and were also
         * unconditional: one VNEXT_B session emitted 24,009 of them in 60 s,
         * and 16 sessions 215,082 -- 83% of all QEMU output.  They are now
         * opt-in and inside the window.  Diagnostic-only: no transport,
         * backpressure, scheduler or reset semantics change.
         * See orchestrator/.ai/EVIDENCE.md E102. */
        vnext_blocked[lane] = true;
        esp32_timg_transport_pause(lane, true);
        cpu_stop_current();
        if (vnext_lane_credit(lane) != 0) {
            vnext_blocked[lane] = false;
            esp32_timg_transport_pause(lane, false);
            cpu_resume(cpu);
        }
    }
}

static void vnext_self_test_publish(CPUState *cpu, uint64_t value) {
    CPUState *saved = current_cpu;
    current_cpu = cpu;
    vnext_write(NULL, 0, value, sizeof(value));
    current_cpu = saved;
}

static const MemoryRegionOps vnext_ops = { .read = vnext_read, .write = vnext_write,
                                           .endianness = DEVICE_LITTLE_ENDIAN };

int vnext_b_main(int argc, char **argv) {
    vnext_transport_ready = false;
    vnext_start_guest_on_core_ready = false;
    vnext_vm_started = false;
    vnext_heartbeat_initialized = false;
    vnext_start_paused = false;
    vnext_heartbeat_timer = NULL;
    if (argc <= 2) { fprintf(stderr, "[VNEXT_B_DIAG] reject=argc\n"); fflush(stderr); return 1; }
    const wchar_t *unused = NULL; (void)unused;
    const char *mapping_name = argv[1];
    const char *core_name = getenv("LASECSIMUL_VNEXT_B_CORE_EVENT");
    const char *artifact_name = getenv("LASECSIMUL_VNEXT_B_ARTIFACT_EVENT");
    if (!core_name || !artifact_name) {
        fprintf(stderr, "[VNEXT_B_DIAG] reject=event_environment core=%s artifact=%s\n",
                core_name ? "set" : "missing", artifact_name ? "set" : "missing");
        fflush(stderr);
        return 1;
    }
    vnext_mapping = OpenFileMappingA(FILE_MAP_ALL_ACCESS, FALSE, mapping_name);
    if (!vnext_mapping) {
        fprintf(stderr, "[VNEXT_B_DIAG] reject=open_mapping name=%s error=%lu\n",
                mapping_name, (unsigned long)GetLastError());
        fflush(stderr);
        return 1;
    }
    vnext_base = MapViewOfFile(vnext_mapping, FILE_MAP_ALL_ACCESS, 0, 0, 0);
    if (!vnext_base) {
        fprintf(stderr, "[VNEXT_B_DIAG] reject=map_view error=%lu\n", (unsigned long)GetLastError());
        fflush(stderr);
        return 1;
    }
    vnext_control = (VnextControl *)vnext_base;
    if (!vnext_valid()) {
        fprintf(stderr, "[VNEXT_B_DIAG] reject=container magic=0x%016" PRIx64
                " major=%u lanes=%u endpoints=%u bytes=%" PRIu64 "\n",
                vnext_control->magic, vnext_control->major,
                vnext_control->laneCount, vnext_control->endpointCount,
                vnext_control->bytes);
        fflush(stderr);
        return 1;
    }
    vnext_core_event = OpenEventA(EVENT_MODIFY_STATE | SYNCHRONIZE, FALSE, core_name);
    vnext_artifact_event = OpenEventA(EVENT_MODIFY_STATE | SYNCHRONIZE, FALSE, artifact_name);
    if (!vnext_core_event || !vnext_artifact_event) {
        fprintf(stderr, "[VNEXT_B_DIAG] reject=open_event core=%s artifact=%s error=%lu\n",
                vnext_core_event ? "ok" : "missing", vnext_artifact_event ? "ok" : "missing",
                (unsigned long)GetLastError());
        fflush(stderr);
        return 1;
    }
    // argv[1] is the vNext mapping name inserted by Core. Keep it as QEMU's argv[0]
    // while removing only the host executable name; qemu_init requires argv[0] before
    // parsing ordinary options.
    for (int i = 2; i < argc; ++i) {
        if (strcmp(argv[i], "-S") == 0) {
            vnext_start_paused = true;
            break;
        }
    }
    autostart = 0;
    qemu_init(argc - 1, argv + 1);
    const unsigned init_delay_ms = vnext_self_test_init_delay_ms();
    if (init_delay_ms) {
        g_usleep((gulong)init_delay_ms * 1000);
    }
    uint32_t count = 0; CPUState *cpu;
    CPU_FOREACH(cpu) {
        if (cpu->cpu_index < VNEXT_LANES) {
            vnext_cpus[cpu->cpu_index] = cpu;
            count = MAX(count, cpu->cpu_index + 1);
        }
    }
    /* 0x60000000 is the ESP32 UART0 AHB FIFO used by ESP-IDF's
     * uart_ll_write_txfifo(), not unused address space. Never shadow the
     * production peripheral with the synthetic transport test endpoint. */
    const char *synthetic_mmio = getenv("LASECSIMUL_VNEXT_B_SYNTHETIC_MMIO");
    if (synthetic_mmio && !strcmp(synthetic_mmio, "1")) {
        memory_region_init_io(&vnext_mmio, NULL, &vnext_ops, NULL, "vnext-b.synthetic-write", 0x1000);
        memory_region_add_subregion(get_system_memory(), UINT64_C(0x60000000), &vnext_mmio);
    }
    const char *self_test = getenv("LASECSIMUL_VNEXT_B_SELF_TEST_WRITES");
    vnext_self_test_writes = self_test ? (unsigned)strtoul(self_test, NULL, 10) : 0;
    vnext_self_test_lane1 = getenv("LASECSIMUL_VNEXT_B_SELF_TEST_LANE1") != NULL;
    vnext_self_test_snapshot = getenv("LASECSIMUL_VNEXT_B_SELF_TEST_SNAPSHOT") != NULL;
    vnext_self_test_gpio = getenv("LASECSIMUL_VNEXT_B_SELF_TEST_GPIO") != NULL;
    vnext_self_test_requests = getenv("LASECSIMUL_VNEXT_B_SELF_TEST_REQUESTS") != NULL;
    if (qemu_add_wait_object(vnext_core_event, vnext_resume, NULL) != 0) return 1;
    vnext_transport_ready = true;
    vnext_start_guest_on_core_ready = !vnext_start_paused;
    qatomic_store_release(&vnext_control->artifactState, 2);
    SetEvent(vnext_artifact_event);
    int status = qemu_main_loop();
    return status;
}
#else
int vnext_b_main(int argc, char **argv) { (void)argc; (void)argv; return 1; }
#endif
