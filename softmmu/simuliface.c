/***************************************************************************
 *   Copyright (C) 2025 by Santiago González                               *
 *                                                                         *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include <sys/stat.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#ifdef _WIN32
#include <windows.h>
#endif

#ifdef __linux__
#include <sys/mman.h>
#include <sys/shm.h>
//#elif defined(_WIN32)
//#include <windows.h>
#endif

#include "simuliface.h"
#include "vnext_b.h"

#include "qemu/osdep.h"
#include "qemu-main.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h"
#include "sysemu/runstate.h"
#include "sysemu/sysemu.h"
#include "sysemu/cpu-timers.h"
#include "hw/irq.h"
#include "hw/core/cpu.h"
#include "qemu/seqlock.h"
#include "qemu/atomic.h"
#include "qemu/thread.h"
#include "timers-state.h"

// ------------------------------------------------
// -------- ARENA ---------------------------------

volatile qemuArena_t* m_arena = NULL;
static volatile qemuArenaDescriptor_t *m_arenaDescriptor;
static unsigned m_arenaAbiMajor;
static uint64_t m_sessionExecutionId;
static uint64_t m_runtimeInstanceId;
static uint64_t m_launchGeneration;
static bool m_runtimeIdentityValid;
static FILE *m_qemuTraceFile;
static uint8_t *m_qemuTraceMapping;
static size_t m_qemuTraceMappingSize;
#ifdef _WIN32
static HANDLE m_qemuTraceFileHandle, m_qemuTraceMappingHandle;
#endif
static uint64_t m_qemuTraceEventSequence;
static uint64_t m_qemuTraceDropped;
static const uint64_t m_qemuTraceCapacity = 8192;
static uint64_t m_qemuTraceQpcFrequency = 1000000000ull;
static uint64_t m_qemuTraceCalls[64], m_qemuTraceTicks[64], m_qemuTraceMaxTicks[64];
static bool m_qemuPrevDurationValid;
static uint16_t m_qemuPrevPhase;
static uint64_t m_qemuPrevSequence, m_qemuPrevDuration;
static uint64_t qemuTraceQpc(void) {
#ifdef _WIN32
    LARGE_INTEGER v, f;
    QueryPerformanceCounter(&v); QueryPerformanceFrequency(&f);
    m_qemuTraceQpcFrequency = (uint64_t)f.QuadPart;
    return (uint64_t)v.QuadPart;
#else
    return get_clock();
#endif
}
static bool m_qemuTraceEnabled;

typedef struct QemuTraceBinaryRecord {
    uint64_t runId, sessionExecutionId, runtimeInstanceId, launchGeneration;
    uint64_t transactionSequence, eventSequence, dependencySequence, virtualNs, qpcTicks;
    uint32_t processId, threadId;
    uint16_t eventType, phase;
    uint32_t fifoState, irqState, timerState, waitReason;
    uint64_t durationQpc;
    uint32_t sourceId;
    uint16_t schemaPhase, reserved0;
} QemuTraceBinaryRecord;
typedef struct QemuTraceBinaryHeader {
    char magic[8]; uint32_t version, recordSize; uint64_t runId, qpcFrequency, capacity, written, dropped;
} QemuTraceBinaryHeader;

static void qemuTraceClose(void) {
#ifdef _WIN32
    if (m_qemuTraceMapping) UnmapViewOfFile(m_qemuTraceMapping);
    if (m_qemuTraceMappingHandle) CloseHandle(m_qemuTraceMappingHandle);
    if (m_qemuTraceFileHandle && m_qemuTraceFileHandle != INVALID_HANDLE_VALUE) CloseHandle(m_qemuTraceFileHandle);
#else
    if (m_qemuTraceMapping) munmap(m_qemuTraceMapping, m_qemuTraceMappingSize);
#endif
    m_qemuTraceMapping = NULL;
}

static void qemuTraceInit(void) {
    static bool initialized;
    if (initialized) return;
    initialized = true;
    const char *mode = getenv("LASECSIMUL_CAUSAL_TRACE");
    if (!mode || strcmp(mode, "detailed") != 0) return;
    const char *configuredPath = getenv("LASECSIMUL_QEMU_TRACE_PATH");
    (void)qemuTraceQpc();
    char derivedPath[192];
    const char *path = configuredPath;
    if (!path || !path[0]) {
        snprintf(derivedPath, sizeof(derivedPath), "lasecsimul-qemu-%" PRIu64 "-%" PRIu64 "-%" PRIu64 ".trace",
                 m_sessionExecutionId, m_runtimeInstanceId, m_launchGeneration);
        path = derivedPath;
    }
#ifdef _WIN32
    m_qemuTraceFileHandle = CreateFileA(path, GENERIC_READ|GENERIC_WRITE, FILE_SHARE_READ|FILE_SHARE_WRITE, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
    if (m_qemuTraceFileHandle == INVALID_HANDLE_VALUE) return;
    m_qemuTraceMappingSize = sizeof(QemuTraceBinaryHeader) + m_qemuTraceCapacity * sizeof(QemuTraceBinaryRecord);
    LARGE_INTEGER sz; sz.QuadPart = (LONGLONG)m_qemuTraceMappingSize;
    if (!SetFilePointerEx(m_qemuTraceFileHandle, sz, NULL, FILE_BEGIN) || !SetEndOfFile(m_qemuTraceFileHandle)) { qemuTraceClose(); return; }
    m_qemuTraceMappingHandle = CreateFileMappingA(m_qemuTraceFileHandle, NULL, PAGE_READWRITE, (DWORD)(sz.QuadPart>>32), (DWORD)sz.QuadPart, NULL);
    if (!m_qemuTraceMappingHandle) { qemuTraceClose(); return; }
    m_qemuTraceMapping = (uint8_t*)MapViewOfFile(m_qemuTraceMappingHandle, FILE_MAP_ALL_ACCESS, 0, 0, m_qemuTraceMappingSize);
#else
    m_qemuTraceFile = fopen(path, "w+b");
    if (!m_qemuTraceFile) return;
    m_qemuTraceMappingSize = sizeof(QemuTraceBinaryHeader) + m_qemuTraceCapacity * sizeof(QemuTraceBinaryRecord);
    int fd = fileno(m_qemuTraceFile); if (ftruncate(fd, (off_t)m_qemuTraceMappingSize) != 0) return;
    m_qemuTraceMapping = (uint8_t*)mmap(NULL, m_qemuTraceMappingSize, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
    if (m_qemuTraceMapping == MAP_FAILED) { m_qemuTraceMapping = NULL; return; }
#endif
    if (m_qemuTraceMapping) {
        QemuTraceBinaryHeader *h = (QemuTraceBinaryHeader*)m_qemuTraceMapping;
        memset(h, 0, sizeof(*h)); memcpy(h->magic, "LSCTRB2", 7); h->version=2; h->recordSize=sizeof(QemuTraceBinaryRecord);
        h->runId=m_sessionExecutionId; h->qpcFrequency=m_qemuTraceQpcFrequency; h->capacity=m_qemuTraceCapacity; m_qemuTraceEnabled=true;
    }
}

static void qemuTraceRecord(uint16_t event, uint64_t sequence, uint64_t virtualNs) {
    if (!m_qemuTraceEnabled || !m_qemuTraceMapping) return;
    if (m_qemuTraceEventSequence >= m_qemuTraceCapacity) { ++m_qemuTraceDropped; return; }
    const uint64_t begin = qemuTraceQpc();
    const uint64_t eventSequence = ++m_qemuTraceEventSequence;
    const uint64_t eventQpc = begin;
    QemuTraceBinaryRecord *r = (QemuTraceBinaryRecord*)(m_qemuTraceMapping + sizeof(QemuTraceBinaryHeader) + (m_qemuTraceEventSequence-1) * sizeof(QemuTraceBinaryRecord));
    memset(r, 0, sizeof(*r)); r->runId=m_sessionExecutionId; r->sessionExecutionId=m_sessionExecutionId; r->runtimeInstanceId=m_runtimeInstanceId; r->launchGeneration=m_launchGeneration; r->transactionSequence=sequence; r->eventSequence=eventSequence; r->virtualNs=virtualNs; r->qpcTicks=eventQpc; r->eventType=event; r->sourceId=2;
    if (event < 64) { const uint64_t elapsed = qemuTraceQpc() - begin; ++m_qemuTraceCalls[event]; m_qemuTraceTicks[event] += elapsed; if (elapsed > m_qemuTraceMaxTicks[event]) m_qemuTraceMaxTicks[event] = elapsed; m_qemuPrevDurationValid = true; m_qemuPrevPhase = event; m_qemuPrevSequence = sequence; m_qemuPrevDuration = elapsed; }
}

typedef enum RuntimeIdentityStartupState {
    RUNTIME_IDENTITY_NONE = 0,
    RUNTIME_IDENTITY_COMPLETE_VALID,
    RUNTIME_IDENTITY_MANAGED_INVALID
} RuntimeIdentityStartupState;

static bool parseIdentityU64Value(const char *value, uint64_t *out) {
    if (!value || !value[0]) return false;
    for (const unsigned char *p = (const unsigned char *)value; *p; ++p) {
        if (*p < '0' || *p > '9') return false;
    }
    errno = 0;
    char *end = NULL;
    unsigned long long parsed = strtoull(value, &end, 10);
    if (errno == ERANGE || end == value || *end != '\0') return false;
    *out = (uint64_t)parsed;
    return true;
}

static RuntimeIdentityStartupState loadRuntimeIdentityFromEnvironment(void) {
    const char *s = getenv("LASECSIMUL_SESSION_EXECUTION_ID");
    const char *r = getenv("LASECSIMUL_RUNTIME_INSTANCE_ID");
    const char *g = getenv("LASECSIMUL_LAUNCH_GENERATION");
    if (!s && !r && !g) {
        fprintf(stderr, "Qemu: runtime identity metadata absent (standalone mode)\n");
        return RUNTIME_IDENTITY_NONE;
    }
    if (!parseIdentityU64Value(s, &m_sessionExecutionId) ||
        !parseIdentityU64Value(r, &m_runtimeInstanceId) ||
        !parseIdentityU64Value(g, &m_launchGeneration) ||
        m_sessionExecutionId == 0 || m_launchGeneration == 0) {
        fprintf(stderr, "Qemu: invalid managed runtime identity metadata; startup rejected\n");
        return RUNTIME_IDENTITY_MANAGED_INVALID;
    }
    m_runtimeIdentityValid = true;
    qemuTraceInit();
    return RUNTIME_IDENTITY_COMPLETE_VALID;
}
#ifdef _WIN32
static HANDLE m_pollDoorbell;
static uint64_t m_doorbellSetAttempts;
static uint64_t m_doorbellSetSuccesses;
static uint64_t m_doorbellSetFailures;
#endif

static void signalPollDoorbell(void)
{
#ifdef _WIN32
    if (!m_pollDoorbell) return;
    ++m_doorbellSetAttempts;
    if (SetEvent(m_pollDoorbell)) ++m_doorbellSetSuccesses;
    else if (++m_doorbellSetFailures <= 3) {
        fprintf(stderr, "Qemu: SetEvent(I2C doorbell) failed: %lu\n",
                (unsigned long)GetLastError());
    }
#endif
}

// ------------------------------------------------

static int configuredArenaAbiMajor(void)
{
    const char *value = getenv("LASECSIMUL_QEMU_ARENA_VERSION");

    if (!value || !value[0] || strcmp(value, "5") == 0) {
        return QEMU_ARENA_ABI_MAJOR;
    }
    if (strcmp(value, "3") == 0) {
        return 3;
    }
    fprintf(stderr,
            "Qemu: invalid LASECSIMUL_QEMU_ARENA_VERSION='%s'; expected 3 or 5\n",
            value);
    return -1;
}

static bool validateArenaV5Descriptor(
    volatile qemuArenaDescriptor_t *descriptor)
{
    const uint64_t coreReady = qatomic_load_acquire(&descriptor->coreReady);

    if (!coreReady) {
        fprintf(stderr, "Qemu: arena ABI v5 descriptor is not ready\n");
        return false;
    }
    if (descriptor->magic != QEMU_ARENA_ABI_MAGIC ||
        descriptor->abiMajor != QEMU_ARENA_ABI_MAJOR ||
        descriptor->descriptorSize != sizeof(qemuArenaDescriptor_t) ||
        descriptor->arenaSize != sizeof(qemuArenaV5Mapping_t) ||
        descriptor->transportSize != sizeof(qemuArena_t) ||
        descriptor->queueDepth != QEMU_ARENA_QUEUE_DEPTH) {
        fprintf(stderr,
                "Qemu: incompatible arena ABI v5 descriptor:"
                " magic=0x%016" PRIx64 " version=%u.%u"
                " descriptor=%" PRIu64 " arena=%" PRIu64
                " transport=%" PRIu64 " queue=%" PRIu64 "\n",
                descriptor->magic, descriptor->abiMajor,
                descriptor->abiMinor, descriptor->descriptorSize,
                descriptor->arenaSize, descriptor->transportSize,
                descriptor->queueDepth);
        return false;
    }

    const uint64_t required = QEMU_ARENA_REQUIRED_CAPABILITIES;
    const uint64_t negotiated =
        descriptor->coreCapabilities & QEMU_ARENA_CAPABILITIES;
    if ((negotiated & required) != required) {
        fprintf(stderr,
                "Qemu: arena ABI v5 lacks required capabilities:"
                " core=0x%016" PRIx64 " required=0x%016" PRIx64 "\n",
                descriptor->coreCapabilities, required);
        return false;
    }
    descriptor->qemuCapabilities = QEMU_ARENA_CAPABILITIES;
    descriptor->negotiatedCapabilities = negotiated;
    return true;
}

uint64_t m_lastQemuTime;

uint64_t period_ns;
uint64_t period_ps;

QEMUTimer* qtimer;
/*
 * With icount QEMU runs both ESP32 vCPUs on one round-robin TCG thread.  MTTCG
 * gives each vCPU its own host thread, so releasing the BQL while waiting for
 * Core turns the arena into a multi-producer channel.  The ABI still has one
 * write cursor and one synchronous read slot; serialize complete arena
 * transactions here instead of changing that stable cross-process layout.
 *
 * Lock order is arena -> BQL.  Callers normally arrive holding the BQL, so
 * arenaTransactionBegin() drops it before acquiring this mutex.  Most arena
 * operations keep it dropped for concurrency.  A synchronous operation issued
 * from a device MMIO callback must, however, reacquire the BQL after acquiring
 * the arena lock: otherwise MTTCG can enter the same MemoryRegion from the
 * other vCPU while the first callback is waiting for Core.  QEMU correctly
 * rejects that re-entrant access, but the guest then loses a real FIFO/register
 * write.  Acquiring arena first preserves the established lock order.
 *
 * Investigated 2026-08-28 (TG0WDT_SYS_RESET investigation): this release-then-reacquire dance
 * still leaves a narrow window with the BQL released while the originating MemoryRegion's
 * dispatch (e.g. esp_soc.uart, nested via uart_send_next() -> writeReg()) is still considered
 * "active" by QEMU's reentrancy guard -- source-confirmed as the mechanism behind observed
 * "Blocked re-entrant IO" warnings that freeze the guest's virtual-time progress. A fix that kept
 * the BQL held straight through the arena-lock acquisition (instead of dropping and reacquiring
 * it) was attempted and reverted: it produced a reproducible ~36-minute hang on the very first
 * validation run. The lock-order proof for that variant did not account for every real contender
 * for m_arenaOrderLock while BQL is held continuously; root cause of the hang not yet confirmed.
 * Do not reattempt that specific variant without first identifying the missing contended path.
 */
static QemuMutex m_arenaOrderLock;
static bool m_arenaOrderLockInitialized;
static bool m_profileEnabled;
static uint64_t m_profileStartWallNs;
static uint64_t m_profileStartVirtualNs;
static uint64_t m_profileLastReportWallNs;
static uint64_t m_profilePublishedEvents;
static uint64_t m_profileReadTransactions;
static uint64_t m_profileQueueWaits;
static uint64_t m_profileReadWaits;
static uint64_t m_profileMaxQueueOccupancy;

typedef struct ArenaTransaction {
    bool restoreIothreadLock;
    bool iothreadLockReacquired;
} ArenaTransaction;

/* REVERTED 2026-08-28: the "hold BQL through the arena-lock acquisition" variant caused a
 * reproducible ~36-minute hang on the very first validation run (frozen: 0 further output past
 * "simulacao iniciada" for the whole window) -- worse than the race it was meant to close. The
 * lock-order/no-new-cycle proof in the removed comment did not account for every real contender
 * for m_arenaOrderLock while BQL is held continuously; root cause of the hang not yet confirmed.
 * Restored to the prior release-then-reacquire behavior pending further investigation. */
static ArenaTransaction arenaTransactionBegin(bool serializeDeviceMmio)
{
    ArenaTransaction transaction = {
        .restoreIothreadLock = qemu_mutex_iothread_locked(),
        .iothreadLockReacquired = false,
    };
    if (transaction.restoreIothreadLock) {
        qemu_mutex_unlock_iothread();
    }
    qemu_mutex_lock(&m_arenaOrderLock);
    if (transaction.restoreIothreadLock && serializeDeviceMmio) {
        qemu_mutex_lock_iothread();
        transaction.iothreadLockReacquired = true;
    }
    return transaction;
}

static void arenaTransactionEnd(ArenaTransaction transaction)
{
    qemu_mutex_unlock(&m_arenaOrderLock);
    if (transaction.restoreIothreadLock && !transaction.iothreadLockReacquired) {
        qemu_mutex_lock_iothread();
    }
}

/*
 * QEMU_CLOCK_VIRTUAL selects icount_get_ns() while -icount is active and the
 * normal monotonic VM clock otherwise.  Calling icount_get_ns() directly in
 * MTTCG leaves the bridge timestamp at zero because instruction counting is
 * disabled, which makes GPIO/heartbeat publication appear permanently stuck.
 */
static uint64_t simuClockNs(void)
{
    const int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    return now > 0 ? (uint64_t)now : 0;
}

static void profileMaybeReport(uint64_t virtualNs)
{
    if (!m_profileEnabled) {
        return;
    }

    const uint64_t wallNs = get_clock();
    if (wallNs - m_profileLastReportWallNs < NANOSECONDS_PER_SECOND) {
        return;
    }

    const uint64_t elapsedWallNs = wallNs - m_profileStartWallNs;
    const uint64_t elapsedVirtualNs = virtualNs >= m_profileStartVirtualNs
                                          ? virtualNs - m_profileStartVirtualNs
                                          : 0;
    const double realtimePercent = elapsedWallNs
                                       ? 100.0 * (double)elapsedVirtualNs / (double)elapsedWallNs
                                       : 0.0;
    printf("[LasecSimul][PROFILE] mode=%s wall_ns=%llu virtual_ns=%llu "
           "realtime_percent=%.2f events=%llu reads=%llu queue_waits=%llu "
           "read_waits=%llu max_queue=%llu\n",
           icount_enabled() ? "deterministic-icount" : "mttcg-realtime",
           (unsigned long long)elapsedWallNs,
           (unsigned long long)elapsedVirtualNs,
           realtimePercent,
           (unsigned long long)m_profilePublishedEvents,
           (unsigned long long)m_profileReadTransactions,
           (unsigned long long)m_profileQueueWaits,
           (unsigned long long)m_profileReadWaits,
           (unsigned long long)m_profileMaxQueueOccupancy);
    fflush(stdout);
    m_profileLastReportWallNs = wallNs;
}

static void pushQueueEntry( uint64_t addr, uint64_t data, uint64_t action, uint64_t simuTimePs );


/* [FIX] queue-full correctness (2026-08-28): returns false only on a genuine
 * waitForSynch() HOST PROCESS-HEALTH BACKSTOP timeout (Core has stopped draining the queue for
 * longer than any legitimate backpressure catch-up could take -- see waitForSynch()). On false,
 * this function does NOT push an entry, does NOT increment queueWriteIndex, and does NOT signal
 * the doorbell -- it only unwinds the transaction (arenaTransactionEnd(), exactly once, restoring
 * the caller's original BQL state) and returns. Callers (writeReg()/writeSimEvent()/simu_event())
 * treat false as terminal: publication failure must never become a silently dropped guest/device
 * write. This is what makes `queueWriteIndex - queueReadIndex > 32` structurally unreachable
 * through this path -- pushQueueEntry() is only ever reached when waitForSynch() has just proven
 * the queue is not full. */
static bool publishQueueEntry( uint64_t addr, uint64_t data, uint64_t action,
                               bool serializeDeviceMmio )
{
    const uint64_t entryHostNs = get_clock();
    const uint64_t entryVirtualNs = simuClockNs();
    ArenaTransaction transaction = arenaTransactionBegin(serializeDeviceMmio);
    const uint64_t bqlReacquireHostNs = get_clock();
    const bool willBackpressureWait = m_arena &&
        (m_arena->queueWriteIndex - qatomic_load_acquire(&m_arena->queueReadIndex) >= QEMU_ARENA_QUEUE_DEPTH);
    if (!waitForSynch()) {
        arenaTransactionEnd(transaction);
        return false;
    }
    /*
     * Timestamp after acquiring the ordering mutex.  If each vCPU sampled time
     * before serialization, thread B could publish its later timestamp first
     * and thread A then append an older event behind it.
     */
    g_assert(m_arena->queueWriteIndex - qatomic_load_acquire(&m_arena->queueReadIndex) < QEMU_ARENA_QUEUE_DEPTH);
    pushQueueEntry(addr, data, action, simuClockNs() * 1000);
    signalPollDoorbell();
    arenaTransactionEnd(transaction);

    if (m_arena->irqNumber) {
        setInterrupt();
    }
    return true;
}

/* v3 (LasecSimul PERF-13): publica uma entrada na fila circular de escritas/heartbeat -- só
 * escreve os campos da entrada e incrementa queueWriteIndex por ÚLTIMO (é isso que torna a
 * entrada visível pro Core, mesmo princípio de "confirma por último" que simuTime != 0 usava
 * em v2 pro slot único). Chamador já garantiu que há espaço (ver waitForSynch()).
 *
 * Achado 2026-07-22 (usuário reporta simulação travando por completo depois de rodar por um
 * tempo, indicador de velocidade continua marcando 100%): `queueWriteIndex` é lido pelo CORE, um
 * PROCESSO SEPARADO -- sem uma store de liberação aqui (pareada com o load de aquisição em
 * QemuArenaBridge::poll(), lado C++), nada no padrão de memória do C impede o compilador ou o
 * hardware de tornar o incremento de `queueWriteIndex` visível pro Core ANTES dos campos da
 * entrada (`regAddr`/`regData`/`simuAction`/`simuTime`) acima -- o Core então leria uma entrada
 * "publicada" com campos ainda não escritos/parcialmente visíveis, potencialmente disparando o
 * branch de overflow de `qemuEventTimeNs()` (Core, McuComponent.cpp) e travando aquela entrada --
 * e tudo atrás dela na fila de 32 -- pra sempre. `qatomic_store_release` garante a ordem: todo
 * store ACIMA desta linha fica visível para qualquer leitor que faça um load de aquisição do
 * mesmo `queueWriteIndex` ANTES de ver este incremento. */
static void pushQueueEntry( uint64_t addr, uint64_t data, uint64_t action, uint64_t simuTimePs )
{
    const uint64_t writeIndex = m_arena->queueWriteIndex;
    const uint64_t slot = writeIndex % QEMU_ARENA_QUEUE_DEPTH;
    m_arena->queue[slot].regAddr    = addr;
    m_arena->queue[slot].regData    = data;
    m_arena->queue[slot].simuAction = action;
    m_arena->queue[slot].simuTime   = simuTimePs;
    qatomic_store_release(&m_arena->queueWriteIndex, writeIndex + 1);

    ++m_profilePublishedEvents;
    const uint64_t occupancy =
        writeIndex + 1 - qatomic_load_acquire(&m_arena->queueReadIndex);
    if (occupancy > m_profileMaxQueueOccupancy) {
        m_profileMaxQueueOccupancy = occupancy;
    }
    profileMaybeReport(simuTimePs / 1000);

    const char *trace = getenv("LASECSIMUL_TRACE_GPIO");
    if (trace && trace[0] && strcmp(trace, "0") != 0 &&
        (addr == 0x3ff44004 || addr == 0x3ff44020 || addr == 0x3ff49038)) {
        printf("[LasecSimul][QUEUE_PUSH] index=%llu slot=%llu cpu=%d bql=%s "
               "addr=0x%08llx data=0x%08llx event_ps=%llu\n",
               (unsigned long long)writeIndex, (unsigned long long)slot,
               current_cpu ? current_cpu->cpu_index : -1,
               qemu_mutex_iothread_locked() ? "yes" : "no",
               (unsigned long long)addr, (unsigned long long)data,
               (unsigned long long)simuTimePs);
        fflush(stdout);
    }
}

/* v3: espera a fila de escritas/heartbeat esvaziar de vez (queueReadIndex alcança
 * queueWriteIndex) -- só readReg() chama isto, pra preservar a ordem leitura-depois-de-escrita
 * sem colocar a própria leitura na fila (ela precisa de um valor de volta, a fila não).
 *
 * `queueReadIndex` é escrito pelo CORE (outro processo, ver QemuArenaBridge::acknowledgeWrite())
 * -- load de aquisição aqui pareia com a store de liberação de lá, garantindo que este processo
 * enxergue o avanço assim que ele acontece, em vez de potencialmente reutilizar um valor
 * cacheado/reordenado (mesmo raciocínio de pushQueueEntry() acima, sentido inverso). */
static void waitForQueueDrain( void )
{
    if( qatomic_load_acquire(&m_arena->queueReadIndex) == m_arena->queueWriteIndex ) return;

    uint64_t timeout = 0;

    while( qatomic_load_acquire(&m_arena->queueReadIndex) != m_arena->queueWriteIndex )
    {
        if( timeout++ > 5e9 ) // Terminate process if timed out
        {
            printf("Qemu: waitForQueueDrain TIMEOUT\n"); fflush( stdout );
            return;
        }
    }
}

uint64_t readReg( uint64_t addr )
{
    if (vnext_b_active()) {
        return vnext_b_register_read(addr);
    }
    /* Standard QEMU mode has no SimulIDE shared-memory arena.  MMIO hooks must
     * remain harmless there instead of dereferencing NULL (0xC0000005 on
     * Windows as soon as Arduino touches GPIO). */
    if (!m_arena) {
        return 0;
    }

    /* v3: leitura sempre retorna só depois de o Core confirmar (nunca deixa m_arena->simuTime
     * pendente de uma chamada pra outra -- diferente de escritas, que são "dispara e esquece"),
     * então não precisa esperar uma leitura ANTERIOR terminar -- só precisa esperar toda
     * escrita/heartbeat pendente NA FILA já ter sido processada (mesmo papel que waitForSynch()
     * tinha em v2 pro slot único, agora dividido porque escritas não usam mais esse slot). */
    /* Every read is synchronous guest MMIO dispatch.  Keep the originating
     * MemoryRegion serialized while waiting for Core, matching the protected
     * I2C burst mailbox path. */
    const uint64_t entryHostNs = get_clock();
    ArenaTransaction transaction = arenaTransactionBegin(true);
    const uint64_t bqlReacquireHostNs = get_clock();
    uint64_t now = simuClockNs();
    ++m_profileReadTransactions;
    m_lastQemuTime = now;
    if( now != 0 )
    {
        waitForQueueDrain();
    }

    m_arena->regAddr    = addr;
    //m_arena->regData    = 0;
    qatomic_store_release(&m_arena->qemuAction, 0);
    m_arena->simuAction = SIM_READ;
    qatomic_store_release(&m_arena->simuTime, simuClockNs()*1000);

    uint64_t timeout = 0;
    bool timedOut = false;
    if( !qatomic_load_acquire(&m_arena->qemuAction) )  // Wait for SimulIDE to execute Read
    {
        ++m_profileReadWaits;
        while( !qatomic_load_acquire(&m_arena->qemuAction) )
        {
            if( timeout++ > 5e9 ) // Terminate process if timed out
            {
                printf("Qemu: readReg TIMEOUT %llu\n", (unsigned long long)addr); fflush( stdout );
                timedOut = true;
                break;
            }
        }
    }
    const uint64_t qemuAction = qatomic_load_acquire(&m_arena->qemuAction);
    const uint64_t regData = m_arena->regData;
    arenaTransactionEnd(transaction);

    if (timedOut) {
        return 0;
    }
    if( qemuAction != SIM_READ )
    {
        printf("Qemu: readReg m_arena->qemuAction != SIM_READ\n"); fflush( stdout );
        return 0;
    }

    if( m_arena->irqNumber ) setInterrupt();

    return regData;
}

/* [FIX] queue-full correctness (2026-08-28): publishQueueEntry() returning false means Core has
 * genuinely stopped draining (HOST PROCESS-HEALTH BACKSTOP, not ESP32 timing) -- treat it as
 * terminal, never as a silently dropped register write. qemu_system_shutdown_request() is safe to
 * call here regardless of BQL state (it acquires no lock itself; see publishQueueEntry(), which
 * has already fully unwound arenaTransactionEnd() before returning false) -- it only flags a
 * request that this fork's own qemu_main_loop() (softmmu/runstate.c) picks up on its next
 * iteration for an orderly shutdown, not an abrupt exit from inside this callback. */
VnextPublishResult writeReg( uint64_t addr, uint64_t value )
{
    if (vnext_b_active()) {
        return vnext_b_gpio_write(addr, value);
    }
    if (!m_arena) {
        return VNEXT_PUBLISHED;
    }
    //printf("Qemu: esp32_gpio_write\n"); fflush( stdout );
    if (!publishQueueEntry( addr, value, SIM_WRITE, true )) {
        qemu_system_shutdown_request(SHUTDOWN_CAUSE_HOST_ERROR);
    }
    return VNEXT_PUBLISHED;
}

void writeSimEvent( uint64_t addr, uint64_t value, uint64_t action )
{
    if (!m_arena) {
        return;
    }
    if (!publishQueueEntry(addr, value, action, true)) {
        qemu_system_shutdown_request(SHUTDOWN_CAUSE_HOST_ERROR);
    }
}

static uint64_t m_i2cRequestCounter;

/* Mailbox de burst I2C (ABI 5, ver simuliface.h/qemu_arena_abi.h). Diferente de writeReg()/readReg()
 * (que espelham registradores individuais do periferico), aqui o pedido inteiro (endereco+dados,
 * ate 32 bytes) atravessa a arena numa unica viagem de ida-e-volta -- e' o Core quem decide, pela
 * topologia do circuito, se ha' um dispositivo conectado que suporta o protocolo (`handled=true`)
 * ou se o chamador deve cair de volta pro caminho eletrico byte-a-byte existente. Mesmo esquema de
 * sequencia monotona nao-zero usado por queueWriteIndex/queueReadIndex: o QEMU publica todos os
 * campos e por ultimo i2cRequestSeq; o Core responde e por ultimo publica i2cResponseSeq. */
bool i2cBurstTransfer( uint32_t bus, const I2cBurstRequest* req, I2cBurstResponse* resp, uint8_t* rxOut )
{
    memset( resp, 0, sizeof(*resp) );
    resp->first_nack = UINT32_MAX;

    if( !m_arena || m_arenaAbiMajor != QEMU_ARENA_ABI_MAJOR ||
        !m_arenaDescriptor ||
        !(m_arenaDescriptor->negotiatedCapabilities & QEMU_ARENA_CAP_I2C_BURST) ) return false;
    if( req->tx_len > 64 || req->rx_len > 32 ) return false;

    /* Called synchronously from esp32.i2c MMIO.  Reacquire the BQL only after
     * the arena lock so another MTTCG vCPU cannot re-enter this device while
     * the mailbox request is in flight. */
    const uint64_t entryHostNs = get_clock();
    const uint64_t entryVirtualNs = simuClockNs();
    ArenaTransaction transaction = arenaTransactionBegin(true);
    const uint64_t bqlReacquireHostNs = get_clock();

    /* Mantém ordem total com GPIO-matrix/IOMUX e demais writes já publicados. Sem esta barreira o
     * Core poderia resolver o burst antes de aplicar o roteamento de SDA/SCL que o precedeu. */
    waitForQueueDrain();

    m_arena->i2cTimePs   = simuClockNs() * 1000;
    m_arena->i2cBus      = bus;
    m_arena->i2cFlags    = req->flags;
    m_arena->i2cPeriodNs = req->period_ns;
    m_arena->i2cTxLen    = req->tx_len;
    m_arena->i2cRxLen    = req->rx_len;
    if( req->tx_len ) memcpy( (void*)m_arena->i2cTx, req->tx, req->tx_len );

    const uint64_t seq = ++m_i2cRequestCounter;
    const uint64_t waitStartWallNs = get_clock();
    qemuTraceRecord(20, seq, simuClockNs()); /* T0: payload ready, before publication */
    qatomic_store_release( &m_arena->i2cRequestSeq, seq );
    qemuTraceRecord(21, seq, simuClockNs()); /* T1: authoritative request publication */
    signalPollDoorbell();

    uint64_t timeout = 0;
    bool timedOut = false;
    while( qatomic_load_acquire(&m_arena->i2cResponseSeq) != seq )
    {
        if( timeout++ > 5e9 ) // Terminate process if timed out
        {
            printf("Qemu: i2cBurstTransfer TIMEOUT\n"); fflush( stdout );
            timedOut = true;
            break;
        }
    }

    if (!timedOut) qemuTraceRecord(23, seq, simuClockNs()); /* T5: completion observed */
    static const char* s_i2cBurstTrace = NULL;
    static bool s_i2cBurstTraceRead = false;
    if( !s_i2cBurstTraceRead )
    {
        s_i2cBurstTrace = getenv("LASECSIMUL_I2C_BURST_TRACE");
        s_i2cBurstTraceRead = true;
    }
    if( s_i2cBurstTrace && s_i2cBurstTrace[0] && strcmp(s_i2cBurstTrace, "0") != 0 )
    {
        const uint64_t waitNs = get_clock() - waitStartWallNs;
        printf("[LasecSimul][I2C burst] seq=%llu waitWallNs=%llu spins=%llu tx=%u rx=%u timedOut=%d\n",
               (unsigned long long)seq, (unsigned long long)waitNs, (unsigned long long)timeout,
               req->tx_len, req->rx_len, timedOut ? 1 : 0);
        fflush(stdout);
    }

    if( !timedOut )
    {
        resp->handled     = (m_arena->i2cStatus & 1u) != 0;
        resp->address_ack = (m_arena->i2cStatus & 2u) != 0;
        resp->first_nack  = m_arena->i2cFirstNack;
        resp->rx_len      = m_arena->i2cRxLen > 32 ? 32 : m_arena->i2cRxLen;
        resp->stretch_ns  = m_arena->i2cStretchNs;
        if( resp->handled && resp->rx_len && rxOut )
            memcpy( rxOut, (const void*)m_arena->i2cRx, resp->rx_len );
    }
    arenaTransactionEnd(transaction);

    if( m_arena->irqNumber ) setInterrupt();

    return !timedOut && resp->handled;
}

void updtCpuFreqHz( uint32_t clock_Hz )
{
    uint64_t now = simuClockNs();

    double clock_MHz = (double)clock_Hz/1000000;
    double ps_instr = 1000000/clock_MHz;
    if (!m_arena) {
        period_ns = 50000;
        period_ps = 1000 * period_ns;
        return;
    }
    if( m_arena->ps_per_inst == ps_instr ) return;
    m_arena->ps_per_inst = ps_instr;

    period_ns = 50000; //ps_instr;
    period_ps = 1000*period_ns;
    m_arena->loop_timeout_ns = ps_instr/10;

    if( now == 0 ){
        printf("Qemu: Timer period: %llu ns loop period: %lld ns\n", (unsigned long long)period_ns,
               (long long)m_arena->loop_timeout_ns );
        printf("Qemu: CPU freq: %f MHz at %llu\n", clock_MHz, (unsigned long long)now );
    }
}

uint64_t getQemu_ps(void)
{
    return simuClockNs()*1000;
}

uint64_t getQemu_ns(void)
{
    return simuClockNs();
}

/* [FIX] queue-full correctness (2026-08-28): the fatal bound below is a HOST PROCESS-HEALTH
 * BACKSTOP, not ESP32/guest timing -- it exists solely to detect a genuinely unresponsive Core
 * (crashed, wedged) and fail loudly instead of silently corrupting the ring (see the caller,
 * publishQueueEntry(), for what a `false` return here now prevents). It must never be reached in
 * healthy operation: derived with a large safety margin over the largest legitimate backpressure
 * wait observed/documented in this investigation (Scheduler's own pacing wait-loop granularity is
 * 5ms, McuComponent.cpp documents a real MTTCG boot-burst gap of ~10.8ms, diagnostic runs in this
 * investigation observed tens-to-a-few-hundred ms) -- ~15-30x that ceiling. Host monotonic time
 * (get_clock(), already used throughout this file for host-elapsed measurement -- see
 * bqlCausalRecord/i2cBurstTransfer's waitWallNs above) on purpose, not QEMU_CLOCK_VIRTUAL: virtual
 * time may not be progressing during exactly this failure mode. */
#define LASECSIMUL_QUEUE_FULL_FATAL_TIMEOUT_NS (UINT64_C(3) * NANOSECONDS_PER_SECOND)

bool waitForSynch(void)
{
    if (!m_arena) {
        return true;
    }
    //printf("Qemu: wait for Action at time %lu\n",  m_lastQemuTime ); fflush( stdout );

    uint64_t now = simuClockNs();
    m_lastQemuTime = now;
    if( now == 0 ) return true;

    /* v3: só espera quando a fila de escritas/heartbeat está CHEIA -- backpressure explícito, não
     * mais um ping-pong completo a cada chamada (protocolo v2 tinha 1 slot só, sempre esperava o
     * anterior confirmar antes de publicar o próximo).
     *
     * `queueReadIndex` é escrito pelo Core (outro processo) -- load de aquisição, mesmo raciocínio
     * de waitForQueueDrain() acima. `queueWriteIndex` só é escrito por ESTE processo (QEMU), então
     * uma leitura simples basta pro lado de cá. */
    if( m_arena->queueWriteIndex - qatomic_load_acquire(&m_arena->queueReadIndex) >= QEMU_ARENA_QUEUE_DEPTH )
    {
        ++m_profileQueueWaits;
        const uint64_t waitStartHostNs = get_clock();
        while( m_arena->queueWriteIndex - qatomic_load_acquire(&m_arena->queueReadIndex) >= QEMU_ARENA_QUEUE_DEPTH )
        {
            if( get_clock() - waitStartHostNs >= LASECSIMUL_QUEUE_FULL_FATAL_TIMEOUT_NS )
            {
                printf("Qemu: waitForSynch TIMEOUT (fila cheia, HOST PROCESS-HEALTH BACKSTOP) at time %llu\n",
                       (unsigned long long)(now*1000) ); fflush( stdout );
                return false;
            }
        }
    }
    return true;
}

static void simu_event( void* opaque )
{
    if (!m_arena) return;
    if( !m_arena->running ) return;

    uint64_t now_ns = simuClockNs();

    //printf("Qemu: simu_event at %lu\n", now_ns ); fflush( stdout );

    if( now_ns > m_lastQemuTime )
    {
        /* Timer heartbeat is not inside a device MMIO dispatch, so holding the
         * BQL through queue backpressure would add contention without guarding
         * a MemoryRegion callback. */
        // heartbeat nao usa regAddr/regData
        if (!publishQueueEntry( 0, 0, SIM_EVENT, false )) {
            /* [FIX] queue-full correctness (2026-08-28) -- see writeReg()/writeSimEvent(). */
            qemu_system_shutdown_request(SHUTDOWN_CAUSE_HOST_ERROR);
        }

        m_lastQemuTime = now_ns;
    }
    else if( m_arena->irqNumber ) setInterrupt();

    //printf("Qemu: simu_event next %lu\n", now_ns+period_ns ); fflush( stdout );

    timer_reload_ns( qtimer, now_ns+period_ns );
    //timer_mod_ns( qtimer, now_ns+period_ns );
}

int simuMain( int argc, char** argv )
{
    if (getenv("LASECSIMUL_TRANSPORT") &&
        strcmp(getenv("LASECSIMUL_TRANSPORT"), "VNEXT_B") == 0) {
        return vnext_b_main(argc, argv);
    }
    const int arenaAbiMajor = configuredArenaAbiMajor();
    if (arenaAbiMajor < 0) {
        return 1;
    }
    const int shMemSize = arenaAbiMajor == QEMU_ARENA_ABI_MAJOR
                               ? sizeof(qemuArenaV5Mapping_t)
                               : sizeof(qemuArena_t);
    const char* shMemKey;

    if( argc > 2 ) // Check if there are any arguments
    {
        shMemKey = argv[1];
        argv = &argv[2];
        argc -= 2;
    } else {
        printf("Qemu Error: No arguments provided.\n");
        return 1;
    }

    void* arena = NULL;

#ifdef __linux__
    int shMemId = shm_open( shMemKey, O_RDWR, 0666 ); // Open the shared memory object
    if( shMemId == -1 )
    {
        printf("Qemu: Error opening arena: %s\n", shMemKey );
        return 1;
    }
    else printf("Qemu: arena ok: %s\n", shMemKey );
    arena = mmap( 0, shMemSize, PROT_READ | PROT_WRITE, MAP_SHARED, shMemId, 0);
#elif defined(_WIN32)
    HANDLE hMapFile = OpenFileMapping( FILE_MAP_ALL_ACCESS,  FALSE, shMemKey );

    if( hMapFile == NULL ) {
        //std::cerr << "Could not create file mapping object: " << GetLastError() << std::endl;
        return 1;
    }
    arena = MapViewOfFile( hMapFile, FILE_MAP_ALL_ACCESS, 0, 0, shMemSize );
    if (arena) {
        const size_t nameLength = strlen(shMemKey) + strlen("-doorbell") + 1;
        char *doorbellName = g_malloc(nameLength);
        snprintf(doorbellName, nameLength, "%s-doorbell", shMemKey);
        m_pollDoorbell = OpenEventA(EVENT_MODIFY_STATE | SYNCHRONIZE, FALSE, doorbellName);
        if (m_pollDoorbell) printf("Qemu: MCU poll doorbell opened: %s\n", doorbellName);
        else fprintf(stderr, "Qemu: MCU poll doorbell unavailable (%lu); timed Core fallback active\n",
                     (unsigned long)GetLastError());
        g_free(doorbellName);
    }
#endif

    if( !arena )
    {
#ifdef _WIN32
        CloseHandle(hMapFile);
#endif
        printf("Qemu: Error mapping arena\n"); fflush( stdout );
        return 1;
    }
    else printf("Qemu: arena mapped %i bytes\n", shMemSize );

    //------------------------------------------------------------------

    m_arenaAbiMajor = arenaAbiMajor;
    m_arenaDescriptor = NULL;
    if (arenaAbiMajor == QEMU_ARENA_ABI_MAJOR) {
        volatile qemuArenaV5Mapping_t *mapping =
            (volatile qemuArenaV5Mapping_t *)arena;
        m_arenaDescriptor = &mapping->descriptor;
        if (!validateArenaV5Descriptor(m_arenaDescriptor)) {
#ifdef __linux__
            munmap(arena, shMemSize);
#elif defined(_WIN32)
            if (m_pollDoorbell) {
                CloseHandle(m_pollDoorbell);
                m_pollDoorbell = NULL;
            }
            UnmapViewOfFile(arena);
            CloseHandle(hMapFile);
#endif
            return 1;
        }
        m_arena = &mapping->transport;
    } else {
        m_arena = (qemuArena_t *)arena;
    }

    printf("Qemu: arena ABI v%u mapped (%i bytes)%s\n",
           m_arenaAbiMajor, shMemSize,
           m_arenaDescriptor ? ", capabilities negotiated" : " (rollback)");
    fflush(stdout);

    /* Identidade cross-processa: leitura/parsing uma única vez no startup. */
    if (loadRuntimeIdentityFromEnvironment() == RUNTIME_IDENTITY_MANAGED_INVALID) {
#ifdef __linux__
        munmap(arena, shMemSize);
#elif defined(_WIN32)
        if (m_pollDoorbell) {
            CloseHandle(m_pollDoorbell);
            m_pollDoorbell = NULL;
        }
        UnmapViewOfFile(arena);
        CloseHandle(hMapFile);
#endif
        m_arena = NULL;
        return 1;
    }

    //------------------------------------------------------------------

    printf("-----------------------------------\n");
    for( int i=0; i<argc; i++)
    {
        printf( "%s",argv[i] );
        if( !(i&1) ) printf("\n");
        else         printf(" ");
    }
    printf("-----------------------------------\n");
    fflush( stdout );

    m_lastQemuTime = 0; //m_resetEvent;
    period_ns = 50000; // placeholder ate qemu_init() analisar "-icount shift=N" da linha de comando
    period_ps = 1000*period_ns;
    m_arena->loop_timeout_ns = 1000;
    m_arena->ps_per_inst = 25000;

    //updtCpuFreqHz( 240000000 );

    qemu_init( argc, argv );
    printf("Qemu: initialized\n" );fflush( stdout );

    qemu_mutex_init(&m_arenaOrderLock);
    m_arenaOrderLockInitialized = true;
    if (m_arenaDescriptor) {
        qatomic_store_release(&m_arenaDescriptor->qemuReady, 1);
    }

    {
        CPUState *cpu;
        unsigned vcpuCount = 0;
        CPU_FOREACH(cpu) {
            ++vcpuCount;
        }
        printf("Qemu: execution mode: %s (vcpus=%u, tcg_threads=%u)\n",
               icount_enabled() ? "deterministic-icount" : "mttcg-realtime",
               vcpuCount,
               qemu_tcg_mttcg_enabled() ? vcpuCount : 1);
        fflush(stdout);

        const char *profile = getenv("LASECSIMUL_QEMU_PROFILE");
        m_profileEnabled =
            profile && profile[0] && strcmp(profile, "0") != 0;
        m_profileStartWallNs = get_clock();
        m_profileStartVirtualNs = simuClockNs();
        m_profileLastReportWallNs = m_profileStartWallNs;
        m_profilePublishedEvents = 0;
        m_profileReadTransactions = 0;
        m_profileQueueWaits = 0;
        m_profileReadWaits = 0;
        m_profileMaxQueueOccupancy = 0;
    }

    /* period_ns precisa escalar com 2^shift, senao o heartbeat periodico (simu_event(), que faz
     * um round-trip completo com o host via waitForSynch()) passa a disparar mais vezes por
     * instrucao real conforme "shift" aumenta -- cancelando o ganho de velocidade que um shift
     * maior deveria trazer (mais tempo virtual por instrucao real executada). O valor original
     * (50000 ns pra shift=4) equivale a ~3125 instrucoes reais entre round-trips; mantemos essa
     * mesma cadencia pra qualquer shift configurado, em vez de um segundo numero fixo dessincronizado
     * do primeiro. Lido de timers_state.icount_time_shift (so valido DEPOIS de qemu_init() acima,
     * que e quem processa "-icount shift=N"). Investigado e documentado em LasecSimul 2026-07-19.
     */
    {
        const int shift = timers_state.icount_time_shift;
        if( shift > 0 && shift < 32 )
        {
            const uint64_t instructionsPerHeartbeat = 3125; // calibrado pro shift=4 original (50000/16)
            period_ns = instructionsPerHeartbeat << shift;
            period_ps = 1000*period_ns;
            printf("Qemu: icount shift=%d -> period_ns=%llu (heartbeat a cada ~%llu instrucoes)\n",
                   shift, (unsigned long long)period_ns, (unsigned long long)instructionsPerHeartbeat ); fflush( stdout );
        }
    }

    qtimer = (QEMUTimer*)malloc( sizeof(QEMUTimer) );
    timer_init_full( qtimer, NULL, QEMU_CLOCK_VIRTUAL, 1, 0, simu_event, NULL );
    timer_mod_ns( qtimer, period_ns );

    qatomic_store_release(&m_arena->running, 1);

    printf("Qemu: starting main loop\n");fflush( stdout );
    int status = qemu_main_loop();

    if (m_arenaOrderLockInitialized) {
        qemu_mutex_destroy(&m_arenaOrderLock);
        m_arenaOrderLockInitialized = false;
    }
    qatomic_store_release(&m_arena->running, 0);
    if (m_arenaDescriptor) {
        qatomic_store_release(&m_arenaDescriptor->qemuReady, 0);
    }

#ifdef __linux__
    munmap( arena, shMemSize ); // Un-map shared memory
#elif defined(_WIN32)
    if (m_pollDoorbell) {
        printf("Qemu: doorbell set attempts=%llu successes=%llu failures=%llu\n",
               (unsigned long long)m_doorbellSetAttempts,
               (unsigned long long)m_doorbellSetSuccesses,
               (unsigned long long)m_doorbellSetFailures);
        CloseHandle(m_pollDoorbell);
        m_pollDoorbell = NULL;
    }
    UnmapViewOfFile( arena );
    CloseHandle( hMapFile );
#endif

    printf("Qemu: process finished %i\n", status );fflush( stdout );

    if (m_qemuTraceEnabled) {
        fprintf(stderr, "Qemu trace recorder: T0 calls=%" PRIu64 " ticks=%" PRIu64 " max=%" PRIu64 "; T1 calls=%" PRIu64 " ticks=%" PRIu64 " max=%" PRIu64 "; T5 calls=%" PRIu64 " ticks=%" PRIu64 " max=%" PRIu64 "\n",
                m_qemuTraceCalls[20], m_qemuTraceTicks[20], m_qemuTraceMaxTicks[20],
                m_qemuTraceCalls[21], m_qemuTraceTicks[21], m_qemuTraceMaxTicks[21],
                m_qemuTraceCalls[23], m_qemuTraceTicks[23], m_qemuTraceMaxTicks[23]);
        if (m_qemuTraceMapping) { QemuTraceBinaryHeader *h=(QemuTraceBinaryHeader*)m_qemuTraceMapping; h->written=m_qemuTraceEventSequence; h->dropped=m_qemuTraceDropped; }
        qemuTraceClose(); m_qemuTraceFile = NULL;
    }
    return 0;
}
