/***************************************************************************
 *   Copyright (C) 2025 by Santiago González                               *
 *                                                                         *
 ***( see copyright.txt file at root folder )*******************************/

#ifndef QEMU_SIMULIFACE_H
#define QEMU_SIMULIFACE_H

#include <stdint.h>
#include <stdbool.h>

#include "qemu/typedefs.h"

// ------------------------------------------------
// -------- ARENA ---------------------------------
// O payload abaixo continua binariamente idêntico à ABI v3. A ABI v4 o encapsula depois de um
// descritor versionado, permitindo validar magic, tamanhos, profundidade e capacidades antes de
// tocar na fila. LASECSIMUL_QEMU_ARENA_VERSION=3 mantém o mapping v3 puro como rollback.

#define QEMU_ARENA_QUEUE_DEPTH 32
#define QEMU_ARENA_ABI_MAGIC UINT64_C(0x4c53444e51415234) /* "LSDNQAR4" */
#define QEMU_ARENA_ABI_MAJOR 4
#define QEMU_ARENA_ABI_MINOR 0

#define QEMU_ARENA_CAP_WRITE_QUEUE          (UINT64_C(1) << 0)
#define QEMU_ARENA_CAP_ORDERED_EVENTS       (UINT64_C(1) << 1)
#define QEMU_ARENA_CAP_SYNC_READ            (UINT64_C(1) << 2)
#define QEMU_ARENA_CAP_MTTCG_MPSC_SERIALIZED (UINT64_C(1) << 3)
#define QEMU_ARENA_CAPABILITIES                                                \
    (QEMU_ARENA_CAP_WRITE_QUEUE | QEMU_ARENA_CAP_ORDERED_EVENTS |             \
     QEMU_ARENA_CAP_SYNC_READ | QEMU_ARENA_CAP_MTTCG_MPSC_SERIALIZED)
#define QEMU_ARENA_REQUIRED_CAPABILITIES QEMU_ARENA_CAPABILITIES

typedef struct qemuQueueEntry{
    uint64_t regAddr;
    uint64_t regData;
    uint64_t simuAction;
    uint64_t simuTime;    // in ps
} qemuQueueEntry_t;

typedef struct qemuArena{
    uint64_t queueWriteIndex;                        // QEMU escreve; nunca reseta
    uint64_t queueReadIndex;                         // Core escreve; nunca reseta
    qemuQueueEntry_t queue[QEMU_ARENA_QUEUE_DEPTH];

    uint64_t simuTime;    // in ps -- só SIM_READ agora
    uint64_t qemuTime;    // in ps
    uint64_t regData;
    uint64_t regAddr;
    uint64_t irqNumber;
    uint64_t irqLevel;
    uint64_t simuAction;
    uint64_t qemuAction;
    uint64_t running;
    int64_t  loop_timeout_ns;
    double   ps_per_inst;
} qemuArena_t;

typedef struct qemuArenaDescriptor {
    uint64_t magic;
    uint32_t abiMajor;
    uint32_t abiMinor;
    uint64_t descriptorSize;
    uint64_t arenaSize;
    uint64_t transportSize;
    uint64_t queueDepth;
    uint64_t coreCapabilities;
    uint64_t qemuCapabilities;
    uint64_t negotiatedCapabilities;
    uint64_t coreReady;
    uint64_t qemuReady;
} qemuArenaDescriptor_t;

typedef struct qemuArenaV4Mapping {
    qemuArenaDescriptor_t descriptor;
    qemuArena_t transport;
} qemuArenaV4Mapping_t;

_Static_assert(sizeof(qemuQueueEntry_t) == 32,
               "QEMU arena queue entry ABI changed");
_Static_assert(sizeof(qemuArena_t) == 1128,
               "QEMU arena v3 payload ABI changed");
_Static_assert(sizeof(qemuArenaDescriptor_t) == 88,
               "QEMU arena v4 descriptor ABI changed");
_Static_assert(sizeof(qemuArenaV4Mapping_t) == 1216,
               "QEMU arena v4 mapping ABI changed");

enum esp32Actions{
    ESP_GPIO_OUT = 1,
    ESP_GPIO_DIR,
    ESP_GPIO_IN,
    ESP_IOMUX,
    ESP_MATRIX_IN,
    ESP_MATRIX_OUT
};

enum arm32Actions{
    ARM_GPIO_OUT = 1,
    ARM_GPIO_CRx,
    ARM_GPIO_IN,
    ARM_ALT_OUT,
    ARM_REMAP
};

enum simAction{
    SIM_NONE=0,
    SIM_READ,
    SIM_WRITE,
    SIM_FREQ,
    SIM_INTERRUPT,
    SIM_I2C=10,
    SIM_SPI,
    SIM_USART,
    SIM_TIMER,
    SIM_GPIO_IN,
    SIM_EVENT=1<<7,
};

extern volatile qemuArena_t* m_arena;
// ------------------------------------------------

extern uint64_t m_timeout;


uint64_t getQemu_ps(void);
uint64_t getQemu_ns(void);

//bool waitEvent(void);
void waitForSynch(void);

uint64_t readReg( uint64_t addr );
void writeReg( uint64_t addr, uint64_t value );
void writeSimEvent( uint64_t addr, uint64_t value, uint64_t action );

int simuMain( int argc, char** argv );

void updtCpuFreqHz( uint32_t clockHz );

void setInterrupt(void);

#endif
