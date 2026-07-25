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
// v3 (LasecSimul PERF-13): fila circular pra escritas/heartbeat (SIM_WRITE/SIM_EVENT --
// "dispara e esquece"), leitura (SIM_READ) continua fora da fila, síncrona, um slot só. Ver
// comentário completo em qemu_arena_abi.h (LasecSimul/core/include/lasecsimul/) -- os dois lados
// são mantidos como espelhos manuais, não um header compartilhado.

#define QEMU_ARENA_QUEUE_DEPTH 32

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
