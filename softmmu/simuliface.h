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

typedef struct qemuArena{
    uint64_t simuTime;    // in ps
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

int simuMain( int argc, char** argv );

void updtCpuFreqHz( uint32_t clockHz );

void setInterrupt(void);

#endif
