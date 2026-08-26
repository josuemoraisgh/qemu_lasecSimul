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
// A ABI v5 mantém o descritor negociado introduzido na v4 e acrescenta o mailbox I2C ao fim do
// payload v3. Core e QEMU precisam usar o mesmo layout; LASECSIMUL_QEMU_ARENA_VERSION=3 mantém o
// mapping v3 puro como rollback.

#define QEMU_ARENA_QUEUE_DEPTH 32
#define QEMU_ARENA_ABI_MAGIC UINT64_C(0x4c53444e51415235) /* "LSDNQAR5" */
#define QEMU_ARENA_ABI_MAJOR 5
#define QEMU_ARENA_ABI_MINOR 0

#define QEMU_ARENA_CAP_WRITE_QUEUE          (UINT64_C(1) << 0)
#define QEMU_ARENA_CAP_ORDERED_EVENTS       (UINT64_C(1) << 1)
#define QEMU_ARENA_CAP_SYNC_READ            (UINT64_C(1) << 2)
#define QEMU_ARENA_CAP_MTTCG_MPSC_SERIALIZED (UINT64_C(1) << 3)
#define QEMU_ARENA_CAP_I2C_BURST             (UINT64_C(1) << 4)
#define QEMU_ARENA_CAPABILITIES                                                \
    (QEMU_ARENA_CAP_WRITE_QUEUE | QEMU_ARENA_CAP_ORDERED_EVENTS |             \
     QEMU_ARENA_CAP_SYNC_READ | QEMU_ARENA_CAP_MTTCG_MPSC_SERIALIZED |        \
     QEMU_ARENA_CAP_I2C_BURST)
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

    /* ABI 5: mailbox I2C de um produtor/um consumidor -- layout espelha EXATAMENTE
     * LsdnQemuArena em core/include/lasecsimul/qemu_arena_abi.h (mesmo comentario de topo
     * daquele header se aplica aqui: nao reordenar, nao inserir campo). O QEMU publica todos os
     * campos e por ultimo i2cRequestSeq; o Core responde e por ultimo publica i2cResponseSeq. */
    uint64_t i2cRequestSeq;
    uint64_t i2cResponseSeq;
    uint64_t i2cTimePs;
    uint32_t i2cBus;
    uint32_t i2cFlags;       /* bit0 START, bit1 STOP, bit2 READ */
    uint64_t i2cPeriodNs;
    uint32_t i2cTxLen;
    uint32_t i2cRxLen;
    uint8_t  i2cTx[64];
    uint8_t  i2cRx[32];
    uint32_t i2cStatus;      /* bit0 handled, bit1 address ACK */
    uint32_t i2cFirstNack;   /* UINT32_MAX quando todos os payloads deram ACK */
    uint64_t i2cStretchNs;
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

typedef struct qemuArenaV5Mapping {
    qemuArenaDescriptor_t descriptor;
    qemuArena_t transport;
} qemuArenaV5Mapping_t;

_Static_assert(sizeof(qemuQueueEntry_t) == 32,
               "QEMU arena queue entry ABI changed");
_Static_assert(sizeof(qemuArena_t) == 1288,
               "QEMU arena v5 payload ABI changed");
_Static_assert(sizeof(qemuArenaDescriptor_t) == 88,
               "QEMU arena descriptor ABI changed");
_Static_assert(sizeof(qemuArenaV5Mapping_t) == 1376,
               "QEMU arena v5 mapping ABI changed");

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

/* Mailbox de burst I2C (ABI 5) -- um pedido cobre um trecho inteiro do FIFO/lista de comandos
 * (endereco + dados, ate 32 bytes, limite real do periferico ESP32) numa unica viagem ao Core,
 * em vez de uma viagem por byte. Ver hw/i2c/esp32_i2c.c (esp32_i2c_try_burst) pro chamador. */
typedef struct I2cBurstRequest {
    uint32_t flags;       /* bit0 START, bit1 STOP, bit2 READ */
    uint64_t period_ns;
    const uint8_t *tx;    /* primeiro byte sempre = endereco+RW; em continuação é metadado */
    uint32_t tx_len;
    uint32_t rx_len;      /* bytes pedidos de volta quando bit2 (READ) esta setado */
} I2cBurstRequest;

typedef struct I2cBurstResponse {
    bool     handled;      /* false: nada no Core suporta o protocolo de burst -- caia pro eletrico */
    bool     address_ack;
    uint32_t first_nack;   /* indice do primeiro byte de dado NACKado; UINT32_MAX se nenhum */
    uint32_t rx_len;
    uint64_t stretch_ns;
} I2cBurstResponse;

bool i2cBurstTransfer( uint32_t bus, const I2cBurstRequest* req, I2cBurstResponse* resp, uint8_t* rxOut );

int simuMain( int argc, char** argv );

void updtCpuFreqHz( uint32_t clockHz );

void setInterrupt(void);

#endif
