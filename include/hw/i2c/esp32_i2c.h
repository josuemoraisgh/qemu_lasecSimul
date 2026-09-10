#ifndef ESP32_I2C_H
#define ESP32_I2C_H

#include "hw/sysbus.h"
#include "hw/i2c/i2c.h"
#include "hw/registerfields.h"
#include "qemu/fifo8.h"
#include "qemu/timer.h"

#define TYPE_ESP32_I2C "esp32.i2c"
#define Esp32_I2C(obj) OBJECT_CHECK(Esp32I2CState, (obj), TYPE_ESP32_I2C)


#define ESP32_I2C_MEM_SIZE 0x100
#define ESP32_I2C_FIFO_LENGTH 32
#define ESP32_I2C_CMD_COUNT 16


typedef struct Esp32I2CState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
    I2CBus *bus;

    Fifo8 rx_fifo;
    Fifo8 tx_fifo;
    bool trans_ongoing;

    QEMUTimer event_timer;
    uint64_t period_ns;
    uint8_t lastCMD;
    uint8_t lastOpcode;
    uint8_t bytesTx;
    uint8_t bytesRx;
    bool ackSamplePending;
    uint8_t busIndex;               /* 0=I2C0, 1=I2C1 -- setado por esp32_soc_realize() */

    /* Estado do burst rapido em andamento (ver esp32_i2c_try_burst/esp32_i2c_finish_burst em
     * esp32_i2c.c) -- so' um burst por vez, mesma premissa de exclusividade que event_timer ja' tem
     * pra transacao byte-a-byte. */
    bool burstActive;
    bool burstAddressAck;
    bool burstAddressValid;
    bool burstPartial;
    bool burstStop;
    uint8_t burstAddressByte;
    uint32_t burstFirstNack;
    uint32_t burstWriteCmdCount;
    uint32_t burstReadCmdCount;
    uint8_t burstRxBuf[32];
    uint32_t burstRxLen;

    uint32_t sr_reg;

    uint32_t ctr_reg;
    uint32_t timeout_reg;
    uint32_t int_ena_reg;
    uint32_t int_raw_reg;
    uint32_t sda_hold_reg;
    uint32_t sda_sample_reg;
    uint32_t high_period_reg;
    uint32_t low_period_reg;
    uint32_t start_hold_reg;
    uint32_t rstart_setup_reg;
    uint32_t stop_hold_reg;
    uint32_t stop_setup_reg;
    uint32_t cmd_reg[ESP32_I2C_CMD_COUNT];
    /* E118-AUDIT (EVIDENCE.md, 2026-09-05): true when esp32_i2c_do_transaction()'s electrical-path
     * writeReg() observed VNEXT_WOULD_BLOCK on a current_cpu==NULL continuation step (reached via
     * esp32_i2c_event()'s timer, not synchronously from a guest MMIO write -- see esp32_i2c_event()
     * for when that happens). No lastCMD/bytesTx/time/interrupt/ACK state is advanced while this is
     * set; esp32_i2c_vnext_credit_available() retries the exact same step once lane 0 regains
     * credit. Cleared by esp32_i2c_reset() (via timer_del(&event_timer), which already cancels any
     * pending step) so a reset device is never woken by a stale continuation -- single-threaded
     * under the BQL, so a plain bool (no generation counter) is sufficient: nothing can observe
     * this flag as true again after reset without a fresh do_transaction() call setting it. */
    bool vnextContinuationBacklogged;
} Esp32I2CState;

void esp32_i2c_vnext_bind(Esp32I2CState *s);
void esp32_i2c_vnext_complete(uint32_t bus, uint32_t status, uint32_t first_nack,
                              uint64_t stretch_ns, const uint8_t *rx, uint32_t rx_len);
/* E118-AUDIT (EVIDENCE.md, 2026-09-05): called from vnext_b.c's vnext_resume() backlog-notify
 * sweep once lane 0 regains credit. Retries esp32_i2c_do_transaction() for exactly the bound
 * instances with vnextContinuationBacklogged set; a no-op otherwise. Never touches a CPU (this
 * continuation only ever runs with current_cpu==NULL in the first place). */
void esp32_i2c_vnext_credit_available(void);


REG32(I2C_CTR, 0x04);
FIELD(I2C_CTR, MS_MODE, 4, 1);
FIELD(I2C_CTR, TRANS_START, 5, 1);

REG32(I2C_STATUS, 0x08);
FIELD(I2C_STATUS, BUS_BUSY, 4, 1);
FIELD(I2C_STATUS, RXFIFO_CNT, 8, 6);
FIELD(I2C_STATUS, TXFIFO_CNT, 18, 6);

REG32(I2C_TIMEOUT, 0x0c);

REG32(I2C_FIFO_CONF, 0x18);
FIELD(I2C_FIFO_CONF, NONFIFO_EN, 10, 1);
FIELD(I2C_FIFO_CONF, RX_FIFO_RST, 12, 1);
FIELD(I2C_FIFO_CONF, TX_FIFO_RST, 13, 1);

REG32(I2C_FIFO_DATA, 0x1c);

REG32(I2C_INT_RAW, 0x20);
FIELD(I2C_INT_RAW, ACK_ERR, 10, 1);
FIELD(I2C_INT_RAW, TRANS_COMPLETE, 7, 1);
FIELD(I2C_INT_RAW, END_DETECT, 3, 1);

REG32(I2C_INT_CLR, 0x24);
FIELD(I2C_INT_CLR, ACK_ERR, 10, 1);
FIELD(I2C_INT_CLR, TRANS_COMPLETE, 7, 1);
FIELD(I2C_INT_CLR, END_DETECT, 3, 1);

REG32(I2C_INT_ENA, 0x28);
FIELD(I2C_INT_ENA, ACK_ERR, 10, 1);
FIELD(I2C_INT_ENA, TRANS_COMPLETE, 7, 1);
FIELD(I2C_INT_ENA, END_DETECT, 3, 1);

REG32(I2C_INT_ST, 0x2c);
FIELD(I2C_INT_ST, ACK_ERR, 10, 1);
FIELD(I2C_INT_ST, TRANS_COMPLETE, 7, 1);
FIELD(I2C_INT_ST, END_DETECT, 3, 1);

REG32(I2C_SDA_HOLD, 0x30);
REG32(I2C_SDA_SAMPLE, 0x34);
REG32(I2C_HIGH_PERIOD, 0x38);
REG32(I2C_LOW_PERIOD, 0x00);  // 0x00 is not a typo
REG32(I2C_START_HOLD, 0x40);
REG32(I2C_RSTART_SETUP, 0x44);
REG32(I2C_STOP_HOLD, 0x48);
REG32(I2C_STOP_SETUP, 0x4c);

REG32(I2C_CMD, 0x58);
FIELD(I2C_CMD, BYTE_NUM, 0, 8);
FIELD(I2C_CMD, ACK_CHECK_EN, 8, 1);
FIELD(I2C_CMD, ACK_EXP, 9, 1);
FIELD(I2C_CMD, ACK_VAL, 10, 1);
FIELD(I2C_CMD, OPCODE, 11, 3);
FIELD(I2C_CMD, DONE, 31, 1);
/* 15 more command registers omitted */

/* I2C_CMD.OPCODE values */
typedef enum {
    I2C_OPCODE_RSTART = 0,
    I2C_OPCODE_WRITE  = 1,
    I2C_OPCODE_READ   = 2,
    I2C_OPCODE_STOP   = 3,
    I2C_OPCODE_END    = 4,
} i2c_opcode_t;

#endif /* ESP32_I2C_H */
