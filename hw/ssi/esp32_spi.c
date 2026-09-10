/*
 * ESP32 SPI controller
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "sysemu/sysemu.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/ssi/ssi.h"
#include "hw/ssi/esp32_spi.h"
#include "hw/misc/esp32_flash_enc.h"
#include "hw/xtensa/esp32_clk.h"

#include "../softmmu/simuliface.h"

enum {
    CMD_RES = 0xab,
    CMD_DP = 0xb9,
    CMD_CE = 0x60,
    CMD_BE = 0xD8,
    CMD_SE = 0x20,
    CMD_PP = 0x02,
    CMD_WRSR = 0x1,
    CMD_RDSR = 0x5,
    CMD_RDID = 0x9f,
    CMD_WRDI = 0x4,
    CMD_WREN = 0x6,
    CMD_READ = 0x03,
};


#define ESP32_SPI_REG_SIZE    0x1000

static void esp32_spi_do_command(Esp32SpiState* state, uint32_t cmd_reg);

static void esp32_spi_event( void* opaque ) // Timer event
{
    Esp32SpiState *s = ESP32_SPI(opaque);

    s->dataBytes--;
    //if( s->dataBytes ) m_arena->regData = s->data_reg[s->bytesDone+1];
    //else               m_arena->regData = -1;

    // Reading gets timed out in some cases, by now just write
    //s->data_reg[s->bytesDone] = readReg( (s->iomem.addr & 0x000FFFFF)+0x80 ); //SPI Data buffer
    s->bytesDone++;

    int bits = 8;
    if( s->dataBytes )
    {
        writeReg( s->iomem.addr+0x80, s->data_reg[s->bytesDone] );
        timer_mod_ns( &s->event_timer, getQemu_ns()+s->period*bits );
    }
    else{
        s->do_command = 0;
        // Fim real da transacao (numero>=2, HSPI/VSPI -- SPI0/SPI1 usam esp32_spi_cs_set() via
        // qemu_irq interno, nunca chegam aqui): desativa CS0 automaticamente, mesmo bracket que o
        // hardware real aplica ao redor de QUALQUER comando USR, sem exigir nenhum firmware/
        // biblioteca especifica -- ver A_SPI_CMD tambem espelhado no inicio da transacao abaixo.
        writeReg( s->iomem.addr+A_SPI_CMD, 0 );
    }

    //printf("esp32_spi_event %i %i %i %lu\n", s->number, s->bytesDone, s->dataBytes, getQemu_ps() ); fflush( stdout );
}

static void write_clk_reg( Esp32SpiState* s, uint64_t value )
{
    uint32_t CLKDIV_PRE = (value & 0x7FFC0000) >> 18; // bits 18 to 30
    uint32_t CLKCNT_N   = (value & 0x0003F000) >> 12; // bits 12 to 17
    uint32_t divider = (CLKDIV_PRE+1)*(CLKCNT_N+1);
    //if( period_ns == 0 ) return;

    uint32_t apb_freq = esp32_soc_get_apb_freq();
    uint32_t spi_freq = esp32_soc_get_apb_freq()/divider;

    uint32_t period_ns = 1e9/spi_freq;
    //if( period_ns < 500 ) period_ns = 500;

    if( s->period == period_ns ) return;
    s->period = period_ns;
    //printf("write_clk_reg %i %i %i %i\n", CLKDIV_PRE, CLKCNT_N, apb_freq, period_ns);
    writeReg( s->iomem.addr+0x18, period_ns*1000 );
}

static uint64_t esp32_spi_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32SpiState *s = ESP32_SPI(opaque);
    uint64_t r = 0;

    switch (addr) {
    case A_SPI_CMD:
        //if( s->number > 1 )
        r = s->do_command;
        break;
    case A_SPI_ADDR:      r = s->addr_reg; break;
    case A_SPI_CTRL:      r = s->ctrl_reg; break;
    case A_SPI_STATUS:    r = s->status_reg; break;
    case A_SPI_CTRL1:     r = s->ctrl1_reg; break;
    case A_SPI_CTRL2:     r = s->ctrl2_reg; break;
    case A_SPI_USER:      r = s->user_reg; break;
    case A_SPI_USER1:     r = s->user1_reg; break;
    case A_SPI_USER2:     r = s->user2_reg; break;
    case A_SPI_MOSI_DLEN: r = s->mosi_dlen_reg; break;
    case A_SPI_MISO_DLEN: r = s->miso_dlen_reg; break;
    case A_SPI_PIN:       r = s->pin_reg; break;
    case A_SPI_SLAVE:
        r = BIT(R_SPI_SLAVE_TRANS_DONE_SHIFT) | BIT(R_SPI_SLAVE_TRANS_INTEN_SHIFT);
        break;
    case A_SPI_W0 ... A_SPI_W0 + (ESP32_SPI_BUF_WORDS - 1) * sizeof(uint32_t):
        r = s->data_reg[(addr - A_SPI_W0) / sizeof(uint32_t)];
        break;
        //case A_SPI_EXT2: r = 0; break;
    }
    //printf("esp32_spi_read %i %lu %li\n", s->number, addr, r ); fflush( stdout );
    return r;
}

static void esp32_spi_write(void *opaque, hwaddr addr, uint64_t value, unsigned int size)
{
    Esp32SpiState *s = ESP32_SPI(opaque);
    switch (addr) {
    case A_SPI_CMD: esp32_spi_do_command(s, value); break;
    case A_SPI_ADDR:      s->addr_reg      = value; break;
    case A_SPI_CTRL:      s->ctrl_reg      = value; break;
    case A_SPI_CTRL1:     s->ctrl1_reg     = value; break;
    case A_SPI_STATUS:    s->status_reg    = value; break;
    case A_SPI_CTRL2:     s->ctrl2_reg     = value; break;
    case 0x18:            write_clk_reg( s, value );break; //SPI_CLOCK_REG (0x18)
    case A_SPI_USER:      s->user_reg      = value; break;
    case A_SPI_USER1:     s->user1_reg     = value; break;
    case A_SPI_USER2:     s->user2_reg     = value; break;
    case A_SPI_MOSI_DLEN: s->mosi_dlen_reg = value; break;
    case A_SPI_MISO_DLEN: s->miso_dlen_reg = value; break;
    case A_SPI_PIN:
        s->pin_reg = value;
        // Espelha CS0_DIS/CS0_POL (e os demais CS, ainda que so' CS0 seja roteado pro Core hoje --
        // ver Esp32Adapter.cpp) pro lado que de fato pilota o pino eletrico em HSPI/VSPI.
        writeReg( s->iomem.addr+A_SPI_PIN, value );
        break;
    case A_SPI_SLAVE:
        writeReg( s->iomem.addr+A_SPI_SLAVE, value );
        break;
    case A_SPI_W0 ... A_SPI_W0 + (ESP32_SPI_BUF_WORDS - 1) * sizeof(uint32_t):
        s->data_reg[(addr - A_SPI_W0) / sizeof(uint32_t)] = value;
        break;
    }
}

typedef struct Esp32SpiTransaction {
    int cmd_bytes;
    uint32_t cmd;
    int addr_bytes;
    uint32_t addr;
    int data_tx_bytes;
    int data_rx_bytes;
    uint32_t* data;
} Esp32SpiTransaction;

static void esp32_spi_txrx_buffer(Esp32SpiState *s, void *buf, int tx_bytes, int rx_bytes)
{
    int bytes = MAX( tx_bytes, rx_bytes );
    uint8_t* c_buf = (uint8_t*)buf;

    for( int i=0; i<bytes; ++i ) {
        uint8_t byte = 0;
        if( byte < tx_bytes) memcpy( &byte, c_buf + i, 1 );

        uint32_t res = ssi_transfer( s->spi, byte );
        if( byte < rx_bytes ) memcpy( c_buf + i, &res, 1 );
    }
}

static void esp32_spi_cs_set(Esp32SpiState *s, int value)
{
    for( int i=0; i<ESP32_SPI_CS_COUNT; ++i) {
        qemu_set_irq( s->cs_gpio[i], ((s->pin_reg & (1 << i)) == 0) ? value : 1);
    }
}

static void esp32_spi_transaction( Esp32SpiState *s, Esp32SpiTransaction *t )
{
    esp32_spi_cs_set( s, 0 );
    esp32_spi_txrx_buffer( s, &t->cmd , t->cmd_bytes, 0 );
    esp32_spi_txrx_buffer( s, &t->addr, t->addr_bytes, 0 );
    esp32_spi_txrx_buffer( s, t->data , t->data_tx_bytes, t->data_rx_bytes );
    esp32_spi_cs_set( s, 1 );
    s->do_command = 0;
}

/* Convert one of the hardware "bitlen" registers to a byte count */
static inline int bitlen_to_bytes(uint32_t val)
{
    return (val + 1 + 7) / 8; /* bitlen registers hold number of bits, minus one */
}

static void maybe_encrypt_data(Esp32SpiState *s)
{
    Esp32FlashEncryptionState* flash_enc = esp32_flash_encryption_find();
    if( esp32_flash_encryption_enabled(flash_enc)) {
        esp32_flash_encryption_get_result(flash_enc, &s->data_reg[0], 8);
    }
}

static void esp32_spi_do_command( Esp32SpiState* s, uint32_t cmd_reg )
{
    if( (cmd_reg & (1<<18)) == 0 ) return;
    s->do_command = 1<<18;

    Esp32SpiTransaction t = {
        .cmd_bytes = 1
    };
    switch (cmd_reg) {
    case R_SPI_CMD_READ_MASK:
        t.cmd = CMD_READ;
        t.addr_bytes = bitlen_to_bytes(FIELD_EX32(s->user1_reg, SPI_USER1, ADDR_BITLEN));
        t.addr = bswap32(s->addr_reg) >> (32 - t.addr_bytes * 8);
        t.data = &s->data_reg[0];
        t.data_rx_bytes = bitlen_to_bytes(s->miso_dlen_reg);
        break;

    case R_SPI_CMD_WREN_MASK:
        t.cmd = CMD_WREN;
        break;

    case R_SPI_CMD_WRDI_MASK:
        t.cmd = CMD_WRDI;
        break;

    case R_SPI_CMD_RDID_MASK:
        t.cmd = CMD_RDID;
        t.data = &s->data_reg[0];
        t.data_rx_bytes = 3;
        break;

    case R_SPI_CMD_RDSR_MASK:
        t.cmd = CMD_RDSR;
        t.data = &s->status_reg;
        t.data_rx_bytes = 1;
        break;

    case R_SPI_CMD_WRSR_MASK:
        t.cmd = CMD_WRSR;
        t.data = &s->status_reg;
        t.data_tx_bytes = 1;
        break;

    case R_SPI_CMD_PP_MASK:
        maybe_encrypt_data(s);
        t.cmd = CMD_PP;
        t.data = &s->data_reg[0];
        t.addr_bytes = bitlen_to_bytes(FIELD_EX32(s->user1_reg, SPI_USER1, ADDR_BITLEN));
        t.addr = bswap32(s->addr_reg) >> 8;
        t.data = &s->data_reg[0];
        t.data_tx_bytes = s->addr_reg >> 24;
        break;

    case R_SPI_CMD_SE_MASK:
        t.cmd = CMD_SE;
        t.addr_bytes = bitlen_to_bytes(FIELD_EX32(s->user1_reg, SPI_USER1, ADDR_BITLEN));
        t.addr = bswap32(s->addr_reg) >> (32 - t.addr_bytes * 8);
        break;

    case R_SPI_CMD_BE_MASK:
        t.cmd = CMD_BE;
        t.addr_bytes = bitlen_to_bytes(FIELD_EX32(s->user1_reg, SPI_USER1, ADDR_BITLEN));
        t.addr = bswap32(s->addr_reg) >> (32 - t.addr_bytes * 8);
        break;

    case R_SPI_CMD_CE_MASK:
        t.cmd = CMD_CE;
        break;

    case R_SPI_CMD_DP_MASK:
        t.cmd = CMD_DP;
        break;

    case R_SPI_CMD_RES_MASK:
        t.cmd = CMD_RES;
        t.data = &s->data_reg[0];
        t.data_rx_bytes = 3;
        break;

    case R_SPI_CMD_USR_MASK:
        maybe_encrypt_data(s);
        if( FIELD_EX32(s->user_reg, SPI_USER, COMMAND) || FIELD_EX32(s->user2_reg, SPI_USER2, COMMAND_BITLEN)) {
            t.cmd = FIELD_EX32(s->user2_reg, SPI_USER2, COMMAND_VALUE);
            t.cmd_bytes = bitlen_to_bytes(FIELD_EX32(s->user2_reg, SPI_USER2, COMMAND_BITLEN));
        } else {
            t.cmd_bytes = 0;
        }
        if( FIELD_EX32(s->user_reg, SPI_USER, ADDR)) {
            t.addr_bytes = bitlen_to_bytes(FIELD_EX32(s->user1_reg, SPI_USER1, ADDR_BITLEN));
            t.addr = bswap32(s->addr_reg);
        }
        if( FIELD_EX32(s->user_reg, SPI_USER, MOSI)) {
            t.data = &s->data_reg[0];
            t.data_tx_bytes = bitlen_to_bytes(s->mosi_dlen_reg);
        }
        if( FIELD_EX32(s->user_reg, SPI_USER, MISO)) {
            t.data = &s->data_reg[0];
            t.data_rx_bytes = bitlen_to_bytes(s->miso_dlen_reg);
        }
        break;
    default:
        return;
    }
    if( s->number < 2 ) esp32_spi_transaction( s, &t );
    else{
        s->dataBytes = MAX( t.data_tx_bytes, t.data_rx_bytes );
        if( s->dataBytes == 0 ) return;

        //printf("\nesp32_spi_cmd %i %i %i\n", s->number, s->bytesDone, s->dataBytes ); fflush( stdout );

        s->bytesDone = 0;
        // Inicio real da transacao (numero>=2, HSPI/VSPI): ativa CS0 automaticamente ANTES do
        // primeiro byte, mesmo bracket que o hardware real aplica ao redor de qualquer comando USR
        // -- ver o fim espelhado em esp32_spi_event() acima. Nenhum firmware/biblioteca especifica
        // e' assumida: isto vale pra qualquer transacao SPI, USR ou nao, que chegue por este caminho.
        writeReg( s->iomem.addr+A_SPI_CMD, 1 );
        writeReg( s->iomem.addr+0x80, s->data_reg[0] );
        int bits = 8;
        timer_mod_ns( &s->event_timer, getQemu_ns()+s->period*bits);
    }
}

static const MemoryRegionOps esp32_spi_ops = {
    .read =  esp32_spi_read,
    .write = esp32_spi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_spi_reset(DeviceState *dev)
{
    Esp32SpiState *s = ESP32_SPI(dev);
    s->pin_reg = 0x6;
    s->user1_reg = FIELD_DP32(0, SPI_USER1, ADDR_BITLEN, 23);
    s->user1_reg = FIELD_DP32(s->user1_reg, SPI_USER1, DUMMY_CYCLELEN, 7);
    s->user2_reg = FIELD_DP32(0, SPI_USER2, COMMAND_BITLEN, 4);
    s->user2_reg = FIELD_DP32(s->user2_reg, SPI_USER2, COMMAND_VALUE, 0);
    s->status_reg = 0;
    s->do_command = 0;
}

static void esp32_spi_realize(DeviceState *dev, Error **errp)
{
}

static void esp32_spi_init(Object *obj)
{
    Esp32SpiState *s = ESP32_SPI(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp32_spi_ops, s,
                          TYPE_ESP32_SPI, ESP32_SPI_REG_SIZE);
    /* E118-AUDIT-2 (EVIDENCE.md, 2026-09-05): see the matching comment in hw/i2c/esp32_i2c.c --
     * writeReg()'s VNEXT_WOULD_BLOCK path can cpu_loop_exit_restore() out of this device's own
     * dispatch, permanently wedging softmmu/memory.c's per-device reentrancy guard otherwise.
     * Safe here for the same reason: this retry path never drops the BQL. */
    s->iomem.disable_reentrancy_guard = true;
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    s->spi = ssi_create_bus(DEVICE(s), "spi");
    qdev_init_gpio_out_named(DEVICE(s), &s->cs_gpio[0], SSI_GPIO_CS, ESP32_SPI_CS_COUNT);

    timer_init_ns( &s->event_timer, QEMU_CLOCK_VIRTUAL, esp32_spi_event, s );

    s->do_command = 0;
}

static Property esp32_spi_properties[] = {
    //DEFINE_PROP_BOOL("xfer_32_bits",Esp32SpiState,xfer_32_bits,false),
    //DEFINE_PROP_BOOL("use_cs", Esp32SpiState, use_cs, false),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32_spi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = esp32_spi_reset;
    dc->realize = esp32_spi_realize;
    device_class_set_props(dc, esp32_spi_properties);
}

static const TypeInfo esp32_spi_info = {
    .name = TYPE_ESP32_SPI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32SpiState),
    .instance_init = esp32_spi_init,
    .class_init = esp32_spi_class_init
};

static void esp32_spi_register_types(void)
{
    type_register_static(&esp32_spi_info);
}

type_init(esp32_spi_register_types)
