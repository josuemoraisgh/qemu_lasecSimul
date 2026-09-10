/*
 * ESP32 IOMUX emulation
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "ui/console.h"
#include "hw/hw.h"
#include "ui/input.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/misc/esp32_iomux.h"
#include "sysemu/runstate.h"

#include "../softmmu/simuliface.h"

#define IOMUX_OFFSET 0x3FF49000

static int getMuxGpio( int addr )
{
    switch (addr) {
    case 0x04: return 36;
    case 0x08: return 37;
    case 0x0C: return 38;
    case 0x10: return 39;
    case 0x1C: return 32;
    case 0x14: return 34;
    case 0x18: return 35;
    case 0x20: return 33;
    case 0x24: return 25;
    case 0x28: return 26;
    case 0x2C: return 27;
    case 0x30: return 14;  //MTMS
    case 0x34: return 12;  //MTDI
    case 0x38: return 13;  //MTCK
    case 0x3C: return 15;  //MTDO
    case 0x40: return  2;
    case 0x44: return  0;
    case 0x48: return  4;
    case 0x4C: return 16;
    case 0x50: return 17;
    case 0x54: return  9;  //SD_DATA2
    case 0x58: return 10;  //SD_DATA3
    case 0x5C: return 11;  //SD_CMD
    case 0x60: return  6;  //SD_CLK
    case 0x64: return  7;  //SD_DATA0
    case 0x68: return  8;  //SD_DATA1
    case 0x6C: return  5;
    case 0x70: return 18;
    case 0x74: return 19;
    case 0x78: return 20;
    case 0x7C: return 21;
    case 0x80: return 22;
    case 0x84: return  3;  //U0RXD
    case 0x88: return  1;  //U0TXD
    case 0x8C: return 23;
    case 0x90: return 24;
    default: break;
    }
    return -1;
}

static uint64_t esp32_iomux_read( void *opaque, hwaddr addr, unsigned int size )
{
    Esp32IomuxState *s = ESP32_IOMUX(opaque);

    uint64_t r = getMuxGpio( addr );
    if( r != -1 ) return s->muxgpios[r];
    return 0;
}


static void esp32_iomux_write( void *opaque, hwaddr addr, uint64_t value, unsigned int size )
{
    Esp32IomuxState *s = ESP32_IOMUX(opaque);

    int pin = getMuxGpio( addr );
    if( pin == -1 ) return;

    if( s->muxgpios[pin] == value ) return;
    s->muxgpios[pin]= value;

    writeReg( IOMUX_OFFSET+addr, value );
}

static const MemoryRegionOps iomux_ops = {
    .read  = esp32_iomux_read,
    .write = esp32_iomux_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_iomux_reset( DeviceState *dev )
{
    Esp32IomuxState *s = ESP32_IOMUX(dev);
    for( int i=0; i<40; i++ ) s->muxgpios[i] = 0x800;
}

static void esp32_iomux_realize( DeviceState *dev, Error **errp ) {}

static void esp32_iomux_init( Object *obj )
{
    Esp32IomuxState *s = ESP32_IOMUX(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io( &s->iomem, obj, &iomux_ops, s, TYPE_ESP32_IOMUX, 0x1000 );
    /* E118-AUDIT-2 (EVIDENCE.md, 2026-09-05): see the matching comment in hw/i2c/esp32_i2c.c. */
    s->iomem.disable_reentrancy_guard = true;
    sysbus_init_mmio( sbd, &s->iomem );

    for( int i=0; i<40; i++ ) s->muxgpios[i] = 0x800;
}

static void esp32_iomux_class_init( ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->reset = esp32_iomux_reset;
    dc->realize = esp32_iomux_realize;
}

static const TypeInfo esp32_iomux_info = {
    .name   = TYPE_ESP32_IOMUX,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32IomuxState),
    .instance_init = esp32_iomux_init,
    .class_init = esp32_iomux_class_init};

static void esp32_iomux_register_types(void) {
    type_register_static(&esp32_iomux_info);
}

type_init( esp32_iomux_register_types )
