/*
 * ESP32 GPIO emulation
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */
// Modified by Santiago Gonzalez, April 2025

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/gpio/esp32_gpio.h"

#include "../softmmu/simuliface.h"


#define GPIO_OFFSET 0x3FF44000

static bool esp32_gpio_trace_enabled(void)
{
    static int enabled = -1;

    if (enabled < 0) {
        const char *value = getenv("LASECSIMUL_TRACE_GPIO");
        enabled = value && value[0] && strcmp(value, "0") != 0;
    }
    return enabled;
}

static void inputChanged( Esp32GpioState *gpioS, int pin, int val )
{
    int n1 = pin;
    int i = 0;
    if( pin >= 32 ) { n1 -= 32; i = 1; }
    uint32_t pinMask = 1 << n1;

    uint32_t* pcpuInt = &gpioS->gpio_pcpu_int[i];
    uint32_t* acpuInt = &gpioS->gpio_acpu_int[i];
    uint32_t* gpioIn  = &gpioS->gpio_in[i];

    int oldval = *gpioIn & pinMask;
    *gpioIn &= ~pinMask;
    *gpioIn |= val;

    uint32_t gpioPin = gpioS->gpio_pin[pin];
    int int_type = (gpioPin >> 7) & 7;

    bool irq = 0;
    switch( int_type )
    {
    case 1: irq = (val >  oldval); break; //rising edge trigger
    case 2: irq = (val <  oldval); break; //falling edge trigger
    case 3: irq = (val != oldval); break; //any edge trigger
    case 4: irq = (val == 0);      break; //low level trigger
    case 5: irq = (val == 1);      break; //high level trigger
    }
    if( !irq ) return;                    // RETURN, no irq

    qemu_set_irq( gpioS->irq, 1);

    if( gpioPin & (1 << 15) ) *pcpuInt |= pinMask; // pro cpu int enable // says bit 16 in the ref manual, is that wrong?
    if( gpioPin & (1 << 13) ) *acpuInt |= pinMask; // app cpu int enable
}

static void readInput( Esp32GpioState *gpioS, int n )
{
    //printf("Qemu: readInput %i\n", n); fflush( stdout );

    uint64_t addr = n ? 0x40 : 0x3C;

    uint32_t newVal = readReg( GPIO_OFFSET+addr );
    uint32_t oldVal = gpioS->gpio_in[n];
    gpioS->gpio_in[n] = newVal;

    uint32_t changedMask = newVal ^ oldVal;
    //printf("Input      %lu\n", changedMask ); fflush( stdout );
    if( changedMask == 0 ) return;

    int start = n ? 32 : 0;
    int end   = n ? 40 : 32;
    for( int pin=start; pin<end; ++pin )
    {
        uint32_t PinMask = 1<<pin;
        if( changedMask & PinMask )         // Pin changed
        {
            uint64_t state = (newVal & PinMask) >> start;
            inputChanged( gpioS, pin, state );
        }
    }
}

static uint64_t esp32_gpio_read( void *opaque, hwaddr addr, unsigned int size )
{
    //printf("Qemu: esp32_gpio_read\n"); fflush( stdout );

    Esp32GpioState *gpioS = ESP32_GPIO(opaque);
    uint64_t r = 0;

    if( addr < 0x88 ){
        switch( addr ){
        case 0x04: r = gpioS->gpio_out;         break; //GPIO_OUT_REG
        case 0x20: r = gpioS->gpio_enable;      break; //GPIO_ENABLE_REG
        case 0x38: r = gpioS->strap_mode;       break; //A_GPIO_STRAP:
        case 0x3C:                                     //GPIO_IN_REG
            readInput( gpioS, 0 );   // Read pins
            r = gpioS->gpio_in[0];
            break;
        case 0x40:                                     //GPIO_IN1_REG
            readInput( gpioS, 1 );   // Read pins
            r = gpioS->gpio_in[1];
            break;
        case 0x44: r = gpioS->gpio_status[0];   break; //GPIO_STATUS_REG
        case 0x50: r = gpioS->gpio_status[1];   break; //gpio_status[1]_REG
        case 0x60: r = gpioS->gpio_acpu_int[0]; break; //GPIO_ACPU_INT_REG
        case 0x68: r = gpioS->gpio_pcpu_int[0]; break; //GPIO_PCPU_INT_REG
        case 0x74: r = gpioS->gpio_acpu_int[1]; break; //GPIO_ACPU_INT1_REG
        case 0x7c: r = gpioS->gpio_pcpu_int[1]; break; //GPIO_PCPU_INT1_REG
        }
    }
    else if( addr < 0x130 ) r = gpioS->gpio_pin[(addr-0x88)/4];     //GPIO_PINXX_REG
    else if( addr < 0x530 ) r = gpioS->gpio_in_sel[(addr-0x130)/4]; //GPIO_FUNCY_IN_SEL_CFG_REG
    else if( addr < 0x5d0 ) r = gpioS->gpio_out_sel[(addr-0x530)/4];//GPIO_FUNCX_OUT_SEL_CFG_REG

    return r;
}

static void clearStatus( Esp32GpioState* gpioS, int n, uint64_t value ) // GPIO_STATUSx_W1TC_REG
{
    int start = n ? 32 : 0;
    int end   = n ? 40 : 32;

    for( int i=start; i<end; i++ )  // GPIO interrupts
    {
        uint32_t mask = 1 << (i-start);
        if( value & mask )
        {
            int int_type = (gpioS->gpio_pin[i] >> 7) & 7;
            uint32_t gpioMask = gpioS->gpio_in[n] & mask;

            if( (int_type == 4 && !gpioMask)
                || (int_type == 5 &&  gpioMask) )
                return;                        // RETURN, no clear, no irq
            //      ????????????????????
        }
    }
    gpioS->gpio_status[n]   &= ~value;
    gpioS->gpio_pcpu_int[n] &= ~value;
    gpioS->gpio_acpu_int[n] &= ~value;
    qemu_set_irq( gpioS->irq, 0 );
}

static void esp32_gpio_write( void *opaque, hwaddr addr, uint64_t value, unsigned int size )
{
    Esp32GpioState *gpioS = ESP32_GPIO( opaque );

    if( addr  < 0x20 )
    {
        uint32_t oldOut    = gpioS->gpio_out;
        switch( addr ){
        case 0x04: gpioS->gpio_out  =  value; break; // GPIO_OUT_REG
        case 0x08: gpioS->gpio_out |=  value; break; // GPIO_OUT_W1TS_REG
        case 0x0C: gpioS->gpio_out &= ~value; break; // GPIO_OUT_W1TC_REG
        }
        if( gpioS->gpio_out != oldOut) {
            if (esp32_gpio_trace_enabled()) {
                printf("[LasecSimul][GPIO] OUT addr=0x%08x old=0x%08x new=0x%08x\n",
                       GPIO_OFFSET + 0x04, oldOut, gpioS->gpio_out);
                fflush(stdout);
            }
            writeReg( GPIO_OFFSET+0x04, gpioS->gpio_out );
        }
    }
    else if( addr < 0x38 )
    {
        uint32_t oldEnable = gpioS->gpio_enable;
        switch( addr ){
        case 0x20: gpioS->gpio_enable  =  value; break; // GPIO_ENABLE_REG
        case 0x24: gpioS->gpio_enable |=  value; break; // GPIO_ENABLE_W1TS_REG
        case 0x28: gpioS->gpio_enable &= ~value; break; // GPIO_ENABLE_W1TC_REG
        }
        if( gpioS->gpio_enable != oldEnable ) {
            if (esp32_gpio_trace_enabled()) {
                printf("[LasecSimul][GPIO] ENABLE addr=0x%08x old=0x%08x new=0x%08x\n",
                       GPIO_OFFSET + 0x20, oldEnable, gpioS->gpio_enable);
                fflush(stdout);
            }
            writeReg( GPIO_OFFSET+0x20, gpioS->gpio_enable );
        }
    }
    else if( addr < 0x88 )
    {
        switch( addr ){
        case 0x38: gpioS->strap_mode      =  value; break; // A_GPIO_STRAP
        case 0x44: gpioS->gpio_status[0]  =  value; break; // GPIO_STATUS_REG
        case 0x48: gpioS->gpio_status[0] |=  value; break; // GPIO_STATUS_W1TS_REG
        case 0x4C: clearStatus( gpioS, 0, value );  break; // GPIO_STATUS_W1TC_REG
        case 0x50: gpioS->gpio_status[1]  =  value; break; // GPIO_STATUS1_REG
        case 0x54: gpioS->gpio_status[1] |=  value; break; // GPIO_STATUS1_W1TS_REG
        case 0x58: clearStatus( gpioS, 1, value );  break; // GPIO_STATUS1_W1TC_REG
        }
    }
    else if( addr < 0x130 )                                //GPIO_PINXX_REG
    {
        int func = (addr-0x88)/4;
        uint32_t oldFunc = gpioS->gpio_pin[func];
        if( oldFunc != value ){
            gpioS->gpio_pin[func] = value;
            writeReg( GPIO_OFFSET+addr, value );
        }
    }
    else if( addr < 0x530 )                                //GPIO_FUNCY_IN_SEL_CFG_REG
    {
        int func = (addr-0x130)/4;
        uint32_t oldFunc = gpioS->gpio_in_sel[func];
        if( oldFunc != value ){
            gpioS->gpio_in_sel[func] = value;
            writeReg( GPIO_OFFSET+addr, value );
        }
    }
    else if( addr < 0x5d0 )                                //GPIO_FUNCX_OUT_SEL_CFG_REG
    {
        int func = (addr-0x530)/4;
        uint32_t oldFunc = gpioS->gpio_out_sel[func];
        if( oldFunc != value ){
            gpioS->gpio_out_sel[func] = value;
            writeReg( GPIO_OFFSET+addr, value );
        }
    }
}

static const MemoryRegionOps uart_ops = {
    .read  = esp32_gpio_read,
    .write = esp32_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_gpio_reset( DeviceState *dev )
{
    /// TODO
}

static void esp32_gpio_realize( DeviceState *dev, Error **errp )
{}

static void esp32_gpio_init( Object *obj )
{
    Esp32GpioState *gpioS = ESP32_GPIO(obj);
    SysBusDevice   *sbd   = SYS_BUS_DEVICE(obj);

    /* Set the default value for the strap_mode property */
    object_property_set_int( obj, "strap_mode", ESP32_STRAP_MODE_FLASH_BOOT, &error_fatal );

    memory_region_init_io( &gpioS->iomem, obj, &uart_ops, gpioS, TYPE_ESP32_GPIO, 0x1000 );
    sysbus_init_mmio( sbd, &gpioS->iomem );
    sysbus_init_irq( sbd, &gpioS->irq );

    qdev_init_gpio_out_named( DEVICE(gpioS), &gpioS->irq     , SYSBUS_DEVICE_GPIO_IRQ, 1 );

    gpioS->gpio_in[0] = 0x1;
    gpioS->gpio_in[1] = 0x8;
}

static Property esp32_gpio_properties[] = {
    /* The strap_mode needs to be explicitly set in the instance init, thus, set the default value to 0. */
    DEFINE_PROP_UINT32("strap_mode", Esp32GpioState, strap_mode, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32_gpio_class_init( ObjectClass *klass, void *data )
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset   = esp32_gpio_reset;
    dc->realize = esp32_gpio_realize;
    device_class_set_props( dc, esp32_gpio_properties );
}

static const TypeInfo esp32_gpio_info = {
    .name          = TYPE_ESP32_GPIO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32GpioState),
    .instance_init = esp32_gpio_init,
    .class_init    = esp32_gpio_class_init,
    .class_size    = sizeof(Esp32GpioClass),
};

static void esp32_gpio_register_types(void)
{
    type_register_static( &esp32_gpio_info );
}

type_init( esp32_gpio_register_types )
