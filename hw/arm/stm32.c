/*
 * STM32 Microcontroller
 *
 * Copyright (C) 2010 Andre Beckus
 *
 * Implementation based on ST Microelectronics "RM0008 Reference Manual Rev 10"
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

//#include "stm32devices.h"
#include "hw/arm/stm32.h"
#include "exec/address-spaces.h"
#include "exec/gdbstub.h"
#include "hw/arm/armv7m.h"
//#include "hw/misc/unimp.h"
#include "qapi/error.h"
#include "sysemu/blockdev.h" // drive_get

#include "sysemu/sysemu.h"
#include "hw/boards.h"
#include "hw/irq.h"
#include "hw/arm/boot.h"

//#include "sysemu/cpu-timers.h"
#include "../softmmu/simuliface.h"


/* DEFINITIONS */

#define ARMV7M_NUM_IRQS 61

#define IOMEM_BASE_ADDRESS 0x40000000
#define IOMEM_SIZE         0x00023400

struct Stm32F1xxState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    char *cpu_type;
    char *kernel_file;
    uint64_t flash_size;
    uint64_t ram_size;
    uint32_t osc_freq;
    uint32_t osc32_freq;

    ARMv7MState armv7m;

    MemoryRegion sram;
    MemoryRegion ioMem;
    MemoryRegion flash;
    MemoryRegion flash_alias;

    qemu_irq irqs[ARMV7M_NUM_IRQS];

   // DeviceState *gpio[7];
   // DeviceState *uart[5];
   // DeviceState *i2c[2];
   // DeviceState *spi[3];
   //Stm32Timer* timer[8];
   // DeviceState *adc[3];
   // DeviceState *afio;

    Clock *sysclk;
    Clock *refclk;
};

Stm32F1xxState* mcu;

typedef struct STM32Model{
    uint32_t oscFreq;
    uint32_t flashSize;
    uint32_t sramSize;
} STM32Model_t;

//Stm32Timer* stm32_get_timer( int n )
//{
//    n -= 1;
//    //if( number > (mcu->model.timerM & 1<<number) ) return NULL;
//    if( n > 8 ) return NULL;
//    return mcu->timer[n];
//}

void setInterrupt(void)
{
    uint8_t irqNumber = m_arena->irqNumber;
    uint8_t irqLevel  = m_arena->irqLevel;

    m_arena->irqNumber = 0;
    //m_arena->irqLevel  = 0;

    qemu_irq irq = mcu->irqs[irqNumber];
    //printf("Qemu: setInterrupt %i %i", irqNumber, level ); fflush( stdout );
    if( irq ) qemu_set_irq( irq, irqLevel );
    //else      printf(" Not found\n" );
}

static uint64_t stm32_ioMem_read( void *arg, hwaddr offset, unsigned size )
{
    uint64_t value = readReg( offset );
    //printf("Qemu: stm32_ioMem_read %lu %lu\n", offset, value ); fflush( stdout );
    return value;
}

static void stm32_ioMem_write( void *arg, hwaddr offset, uint64_t data, unsigned size )
{
    //printf("\nQemu: stm32_ioMem_write %lu %lu\n", offset, data ); fflush( stdout );
    writeReg( offset, data );
}

static const MemoryRegionOps stm32_ioMem_ops = {
    .read  = stm32_ioMem_read,
    .write = stm32_ioMem_write,
    .endianness = DEVICE_NATIVE_ENDIAN
};

static STM32Model_t stm32_f10xx_get_model( uint32_t id )
{
    //printf("stm32_get_model %i\n", id); fflush( stdout );
    uint32_t oscFreq ;
    uint16_t flashSize;
    uint8_t  ramSize;

    uint8_t family = (id & 0xFF0000) >> 16;
    uint8_t variant = id & 0x0000FF;

    //printf("stm32_get_model %i %i\n", family, variant); fflush( stdout );

    switch( family )
    {
    case 1: // F101
    {
        oscFreq = 4500000; break;
        switch( variant ) // 4=0, 6=1, 8=2, B=3, C=4, D=5, E=6, F=7, G=8
        {
        case 0: flashSize =  16; ramSize =  6; break; // 4
        case 1: flashSize =  32; ramSize = 10; break; // 6
        case 2: flashSize =  64; ramSize = 20; break; // 8
        case 3: flashSize = 128; ramSize = 20; break; // B
        case 4: flashSize = 256; ramSize = 48; break; // C
        case 5: flashSize = 384; ramSize = 64; break; // D
        case 6: flashSize = 512; ramSize = 64; break; // E

        default: flashSize = 512; ramSize = 64; break; // E
        }
    }break;
    case 2: // F102
    {
        oscFreq = 6000000; break;
        switch( variant ) // 4=0, 6=1, 8=2, B=3, C=4, D=5, E=6, F=7, G=8
        {
        case 0: flashSize =  16; ramSize =  6; break; // 4
        case 1: flashSize =  32; ramSize = 10; break; // 6
        case 2: flashSize =  64; ramSize = 20; break; // 8
        case 3: flashSize = 128; ramSize = 20; break; // B

        default: flashSize = 128; ramSize = 20; break; // B
        }
    }break;
    case 3: // F103
    {
        oscFreq = 8000000; break;
        switch( variant ) // 4=0, 6=1, 8=2, B=3, C=4, D=5, E=6, F=7, G=8
        {
        case 0: flashSize =  16; ramSize =  6; break; // 4
        case 1: flashSize =  32; ramSize = 10; break; // 6
        case 2: flashSize =  64; ramSize = 20; break; // 8
        case 3: flashSize = 128; ramSize = 20; break; // B
        case 4: flashSize = 256; ramSize = 48; break; // C
        case 5: flashSize = 384; ramSize = 64; break; // D
        case 6: flashSize = 512; ramSize = 64; break; // E

        default: flashSize = 128; ramSize = 20; break; // B
        }
    } break;
    default:
        oscFreq = 8000000; flashSize = 128; ramSize = 20; break; // F103x8
    }

    STM32Model_t model = { oscFreq, flashSize*1024, ramSize*1024 };
    return model;
}

/* COMMON */

void stm32_hw_warn(const char *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    fprintf(stderr, "qemu stm32: hardware warning: ");
    vfprintf(stderr, fmt, ap);
    fprintf(stderr, "\n");
    cpu_dump_state(first_cpu, stderr, 0);
    va_end(ap);
}

/* INITIALIZATION */

static DeviceState *stm32_init_periph( DeviceState *dev, stm32_periph_t periph, hwaddr addr, qemu_irq irq)
{
    // qdev_init_nofail(dev);
    sysbus_mmio_map( SYS_BUS_DEVICE(dev), 0, addr );
    if( irq ) sysbus_connect_irq( SYS_BUS_DEVICE(dev), 0, irq );

    return dev;
}

static void create_timer( Object *stm32_container, stm32_periph_t periph, int timer_num,
                          DeviceState *rcc_dev, hwaddr addr, qemu_irq irq, Error **errp) {
    char child_name[9];
    DeviceState *timer_dev = qdev_new("stm32-timer");

    QDEV_PROP_SET_PERIPH_T( timer_dev, "periph", periph );

    stm32_timer_set_rcc( STM32_TIMER(timer_dev), STM32_RCC(rcc_dev) );
    stm32_timer_set_number( STM32_TIMER(timer_dev), timer_num );

    snprintf( child_name, sizeof(child_name), "timer[%i]", timer_num );
    object_property_add_child( stm32_container, child_name, OBJECT(timer_dev) );

    if( !sysbus_realize(SYS_BUS_DEVICE(timer_dev), errp) )
        return;

    stm32_init_periph( timer_dev, periph, addr, NULL );
    sysbus_connect_irq( SYS_BUS_DEVICE(timer_dev), 0, irq);
}

static void stm32f10xx_soc_initfn( Object *obj )
{
    Stm32F1xxState *s = STM32F1XX_SOC(obj);

    object_initialize_child(obj, "armv7m", &s->armv7m, TYPE_ARMV7M);

    s->sysclk = qdev_init_clock_in(DEVICE(s), "sysclk", NULL, NULL, 0);
    s->refclk = qdev_init_clock_in(DEVICE(s), "refclk", NULL, NULL, 0);
}

#define SYSCLK_FRQ 24000000ULL /* Main SYSCLK frequency in Hz (24MHz) */

static void stm32_f10xx_init_machine( MachineState *machine )
{
    DeviceState *dev = qdev_new(TYPE_STM32F1XX_SOC);
    mcu = STM32F1XX_SOC( dev );                          /// MCU DECLARATION --------------------------------------

    Clock* sysclk = clock_new( OBJECT(machine), "SYSCLK");
    clock_set_hz( sysclk, SYSCLK_FRQ );

    STM32Model_t model = stm32_f10xx_get_model( m_arena->regData ); // 0x030202

    qdev_prop_set_string( dev, "cpu-type", ARM_CPU_TYPE_NAME("cortex-m3") );

    //if( machine->kernel_filename )
    //    qdev_prop_set_string(dev, "kernel-file", machine->kernel_filename );

    qdev_prop_set_uint64(  dev, "flash_size", model.flashSize );
    qdev_prop_set_uint64(  dev, "ram_size"  , model.sramSize );
    qdev_prop_set_uint32(  dev, "osc_freq"  , model.oscFreq );
    qdev_prop_set_uint32(  dev, "osc32_freq", 32768 );
    qdev_connect_clock_in( dev, "sysclk"    , sysclk);
    sysbus_realize_and_unref( SYS_BUS_DEVICE(dev), &error_fatal );

    armv7m_load_kernel( ARM_CPU(first_cpu), machine->kernel_filename, 0, mcu->flash_size );
}

static void stm32f10xx_soc_realize( DeviceState *dev_soc, Error **errp )
{
    Stm32F1xxState *s = STM32F1XX_SOC(dev_soc);
    MemoryRegion *system_memory = get_system_memory();

    if( clock_has_source(s->refclk) ) {
        error_setg( errp, "refclk clock must not be wired up by the board code");
        return;
    }

    if( !clock_has_source(s->sysclk) ) {
        error_setg( errp, "sysclk clock must be wired up by the board code");
        return;
    }

    /* The refclk always runs at frequency HCLK / 8 */
    clock_set_mul_div( s->refclk, 8, 1 );
    clock_set_source( s->refclk, s->sysclk );

    //MemoryRegion *sram = g_new(MemoryRegion, 1);
    memory_region_init_ram( &s->sram, NULL, "stm32.sram", s->ram_size, &error_fatal );
    memory_region_add_subregion( system_memory, 0x20000000, &s->sram );

    //if( s->kernel_file) // Use Legacy mode without reset support
    //{
    //    printf("stm32f10xx_soc_realize Legacy mode\n"); fflush( stdout );
    //    MemoryRegion *flash_alias_mem = g_malloc(sizeof(MemoryRegion));
    //    /* The STM32 family stores its Flash memory at some base address in memory
    //     * (0x08000000 for medium density devices), and then aliases it to the
    //     * boot memory space, which starts at 0x00000000 (the "System Memory" can
    //     * also be aliased to 0x00000000, but this is not implemented here). The
    //     * processor executes the code in the aliased memory at 0x00000000.  We need
    //     * to make a QEMU alias so that reads in the 0x08000000 area are passed
    //     * through to the 0x00000000 area. Note that this is the opposite of real
    //     * hardware, where the memory at 0x00000000 passes reads through the "real"
    //     * flash memory at 0x08000000, but it works the same either way. */
    //    /* TODO: Parameterize the base address of the aliased memory. */
    //    memory_region_init_alias(flash_alias_mem, NULL, "stm32-flash-alias-mem", system_memory, 0, s->flash_size);
    //    memory_region_add_subregion(system_memory, STM32_FLASH_ADDR_START, flash_alias_mem);
    //}
    //else               // Use new FLASH mode with reset support
    {
        //printf("stm32f10xx_soc_realize FLASH mode\n"); fflush( stdout );
        DriveInfo *dinfo = drive_get( IF_PFLASH, 0, 0 );
        if( dinfo )
          stm32_flash_register( blk_by_legacy_dinfo(dinfo), STM32_FLASH_ADDR_START, s->flash_size );

        MemoryRegionSection mrs = memory_region_find( system_memory, STM32_FLASH_ADDR_START, 4 /*WORD_ACCESS_SIZE*/);
        //MemoryRegion *flash_alias_mem = g_new(MemoryRegion, 1);
        memory_region_init_alias( &s->flash_alias , NULL, "stm32-flash-alias-mem", mrs.mr, 0, s->flash_size);
        memory_region_add_subregion( system_memory, 0, &s->flash_alias );
        /// memory_region_add_subregion( system_memory, STM32_FLASH_ADDR_START, &s->flash);
    }

    /* Init ARMv7m */
    DeviceState *armv7m = DEVICE(&s->armv7m);
    qdev_prop_set_uint32( armv7m, "num-irq", ARMV7M_NUM_IRQS );            /// TODO: missing Timer IRQs
    qdev_prop_set_string( armv7m, "cpu-type", s->cpu_type);
    qdev_prop_set_bit( armv7m, "enable-bitband", true);
    qdev_connect_clock_in( armv7m, "cpuclk", s->sysclk);
    qdev_connect_clock_in( armv7m, "refclk", s->refclk);

    object_property_set_link( OBJECT(&s->armv7m ), "memory", OBJECT(get_system_memory()), &error_abort);

    if( !sysbus_realize(SYS_BUS_DEVICE(&s->armv7m), errp))
      return;

    // Init Peripheral region
    memory_region_init_io( &s->ioMem, NULL, &stm32_ioMem_ops, s, "STM32F1XX.perif", IOMEM_SIZE );
    memory_region_add_subregion( system_memory, IOMEM_BASE_ADDRESS, &s->ioMem );


    // Clock
    Object *stm32_container = container_get( qdev_get_machine(), "/stm32" );
    DeviceState *rcc_dev = qdev_new("stm32-rcc");
    qdev_prop_set_uint32( rcc_dev, "osc_freq"  , s->osc_freq);
    qdev_prop_set_uint32( rcc_dev, "osc32_freq", s->osc32_freq);
    stm32_rcc_set_sysclk( STM32_RCC(rcc_dev), s->sysclk);
    object_property_add_child( stm32_container, "rcc", OBJECT(rcc_dev));
    if( !sysbus_realize( SYS_BUS_DEVICE(rcc_dev), errp) )
        return;
    stm32_init_periph( rcc_dev, STM32_RCC_PERIPH, 0x40021000, qdev_get_gpio_in(armv7m, STM32_RCC_IRQ));


    // Interrupts
    for( int i=0; i<ARMV7M_NUM_IRQS; ++i )
    {
        sysbus_init_irq( SYS_BUS_DEVICE(dev_soc), &s->irqs[i] );
        sysbus_connect_irq( SYS_BUS_DEVICE(dev_soc), i, qdev_get_gpio_in(armv7m, i) );
    }

    /* Timer 1 has four interrupts but only the TIM1 Update interrupt is
   * implemented. */
    /*qemu_irq tim1_irqs[] = { pic[TIM1_BRK_IRQn], pic[TIM1_UP_IRQn],
   * pic[TIM1_TRG_COM_IRQn], pic[TIM1_CC_IRQn]};*/
    /*if( mcu->model.timerM & 1<<0 )*/ create_timer( stm32_container, STM32_TIM1, 1, rcc_dev, 0x40012C00, qdev_get_gpio_in(armv7m, TIM1_UP_IRQn), errp);
    /*if( mcu->model.timerM & 1<<1 )*/ create_timer( stm32_container, STM32_TIM2, 2, rcc_dev, 0x40000000, qdev_get_gpio_in(armv7m, TIM2_IRQn), errp);
    /*if( mcu->model.timerM & 1<<2 )*/ create_timer( stm32_container, STM32_TIM3, 3, rcc_dev, 0x40000400, qdev_get_gpio_in(armv7m, TIM3_IRQn), errp);
    /*if( mcu->model.timerM & 1<<3 )*/ create_timer( stm32_container, STM32_TIM4, 4, rcc_dev, 0x40000800, qdev_get_gpio_in(armv7m, TIM4_IRQn), errp);
    /*if( mcu->model.timerM & 1<<4 )*/ create_timer( stm32_container, STM32_TIM5, 5, rcc_dev, 0x40000C00, qdev_get_gpio_in(armv7m, TIM5_IRQn), errp);
    /*if( mcu->model.timerM & 1<<5 )*/ create_timer( stm32_container, STM32_TIM6, 6, rcc_dev, 0x40001000, qdev_get_gpio_in(armv7m, TIM6_DAC_IRQn), errp);
    /*if( mcu->model.timerM & 1<<6 )*/ create_timer( stm32_container, STM32_TIM7, 7, rcc_dev, 0x40001400, qdev_get_gpio_in(armv7m, TIM7_IRQn), errp);
    /*if( mcu->model.timerM & 1<<7 )*/ create_timer( stm32_container, STM32_TIM8, 8, rcc_dev, 0x40013400, qdev_get_gpio_in(armv7m, TIM8_UP_TIM13_IRQn), errp);

    /* FLASH regs */
    DeviceState *flash_regs = qdev_new( TYPE_STM32_FLASH_REGS );
    stm32_init_periph( flash_regs, STM32_FLASH_REGS, 0x40022000, NULL );
}

static Property stm32f10xx_soc_properties[] = {
    DEFINE_PROP_STRING("cpu-type"   , Stm32F1xxState, cpu_type),
    DEFINE_PROP_STRING("kernel-file", Stm32F1xxState, kernel_file),
    DEFINE_PROP_UINT64("flash_size" , Stm32F1xxState, flash_size, 0x00020000),
    DEFINE_PROP_UINT64("ram_size"   , Stm32F1xxState, ram_size,   0x00005000),
    DEFINE_PROP_UINT32("osc_freq"   , Stm32F1xxState, osc_freq,   8000000),
    DEFINE_PROP_UINT32("osc32_freq" , Stm32F1xxState, osc32_freq, 32768),
    DEFINE_PROP_END_OF_LIST(),
};

static void stm32f10xx_soc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = stm32f10xx_soc_realize;
    device_class_set_props( dc, stm32f10xx_soc_properties );
}

static const TypeInfo stm32f10xx_soc_info = {
    .name = TYPE_STM32F1XX_SOC,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Stm32F1xxState),
    .instance_init = stm32f10xx_soc_initfn,
    .class_init = stm32f10xx_soc_class_init,
};

static void stm32f10xx_soc_types(void) {
    type_register_static( &stm32f10xx_soc_info );
}

static void stm32_f10xx_machine_init(MachineClass *mc)
{
    mc->desc = "STM32F10xx soc machine";
    mc->init = stm32_f10xx_init_machine;
    mc->block_default_type = IF_IDE;
    mc->ignore_memory_transaction_failures = true;
}

DEFINE_MACHINE("stm32-f10xx", stm32_f10xx_machine_init)

type_init(stm32f10xx_soc_types)

