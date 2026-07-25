/*
 * STM32 Microcontroller Timer module
 *
 * Copyright (C) 2010 Andrew Hankins
 *
 * Source code based on pl011.c
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

/*
 * Modified by Santiago Gonzalez 2025
 */

#include <math.h>

#include "hw/arm/stm32.h"
#include "migration/vmstate.h"
#include "hw/sysbus.h"
#include "qemu/timer.h"
#include "sysemu/sysemu.h"
#include "hw/irq.h"
#include "hw/ptimer.h"

#include "../softmmu/simuliface.h"


/* DEFINITIONS*/

/* See the README file for details on these settings. */
//#define DEBUG_STM32_TIMER

#ifdef DEBUG_STM32_TIMER
#define DPRINTF(fmt, ...)                                       \
    do { fprintf(stderr, "STM32_TIMER: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...)
#endif

#define TIMER_CR1_OFFSET   0x00
#define TIMER_CR2_OFFSET   0x04
#define TIMER_SMCR_OFFSET  0x08
#define TIMER_DIER_OFFSET  0x0c
#define TIMER_SR_OFFSET    0x10
#define TIMER_EGR_OFFSET   0x14
#define TIMER_CCMR1_OFFSET 0x18
#define TIMER_CCMR2_OFFSET 0x1c
#define TIMER_CCER_OFFSET  0x20
#define TIMER_CNT_OFFSET   0x24
#define TIMER_PSC_OFFSET   0x28
#define TIMER_ARR_OFFSET   0x2c
#define TIMER_RCR_OFFSET   0x30
#define TIMER_CCR1_OFFSET  0x34
#define TIMER_CCR2_OFFSET  0x38
#define TIMER_CCR3_OFFSET  0x3c
#define TIMER_CCR4_OFFSET  0x40
#define TIMER_BDTR_OFFSET  0x44
#define TIMER_DCR_OFFSET   0x48
#define TIMER_DMAR_OFFSET  0x4C

#define CR1_CEN 1<<0
#define CR1_OPM 1<<3

#define STM32_TIM_SYNC "stm32_tim_sync"

enum {
    TIMER_UP_COUNT   = 0,
    TIMER_DOWN_COUNT = 1
};

typedef struct Stm32Timer Stm32Timer;

typedef struct Stm32Channel {

    uint16_t CCR; // Capture/Compare register
    uint8_t CCMR;

    uint8_t CCS;
    uint8_t OCM;

    uint8_t IC_presc; // Input Capture Prescaler

    uint8_t CCE;      // Input/Output Enable
    uint8_t inverted; // Polarity

    uint8_t number;

    uint8_t ioPort;
    uint8_t ioPin;
    bool lastState;

    Stm32Timer* parent_timer;
    QEMUTimer   match_timer;

} Stm32Channel;

struct Stm32Timer {

    /* Inherited */
    SysBusDevice busdev;

    MemoryRegion  iomem;
    ptimer_state *timer;
    qemu_irq      irq;

    /* Properties */
    stm32_periph_t periph;

    Stm32Rcc *stm32_rcc;
    //Stm32Gpio **stm32_gpio;
    //Stm32Afio *stm32_afio;

    Stm32Channel channels[4];

    int number;

    uint8_t enabled;
    //int period;
    uint8_t direction;
    //int itr;
    uint8_t oneShot;
    uint8_t bidirectional;

    double frequency;
    double ps_per_tick;

    uint16_t cr1;
    uint16_t cr2;  //Extended modes not supported
    uint16_t smcr; //Slave mode not supported
    uint16_t dier;
    uint16_t sr;
    uint16_t egr;
    uint16_t ccmr1;
    uint16_t ccmr2;
    uint16_t ccer;
    /* uint16_t cnt; //Handled by ptimer */
    uint16_t psc;
    uint16_t arr;
    uint16_t rcr; //Repetition count not supported

    uint16_t bdtr; //Break and deadtime not supported
    uint16_t dcr;  //DMA mode not supported
    uint16_t dmar; //DMA mode not supported
};

// -------------------------------------------------------
// -------------------------------------------------------

static inline void Stm32_channel_setPin( Stm32Channel* ch, bool state )
{
    //printf("Stm32_channel_setPin %i %i\n", ch->number, state );

    uint16_t data = ch->number | (state ? 1<<8 : 0);
    uint64_t addr = ch->parent_timer->iomem.addr & 0x000FFFFF;

    //printf("Stm32_channel_setPin %i %lu\n", data, addr );
    writeReg( addr, data );

    /// m_arena->simuAction = ARM_ALT_OUT;
    /// m_arena->data16 = state;   // 1 bit per pin: 1 = High
    /// m_arena->data8  = ch->ioPort;  // We have to send Port number, PortA = 1
    /// m_arena->mask8  = ch->ioPin;

    /// doAction();
}

static void Stm32_channel_match_event( void* opaque )
{
    Stm32Channel* ch = (Stm32Channel*) opaque;

    bool state = ch->lastState;

    switch( ch->OCM ) {
    case 0: return;  // Frozen: No output
    case 1: state = true;   break;   // OCxREF High at match
    case 2: state = false;  break;   // OCxREF Low  at match
    case 3: state = !state; break;   // OCxREF Toggle at match
    case 4:                 return;  // OCxREF is forced low
    case 5:                 return;  // OCxREF is forced high
    case 6: {                        // PWM mode 1, Birirectional: High below math
        if( ch->parent_timer->direction == TIMER_UP_COUNT )
            state = true;
        else state = false;
    }break;
    case 7: {                        // PWM mode 2, Birirectional: Low below math
        if( ch->parent_timer->direction == TIMER_UP_COUNT )
            state = false;
        else state = true;
    }break;
    default: break;
    }
    if( ch->lastState == state ) return;
    ch->lastState = state;
    //printf("Stm32_channel_match_event %i OCM: %i at %lu\n", ch->number, ch->OCM, getQemu_ps() );fflush( stdout );

    Stm32_channel_setPin( ch, state );
}

static void Stm32_channel_ovf_event( Stm32Channel* ch )
{
    /// TODO: set Interrupt flag

    double match_count;
    Stm32Timer* p_timer = ch->parent_timer;

    if( p_timer->direction == TIMER_UP_COUNT ) match_count = ch->CCR;
    else                                       match_count = p_timer->arr - ch->CCR;

    double nextTime_ns = match_count * p_timer->ps_per_tick / 1000; // Convert to ns

    if( nextTime_ns == 0 ) return;

    double time = getQemu_ps();
    time = time/1000 +  nextTime_ns;
    timer_mod_ns( &ch->match_timer, time );    // Schedule next match event

    //printf("Stm32_channel_ovf_event %i %lu\n", ch->number, getQemu_ps() );fflush( stdout );
    //if( !ch->CCE ) return;

    bool state = ch->lastState;
    switch( ch->OCM ) {
    case 0: return;  // Frozen: No output
    case 1: state = false; break;   // OCxREF High at match
    case 2: state = true;  break;   // OCxREF Low  at match
    case 3:                return;  // OCxREF Toggle at match, Nothing on OVF
    case 4:                return;  // OCxREF is forced low
    case 5:                return;  // OCxREF is forced high
    case 6: {                        // PWM mode 1, Birirectional: High below math
        if( ch->parent_timer->direction == TIMER_UP_COUNT )
            state = false;
        else state = true;
    }break;
    case 7: {                        // PWM mode 2, Birirectional: Low below math
        if( ch->parent_timer->direction == TIMER_UP_COUNT )
            state = true;
        else state = false;
    }break;
    default: break;
    }
    if( ch->lastState == state ) return;
    ch->lastState = state;
    Stm32_channel_setPin( ch, state );
}

static inline void Stm32_channel_write_CCER( Stm32Channel* ch, uint8_t ccer )
{
    uint8_t cce = ccer & 0b01; // Input/Output Enable
    if( ch->CCE != cce )
    {
        ch->CCE = cce;
        /// Set actual pin out enabled
    }
    ch->inverted = ccer & 0b10; // Polarity
}

static inline void Stm32_channel_write_CCMR( Stm32Channel* ch, uint8_t ccmr )
{
    if( ch->CCMR == ccmr ) return;
    ch->CCMR = ccmr;

    uint8_t ccs = ccmr & 0b11;
    if( ch->CCS != ccs )
    {
        ch->CCS = ccs;

        switch( ccs ) {
        case 0: break; // Output
        case 1: break; // Input TI1
        case 2: break; // Input TI2
        case 3: break; // Input TRC
        default: break;
        }
    }

    if( ccs ) // Input
    {
        uint8_t icpsc = (ccmr & 0b00001100) >> 2;
        ch->IC_presc = pow( 2, icpsc );

        //uint8_t icf   = (ccmr & 0b11110000) >> 4;  // Filter not implemented
    }
    else      // Output
    {
        //uint8_t ocfe = (ccmr & 0b00000100) >> 2; // Output compare fast enable
        //uint8_t ocpe = (ccmr & 0b00001000) >> 3; // Output compare preload enable
        uint8_t ocm  = (ccmr & 0b01110000) >> 4;
        if( ch->OCM != ocm )
        {                    // OCxREF: signal from which OCx and OCxN are derived

            /// FIXME: should we initialize pin staes in all cases?

            ch->OCM = ocm;
            switch( ocm ) {
            case 0:                                    break;   // Frozen: No output
            case 1:                                    break;   // OCxREF High at match
            case 2:                                    break;   // OCxREF Low  at match
            case 3:                                    break;   // OCxREF Toggle at match
            case 4: Stm32_channel_setPin( ch, false ); break;   // OCxREF is forced low
            case 5: Stm32_channel_setPin( ch, true );  break;   // OCxREF is forced high
            case 6:                                    break;   // PWM mode 1, Birirectional: High below math
            case 7:                                    break;   // PWM mode 2, Birirectional: Low below math
            default: break;
            }
        }
        //uint8_t occe = (ccmr & 0b10000000) >> 7;
    }

    //printf("Stm32_channel_write_CCMR ch: %i %i %i\n", ch->number, ch->CCS, ch->OCM);fflush( stdout );
}

static void Stm32_channel_init( Stm32Channel* ch, Stm32Timer* timer, uint8_t ch_number )
{
    timer_init_full( &ch->match_timer, NULL, QEMU_CLOCK_VIRTUAL, 1, 0, Stm32_channel_match_event, ch );

    ch->number = ch_number;
    ch->parent_timer = timer;
    // Initialize variables
}

// -------------------------------------------------------
// -------------------------------------------------------


static void stm32_timer_updt_freq( Stm32Timer *s )
{
    // Why do we need to multiply the frequency by 2? This is how real hardware behaves.
    double freq = 2*(double)stm32_rcc_get_periph_freq(s->stm32_rcc, s->periph) / (double)(s->psc + 1);

    if( s->frequency == freq ) return;
    s->frequency = freq;

    double ps_tick = (double)freq/1000000;
    ps_tick = 1000000/ps_tick;
    s->ps_per_tick = ps_tick;

    if( freq != 0 ) {
        ptimer_set_freq( s->timer, freq );
        //printf( "%s Update freq = %d presc = %d\n", stm32_periph_name(s->periph), clk_freq, (s->psc + 1) );
    }
}

static void stm32_timer_clk_irq_handler( void *opaque, int n, int level )
{
    Stm32Timer *s = (Stm32Timer *)opaque;
    //printf("clk_irq %i %i\n", n, level);
    assert(n == 0);
    ptimer_transaction_begin(s->timer);
    stm32_timer_updt_freq(s);
    ptimer_transaction_commit(s->timer);
}

static uint32_t stm32_timer_get_count( Stm32Timer *s )
{
    uint64_t cnt = ptimer_get_count( s->timer );

    if (s->direction == TIMER_UP_COUNT) return s->arr - cnt;
    else                                return cnt;
}

static void stm32_timer_set_count( Stm32Timer *s, uint16_t cnt )
{
    ptimer_transaction_begin( s->timer );
    if (s->direction == TIMER_UP_COUNT) ptimer_set_count( s->timer, s->arr - cnt);
    else                                ptimer_set_count( s->timer, cnt );
    ptimer_transaction_commit( s->timer );

    DPRINTF("%s cnt = %x\n", stm32_periph_name(s->periph), stm32_timer_get_count(s));
}

static void stm32_timer_update_UIF( Stm32Timer *s, uint8_t value )
{
    //printf("stm32_timer_update_UIF %i\n", s->sr & 0x1);
    if( !(s->sr & 0x1) )
    {
        //s->sr &= ~0x1; /* update interrupt flag in status reg */
        s->sr |= (value & 0x1);
        s->dier |= 0x01;                 //FIXME what value must be used? RESET = 0
        qemu_set_irq( s->irq, value );
    }
}

static void stm32_timer_ovf( void *opaque ) // overflow
{
    Stm32Timer *s = (Stm32Timer*)opaque;

    if( !s->enabled ) return;
    DPRINTF("%s Alarm raised\n", stm32_periph_name(s->periph));

    for( int i=0; i<4; ++i ) Stm32_channel_ovf_event( &s->channels[i] );

    //static uint64_t lastTime=0;
    //uint64_t qemuTime_ns = qemu_clock_get_ns( QEMU_CLOCK_VIRTUAL ); // ns
    //printf("%s overflow at %lu ns\n", stm32_periph_name(s->periph), qemuTime_ns - lastTime);
    ////printf("%lu\n", qemuTime_ns - lastTime);
    //printf("%i\n", s->arr);
    //lastTime = qemuTime_ns;

    //s->itr = 1;
    stm32_timer_update_UIF( s, 1 );

    //ptimer_set_count( s->timer, s->arr ); // Reset Qemu timer directly (it is count down)

    if( s->bidirectional )
    {
        if( s->direction == TIMER_UP_COUNT ) s->direction = TIMER_DOWN_COUNT;
        else                                 s->direction = TIMER_UP_COUNT;
    }

    if( s->oneShot ) s->cr1 &= ~CR1_CEN; // One shot, clear Enable flag
}

static void stm32_timer_updt_period( Stm32Timer *s ) // Set overflow period
{
    //s->period = s->arr+1;
    ptimer_transaction_begin( s->timer );
    ptimer_set_limit( s->timer, s->arr, 1 );
    ptimer_transaction_commit( s->timer );
}

static inline void stm32_timer_write_CR1( Stm32Timer *s, uint64_t value )
{
    s->cr1 = value & 0x3FF;

    uint8_t enabled = value & CR1_CEN;
    uint8_t oneShot = value & CR1_OPM;

    if( s->enabled != enabled || s->oneShot != oneShot )
    {
        s->enabled = enabled;
        s->oneShot = oneShot;

        if( !enabled ) //Can't switch from edge-aligned to center-aligned if enabled (CEN=1)
        {
            uint8_t CMS = (value & 0b1100000) >> 5;

            switch( CMS ) {
            case 0: s->bidirectional = 0; break;
            case 1: //break;
            case 2: //break;
            case 3: s->bidirectional = 1; break;
            default: break;
            }
        }

        ptimer_transaction_begin(s->timer);
        if( enabled ) ptimer_run( s->timer, s->oneShot ); // DPRINTF("%s Enabling timer\n", stm32_periph_name(s->periph));
        else          ptimer_stop( s->timer );            // DPRINTF("%s Disabling timer\n", stm32_periph_name(s->periph));
        ptimer_transaction_commit(s->timer);
    }
    if( !s->bidirectional )
    {
        if( s->cr1 & 1<<4 ) s->direction = TIMER_DOWN_COUNT; // DIR bit
        else                s->direction = TIMER_UP_COUNT;
    }
    DPRINTF("%s cr1 = %x\n", stm32_periph_name(s->periph), s->cr1);
}

static inline void stm32_timer_write_CCER( Stm32Timer *s, uint16_t ccer )
{
    if( s->ccer == ccer ) return;
    s->ccer = ccer;

    for( int i=0; i<4; ++i )
    {
        Stm32_channel_write_CCER( &s->channels[i], ccer & 0x0F );
        ccer >>= 4;
    }
    DPRINTF("%s ccer = %x\n", stm32_periph_name(s->periph), s->ccer);
}

static uint64_t stm32_timer_read(void *opaque, hwaddr offset, unsigned size)
{
    Stm32Timer *s = (Stm32Timer*)opaque;

    uint64_t val = 0;

    switch( offset )
    {
    case TIMER_CR1_OFFSET:   val = s->cr1;                    break; //DPRINTF("%s cr1 = %x\n", stm32_periph_name(s->periph), s->cr1); break;
    case TIMER_CR2_OFFSET:   val = s->cr2;                    break; //qemu_log_mask(LOG_GUEST_ERROR, "stm32_timer: CR2 not supported"); break;
    case TIMER_SMCR_OFFSET:  val = s->smcr;                   break; //qemu_log_mask(LOG_GUEST_ERROR, "stm32_timer: SMCR not supported"); break;
    case TIMER_DIER_OFFSET:  val = s->dier;                   break; //DPRINTF("%s dier = %x\n", stm32_periph_name(s->periph), s->dier); break;
    case TIMER_SR_OFFSET:    val = s->sr;                     break; //DPRINTF("%s sr = %x\n", stm32_periph_name(s->periph), s->sr); break;
    case TIMER_EGR_OFFSET:   val = s->egr;                    break; //qemu_log_mask(LOG_GUEST_ERROR, "stm32_timer: EGR write only"); break;
    case TIMER_CCMR1_OFFSET: val = s->ccmr1;                  break; //DPRINTF("%s ccmr1 = %x\n", stm32_periph_name(s->periph), s->ccmr1); break;
    case TIMER_CCMR2_OFFSET: val = s->ccmr2;                  break; //DPRINTF("%s ccmr2 = %x\n", stm32_periph_name(s->periph), s->ccmr2); break;
    case TIMER_CCER_OFFSET:  val = s->ccer;                   break; //DPRINTF("%s ccer = %x\n", stm32_periph_name(s->periph), s->ccer); break;
    case TIMER_CNT_OFFSET:   val = stm32_timer_get_count(s);  break; //DPRINTF("%s cnt = %x\n", stm32_periph_name(s->periph), stm32_timer_get_count(s)); break;
    case TIMER_PSC_OFFSET:   val = s->psc;                    break; //DPRINTF("%s psc = %x\n", stm32_periph_name(s->periph), s->psc); break;
    case TIMER_ARR_OFFSET:   val = s->arr;                    break; //DPRINTF("%s arr = %x\n", stm32_periph_name(s->periph), s->arr); break;
    case TIMER_RCR_OFFSET:   val = s->rcr;                    break; //qemu_log_mask(LOG_GUEST_ERROR, "stm32_timer: RCR not supported"); break;
    case TIMER_CCR1_OFFSET:  val = s->channels[0].CCR;        break; //DPRINTF("%s ccr1 = %x\n", stm32_periph_name(s->periph), s->ccr1); break;
    case TIMER_CCR2_OFFSET:  val = s->channels[1].CCR;        break; //DPRINTF("%s ccr2 = %x\n", stm32_periph_name(s->periph), s->ccr2); break;
    case TIMER_CCR3_OFFSET:  val = s->channels[2].CCR;        break; //DPRINTF("%s ccr3 = %x\n", stm32_periph_name(s->periph), s->ccr3); break;
    case TIMER_CCR4_OFFSET:  val = s->channels[3].CCR;        break; //DPRINTF("%s ccr4 = %x\n", stm32_periph_name(s->periph), s->ccr4); break;
    case TIMER_BDTR_OFFSET:  val = s->bdtr;                   break; //qemu_log_mask(LOG_GUEST_ERROR, "stm32_timer: BDTR not supported"); break;
    case TIMER_DCR_OFFSET:   val = s->dcr;                    break; //qemu_log_mask(LOG_GUEST_ERROR, "stm32_timer: DCR not supported"); break;
    case TIMER_DMAR_OFFSET:  val = s->dmar;                   break; //qemu_log_mask(LOG_GUEST_ERROR, "stm32_timer: DMAR not supported"); break;
    default:                                                  break; //qemu_log_mask(LOG_GUEST_ERROR, "stm32_read: Bad offset 0x%x\n", (int)offset); break;
    }
    return val;
}

static void stm32_timer_write( void * opaque, hwaddr offset, uint64_t value, unsigned size )
{
    Stm32Timer *s = (Stm32Timer*)opaque;

    switch( offset )
    {
    case TIMER_CR1_OFFSET:  stm32_timer_write_CR1( s, value );   break;
    case TIMER_CR2_OFFSET:  s->cr2  = value & 0xF8;              qemu_log_mask(LOG_GUEST_ERROR, "stm32_timer: CR2 not supported"); break;
    case TIMER_SMCR_OFFSET: s->smcr = value;                     qemu_log_mask(LOG_GUEST_ERROR, "stm32_timer: SMCR not supported"); break;
    case TIMER_DIER_OFFSET: s->dier = value & 0x5F5F;            DPRINTF("%s dier = %x\n", stm32_periph_name(s->periph), s->dier); break;
    case TIMER_SR_OFFSET:{
            s->sr ^= (value ^ 0xFFFF);
            s->sr &= 0x1eFF;                                     DPRINTF("%s sr = %x\n", stm32_periph_name(s->periph), s->sr);
            stm32_timer_update_UIF(s, s->sr & 0x1);
        }break;
    case TIMER_EGR_OFFSET:{
            uint16_t egr = value & 0x1E;
            if( s->egr == egr ) return;
            s->egr = egr;                                        DPRINTF("%s egr = %x\n", stm32_periph_name(s->periph), s->egr);
            if (value & 0x40) s->sr |= 0x40;                     // TG bit
            if (value & 0x1)  stm32_timer_updt_period( s );      // UG bit - reload count
        } break;
    case TIMER_CCMR1_OFFSET:{
            uint16_t ccmr1 = value;
            if( s->ccmr1 == ccmr1 ) return;
            s->ccmr1 = ccmr1;                                    DPRINTF("%s ccmr1 = %x\n", stm32_periph_name(s->periph), s->ccmr1);
            Stm32_channel_write_CCMR( &s->channels[0], ccmr1    );
            Stm32_channel_write_CCMR( &s->channels[1], ccmr1>>8 );
        }break;
    case TIMER_CCMR2_OFFSET:{
            uint16_t ccmr2 = value;
            if( s->ccmr2 == ccmr2 ) return;
            s->ccmr2 = ccmr2;                                    DPRINTF("%s ccmr2 = %x\n", stm32_periph_name(s->periph), s->ccmr2);
            Stm32_channel_write_CCMR( &s->channels[2], ccmr2    );
            Stm32_channel_write_CCMR( &s->channels[3], ccmr2>>8 );
        }break;
    case TIMER_CCER_OFFSET: stm32_timer_write_CCER( s, value );   break;
    case TIMER_CNT_OFFSET:  stm32_timer_set_count( s, value );   break;
    case TIMER_PSC_OFFSET:{
            s->psc = value;                                      DPRINTF("%s psc = %x\n", stm32_periph_name(s->periph), s->psc);
            ptimer_transaction_begin(s->timer);
            stm32_timer_updt_freq(s);
            ptimer_transaction_commit(s->timer);
        }break;
    case TIMER_ARR_OFFSET:{
            s->arr = value;                                      DPRINTF("%s arr = %x\n", stm32_periph_name(s->periph), s->arr);
            stm32_timer_updt_period( s );
        }break;
    case TIMER_RCR_OFFSET:  s->rcr = value;                      qemu_log_mask(LOG_GUEST_ERROR, "stm32_timer: RCR not supported"); break;
    case TIMER_CCR1_OFFSET: s->channels[0].CCR = value;          DPRINTF("%s ccr1 = %x\n", stm32_periph_name(s->periph), s->channels[0].CCR); break;
    case TIMER_CCR2_OFFSET: s->channels[1].CCR = value;          DPRINTF("%s ccr2 = %x\n", stm32_periph_name(s->periph), s->channels[1].CCR); break;
    case TIMER_CCR3_OFFSET: s->channels[2].CCR = value;          DPRINTF("%s ccr3 = %x\n", stm32_periph_name(s->periph), s->channels[2].CCR); break;
    case TIMER_CCR4_OFFSET: s->channels[3].CCR = value;          DPRINTF("%s ccr4 = %x\n", stm32_periph_name(s->periph), s->channels[3].CCR); break;
    case TIMER_BDTR_OFFSET: s->bdtr = value;                     qemu_log_mask(LOG_GUEST_ERROR, "stm32_timer: BDTR not supported"); break;
    case TIMER_DCR_OFFSET:  s->dcr  = value;                     qemu_log_mask(LOG_GUEST_ERROR, "stm32_timer: DCR not supported"); break;
    case TIMER_DMAR_OFFSET: s->dmar = value;                     qemu_log_mask(LOG_GUEST_ERROR, "stm32_timer: DMAR not supported"); break;
    default:                                                     qemu_log_mask(LOG_GUEST_ERROR, "stm32_read: Bad offset 0x%x\n", (int)offset); break;
    }
}

static const MemoryRegionOps stm32_timer_ops = {
    .read = stm32_timer_read,
    .write = stm32_timer_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void stm32_timer_init(Object *obj)
{
    SysBusDevice *dev= SYS_BUS_DEVICE(obj);
    Stm32Timer    *s = STM32_TIMER(dev);

    memory_region_init_io( &s->iomem, OBJECT(s), &stm32_timer_ops, s, "stm32-timer", 0x3ff );
    sysbus_init_mmio( dev, &s->iomem );

    sysbus_init_irq( dev, &s->irq );

    return ;
}

static void stm32_timer_realize(DeviceState *dev, Error **errp)
{
    Stm32Timer *s = STM32_TIMER(dev);
    qemu_irq *clk_irq;
    
    clk_irq = qemu_allocate_irqs( stm32_timer_clk_irq_handler, (void*)s, 1 );
    stm32_rcc_set_periph_clk_irq( s->stm32_rcc, s->periph, clk_irq[0] );

    s->timer = ptimer_init(stm32_timer_ovf, s, PTIMER_POLICY_LEGACY);
                           //PTIMER_POLICY_WRAP_AFTER_ONE_PERIOD |
                           //PTIMER_POLICY_TRIGGER_ONLY_ON_DECREMENT |
                           //PTIMER_POLICY_NO_IMMEDIATE_RELOAD |
                           //PTIMER_POLICY_NO_COUNTER_ROUND_DOWN);

    s->cr1   = 0;
    s->dier  = 0;
    s->sr    = 0;
    s->egr   = 0;
    s->ccmr1 = 0;
    s->ccmr2 = 0;
    s->ccer  = 0;
    s->psc   = 0;
    s->arr   = 0;

    s->enabled = 0;
    s->oneShot = 0;

    s->frequency = 0;

    s->direction = TIMER_UP_COUNT;

    for( int i=0; i<4; ++i ) Stm32_channel_init( &s->channels[i], s, i );


}

static int stm32_timer_pre_save( void *opaque )
{
    //Stm32Timer *s = opaque;

    /* tick_offset is base_time - rtc_clock base time.  Instead, we want to
     * store the base time relative to the vm_clock for backwards-compatibility.  */
    //int64_t delta = qemu_get_clock_ns(rtc_clock) - qemu_get_clock_ns(vm_clock);
    //s->tick_offset_vmstate = s->tick_offset + delta / get_ticks_per_sec();
    return 0;
}

static int stm32_timer_post_load( void *opaque, int version_id )
{
    //Stm32Timer *s = opaque;

    //int64_t delta = qemu_get_clock_ns(rtc_clock) - qemu_get_clock_ns(vm_clock);
    //s->tick_offset = s->tick_offset_vmstate - delta / get_ticks_per_sec();
    //stm32_timer_set_alarm(s);
    return 0;
}
//---------------------------------------------------------------------------

//void stm32_timer_remap( int number, uint8_t value )
//{
//    //printf("stm32_timer_remap %i %i\n", number, value ); fflush( stdout );
//    Stm32Timer* timer = stm32_get_timer( number );
//    if( !timer ) return;
//
//    switch( number )
//    {
//    case 1:{
//        switch ( value ) {
//            case 0:{       //00: No remap (ETR/PA12, CH1/PA8, CH2/PA9, CH3/PA10, CH4/PA11, BKIN/PB12, CH1N/PB13, CH2N/PB14, CH3N/PB15)
//                timer->channels[0].ioPort = 1;
//                timer->channels[0].ioPin  = 8;
//                timer->channels[1].ioPort = 1;
//                timer->channels[1].ioPin  = 9;
//                timer->channels[2].ioPort = 1;
//                timer->channels[2].ioPin  = 10;
//                timer->channels[3].ioPort = 1;
//                timer->channels[3].ioPin  = 11;
//            }break;
//            case 1:{       //01: Partial remap (ETR/PA12, CH1/PA8, CH2/PA9, CH3/PA10, CH4/PA11, BKIN/PA6, CH1N/PA7, CH2N/PB0, CH3N/PB1)
//                timer->channels[0].ioPort = 1;
//                timer->channels[0].ioPin  = 8;
//                timer->channels[1].ioPort = 1;
//                timer->channels[1].ioPin  = 9;
//                timer->channels[2].ioPort = 1;
//                timer->channels[2].ioPin  = 10;
//                timer->channels[3].ioPort = 1;
//                timer->channels[3].ioPin  = 11;
//            }break;
//            case 2: break; //10: not used
//            case 3:{       //11: Full remap (ETR/PE7, CH1/PE9, CH2/PE11, CH3/PE13, CH4/PE14, BKIN/PE15, CH1N/PE8, CH2N/PE10, CH3N/PE12)
//                timer->channels[0].ioPort = 5;
//                timer->channels[0].ioPin  = 9;
//                timer->channels[1].ioPort = 5;
//                timer->channels[1].ioPin  = 11;
//                timer->channels[2].ioPort = 5;
//                timer->channels[2].ioPin  = 13;
//                timer->channels[3].ioPort = 5;
//                timer->channels[3].ioPin  = 14;
//            }break;
//        }
//    }break;
//    case 2:{
//        switch ( value ) {
//            case 0:{       //00: No remap (CH1/ETR/PA0, CH2/PA1, CH3/PA2, CH4/PA3)
//                timer->channels[0].ioPort = 1;
//                timer->channels[0].ioPin  = 0;
//                timer->channels[1].ioPort = 1;
//                timer->channels[1].ioPin  = 1;
//                timer->channels[2].ioPort = 1;
//                timer->channels[2].ioPin  = 2;
//                timer->channels[3].ioPort = 1;
//                timer->channels[3].ioPin  = 3;
//            }break;
//            case 1:{       //01: Partial remap (CH1/ETR/PA15, CH2/PB3, CH3/PA2, CH4/PA3)
//                timer->channels[0].ioPort = 1;
//                timer->channels[0].ioPin  = 15;
//                timer->channels[1].ioPort = 2;
//                timer->channels[1].ioPin  = 3;
//                timer->channels[2].ioPort = 1;
//                timer->channels[2].ioPin  = 2;
//                timer->channels[3].ioPort = 1;
//                timer->channels[3].ioPin  = 3;
//            }break;
//            case 2:{       //10: Partial remap (CH1/ETR/PA0, CH2/PA1, CH3/PB10, CH4/PB11)
//                timer->channels[0].ioPort = 1;
//                timer->channels[0].ioPin  = 0;
//                timer->channels[1].ioPort = 1;
//                timer->channels[1].ioPin  = 1;
//                timer->channels[2].ioPort = 2;
//                timer->channels[2].ioPin  = 10;
//                timer->channels[3].ioPort = 2;
//                timer->channels[3].ioPin  = 11;
//            }break;
//            case 3:{       //11: Full remap (CH1/ETR/PA15, CH2/PB3, CH3/PB10, CH4/PB11)
//            }break;
//        }
//    }break;
//    case 3:{
//        switch ( value ) {
//            case 0:{       //00: No remap (CH1/PA6, CH2/PA7, CH3/PB0, CH4/PB1)
//                timer->channels[0].ioPort = 1;
//                timer->channels[0].ioPin  = 6;
//                timer->channels[1].ioPort = 1;
//                timer->channels[1].ioPin  = 7;
//                timer->channels[2].ioPort = 2;
//                timer->channels[2].ioPin  = 0;
//                timer->channels[3].ioPort = 2;
//                timer->channels[3].ioPin  = 1;
//            }break;
//            case 1: break; //01: Not used
//            case 2:{       //10: Partial remap (CH1/PB4, CH2/PB5, CH3/PB0, CH4/PB1)
//                timer->channels[0].ioPort = 2;
//                timer->channels[0].ioPin  = 4;
//                timer->channels[1].ioPort = 2;
//                timer->channels[1].ioPin  = 5;
//                timer->channels[2].ioPort = 2;
//                timer->channels[2].ioPin  = 0;
//                timer->channels[3].ioPort = 2;
//                timer->channels[3].ioPin  = 1;
//            }break;
//            case 3:{       //11: Full remap (CH1/PC6, CH2/PC7, CH3/PC8, CH4/PC9)
//                timer->channels[0].ioPort = 3;
//                timer->channels[0].ioPin  = 6;
//                timer->channels[1].ioPort = 3;
//                timer->channels[1].ioPin  = 7;
//                timer->channels[2].ioPort = 3;
//                timer->channels[2].ioPin  = 8;
//                timer->channels[3].ioPort = 3;
//                timer->channels[3].ioPin  = 9;
//            }break;
//        }
//    }break;
//    case 4:{
//        switch ( value ) {
//            case 0:{       // No remap (CH1/PB6, CH2/PB7, CH3/PB8, CH4/PB9)
//                timer->channels[0].ioPort = 2;
//                timer->channels[0].ioPin  = 6;
//                timer->channels[1].ioPort = 2;
//                timer->channels[1].ioPin  = 7;
//                timer->channels[2].ioPort = 2;
//                timer->channels[2].ioPin  = 8;
//                timer->channels[3].ioPort = 2;
//                timer->channels[3].ioPin  = 9;
//            }break;
//            case 1:{       // Full remap (CH1/PD12, CH2/PD13, CH3/PD14, CH4/PD15)
//                timer->channels[0].ioPort = 4;
//                timer->channels[0].ioPin  = 12;
//                timer->channels[1].ioPort = 4;
//                timer->channels[1].ioPin  = 13;
//                timer->channels[2].ioPort = 4;
//                timer->channels[2].ioPin  = 14;
//                timer->channels[3].ioPort = 4;
//                timer->channels[3].ioPin  = 15;
//            }break;
//        }
//    }break;
//    case 5:{
//
//    }break;
//    }
//}

//void stm32_timer_set_gpio(Stm32Timer *tim, Stm32Gpio** gpio){
//    tim->stm32_gpio = gpio;
//}
//void stm32_timer_set_afio(Stm32Timer *tim, Stm32Afio* afio){
//    tim->stm32_afio = afio;
//}
void stm32_timer_set_rcc(Stm32Timer *tim, Stm32Rcc* rcc){
    tim->stm32_rcc = rcc;
}

void stm32_timer_set_number( Stm32Timer *tim, int tim_num ) {
    tim->number= tim_num;
}

static Property stm32_timer_properties[] = {
    DEFINE_PROP_PERIPH_T("periph", Stm32Timer, periph, STM32_PERIPH_UNDEFINED),
    DEFINE_PROP_END_OF_LIST()
};

static const VMStateDescription vmstate_stm32 = {
    .name = "stm32-timer",
    .version_id = 1,
    .minimum_version_id = 1,
    .pre_save = stm32_timer_pre_save,
    .post_load = stm32_timer_post_load,
    .fields = (VMStateField[]) {
        //VMSTATE_UINT32(tick_offset_vmstate, stm32_timer_tm_state),
        //VMSTATE_UINT32(mr, Stm32Timer),
        //VMSTATE_UINT32(lr, Stm32Timer),
        //VMSTATE_UINT32(cr, Stm32Timer),
        //VMSTATE_UINT32(im, Stm32Timer),
        //VMSTATE_UINT32(is, Stm32Timer),
        VMSTATE_END_OF_LIST()
    }
};

static void stm32_timer_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    
    //dc->no_user = 1;
    dc->vmsd = &vmstate_stm32;
    dc->realize = stm32_timer_realize;
    device_class_set_props(dc, stm32_timer_properties);
    
}

static const TypeInfo stm32_timer_info = {
    .name          = "stm32-timer",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Stm32Timer),
    .instance_init = stm32_timer_init,
    .class_init    = stm32_timer_class_init,
};

static void stm32_timer_register_types(void)
{
    type_register_static(&stm32_timer_info);
}

type_init(stm32_timer_register_types)
