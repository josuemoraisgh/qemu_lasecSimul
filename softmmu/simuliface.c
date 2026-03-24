/***************************************************************************
 *   Copyright (C) 2025 by Santiago González                               *
 *                                                                         *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include <sys/stat.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>

#ifdef __linux__
#include <sys/mman.h>
#include <sys/shm.h>
//#elif defined(_WIN32)
//#include <windows.h>
#endif

#include "simuliface.h"

#include "qemu/osdep.h"
#include "qemu-main.h"
#include "qemu/timer.h"
#include "sysemu/runstate.h"
#include "sysemu/sysemu.h"
#include "sysemu/cpu-timers.h"
#include "hw/irq.h"

// ------------------------------------------------
// -------- ARENA ---------------------------------

volatile qemuArena_t* m_arena = NULL;

// ------------------------------------------------

uint64_t m_timeout;
uint64_t m_lastQemuTime;

uint64_t period_ns;
uint64_t period_ps;

QEMUTimer* qtimer;

uint64_t readReg( uint64_t addr )
{
    waitForSynch();
    m_arena->regAddr    = addr;
    //m_arena->regData    = 0;
    m_arena->qemuAction = 0;
    m_arena->simuAction = SIM_READ;
    m_arena->simuTime = icount_get_ns()*1000;

    uint64_t timeout = 0;
    while( !m_arena->qemuAction )  // Wait for SimulIDE to execute Read
    {
        if( timeout++ > 5e9 ) // Terminate process if timed out
        {
            printf("Qemu: readReg TIMEOUT %lu\n", addr); fflush( stdout );
            return 0;
        }
    }
    if( m_arena->qemuAction != SIM_READ )
    {
        printf("Qemu: readReg m_arena->qemuAction != SIM_READ\n"); fflush( stdout );
        return 0;
    }

    if( m_arena->irqNumber ) setInterrupt();

    return m_arena->regData;
}

void writeReg( uint64_t addr, uint64_t value )
{
    //printf("Qemu: esp32_gpio_write\n"); fflush( stdout );
    waitForSynch();
    m_arena->regAddr    = addr;
    m_arena->regData    = value;
    m_arena->simuAction = SIM_WRITE;
    m_arena->simuTime = getQemu_ps();
}

void updtCpuFreqHz( uint32_t clock_Hz )
{
    uint64_t now = icount_get_ns();

    double clock_MHz = (double)clock_Hz/1000000;
    double ps_instr = 1000000/clock_MHz;
    if( m_arena->ps_per_inst == ps_instr ) return;
    m_arena->ps_per_inst = ps_instr;

    period_ns = 50000; //ps_instr;
    period_ps = 1000*period_ns;
    m_arena->loop_timeout_ns = ps_instr/10;

    if( now == 0 ){
        printf("Qemu: Timer period: %lu ns loop period: %li ns\n", period_ns, m_arena->loop_timeout_ns );
        printf("Qemu: CPU freq: %f MHz at %lu\n", clock_MHz, now );
    }
}

uint64_t getQemu_ps(void)
{
    uint64_t qemuTime = icount_get_ns()*1000; //qemu_clock_get_ns( QEMU_CLOCK_VIRTUAL ); // ns //icount_get_ps; //
    //qemuTime *= 1000;
    return qemuTime;
}

uint64_t getQemu_ns(void)
{
    return icount_get_ns();
}

void waitForSynch(void)
{
    //printf("Qemu: wait for Action at time %lu\n",  m_lastQemuTime ); fflush( stdout );

    uint64_t now = icount_get_ns();
    m_lastQemuTime = now;
    if( now == 0 ) return;

    m_timeout = 0;
    while( m_arena->simuTime )  // Wait for SimulIDE to execute previous action
    {
        if( m_timeout++ > 2e9 ) // Terminate process if timed out
        {
            printf("Qemu: waitForSynch TIMEOUT at time %lu\n", now*1000 ); fflush( stdout );
            return;
        }
    }

    if( m_arena->irqNumber ) setInterrupt();
}

static void simu_event( void* opaque )
{
    if( !m_arena->running ) return;

    uint64_t now_ns = icount_get_ns();

    //printf("Qemu: simu_event at %lu\n", now_ns ); fflush( stdout );

    if( now_ns > m_lastQemuTime )
    {
        waitForSynch();
        m_arena->simuAction = SIM_EVENT;
        m_arena->simuTime = now_ns*1000;

        m_lastQemuTime = now_ns;
    }
    else if( m_arena->irqNumber ) setInterrupt();

    //printf("Qemu: simu_event next %lu\n", now_ns+period_ns ); fflush( stdout );

    timer_reload_ns( qtimer, now_ns+period_ns );
    //timer_mod_ns( qtimer, now_ns+period_ns );
}

int simuMain( int argc, char** argv )
{
    const int   shMemSize = sizeof( qemuArena_t );
    const char* shMemKey;

    if( argc > 2 ) // Check if there are any arguments
    {
        shMemKey = argv[1];
        argv = &argv[2];
        argc -= 2;
    } else {
        printf("Qemu Error: No arguments provided.\n");
        return 1;
    }

    void* arena = NULL;

#ifdef __linux__
    int shMemId = shm_open( shMemKey, O_RDWR, 0666 ); // Open the shared memory object
    if( shMemId == -1 )
    {
        printf("Qemu: Error opening arena: %s\n", shMemKey );
        return 1;
    }
    else printf("Qemu: arena ok: %s\n", shMemKey );
    arena = mmap( 0, shMemSize, PROT_READ | PROT_WRITE, MAP_SHARED, shMemId, 0);
#elif defined(_WIN32)
    HANDLE hMapFile = OpenFileMapping( FILE_MAP_ALL_ACCESS,  FALSE, shMemKey );

    if( hMapFile == NULL ) {
        //std::cerr << "Could not create file mapping object: " << GetLastError() << std::endl;
        return 1;
    }
    arena = MapViewOfFile( hMapFile, FILE_MAP_ALL_ACCESS, 0, 0, shMemSize );
#endif

    if( !arena )
    {
        printf("Qemu: Error mapping arena\n"); fflush( stdout );
        return 1;
    }
    else printf("Qemu: arena mapped %i bytes\n", shMemSize );

    //------------------------------------------------------------------

    m_arena = (qemuArena_t*)arena;

    //------------------------------------------------------------------

    printf("-----------------------------------\n");
    for( int i=0; i<argc; i++)
    {
        printf( "%s",argv[i] );
        if( !(i&1) ) printf("\n");
        else         printf(" ");
    }
    printf("-----------------------------------\n");
    fflush( stdout );

    m_lastQemuTime = 0; //m_resetEvent;
    period_ns = 50000;
    period_ps = 1000*period_ns;
    m_arena->loop_timeout_ns = 1000;
    m_arena->ps_per_inst = 25000;

    //updtCpuFreqHz( 240000000 );

    qemu_init( argc, argv );
    printf("Qemu: initialized\n" );fflush( stdout );

    qtimer = (QEMUTimer*)malloc( sizeof(QEMUTimer) );
    timer_init_full( qtimer, NULL, QEMU_CLOCK_VIRTUAL, 1, 0, simu_event, NULL );
    timer_mod_ns( qtimer, period_ns );

    m_arena->running = 1;

    printf("Qemu: starting main loop\n");fflush( stdout );
    int status = qemu_main_loop();

#ifdef __linux__
    munmap( arena, shMemSize ); // Un-map shared memory
#elif defined(_WIN32)
    UnmapViewOfFile( arena );
    CloseHandle( hMapFile );
#endif

    printf("Qemu: process finished %i\n", status );fflush( stdout );

    return 0;
}
