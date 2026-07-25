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
#include "qemu/main-loop.h"
#include "sysemu/runstate.h"
#include "sysemu/sysemu.h"
#include "sysemu/cpu-timers.h"
#include "hw/irq.h"
#include "hw/core/cpu.h"
#include "qemu/seqlock.h"
#include "qemu/atomic.h"
#include "timers-state.h"

// ------------------------------------------------
// -------- ARENA ---------------------------------

volatile qemuArena_t* m_arena = NULL;

// ------------------------------------------------

uint64_t m_timeout;
uint64_t m_lastQemuTime;

uint64_t period_ns;
uint64_t period_ps;

QEMUTimer* qtimer;
static void pushQueueEntry( uint64_t addr, uint64_t data, uint64_t action, uint64_t simuTimePs );

static void publishQueueEntry( uint64_t addr, uint64_t data, uint64_t action, uint64_t simuTimePs )
{
    waitForSynch();
    pushQueueEntry(addr, data, action, simuTimePs);
}

/* v3 (LasecSimul PERF-13): publica uma entrada na fila circular de escritas/heartbeat -- só
 * escreve os campos da entrada e incrementa queueWriteIndex por ÚLTIMO (é isso que torna a
 * entrada visível pro Core, mesmo princípio de "confirma por último" que simuTime != 0 usava
 * em v2 pro slot único). Chamador já garantiu que há espaço (ver waitForSynch()).
 *
 * Achado 2026-07-22 (usuário reporta simulação travando por completo depois de rodar por um
 * tempo, indicador de velocidade continua marcando 100%): `queueWriteIndex` é lido pelo CORE, um
 * PROCESSO SEPARADO -- sem uma store de liberação aqui (pareada com o load de aquisição em
 * QemuArenaBridge::poll(), lado C++), nada no padrão de memória do C impede o compilador ou o
 * hardware de tornar o incremento de `queueWriteIndex` visível pro Core ANTES dos campos da
 * entrada (`regAddr`/`regData`/`simuAction`/`simuTime`) acima -- o Core então leria uma entrada
 * "publicada" com campos ainda não escritos/parcialmente visíveis, potencialmente disparando o
 * branch de overflow de `qemuEventTimeNs()` (Core, McuComponent.cpp) e travando aquela entrada --
 * e tudo atrás dela na fila de 32 -- pra sempre. `qatomic_store_release` garante a ordem: todo
 * store ACIMA desta linha fica visível para qualquer leitor que faça um load de aquisição do
 * mesmo `queueWriteIndex` ANTES de ver este incremento. */
static void pushQueueEntry( uint64_t addr, uint64_t data, uint64_t action, uint64_t simuTimePs )
{
    const uint64_t writeIndex = m_arena->queueWriteIndex;
    const uint64_t slot = writeIndex % QEMU_ARENA_QUEUE_DEPTH;
    m_arena->queue[slot].regAddr    = addr;
    m_arena->queue[slot].regData    = data;
    m_arena->queue[slot].simuAction = action;
    m_arena->queue[slot].simuTime   = simuTimePs;
    qatomic_store_release(&m_arena->queueWriteIndex, writeIndex + 1);

    const char *trace = getenv("LASECSIMUL_TRACE_GPIO");
    if (trace && trace[0] && strcmp(trace, "0") != 0 &&
        (addr == 0x3ff44004 || addr == 0x3ff44020 || addr == 0x3ff49038)) {
        printf("[LasecSimul][QUEUE_PUSH] index=%llu slot=%llu cpu=%d bql=%s "
               "addr=0x%08llx data=0x%08llx event_ps=%llu\n",
               (unsigned long long)writeIndex, (unsigned long long)slot,
               current_cpu ? current_cpu->cpu_index : -1,
               qemu_mutex_iothread_locked() ? "yes" : "no",
               (unsigned long long)addr, (unsigned long long)data,
               (unsigned long long)simuTimePs);
        fflush(stdout);
    }
}

/* v3: espera a fila de escritas/heartbeat esvaziar de vez (queueReadIndex alcança
 * queueWriteIndex) -- só readReg() chama isto, pra preservar a ordem leitura-depois-de-escrita
 * sem colocar a própria leitura na fila (ela precisa de um valor de volta, a fila não).
 *
 * `queueReadIndex` é escrito pelo CORE (outro processo, ver QemuArenaBridge::acknowledgeWrite())
 * -- load de aquisição aqui pareia com a store de liberação de lá, garantindo que este processo
 * enxergue o avanço assim que ele acontece, em vez de potencialmente reutilizar um valor
 * cacheado/reordenado (mesmo raciocínio de pushQueueEntry() acima, sentido inverso). */
static void waitForQueueDrain( void )
{
    if( qatomic_load_acquire(&m_arena->queueReadIndex) == m_arena->queueWriteIndex ) return;

    uint64_t timeout = 0;
    const bool hadIothreadLock = qemu_mutex_iothread_locked();
    if( hadIothreadLock ) qemu_mutex_unlock_iothread();

    while( qatomic_load_acquire(&m_arena->queueReadIndex) != m_arena->queueWriteIndex )
    {
        if( timeout++ > 5e9 ) // Terminate process if timed out
        {
            printf("Qemu: waitForQueueDrain TIMEOUT\n"); fflush( stdout );
            if( hadIothreadLock ) qemu_mutex_lock_iothread();
            return;
        }
    }

    if( hadIothreadLock ) qemu_mutex_lock_iothread();
}

uint64_t readReg( uint64_t addr )
{
    /* Standard QEMU mode has no SimulIDE shared-memory arena.  MMIO hooks must
     * remain harmless there instead of dereferencing NULL (0xC0000005 on
     * Windows as soon as Arduino touches GPIO). */
    if (!m_arena) {
        return 0;
    }

    /* v3: leitura sempre retorna só depois de o Core confirmar (nunca deixa m_arena->simuTime
     * pendente de uma chamada pra outra -- diferente de escritas, que são "dispara e esquece"),
     * então não precisa esperar uma leitura ANTERIOR terminar -- só precisa esperar toda
     * escrita/heartbeat pendente NA FILA já ter sido processada (mesmo papel que waitForSynch()
     * tinha em v2 pro slot único, agora dividido porque escritas não usam mais esse slot). */
    uint64_t now = icount_get_ns();
    m_lastQemuTime = now;
    if( now != 0 )
    {
        waitForQueueDrain();
        if( m_arena->irqNumber ) setInterrupt();
    }

    m_arena->regAddr    = addr;
    //m_arena->regData    = 0;
    m_arena->qemuAction = 0;
    m_arena->simuAction = SIM_READ;
    m_arena->simuTime = icount_get_ns()*1000;

    uint64_t timeout = 0;
    if( !m_arena->qemuAction )  // Wait for SimulIDE to execute Read
    {
        /* This spins on a reply from Core, a separate OS process -- not on any QEMU-internal
         * state. readReg() only ever runs as an MMIO device callback, invoked from cputlb.c under
         * the BQL (qemu/main-loop.h: "should be unlocked as soon as possible... because it
         * prevents the main loop from processing callbacks"). Release it for the wait so QEMU's
         * own iothread isn't blocked out for however long Core takes to reply, same
         * unlock/block/relock pattern QEMU's own blk_pread()/aio_poll() already use elsewhere in
         * this same MMIO dispatch path (see esp32_cache_data_sync). */
        const bool hadIothreadLock = qemu_mutex_iothread_locked();
        if( hadIothreadLock ) qemu_mutex_unlock_iothread();

        while( !m_arena->qemuAction )
        {
            if( timeout++ > 5e9 ) // Terminate process if timed out
            {
                printf("Qemu: readReg TIMEOUT %llu\n", (unsigned long long)addr); fflush( stdout );
                if( hadIothreadLock ) qemu_mutex_lock_iothread();
                return 0;
            }
        }

        if( hadIothreadLock ) qemu_mutex_lock_iothread();
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
    if (!m_arena) {
        return;
    }
    //printf("Qemu: esp32_gpio_write\n"); fflush( stdout );
    publishQueueEntry( addr, value, SIM_WRITE, getQemu_ps() );
}

void writeSimEvent( uint64_t addr, uint64_t value, uint64_t action )
{
    if (!m_arena) {
        return;
    }
    publishQueueEntry(addr, value, action, getQemu_ps());
}

void updtCpuFreqHz( uint32_t clock_Hz )
{
    uint64_t now = icount_get_ns();

    double clock_MHz = (double)clock_Hz/1000000;
    double ps_instr = 1000000/clock_MHz;
    if (!m_arena) {
        period_ns = 50000;
        period_ps = 1000 * period_ns;
        return;
    }
    if( m_arena->ps_per_inst == ps_instr ) return;
    m_arena->ps_per_inst = ps_instr;

    period_ns = 50000; //ps_instr;
    period_ps = 1000*period_ns;
    m_arena->loop_timeout_ns = ps_instr/10;

    if( now == 0 ){
        printf("Qemu: Timer period: %llu ns loop period: %lld ns\n", (unsigned long long)period_ns,
               (long long)m_arena->loop_timeout_ns );
        printf("Qemu: CPU freq: %f MHz at %llu\n", clock_MHz, (unsigned long long)now );
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
    if (!m_arena) {
        return;
    }
    //printf("Qemu: wait for Action at time %lu\n",  m_lastQemuTime ); fflush( stdout );

    uint64_t now = icount_get_ns();
    m_lastQemuTime = now;
    if( now == 0 ) return;

    m_timeout = 0;
    /* v3: só espera quando a fila de escritas/heartbeat está CHEIA -- backpressure explícito, não
     * mais um ping-pong completo a cada chamada (protocolo v2 tinha 1 slot só, sempre esperava o
     * anterior confirmar antes de publicar o próximo).
     *
     * `queueReadIndex` é escrito pelo Core (outro processo) -- load de aquisição, mesmo raciocínio
     * de waitForQueueDrain() acima. `queueWriteIndex` só é escrito por ESTE processo (QEMU), então
     * uma leitura simples basta pro lado de cá. */
    if( m_arena->queueWriteIndex - qatomic_load_acquire(&m_arena->queueReadIndex) >= QEMU_ARENA_QUEUE_DEPTH )
    {
        /* Same reasoning as the wait in readReg() above: this spins on Core (a separate process),
         * not on QEMU state, and runs either as an MMIO callback or inline from simu_event()'s
         * timer callback -- both BQL-held contexts. Release it for the spin. */
        const bool hadIothreadLock = qemu_mutex_iothread_locked();
        if( hadIothreadLock ) qemu_mutex_unlock_iothread();

        while( m_arena->queueWriteIndex - qatomic_load_acquire(&m_arena->queueReadIndex) >= QEMU_ARENA_QUEUE_DEPTH )
        {
            if( m_timeout++ > 2e9 ) // Terminate process if timed out
            {
                printf("Qemu: waitForSynch TIMEOUT (fila cheia) at time %llu\n", (unsigned long long)(now*1000) ); fflush( stdout );
                if( hadIothreadLock ) qemu_mutex_lock_iothread();
                return;
            }
        }

        if( hadIothreadLock ) qemu_mutex_lock_iothread();
    }

    if( m_arena->irqNumber ) setInterrupt();
}

static void simu_event( void* opaque )
{
    if (!m_arena) return;
    if( !m_arena->running ) return;

    uint64_t now_ns = icount_get_ns();

    //printf("Qemu: simu_event at %lu\n", now_ns ); fflush( stdout );

    if( now_ns > m_lastQemuTime )
    {
        publishQueueEntry( 0, 0, SIM_EVENT, now_ns*1000 ); // heartbeat nao usa regAddr/regData

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
    period_ns = 50000; // placeholder ate qemu_init() analisar "-icount shift=N" da linha de comando
    period_ps = 1000*period_ns;
    m_arena->loop_timeout_ns = 1000;
    m_arena->ps_per_inst = 25000;

    //updtCpuFreqHz( 240000000 );

    qemu_init( argc, argv );
    printf("Qemu: initialized\n" );fflush( stdout );

    /* period_ns precisa escalar com 2^shift, senao o heartbeat periodico (simu_event(), que faz
     * um round-trip completo com o host via waitForSynch()) passa a disparar mais vezes por
     * instrucao real conforme "shift" aumenta -- cancelando o ganho de velocidade que um shift
     * maior deveria trazer (mais tempo virtual por instrucao real executada). O valor original
     * (50000 ns pra shift=4) equivale a ~3125 instrucoes reais entre round-trips; mantemos essa
     * mesma cadencia pra qualquer shift configurado, em vez de um segundo numero fixo dessincronizado
     * do primeiro. Lido de timers_state.icount_time_shift (so valido DEPOIS de qemu_init() acima,
     * que e quem processa "-icount shift=N"). Investigado e documentado em LasecSimul 2026-07-19.
     */
    {
        const int shift = timers_state.icount_time_shift;
        if( shift > 0 && shift < 32 )
        {
            const uint64_t instructionsPerHeartbeat = 3125; // calibrado pro shift=4 original (50000/16)
            period_ns = instructionsPerHeartbeat << shift;
            period_ps = 1000*period_ns;
            printf("Qemu: icount shift=%d -> period_ns=%llu (heartbeat a cada ~%llu instrucoes)\n",
                   shift, (unsigned long long)period_ns, (unsigned long long)instructionsPerHeartbeat ); fflush( stdout );
        }
    }

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
