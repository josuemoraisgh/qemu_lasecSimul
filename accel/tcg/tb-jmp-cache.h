/*
 * The per-CPU TranslationBlock jump cache.
 *
 *  Copyright (c) 2003 Fabrice Bellard
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#ifndef ACCEL_TCG_TB_JMP_CACHE_H
#define ACCEL_TCG_TB_JMP_CACHE_H

/* Experimento 2026-07-19: live profiling (GDB, thread-sampling contra um processo QEMU real
 * rodando firmware ESP32) mostrou tb_lookup_cmp/tb_htable_lookup (o caminho LENTO de fallback via
 * hash table -- ver cpu-exec.c) genuinely quente no thread de execucao, nao dominado por espera de
 * lock (isso ja foi corrigido separadamente, ver simuliface.c). O cache rapido abaixo
 * (CPUJumpCache::array) e' mapeado diretamente (1 slot por hash, sem associatividade) -- dois PCs
 * quentes que colidem no mesmo slot ficam se despejando um ao outro a cada lookup, forcando o
 * fallback caro toda vez. Testando se aumentar o cache (12->14 bits, 4096->16384 entradas, 64KB->
 * 256KB por vCPU, negligivel em hardware moderno) reduz essa taxa de colisao pra este workload
 * (ESP32 real, código espalhado entre boot ROM/IRAM/DRAM/cache de flash). Medir antes de manter.
 */
#define TB_JMP_CACHE_BITS 14
#define TB_JMP_CACHE_SIZE (1 << TB_JMP_CACHE_BITS)

/*
 * Accessed in parallel; all accesses to 'tb' must be atomic.
 * For CF_PCREL, accesses to 'pc' must be protected by a
 * load_acquire/store_release to 'tb'.
 */
struct CPUJumpCache {
    struct rcu_head rcu;
    struct {
        TranslationBlock *tb;
        vaddr pc;
    } array[TB_JMP_CACHE_SIZE];
};

#endif /* ACCEL_TCG_TB_JMP_CACHE_H */
