/*
 * ESP32 Interrupt Matrix
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
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/xtensa/esp32_intc.h"
#include "hw/misc/esp32_dport.h"

#define INTMATRIX_UNINT_VALUE   6

#define IRQ_MAP(cpu, input) s->irq_map[cpu][input]

/* [CACHE-TRACE] instrumentacao temporaria, ver .spec secao 32.5.12/32.5.13 -- escalada explicita do
 * usuario. Achado real (32.5.12, leitura direta de highint_hdl.S da v5.5.4 do ESP-IDF real +
 * confirmado por desmontagem do merger.elf): neste build (CONFIG_ESP_SYSTEM_CHECK_INT_LEVEL_5), o
 * watchdog TG1 (fonte 20, ETS_TG1_WDT_LEVEL_INTR_SOURCE) e a interrupcao de acesso ilegal ao cache
 * (fonte 68, ETS_CACHE_IA_INTR_SOURCE) sao deliberadamente roteados pela ESP-IDF real pra MESMA linha
 * de interrupcao de CPU (26, ETS_T1_WDT_CACHEERR_INUM) -- o handler de interrupcao (xt_highint5)
 * desambigua lendo DPORT_PRO/APP_INTR_STATUS_0_REG bit 20, que este fork NAO modela em lugar nenhum
 * (grep confirmado, sempre le zero). Registra aqui QUALQUER fonte que acabe mapeada pra linha de CPU
 * 26 em qualquer nucleo, pra determinar se quem de fato ativa a linha 26 nas reproducoes reais do
 * "Cache error" e o watchdog TG1 genuino (fonte 20), o cache-IA genuino (fonte 68, ja refutado
 * repetidas vezes desde 32.5.7 por outro caminho), ou uma fonte inesperada (bug de roteamento). Sera
 * revertido apos a causa raiz ser confirmada. */
#define ESP32_INTC_TRACE_LINE_TG1_WDT_CACHEERR 26

static void esp32_intmatrix_irq_handler(void *opaque, int n, int level)
{
    Esp32IntMatrixState *s = ESP32_INTMATRIX(opaque);

    s->irq_raw[n] = level;
    for (int i = 0; i < ESP32_CPU_COUNT; ++i) {
        if (s->outputs[i] == NULL) {
            continue;
        }
        int out_index = IRQ_MAP(i, n);
        if (out_index == ESP32_INTC_TRACE_LINE_TG1_WDT_CACHEERR) {
            esp32_cache_trace_generic_event("intmatrix_line26", i, (uint64_t)n, (uint32_t)level);
        }
        for (int int_index = 0; int_index < s->cpu[i]->env.config->nextint; ++int_index) {
            if (s->cpu[i]->env.config->extint[int_index] == out_index) {
                qemu_set_irq(s->outputs[i][int_index], level);
                break;
            }
        }
    }
}

static inline uint8_t* get_map_entry(Esp32IntMatrixState* s, hwaddr addr)
{
    int source_index = addr / sizeof(uint32_t);
    if (source_index > ESP32_INT_MATRIX_INPUTS * ESP32_CPU_COUNT) {
        error_report("%s: source_index %d out of range", __func__, source_index);
        return NULL;
    }
    int cpu_index = source_index / ESP32_INT_MATRIX_INPUTS;
    source_index = source_index % ESP32_INT_MATRIX_INPUTS;
    return &IRQ_MAP(cpu_index, source_index);
}

static uint64_t esp32_intmatrix_read(void* opaque, hwaddr addr, unsigned int size)
{
    Esp32IntMatrixState *s = ESP32_INTMATRIX(opaque);
    uint8_t* map_entry = get_map_entry(s, addr);
    return (map_entry != NULL) ? *map_entry : 0;
}

static void esp32_intmatrix_write(void* opaque, hwaddr addr, uint64_t value, unsigned int size)
{
    Esp32IntMatrixState *s = ESP32_INTMATRIX(opaque);
    int source_index = (addr / sizeof(uint32_t)) % ESP32_INT_MATRIX_INPUTS;
    int cpu_index = (addr / sizeof(uint32_t)) / ESP32_INT_MATRIX_INPUTS;
    uint8_t* map_entry = get_map_entry(s, addr);
    /* [CACHE-TRACE] ver .spec 32.5.12/32.5.13 -- registra TODO roteamento pra linha de CPU 26
     * (compartilhada entre TG1 WDT e cache-IA neste build), incluindo QUEM configurou (fonte, nucleo)
     * -- confirma se o guest realmente roteia so as duas fontes esperadas (20 e 68) pra essa linha,
     * ou se ha uma terceira fonte inesperada (indicativo de bug de roteamento neste fork). */
    if ((value & 0x1f) == ESP32_INTC_TRACE_LINE_TG1_WDT_CACHEERR) {
        esp32_cache_trace_generic_event("intmatrix_map_to_line26", cpu_index,
                                         (uint64_t)source_index, (uint32_t)value);
    }
    if (value == INTMATRIX_UNINT_VALUE) {
        int si = s->irq_raw[source_index];
        esp32_intmatrix_irq_handler(s, source_index, 0);
        s->irq_raw[source_index] = si;
    }
    if (map_entry != NULL) {
        *map_entry = value & 0x1f;
    }
    if (value != INTMATRIX_UNINT_VALUE && s->irq_raw[source_index]) {
        esp32_intmatrix_irq_handler(s, source_index, 1);
    }
}

/* Ver .spec 32.5.16 -- expoe o estado bruto ("raw", antes de roteamento) das fontes de interrupcao
 * pra hw/misc/esp32_dport.c (codigo "common", nao pode incluir este header por causa de
 * target/xtensa/cpu.h -- ver comentario em include/hw/misc/esp32_dport.h). `opaque` e o mesmo
 * ponteiro guardado em Esp32DportState::intmatrix_opaque, setado em esp32.c apos os dois
 * dispositivos-irmaos serem inicializados. */
uint32_t esp32_intmatrix_get_raw_status_bits(void *opaque, int start_bit, int count)
{
    Esp32IntMatrixState *s = ESP32_INTMATRIX(opaque);
    uint32_t result = 0;

    assert(count > 0 && count <= 32);
    for (int i = 0; i < count; ++i) {
        int source = start_bit + i;
        if (source >= 0 && source < ESP32_INT_MATRIX_INPUTS && s->irq_raw[source]) {
            result |= (1u << i);
        }
    }
    return result;
}

static const MemoryRegionOps esp_intmatrix_ops = {
    .read =  esp32_intmatrix_read,
    .write = esp32_intmatrix_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_intmatrix_reset(DeviceState *dev)
{
    Esp32IntMatrixState *s = ESP32_INTMATRIX(dev);
    memset(s->irq_raw, 0, sizeof(s->irq_raw));
    memset(s->irq_map, INTMATRIX_UNINT_VALUE, sizeof(s->irq_map));
    for (int i = 0; i < ESP32_CPU_COUNT; ++i) {
        if (s->outputs[i] == NULL) {
            continue;
        }
        for (int int_index = 0; int_index < s->cpu[i]->env.config->nextint; ++int_index) {
            qemu_irq_lower(s->outputs[i][int_index]);
        }
    }

}

static void esp32_intmatrix_realize(DeviceState *dev, Error **errp)
{
    Esp32IntMatrixState *s = ESP32_INTMATRIX(dev);

    for (int i = 0; i < ESP32_CPU_COUNT; ++i) {
        if (s->cpu[i]) {
            s->outputs[i] = xtensa_get_extints(&s->cpu[i]->env);
        }
    }
    esp32_intmatrix_reset(dev);
}

static void esp32_intmatrix_init(Object *obj)
{
    Esp32IntMatrixState *s = ESP32_INTMATRIX(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &esp_intmatrix_ops, s,
                          TYPE_ESP32_INTMATRIX, ESP32_INT_MATRIX_INPUTS * ESP32_CPU_COUNT * sizeof(uint32_t));
    sysbus_init_mmio(sbd, &s->iomem);

    qdev_init_gpio_in(DEVICE(s), esp32_intmatrix_irq_handler, ESP32_INT_MATRIX_INPUTS);
}

static Property esp32_intmatrix_properties[] = {
    DEFINE_PROP_LINK("cpu0", Esp32IntMatrixState, cpu[0], TYPE_XTENSA_CPU, XtensaCPU *),
    DEFINE_PROP_LINK("cpu1", Esp32IntMatrixState, cpu[1], TYPE_XTENSA_CPU, XtensaCPU *),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32_intmatrix_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = esp32_intmatrix_reset;
    dc->realize = esp32_intmatrix_realize;
    device_class_set_props(dc, esp32_intmatrix_properties);
}

static const TypeInfo esp32_intmatrix_info = {
    .name = TYPE_ESP32_INTMATRIX,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32IntMatrixState),
    .instance_init = esp32_intmatrix_init,
    .class_init = esp32_intmatrix_class_init
};

static void esp32_intmatrix_register_types(void)
{
    type_register_static(&esp32_intmatrix_info);
}

type_init(esp32_intmatrix_register_types)
