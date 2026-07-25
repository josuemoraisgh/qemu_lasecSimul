#pragma once

#include "qemu/osdep.h"

/*
 * Minimal clock interface for ESP32 peripherals compiled as common code.
 * Keep this header independent from target/xtensa/cpu.h.
 */
uint32_t esp32_soc_get_apb_freq(void);
uint32_t esp32_soc_get_cpu_freq(void);
