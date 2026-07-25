#pragma once

#include "hw/sysbus.h"
#include "hw/hw.h"
#include "hw/registerfields.h"

#define TYPE_ESP32_IOMUX "esp32.iomux"
#define ESP32_IOMUX(obj) OBJECT_CHECK(Esp32IomuxState, (obj), TYPE_ESP32_IOMUX)
#define ESP32_IOMUX_GET_CLASS(obj)   OBJECT_GET_CLASS(Esp32IomuxClass, obj, TYPE_ESP32_IOMUX)
#define ESP32_IOMUX_CLASS(klass)     OBJECT_CLASS_CHECK(Esp32IomuxClass, klass, TYPE_ESP32_IOMUX)

typedef struct Esp32IomuxState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint32_t muxgpios[40];
} Esp32IomuxState;

typedef struct Esp32IomuxClass {
    SysBusDeviceClass parent_class;
} Esp32IomuxClass;
