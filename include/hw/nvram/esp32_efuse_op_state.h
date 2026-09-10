#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct Esp32EfuseOperationState {
    bool operation_active;
    bool busy_observed;
    bool completion_due;
    uint64_t operation_generation;
    uint32_t operation_command;
    int64_t operation_deadline_ns;
} Esp32EfuseOperationState;

static inline void esp32_efuse_operation_reset(Esp32EfuseOperationState *op)
{
    ++op->operation_generation;
    op->operation_active = false;
    op->busy_observed = false;
    op->completion_due = false;
    op->operation_command = 0;
    op->operation_deadline_ns = -1;
}

static inline void esp32_efuse_operation_begin(Esp32EfuseOperationState *op,
                                               uint32_t command,
                                               int64_t deadline_ns)
{
    ++op->operation_generation;
    op->operation_active = true;
    op->busy_observed = false;
    op->completion_due = false;
    op->operation_command = command;
    op->operation_deadline_ns = deadline_ns;
}

static inline bool esp32_efuse_operation_complete(Esp32EfuseOperationState *op,
                                                  uint32_t *completed_command)
{
    if (!op->operation_active) {
        *completed_command = 0;
        return false;
    }

    *completed_command = op->operation_command;
    op->operation_active = false;
    op->busy_observed = false;
    op->completion_due = false;
    op->operation_command = 0;
    op->operation_deadline_ns = -1;
    return *completed_command != 0;
}

static inline bool esp32_efuse_operation_timer_expired(Esp32EfuseOperationState *op,
                                                       uint32_t *completed_command)
{
    *completed_command = 0;
    if (!op->operation_active) {
        return false;
    }

    if (!op->busy_observed) {
        op->completion_due = true;
        return false;
    }

    return esp32_efuse_operation_complete(op, completed_command);
}

static inline uint32_t esp32_efuse_operation_read_cmd(Esp32EfuseOperationState *op,
                                                      uint32_t live_cmd,
                                                      uint32_t *completed_command)
{
    uint32_t ret = live_cmd;

    *completed_command = 0;
    if (op->operation_active && !op->busy_observed) {
        ret = op->operation_command;
        op->busy_observed = true;
        if (op->completion_due) {
            esp32_efuse_operation_complete(op, completed_command);
        }
    }

    return ret;
}
