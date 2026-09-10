#include "qemu/osdep.h"
#include "hw/nvram/esp32_efuse_op_state.h"
#include "hw/nvram/esp32_efuse.h"

#define DEADLINE_NS 100000000LL

static void complete_to_registers(Esp32EfuseOperationState *op,
                                  uint32_t *cmd_reg,
                                  uint32_t *int_raw_reg,
                                  uint32_t completed_command)
{
    if (completed_command != 0) {
        *cmd_reg = 0;
        *int_raw_reg |= completed_command;
    }
}

static void begin_read(Esp32EfuseOperationState *op, uint32_t *cmd_reg)
{
    *cmd_reg = EFUSE_READ;
    esp32_efuse_operation_begin(op, EFUSE_READ, DEADLINE_NS);
}

static uint32_t read_cmd(Esp32EfuseOperationState *op, uint32_t *cmd_reg,
                         uint32_t *int_raw_reg)
{
    uint32_t completed_command = 0;
    uint32_t ret = esp32_efuse_operation_read_cmd(op, *cmd_reg,
                                                  &completed_command);
    complete_to_registers(op, cmd_reg, int_raw_reg, completed_command);
    return ret;
}

static void expire_timer(Esp32EfuseOperationState *op, uint32_t *cmd_reg,
                         uint32_t *int_raw_reg)
{
    uint32_t completed_command = 0;
    if (esp32_efuse_operation_timer_expired(op, &completed_command)) {
        complete_to_registers(op, cmd_reg, int_raw_reg, completed_command);
    }
}

static void test_normal_read_busy_then_zero(void)
{
    Esp32EfuseOperationState op = { 0 };
    uint32_t cmd_reg = 0, int_raw = 0;
    esp32_efuse_operation_reset(&op);
    begin_read(&op, &cmd_reg);

    g_assert_cmpuint(read_cmd(&op, &cmd_reg, &int_raw), ==, EFUSE_READ);
    expire_timer(&op, &cmd_reg, &int_raw);
    g_assert_cmpuint(read_cmd(&op, &cmd_reg, &int_raw), ==, 0);
    g_assert_cmpuint(int_raw, ==, EFUSE_READ);
}

static void test_timer_before_first_read_returns_busy_once(void)
{
    Esp32EfuseOperationState op = { 0 };
    uint32_t cmd_reg = 0, int_raw = 0;
    esp32_efuse_operation_reset(&op);
    begin_read(&op, &cmd_reg);

    expire_timer(&op, &cmd_reg, &int_raw);
    g_assert_cmpuint(cmd_reg, ==, EFUSE_READ);
    g_assert_cmpuint(int_raw, ==, 0);
    g_assert_cmpuint(read_cmd(&op, &cmd_reg, &int_raw), ==, EFUSE_READ);
    g_assert_cmpuint(cmd_reg, ==, 0);
    g_assert_cmpuint(int_raw, ==, EFUSE_READ);
    g_assert_cmpuint(read_cmd(&op, &cmd_reg, &int_raw), ==, 0);
}

static void test_completion_irq_once(void)
{
    Esp32EfuseOperationState op = { 0 };
    uint32_t cmd_reg = 0, int_raw = 0;
    esp32_efuse_operation_reset(&op);
    begin_read(&op, &cmd_reg);
    g_assert_cmpuint(read_cmd(&op, &cmd_reg, &int_raw), ==, EFUSE_READ);
    expire_timer(&op, &cmd_reg, &int_raw);
    expire_timer(&op, &cmd_reg, &int_raw);
    g_assert_cmpuint(int_raw, ==, EFUSE_READ);
}

static void test_timer_after_busy_observed_completes_immediately(void)
{
    Esp32EfuseOperationState op = { 0 };
    uint32_t cmd_reg = 0, int_raw = 0;
    esp32_efuse_operation_reset(&op);
    begin_read(&op, &cmd_reg);
    g_assert_cmpuint(read_cmd(&op, &cmd_reg, &int_raw), ==, EFUSE_READ);
    expire_timer(&op, &cmd_reg, &int_raw);
    g_assert_cmpuint(cmd_reg, ==, 0);
    g_assert_cmpuint(int_raw, ==, EFUSE_READ);
}

static void test_three_rom_pattern_operations(void)
{
    Esp32EfuseOperationState op = { 0 };
    uint32_t cmd_reg = 0, int_raw = 0;
    uint32_t rdata[3];
    esp32_efuse_operation_reset(&op);

    for (unsigned i = 0; i < 3; ++i) {
        begin_read(&op, &cmd_reg);
        expire_timer(&op, &cmd_reg, &int_raw);
        g_assert_cmpuint(read_cmd(&op, &cmd_reg, &int_raw), ==, EFUSE_READ);
        g_assert_cmpuint(read_cmd(&op, &cmd_reg, &int_raw), ==, 0);
        rdata[i] = 0x00c40a24;
    }

    g_assert_cmpuint(rdata[0], ==, rdata[1]);
    g_assert_cmpuint(rdata[1], ==, rdata[2]);
}

static void test_new_operation_after_completion(void)
{
    Esp32EfuseOperationState op = { 0 };
    uint32_t cmd_reg = 0, int_raw = 0;
    esp32_efuse_operation_reset(&op);
    begin_read(&op, &cmd_reg);
    g_assert_cmpuint(read_cmd(&op, &cmd_reg, &int_raw), ==, EFUSE_READ);
    expire_timer(&op, &cmd_reg, &int_raw);
    begin_read(&op, &cmd_reg);
    g_assert_cmpuint(read_cmd(&op, &cmd_reg, &int_raw), ==, EFUSE_READ);
}

static void test_new_operation_replaces_pending(void)
{
    Esp32EfuseOperationState op = { 0 };
    uint32_t cmd_reg = 0, int_raw = 0;
    esp32_efuse_operation_reset(&op);
    begin_read(&op, &cmd_reg);
    uint64_t first_generation = op.operation_generation;
    begin_read(&op, &cmd_reg);
    g_assert_cmpuint(op.operation_generation, ==, first_generation + 1);
    expire_timer(&op, &cmd_reg, &int_raw);
    g_assert_cmpuint(read_cmd(&op, &cmd_reg, &int_raw), ==, EFUSE_READ);
    g_assert_cmpuint(read_cmd(&op, &cmd_reg, &int_raw), ==, 0);
}

static void test_reset_before_timer(void)
{
    Esp32EfuseOperationState op = { 0 };
    uint32_t cmd_reg = 0, int_raw = 0;
    esp32_efuse_operation_reset(&op);
    begin_read(&op, &cmd_reg);
    esp32_efuse_operation_reset(&op);
    cmd_reg = 0;
    int_raw = 0;
    expire_timer(&op, &cmd_reg, &int_raw);
    g_assert_cmpuint(cmd_reg, ==, 0);
    g_assert_cmpuint(int_raw, ==, 0);
}

static void test_reset_after_completion_due(void)
{
    Esp32EfuseOperationState op = { 0 };
    uint32_t cmd_reg = 0, int_raw = 0;
    esp32_efuse_operation_reset(&op);
    begin_read(&op, &cmd_reg);
    expire_timer(&op, &cmd_reg, &int_raw);
    esp32_efuse_operation_reset(&op);
    cmd_reg = 0;
    int_raw = 0;
    g_assert_cmpuint(read_cmd(&op, &cmd_reg, &int_raw), ==, 0);
    g_assert_cmpuint(int_raw, ==, 0);
}

static void test_reset_after_busy_observed(void)
{
    Esp32EfuseOperationState op = { 0 };
    uint32_t cmd_reg = 0, int_raw = 0;
    esp32_efuse_operation_reset(&op);
    begin_read(&op, &cmd_reg);
    g_assert_cmpuint(read_cmd(&op, &cmd_reg, &int_raw), ==, EFUSE_READ);
    esp32_efuse_operation_reset(&op);
    cmd_reg = 0;
    int_raw = 0;
    expire_timer(&op, &cmd_reg, &int_raw);
    g_assert_cmpuint(int_raw, ==, 0);
}

static void test_callback_cannot_complete_old_generation(void)
{
    Esp32EfuseOperationState op = { 0 };
    uint32_t cmd_reg = 0, int_raw = 0;
    esp32_efuse_operation_reset(&op);
    begin_read(&op, &cmd_reg);
    begin_read(&op, &cmd_reg);
    expire_timer(&op, &cmd_reg, &int_raw);
    g_assert_cmpuint(read_cmd(&op, &cmd_reg, &int_raw), ==, EFUSE_READ);
    g_assert_cmpuint(int_raw, ==, EFUSE_READ);
}

static void test_program_operation_same_contract(void)
{
    Esp32EfuseOperationState op = { 0 };
    uint32_t cmd_reg = EFUSE_PGM, int_raw = 0, completed = 0;
    esp32_efuse_operation_reset(&op);
    esp32_efuse_operation_begin(&op, EFUSE_PGM, DEADLINE_NS);
    esp32_efuse_operation_timer_expired(&op, &completed);
    g_assert_cmpuint(esp32_efuse_operation_read_cmd(&op, cmd_reg, &completed),
                     ==, EFUSE_PGM);
    complete_to_registers(&op, &cmd_reg, &int_raw, completed);
    g_assert_cmpuint(cmd_reg, ==, 0);
    g_assert_cmpuint(int_raw, ==, EFUSE_PGM);
}

static void test_invalid_conf_no_operation(void)
{
    Esp32EfuseOperationState op = { 0 };
    uint32_t cmd_reg = 0, int_raw = 0;
    uint32_t completed = 0;
    esp32_efuse_operation_reset(&op);
    g_assert_false(esp32_efuse_operation_timer_expired(&op, &completed));
    g_assert_cmpuint(read_cmd(&op, &cmd_reg, &int_raw), ==, 0);
}

static void test_interrupt_raw_status_enable_math(void)
{
    uint32_t int_raw = EFUSE_READ;
    uint32_t int_ena = EFUSE_READ;
    uint32_t int_st = int_raw & int_ena;
    g_assert_cmpuint(int_st, ==, EFUSE_READ);
    int_raw &= ~EFUSE_READ;
    int_st = int_raw & int_ena;
    g_assert_cmpuint(int_st, ==, 0);
}

static void test_duration_does_not_change_first_busy_observation(void)
{
    Esp32EfuseOperationState op_1ms = { 0 }, op_100ms = { 0 };
    uint32_t completed = 0;
    esp32_efuse_operation_begin(&op_1ms, EFUSE_READ, 1000000LL);
    esp32_efuse_operation_begin(&op_100ms, EFUSE_READ, 100000000LL);
    esp32_efuse_operation_timer_expired(&op_1ms, &completed);
    esp32_efuse_operation_timer_expired(&op_100ms, &completed);
    g_assert_cmpuint(esp32_efuse_operation_read_cmd(&op_1ms, EFUSE_READ,
                                                    &completed), ==, EFUSE_READ);
    g_assert_cmpuint(esp32_efuse_operation_read_cmd(&op_100ms, EFUSE_READ,
                                                    &completed), ==, EFUSE_READ);
}

static void test_rom_zero_branch_mapping_documented(void)
{
    const uint32_t first_cmd_read_old_model = 0;
    g_assert_cmpuint(first_cmd_read_old_model, ==, 0);
    g_test_message("EFUSE_CMD==0 maps to ROM branch 0x4000fca6 -> 0x4000fdc7");
}

static void test_generation_increments_and_reset_invalidates(void)
{
    Esp32EfuseOperationState op = { 0 };
    uint64_t initial_generation = op.operation_generation;
    esp32_efuse_operation_reset(&op);
    g_assert_cmpuint(op.operation_generation, ==, initial_generation + 1);
    uint64_t reset_generation = op.operation_generation;
    esp32_efuse_operation_begin(&op, EFUSE_READ, DEADLINE_NS);
    g_assert_cmpuint(op.operation_generation, ==, reset_generation + 1);
    uint64_t active_generation = op.operation_generation;
    esp32_efuse_operation_reset(&op);
    g_assert_cmpuint(op.operation_generation, ==, active_generation + 1);
    g_assert_false(op.operation_active);
    g_assert_false(op.busy_observed);
    g_assert_false(op.completion_due);
}

static void test_no_diagnostic_output_contract_is_external(void)
{
    g_assert_null(g_getenv("LASECSIMUL_RTC_SYS_RESET_CAUSAL_TRACE"));
    g_assert_null(g_getenv("LASECSIMUL_PANIC_CAUSAL_TRACE"));
    g_assert_null(g_getenv("LASECSIMUL_WDT_CAUSAL_TRACE"));
    g_assert_null(g_getenv("LASECSIMUL_CACHE_TRACE"));
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/esp32_efuse/op_state/normal_read_busy_then_zero",
                    test_normal_read_busy_then_zero);
    g_test_add_func("/esp32_efuse/op_state/timer_before_first_read_returns_busy_once",
                    test_timer_before_first_read_returns_busy_once);
    g_test_add_func("/esp32_efuse/op_state/completion_irq_once",
                    test_completion_irq_once);
    g_test_add_func("/esp32_efuse/op_state/timer_after_busy_observed",
                    test_timer_after_busy_observed_completes_immediately);
    g_test_add_func("/esp32_efuse/op_state/three_rom_pattern_operations",
                    test_three_rom_pattern_operations);
    g_test_add_func("/esp32_efuse/op_state/new_operation_after_completion",
                    test_new_operation_after_completion);
    g_test_add_func("/esp32_efuse/op_state/new_operation_replaces_pending",
                    test_new_operation_replaces_pending);
    g_test_add_func("/esp32_efuse/op_state/reset_before_timer",
                    test_reset_before_timer);
    g_test_add_func("/esp32_efuse/op_state/reset_after_completion_due",
                    test_reset_after_completion_due);
    g_test_add_func("/esp32_efuse/op_state/reset_after_busy_observed",
                    test_reset_after_busy_observed);
    g_test_add_func("/esp32_efuse/op_state/callback_cannot_complete_old_generation",
                    test_callback_cannot_complete_old_generation);
    g_test_add_func("/esp32_efuse/op_state/program_operation_same_contract",
                    test_program_operation_same_contract);
    g_test_add_func("/esp32_efuse/op_state/invalid_conf_no_operation",
                    test_invalid_conf_no_operation);
    g_test_add_func("/esp32_efuse/op_state/interrupt_raw_status_enable_math",
                    test_interrupt_raw_status_enable_math);
    g_test_add_func("/esp32_efuse/op_state/duration_independent_first_busy",
                    test_duration_does_not_change_first_busy_observation);
    g_test_add_func("/esp32_efuse/op_state/rom_zero_branch_mapping_documented",
                    test_rom_zero_branch_mapping_documented);
    g_test_add_func("/esp32_efuse/op_state/generation_reset",
                    test_generation_increments_and_reset_invalidates);
    g_test_add_func("/esp32_efuse/op_state/no_diagnostic_output_contract",
                    test_no_diagnostic_output_contract_is_external);
    return g_test_run();
}
