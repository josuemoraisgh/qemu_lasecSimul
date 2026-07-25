/*
 * QTest tests for the ESP32 network devices.
 *
 * Copyright (c) 2026 LasecSimul contributors
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 */

#include "qemu/osdep.h"
#include "libqtest.h"

#define ESP32_EMAC_BASE        0x3ff69000
#define ESP32_WIFI_BASE        0x3ff73000
#define OPENETH_MODER          0x00
#define OPENETH_TX_BD_NUM      0x20
#define OPENETH_MAC_ADDR0      0x40
#define OPENETH_MAC_ADDR1      0x44

#define OPENETH_MODER_RESET    0x0000a000
#define OPENETH_MODER_TXEN     0x00000002
#define OPENETH_MODER_RXEN     0x00000001

#define WIFI_DMA_IN_STATUS     0x084
#define WIFI_STATUS            0xcc8
#define WIFI_DMA_OUT_STATUS    0xd24

static QTestState *esp32_openeth_start(void)
{
    return qtest_init("-no-user-config -machine esp32 "
                      "-nic user,model=open_eth,mac=02:00:00:12:34:56");
}

static void test_openeth_reset_registers(void)
{
    QTestState *s = esp32_openeth_start();

    g_assert_cmphex(qtest_readl(s, ESP32_EMAC_BASE + OPENETH_MODER),
                    ==, OPENETH_MODER_RESET);
    g_assert_cmphex(qtest_readl(s, ESP32_EMAC_BASE + OPENETH_TX_BD_NUM),
                    ==, 0x40);

    /* MAC_ADDR1 holds bytes 0..1; MAC_ADDR0 holds bytes 2..5. */
    g_assert_cmphex(qtest_readl(s, ESP32_EMAC_BASE + OPENETH_MAC_ADDR1),
                    ==, 0x0200);
    g_assert_cmphex(qtest_readl(s, ESP32_EMAC_BASE + OPENETH_MAC_ADDR0),
                    ==, 0x00123456);

    qtest_quit(s);
}

static void test_openeth_enable_and_system_reset(void)
{
    QTestState *s = esp32_openeth_start();
    uint32_t enabled = OPENETH_MODER_RESET |
                       OPENETH_MODER_TXEN |
                       OPENETH_MODER_RXEN;

    qtest_writel(s, ESP32_EMAC_BASE + OPENETH_MODER, enabled);
    g_assert_cmphex(qtest_readl(s, ESP32_EMAC_BASE + OPENETH_MODER),
                    ==, enabled);

    qtest_qmp_assert_success(s, "{ 'execute': 'system_reset' }");
    qtest_qmp_eventwait(s, "RESET");
    g_assert_cmphex(qtest_readl(s, ESP32_EMAC_BASE + OPENETH_MODER),
                    ==, OPENETH_MODER_RESET);
    g_assert_cmphex(qtest_readl(s, ESP32_EMAC_BASE + OPENETH_MAC_ADDR1),
                    ==, 0x0200);
    g_assert_cmphex(qtest_readl(s, ESP32_EMAC_BASE + OPENETH_MAC_ADDR0),
                    ==, 0x00123456);

    qtest_quit(s);
}

static void test_openeth_and_wifi_coexist(void)
{
    QTestState *s = qtest_init(
        "-no-user-config -machine esp32 "
        "-nic user,model=open_eth,net=192.168.4.0/24,"
        "mac=02:00:00:12:34:56 "
        "-nic user,model=esp32_wifi,net=192.168.5.0/24,"
        "mac=02:00:00:65:43:21");

    g_assert_cmphex(qtest_readl(s, ESP32_EMAC_BASE + OPENETH_MODER),
                    ==, OPENETH_MODER_RESET);
    g_assert_cmphex(qtest_readl(s, ESP32_WIFI_BASE + WIFI_DMA_IN_STATUS),
                    ==, 0);
    g_assert_cmphex(qtest_readl(s, ESP32_WIFI_BASE + WIFI_STATUS), ==, 1);
    g_assert_cmphex(qtest_readl(s, ESP32_WIFI_BASE + WIFI_DMA_OUT_STATUS),
                    ==, 1);

    qtest_quit(s);
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);

    qtest_add_func("/esp32/network/openeth/reset-registers",
                   test_openeth_reset_registers);
    qtest_add_func("/esp32/network/openeth/enable-and-system-reset",
                   test_openeth_enable_and_system_reset);
    qtest_add_func("/esp32/network/openeth-and-wifi/coexist",
                   test_openeth_and_wifi_coexist);

    return g_test_run();
}
