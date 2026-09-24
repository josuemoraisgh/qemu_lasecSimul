/*
 * Unit tests for the ESP32 Wi-Fi transparent uplink logic.
 *
 * Frames marked "captured" were transmitted by the real ESP-IDF 5.5 driver
 * (arduino-esp32 3.3.9) running in this QEMU, dumped from the TX DMA.
 */
#include "qemu/osdep.h"
#include "hw/misc/esp32_wifi_link.h"

static const uint8_t STA[6] = { 0x02, 0x4c, 0x2a, 0x01, 0x12, 0x34 };
static const uint8_t BCAST[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

static size_t unhex(const char *hex, uint8_t *out)
{
    size_t n = 0;
    for (; hex[0] && hex[1]; hex += 2) {
        unsigned v;
        g_assert(sscanf(hex, "%2x", &v) == 1);
        out[n++] = (uint8_t)v;
    }
    return n;
}

static size_t make_mgmt(uint8_t *f, uint8_t subtype, const uint8_t a1[6],
                        const uint8_t a3[6], const uint8_t *body, size_t blen)
{
    memset(f, 0, ESP32_WLAN_HDR_LEN);
    f[0] = (uint8_t)(subtype << 4);
    memcpy(f + 4, a1, 6);
    memcpy(f + 10, STA, 6);
    memcpy(f + 16, a3, 6);
    if (blen) {
        memcpy(f + ESP32_WLAN_HDR_LEN, body, blen);
    }
    return ESP32_WLAN_HDR_LEN + blen;
}

static size_t make_data(uint8_t *f, const uint8_t bssid[6], uint8_t subtype)
{
    static const uint8_t arp[28] = { 0, 1, 8, 0, 6, 4, 0, 1 };
    memset(f, 0, 80);
    f[0] = (uint8_t)((subtype << 4) | (ESP32_WLAN_TYPE_DATA << 2));
    f[1] = ESP32_WLAN_FLAG_TO_DS;
    memcpy(f + 4, bssid, 6);
    memcpy(f + 10, STA, 6);
    memcpy(f + 16, BCAST, 6);
    const uint8_t llc[8] = { 0xaa, 0xaa, 0x03, 0, 0, 0, 0x08, 0x06 };
    memcpy(f + 24, llc, 8);
    memcpy(f + 32, arp, sizeof(arp));
    return 32 + sizeof(arp);
}

static void test_ie_parser(void)
{
    const uint8_t *v;
    uint8_t vl;
    const uint8_t ies[] = { 0, 4, 'a', 'b', 'c', 'd', 1, 2, 0x82, 0x84, 3, 1, 6 };

    g_assert_cmpint(esp32_wlan_find_ie(ies, sizeof(ies), 0, &v, &vl), ==, ESP32_WLAN_OK);
    g_assert_cmpuint(vl, ==, 4);
    g_assert(!memcmp(v, "abcd", 4));
    g_assert_cmpint(esp32_wlan_find_ie(ies, sizeof(ies), 3, &v, &vl), ==, ESP32_WLAN_OK);
    g_assert_cmpuint(v[0], ==, 6);
    g_assert_cmpint(esp32_wlan_find_ie(ies, sizeof(ies), 48, &v, &vl), ==,
                    ESP32_WLAN_ERR_UNSUPPORTED);
    g_assert_cmpint(esp32_wlan_find_ie(ies, 0, 0, &v, &vl), ==, ESP32_WLAN_ERR_UNSUPPORTED);

    const uint8_t overrun[] = { 1, 2, 0x82, 0x84, 0, 32, 'x' };
    g_assert_cmpint(esp32_wlan_find_ie(overrun, sizeof(overrun), 0, &v, &vl), ==,
                    ESP32_WLAN_ERR_MALFORMED);
    const uint8_t dangling[] = { 1, 1, 0x82, 0 };
    g_assert_cmpint(esp32_wlan_find_ie(dangling, sizeof(dangling), 48, &v, &vl), ==,
                    ESP32_WLAN_ERR_MALFORMED);
}

static void test_header_lengths(void)
{
    uint8_t f[64] = { 0 };
    Esp32WlanHeader h;

    f[0] = 0x08;
    g_assert_cmpint(esp32_wlan_parse_header(f, 24, &h), ==, ESP32_WLAN_OK);
    g_assert_cmpuint(h.hdr_len, ==, 24);
    f[0] = 0x88;
    g_assert_cmpint(esp32_wlan_parse_header(f, 26, &h), ==, ESP32_WLAN_OK);
    g_assert_cmpuint(h.hdr_len, ==, 26);
    g_assert_cmpint(esp32_wlan_parse_header(f, 25, &h), ==, ESP32_WLAN_ERR_TRUNCATED);
    f[1] = ESP32_WLAN_FLAG_TO_DS | ESP32_WLAN_FLAG_FROM_DS;
    g_assert_cmpint(esp32_wlan_parse_header(f, 32, &h), ==, ESP32_WLAN_OK);
    g_assert_cmpuint(h.hdr_len, ==, 32);
    f[0] = 0xd4;
    f[1] = 0;
    g_assert_cmpint(esp32_wlan_parse_header(f, 10, &h), ==, ESP32_WLAN_OK);
    g_assert_cmpint(esp32_wlan_parse_header(f, 9, &h), ==, ESP32_WLAN_ERR_TRUNCATED);
}

static void test_data_to_ethernet_captured(void)
{
    uint8_t frame[256], eth[1600];
    size_t len, eth_len;

    /* Captured ARP request; DMA length 64 includes the 4-byte FCS placeholder. */
    len = unhex("08010000100100c40a56100100c40a24ffffffffffff2000aaaa0300000008060001"
                "080006040001100100c40a24000000000000000000000a00020fefbeadde", frame);
    g_assert_cmpuint(len, ==, 64);
    g_assert_cmpint(esp32_wlan_data_to_ethernet(frame, len - ESP32_WLAN_FCS_LEN, eth,
                                                sizeof(eth), &eth_len), ==, ESP32_WLAN_OK);
    g_assert_cmpuint(eth_len, ==, 14 + 28);        /* no legacy "+22" garbage */
    g_assert(!memcmp(eth, BCAST, 6));              /* DA = addr3 */
    g_assert(!memcmp(eth + 6, frame + 10, 6));     /* SA = addr2 */
    g_assert_cmpuint(eth[12], ==, 0x08);
    g_assert_cmpuint(eth[13], ==, 0x06);

    /* Captured DNS query (IPv4 total length 57). */
    len = unhex("08010000100100c40a56100100c40a2452550a0002038000aaaa030000000800450000"
                "3900020000401162a10a00020f0a000203a475003500250146723401000001000000"
                "000000076578616d706c6503636f6d0000010001efbeadde", frame);
    g_assert_cmpint(esp32_wlan_data_to_ethernet(frame, len - ESP32_WLAN_FCS_LEN, eth,
                                                sizeof(eth), &eth_len), ==, ESP32_WLAN_OK);
    g_assert_cmpuint(eth_len, ==, 14 + 57);
    g_assert(!memcmp(eth, frame + 16, 6));         /* real gateway MAC, not the BSSID */

    /* IPv4 total length larger than the frame: rejected, never over-read. */
    frame[32 + 3] = 0xff;
    g_assert_cmpint(esp32_wlan_data_to_ethernet(frame, len - ESP32_WLAN_FCS_LEN, eth,
                                                sizeof(eth), &eth_len), ==,
                    ESP32_WLAN_ERR_MALFORMED);
    frame[32 + 3] = 0x39;

    /* Destination buffer too small: rejected instead of truncated. */
    g_assert_cmpint(esp32_wlan_data_to_ethernet(frame, len - ESP32_WLAN_FCS_LEN, eth, 40,
                                                &eth_len), ==, ESP32_WLAN_ERR_TOO_BIG);

    frame[1] |= ESP32_WLAN_FLAG_PROTECTED;
    g_assert_cmpint(esp32_wlan_data_to_ethernet(frame, len - ESP32_WLAN_FCS_LEN, eth,
                                                sizeof(eth), &eth_len), ==,
                    ESP32_WLAN_ERR_UNSUPPORTED);

    /* Null function carries no payload. */
    uint8_t null_frame[24] = { 0x48, 0x01 };
    g_assert_cmpint(esp32_wlan_data_to_ethernet(null_frame, 24, eth, sizeof(eth), &eth_len),
                    ==, ESP32_WLAN_ERR_NO_PAYLOAD);
}

static void test_ethernet_to_data(void)
{
    uint8_t eth[64], frame[128];
    size_t flen, blen;
    const uint8_t bssid[6] = { 0x06, 0x4c, 0x2a, 0x01, 0x12, 0x6e };
    const uint8_t gw[6] = { 0x52, 0x55, 0x0a, 0x00, 0x02, 0x02 };

    memcpy(eth, STA, 6);
    memcpy(eth + 6, gw, 6);
    eth[12] = 0x08;
    eth[13] = 0x06;
    for (int i = 0; i < 28; i++) {
        eth[14 + i] = (uint8_t)i;
    }
    g_assert_cmpint(esp32_wlan_ethernet_to_data(eth, 42, bssid, 7, frame, sizeof(frame),
                                                &flen), ==, ESP32_WLAN_OK);
    g_assert_cmpuint(flen, ==, 24 + 8 + 28);
    g_assert_cmpuint(frame[0], ==, 0x08);
    g_assert_cmpuint(frame[1], ==, ESP32_WLAN_FLAG_FROM_DS);
    g_assert(!memcmp(frame + 4, STA, 6));          /* addr1 = DA */
    g_assert(!memcmp(frame + 10, bssid, 6));       /* addr2 = BSSID */
    g_assert(!memcmp(frame + 16, gw, 6));          /* addr3 = SA */
    g_assert_cmpuint(frame[22] | (frame[23] << 8), ==, 7 << 4);
    g_assert(!memcmp(frame + 32, eth + 14, 28));

    /* Round trip back through the station direction. */
    uint8_t back[128], eth2[64];
    memcpy(back, frame, flen);
    back[1] = ESP32_WLAN_FLAG_TO_DS;
    memcpy(back + 4, bssid, 6);
    memcpy(back + 10, STA, 6);
    memcpy(back + 16, gw, 6);
    g_assert_cmpint(esp32_wlan_data_to_ethernet(back, flen, eth2, sizeof(eth2), &blen), ==,
                    ESP32_WLAN_OK);
    g_assert_cmpuint(blen, ==, 42);
    g_assert(!memcmp(eth2 + 12, eth + 12, 30));

    eth[12] = 0x00;                                /* 802.3 length field */
    g_assert_cmpint(esp32_wlan_ethernet_to_data(eth, 42, bssid, 0, frame, sizeof(frame),
                                                &flen), ==, ESP32_WLAN_ERR_UNSUPPORTED);
    eth[12] = 0x08;
    g_assert_cmpint(esp32_wlan_ethernet_to_data(eth, 42, bssid, 0, frame, 40, &flen), ==,
                    ESP32_WLAN_ERR_TOO_BIG);
    g_assert_cmpint(esp32_wlan_ethernet_to_data(eth, 10, bssid, 0, frame, sizeof(frame),
                                                &flen), ==, ESP32_WLAN_ERR_TRUNCATED);
}

static void test_builders(void)
{
    uint8_t f[256];
    const uint8_t bssid[6] = { 0x06, 0x4c, 0x2a, 0x01, 0x12, 0x6e };
    const uint8_t *v;
    uint8_t vl;
    size_t n;

    n = esp32_wlan_build_probe_response(f, sizeof(f), STA, bssid, 1,
                                        (const uint8_t *)"qualquer", 8, 6, 1234);
    g_assert_cmpuint(n, >, 36);
    g_assert_cmpuint(f[0], ==, 0x50);
    g_assert_cmpint(esp32_wlan_find_ie(f + 36, n - 36, ESP32_WLAN_IE_SSID, &v, &vl), ==,
                    ESP32_WLAN_OK);
    g_assert_cmpuint(vl, ==, 8);
    g_assert(!memcmp(v, "qualquer", 8));
    g_assert_cmpint(esp32_wlan_find_ie(f + 36, n - 36, ESP32_WLAN_IE_DS_PARAMS, &v, &vl), ==,
                    ESP32_WLAN_OK);
    g_assert_cmpuint(v[0], ==, 6);
    g_assert_cmpuint(f[34] & 0x10, ==, 0);         /* capability: no privacy bit */
    g_assert_cmpuint(esp32_wlan_build_probe_response(f, 20, STA, bssid, 1,
                                                     (const uint8_t *)"x", 1, 6, 0), ==, 0);

    n = esp32_wlan_build_beacon(f, sizeof(f), bssid, 2, 6, 0);
    g_assert_cmpint(esp32_wlan_find_ie(f + 36, n - 36, ESP32_WLAN_IE_SSID, &v, &vl), ==,
                    ESP32_WLAN_OK);
    g_assert_cmpuint(vl, ==, 0);                   /* hidden: no network announced */

    n = esp32_wlan_build_assoc_response(f, sizeof(f), STA, bssid, 3, false, 1);
    g_assert_cmpuint(f[0], ==, 0x10);
    g_assert_cmpuint(f[24] | (f[25] << 8), ==, 0x0001);
    g_assert_cmpuint(f[26] | (f[27] << 8), ==, 0);
    g_assert_cmpuint(f[28] | (f[29] << 8), ==, 0xc001);

    n = esp32_wlan_build_auth_response(f, sizeof(f), STA, bssid, 4, 0, 0);
    g_assert_cmpuint(n, ==, 30);
    g_assert_cmpuint(f[26], ==, 2);
}

static Esp32WifiLinkAction tx(Esp32WifiLink *l, const uint8_t *f, size_t n)
{
    return esp32_wifi_link_on_tx(l, f, n);
}

static void connect_link(Esp32WifiLink *l)
{
    uint8_t f[128];
    const uint8_t probe[] = { 0, 3, 'x', 'y', 'z', 1, 1, 0x82 };
    const uint8_t auth[] = { 0, 0, 1, 0, 0, 0 };
    const uint8_t assoc[] = { 0x21, 0x04, 0x03, 0x00, 0, 3, 'x', 'y', 'z' };

    g_assert_cmpint(tx(l, f, make_mgmt(f, ESP32_WLAN_MGT_PROBE_REQ, BCAST, BCAST, probe,
                                       sizeof(probe))), ==, ESP32_WIFI_ACT_PROBE_RESPONSE);
    g_assert_cmpint(tx(l, f, make_mgmt(f, ESP32_WLAN_MGT_AUTH, l->bssid, l->bssid, auth,
                                       sizeof(auth))), ==, ESP32_WIFI_ACT_AUTH_RESPONSE);
    g_assert_cmpint(l->state, ==, ESP32_WIFI_LINK_AUTHENTICATED);
    g_assert_cmpint(tx(l, f, make_mgmt(f, ESP32_WLAN_MGT_ASSOC_REQ, l->bssid, l->bssid, assoc,
                                       sizeof(assoc))), ==, ESP32_WIFI_ACT_ASSOC_RESPONSE);
    g_assert(esp32_wifi_link_is_up(l));
}

static void test_state_machine(void)
{
    Esp32WifiLink l = { 0 };
    uint8_t f[128];
    esp32_wifi_link_reset(&l, STA);
    g_assert_cmpint(l.state, ==, ESP32_WIFI_LINK_IDLE);
    g_assert(memcmp(l.bssid, STA, 6));
    g_assert_cmpuint(l.bssid[0] & 0x03, ==, 0x02);

    /* Wildcard scan: nothing is announced. */
    const uint8_t wildcard[] = { 0, 0, 1, 1, 0x82 };
    g_assert_cmpint(tx(&l, f, make_mgmt(f, ESP32_WLAN_MGT_PROBE_REQ, BCAST, BCAST, wildcard,
                                        sizeof(wildcard))), ==, ESP32_WIFI_ACT_NONE);

    /* Data before association is refused with a deauthentication. */
    g_assert_cmpint(tx(&l, f, make_data(f, l.bssid, 0)), ==, ESP32_WIFI_ACT_DEAUTH);

    connect_link(&l);
    g_assert_cmpuint(l.link_ups, ==, 1);
    g_assert_cmpint(tx(&l, f, make_data(f, l.bssid, 0)), ==, ESP32_WIFI_ACT_FORWARD_DATA);
    g_assert_cmpint(tx(&l, f, make_data(f, l.bssid, 4)), ==, ESP32_WIFI_ACT_NONE);

    /* Station leaves: deterministic return to idle, counted once. */
    const uint8_t reason[] = { 8, 0 };
    g_assert_cmpint(tx(&l, f, make_mgmt(f, ESP32_WLAN_MGT_DEAUTH, l.bssid, l.bssid, reason,
                                        sizeof(reason))), ==, ESP32_WIFI_ACT_NONE);
    g_assert_cmpint(l.state, ==, ESP32_WIFI_LINK_IDLE);
    g_assert_cmpuint(l.link_downs, ==, 1);

    /* Non-open authentication (SAE) is rejected, never negotiated. */
    const uint8_t sae[] = { 3, 0, 1, 0, 0, 0 };
    g_assert_cmpint(tx(&l, f, make_mgmt(f, ESP32_WLAN_MGT_AUTH, l.bssid, l.bssid, sae,
                                        sizeof(sae))), ==, ESP32_WIFI_ACT_AUTH_REJECT);

    /* Station beaconing (SoftAP) is reported as unsupported. */
    g_assert_cmpint(tx(&l, f, make_mgmt(f, ESP32_WLAN_MGT_BEACON, BCAST, STA, NULL, 0)), ==,
                    ESP32_WIFI_ACT_SOFTAP_UNSUPPORTED);

    /* 100 reconnect/reset cycles stay deterministic. */
    for (int i = 0; i < 100; i++) {
        connect_link(&l);
        esp32_wifi_link_reset(&l, STA);
        g_assert_cmpint(l.state, ==, ESP32_WIFI_LINK_IDLE);
    }
    g_assert_cmpuint(l.link_ups, ==, 101);
    g_assert_cmpuint(l.link_downs, ==, 101);
}

/*
 * With the explicit ignore-password option the security fields of the station
 * config the driver built are cleared to WIFI_AUTH_OPEN before the driver uses
 * them; the SSID is left untouched. Verifies the byte ranges only.
 */
static void test_open_config_ranges(void)
{
    const Esp32WifiDriverProfile *p = &esp32_wifi_driver_profiles[0];
    uint8_t cfg[256];
    memset(cfg, 0x5a, sizeof(cfg));

    esp32_wifi_neutralize_sta_config(p, cfg);

    /* SSID (offset 0..31) preserved. */
    for (uint32_t i = 0; i < p->off_password && i < 32; i++) {
        g_assert_cmpuint(cfg[i], ==, 0x5a);
    }
    for (uint32_t i = 0; i < p->password_len; i++) {
        g_assert_cmpuint(cfg[p->off_password + i], ==, 0);
    }
    g_assert_cmpuint(cfg[p->off_bssid_set], ==, 0);
    for (int i = 0; i < 4; i++) {
        g_assert_cmpuint(cfg[p->off_authmode + i], ==, 0);   /* WIFI_AUTH_OPEN */
    }
    g_assert_cmpuint(cfg[p->off_pmf_required], ==, 0);
    /* A byte just past the last cleared field stays untouched. */
    g_assert_cmpuint(cfg[p->off_authmode + 8], ==, 0x5a);

    Esp32WifiConfigRange ranges[ESP32_WIFI_CONFIG_RANGES];
    g_assert_cmpint(esp32_wifi_sta_config_ranges(p, ranges), ==, ESP32_WIFI_CONFIG_RANGES);
    g_assert_cmpuint(p->config_size, ==, 184);
    for (int i = 0; i < ESP32_WIFI_CONFIG_RANGES; i++) {
        g_assert_cmpuint(ranges[i].offset + ranges[i].len, <=, p->config_size);
    }
}

int main(int argc, char **argv)
{
    g_test_init(&argc, &argv, NULL);
    g_test_add_func("/esp32-wifi-link/ie-parser", test_ie_parser);
    g_test_add_func("/esp32-wifi-link/header-lengths", test_header_lengths);
    g_test_add_func("/esp32-wifi-link/data-to-ethernet", test_data_to_ethernet_captured);
    g_test_add_func("/esp32-wifi-link/ethernet-to-data", test_ethernet_to_data);
    g_test_add_func("/esp32-wifi-link/builders", test_builders);
    g_test_add_func("/esp32-wifi-link/state-machine", test_state_machine);
    g_test_add_func("/esp32-wifi-link/open-config-ranges", test_open_config_ranges);
    return g_test_run();
}
