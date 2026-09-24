/*
 * ESP32 Wi-Fi transparent open uplink: device glue (docs/47).
 *
 * Turns the pure decisions of esp32_wifi_link.c into the device's existing
 * DMA-inject queue (frames towards the guest driver) and NIC backend traffic
 * (Ethernet frames towards the host network). Used instead of the legacy
 * beacon-driven AP (esp32_wifi_ap.c) when s->direct_uplink is set.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "net/net.h"
#include "hw/misc/esp32_wifi.h"
#include "esp32_wlan.h"
#include "esp32_wlan_packet.h"

#define TRANSPARENT_SIGNAL   -30

/* Wrap a contiguous 802.11 frame (header + body, no FCS) in the device's
 * mac80211_frame carrier and hand it to the paced inject queue, which prepends
 * the RX control header and DMAs it to the guest. The first 24 bytes of the
 * (packed) carrier are the wire header, so a straight copy lines the body up in
 * data_and_fcs; insert_frame() appends the (zero) FCS. */
static void transparent_queue(Esp32WifiState *s, const uint8_t *raw, size_t len)
{
    if (len < ESP32_WLAN_HDR_LEN || len > sizeof(((mac80211_frame *)0)->data_and_fcs)) {
        return;
    }
    mac80211_frame *frame = g_malloc0(sizeof(*frame));
    memcpy(frame, raw, len);
    frame->frame_length = len;
    frame->signal_strength = TRANSPARENT_SIGNAL;
    frame->next_frame = NULL;
    frame->pos = 0;
    Esp32_WLAN_insert_frame(s, frame);
}

static uint8_t transparent_channel(void)
{
    return esp32_wifi_channel > 0 ? (uint8_t)esp32_wifi_channel : 1;
}

static uint64_t transparent_tsf(void)
{
    return qemu_clock_get_ns(QEMU_CLOCK_REALTIME) / 1000;
}

void Esp32_WLAN_transparent_reset(Esp32WifiState *s)
{
    esp32_wifi_link_reset(&s->link, s->macaddr);
}

void Esp32_WLAN_transparent_tx(Esp32WifiState *s, const uint8_t *frame, size_t len)
{
    uint8_t out[ESP32_WLAN_MAX_FRAME];
    uint8_t eth[ESP32_ETH_MAX_FRAME];
    Esp32WlanHeader h;
    size_t n = 0, eth_len = 0;

    const Esp32WifiLinkAction action = esp32_wifi_link_on_tx(&s->link, frame, len);
    const uint16_t seq = (uint16_t)(s->inject_sequence_number++ & 0xfff);
    const uint8_t *sta = s->link.sta;

    switch (action) {
    case ESP32_WIFI_ACT_PROBE_RESPONSE: {
        const uint8_t *ssid;
        uint8_t ssid_len;
        if (esp32_wlan_parse_header(frame, len, &h) != ESP32_WLAN_OK || !h.addr2) {
            break;
        }
        if (esp32_wlan_find_ie(frame + h.hdr_len, len - h.hdr_len, ESP32_WLAN_IE_SSID,
                               &ssid, &ssid_len) != ESP32_WLAN_OK) {
            break;
        }
        n = esp32_wlan_build_probe_response(out, sizeof(out), h.addr2, s->link.bssid, seq,
                                            ssid, ssid_len, transparent_channel(),
                                            transparent_tsf());
        break;
    }
    case ESP32_WIFI_ACT_AUTH_RESPONSE:
        n = esp32_wlan_build_auth_response(out, sizeof(out), sta, s->link.bssid, seq, 0, 0);
        break;
    case ESP32_WIFI_ACT_AUTH_REJECT:
        /* 13 = unsupported authentication algorithm (the link is open only). */
        n = esp32_wlan_build_auth_response(out, sizeof(out), sta, s->link.bssid, seq, 0, 13);
        break;
    case ESP32_WIFI_ACT_ASSOC_RESPONSE:
        n = esp32_wlan_build_assoc_response(out, sizeof(out), sta, s->link.bssid, seq,
                                            s->link.reassoc, s->link.aid);
        break;
    case ESP32_WIFI_ACT_DEAUTH:
        /* 7 = class-3 frame received from a nonassociated station. */
        n = esp32_wlan_build_deauth(out, sizeof(out), sta, s->link.bssid, seq, 7);
        break;
    case ESP32_WIFI_ACT_FORWARD_DATA:
        if (esp32_wlan_data_to_ethernet(frame, len, eth, sizeof(eth), &eth_len) ==
                ESP32_WLAN_OK) {
            qemu_send_packet(qemu_get_queue(s->nic), eth, eth_len);
        }
        break;
    case ESP32_WIFI_ACT_SOFTAP_UNSUPPORTED:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_wifi: SoftAP/beacon transmit ignored -- the transparent "
                      "uplink only supports station mode\n");
        break;
    case ESP32_WIFI_ACT_NONE:
        break;
    }

    if (n) {
        transparent_queue(s, out, n);
    }
    /* Acknowledge the guest transmit so the driver's TX path completes. */
    Esp32_WLAN_frame_delivered(s);
}

ssize_t Esp32_WLAN_transparent_rx(Esp32WifiState *s, const uint8_t *eth, size_t size)
{
    uint8_t out[ESP32_WLAN_MAX_FRAME];
    size_t n = 0;

    if (!esp32_wifi_link_is_up(&s->link)) {
        return size;                               /* no link yet: drop silently */
    }
    const uint16_t seq = (uint16_t)(s->inject_sequence_number++ & 0xfff);
    if (esp32_wlan_ethernet_to_data(eth, size, s->link.bssid, seq, out, sizeof(out), &n) ==
            ESP32_WLAN_OK) {
        transparent_queue(s, out, n);
    }
    return size;
}
