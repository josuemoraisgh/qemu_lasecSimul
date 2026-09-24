/*
 * ESP32 Wi-Fi transparent uplink: pure frame/link logic.
 * See include/hw/misc/esp32_wifi_link.h for the model.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */
#include "qemu/osdep.h"
#include "hw/misc/esp32_wifi_link.h"

static const uint8_t broadcast_mac[6] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
static const uint8_t llc_snap[6] = { 0xaa, 0xaa, 0x03, 0x00, 0x00, 0x00 };
static const uint8_t llc_bridge_tunnel[6] = { 0xaa, 0xaa, 0x03, 0x00, 0x00, 0xf8 };
/* 1, 2, 5.5, 11 Mbps (basic) + 6, 9, 12, 18 Mbps; 24, 36, 48, 54 Mbps extended. */
static const uint8_t supported_rates[8] = { 0x82, 0x84, 0x8b, 0x96, 0x0c, 0x12, 0x18, 0x24 };
static const uint8_t extended_rates[4] = { 0x30, 0x48, 0x60, 0x6c };

static inline uint16_t le16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

static inline uint16_t be16(const uint8_t *p)
{
    return (uint16_t)((p[0] << 8) | p[1]);
}

static inline void put_le16(uint8_t *p, uint16_t v)
{
    p[0] = v & 0xff;
    p[1] = v >> 8;
}

int esp32_wlan_parse_header(const uint8_t *frame, size_t len, Esp32WlanHeader *h)
{
    if (len < 10) {
        return ESP32_WLAN_ERR_TRUNCATED;
    }
    memset(h, 0, sizeof(*h));
    h->type = (frame[0] >> 2) & 0x3;
    h->subtype = (frame[0] >> 4) & 0xf;
    h->flags = frame[1];
    h->addr1 = frame + 4;
    if ((frame[0] & 0x3) != 0) {
        return ESP32_WLAN_ERR_UNSUPPORTED;        /* protocol version must be 0 */
    }
    if (h->type == ESP32_WLAN_TYPE_CTL) {
        h->hdr_len = len < 16 ? 10 : 16;
        h->addr2 = len >= 16 ? frame + 10 : NULL;
        return ESP32_WLAN_OK;
    }
    if (h->type != ESP32_WLAN_TYPE_MGT && h->type != ESP32_WLAN_TYPE_DATA) {
        return ESP32_WLAN_ERR_UNSUPPORTED;
    }
    size_t hdr = ESP32_WLAN_HDR_LEN;
    if (h->type == ESP32_WLAN_TYPE_DATA) {
        if ((h->flags & (ESP32_WLAN_FLAG_TO_DS | ESP32_WLAN_FLAG_FROM_DS)) ==
            (ESP32_WLAN_FLAG_TO_DS | ESP32_WLAN_FLAG_FROM_DS)) {
            hdr += 6;                              /* addr4 */
        }
        if (h->subtype & 0x8) {
            hdr += 2;                              /* QoS control */
            if (h->flags & ESP32_WLAN_FLAG_ORDER) {
                hdr += 4;                          /* HT control */
            }
        }
    } else if (h->flags & ESP32_WLAN_FLAG_ORDER) {
        hdr += 4;
    }
    if (len < hdr) {
        return ESP32_WLAN_ERR_TRUNCATED;
    }
    h->addr2 = frame + 10;
    h->addr3 = frame + 16;
    h->seq_ctl = le16(frame + 22);
    h->hdr_len = hdr;
    return ESP32_WLAN_OK;
}

int esp32_wlan_find_ie(const uint8_t *ies, size_t len, uint8_t id,
                       const uint8_t **value, uint8_t *value_len)
{
    size_t off = 0;
    while (off < len) {
        if (len - off < 2) {
            return ESP32_WLAN_ERR_MALFORMED;
        }
        const uint8_t eid = ies[off];
        const uint8_t elen = ies[off + 1];
        if (len - off - 2 < elen) {
            return ESP32_WLAN_ERR_MALFORMED;
        }
        if (eid == id) {
            *value = ies + off + 2;
            *value_len = elen;
            return ESP32_WLAN_OK;
        }
        off += 2u + elen;
    }
    return ESP32_WLAN_ERR_UNSUPPORTED;
}

int esp32_wlan_data_to_ethernet(const uint8_t *frame, size_t len,
                                uint8_t *eth, size_t cap, size_t *eth_len)
{
    Esp32WlanHeader h;
    int r = esp32_wlan_parse_header(frame, len, &h);
    if (r != ESP32_WLAN_OK) {
        return r;
    }
    if (h.type != ESP32_WLAN_TYPE_DATA) {
        return ESP32_WLAN_ERR_UNSUPPORTED;
    }
    if (h.subtype & 0x4) {
        return ESP32_WLAN_ERR_NO_PAYLOAD;          /* (QoS) null function */
    }
    if (h.flags & ESP32_WLAN_FLAG_PROTECTED) {
        return ESP32_WLAN_ERR_UNSUPPORTED;         /* the transparent link is open */
    }
    if ((h.flags & (ESP32_WLAN_FLAG_TO_DS | ESP32_WLAN_FLAG_FROM_DS)) != ESP32_WLAN_FLAG_TO_DS) {
        return ESP32_WLAN_ERR_UNSUPPORTED;         /* a station only transmits ToDS */
    }
    const uint8_t *body = frame + h.hdr_len;
    const size_t body_len = len - h.hdr_len;
    if (body_len < ESP32_WLAN_LLC_LEN) {
        return ESP32_WLAN_ERR_TRUNCATED;
    }
    if (memcmp(body, llc_snap, 6) != 0 && memcmp(body, llc_bridge_tunnel, 6) != 0) {
        return ESP32_WLAN_ERR_UNSUPPORTED;
    }
    const size_t payload_len = body_len - ESP32_WLAN_LLC_LEN;
    const uint8_t *payload = body + ESP32_WLAN_LLC_LEN;
    const uint16_t ethertype = be16(body + 6);
    if (ethertype == 0x0800) {
        if (payload_len < 20 || (payload[0] >> 4) != 4) {
            return ESP32_WLAN_ERR_MALFORMED;
        }
        if (be16(payload + 2) > payload_len) {
            return ESP32_WLAN_ERR_MALFORMED;       /* IPv4 total length overruns the frame */
        }
    } else if (ethertype == 0x0806 && payload_len < 28) {
        return ESP32_WLAN_ERR_MALFORMED;
    }
    const size_t out_len = ESP32_ETH_HDR_LEN + payload_len;
    if (out_len > cap || out_len > ESP32_ETH_MAX_FRAME) {
        return ESP32_WLAN_ERR_TOO_BIG;
    }
    memcpy(eth, h.addr3, 6);                       /* DA */
    memcpy(eth + 6, h.addr2, 6);                   /* SA */
    eth[12] = body[6];
    eth[13] = body[7];
    memcpy(eth + ESP32_ETH_HDR_LEN, payload, payload_len);
    *eth_len = out_len;
    return ESP32_WLAN_OK;
}

static uint8_t *put_header(uint8_t *out, uint8_t type, uint8_t subtype, uint8_t flags,
                           const uint8_t *a1, const uint8_t *a2, const uint8_t *a3,
                           uint16_t seq)
{
    out[0] = (uint8_t)((subtype << 4) | (type << 2));
    out[1] = flags;
    put_le16(out + 2, 0);                          /* duration */
    memcpy(out + 4, a1, 6);
    memcpy(out + 10, a2, 6);
    memcpy(out + 16, a3, 6);
    put_le16(out + 22, (uint16_t)((seq & 0xfff) << 4));
    return out + ESP32_WLAN_HDR_LEN;
}

int esp32_wlan_ethernet_to_data(const uint8_t *eth, size_t eth_len,
                                const uint8_t bssid[6], uint16_t seq,
                                uint8_t *frame, size_t cap, size_t *frame_len)
{
    if (eth_len < ESP32_ETH_HDR_LEN) {
        return ESP32_WLAN_ERR_TRUNCATED;
    }
    if (be16(eth + 12) < 0x0600) {
        return ESP32_WLAN_ERR_UNSUPPORTED;         /* 802.3 length field / raw LLC */
    }
    const size_t payload_len = eth_len - ESP32_ETH_HDR_LEN;
    const size_t out_len = ESP32_WLAN_HDR_LEN + ESP32_WLAN_LLC_LEN + payload_len;
    if (out_len > cap || out_len + ESP32_WLAN_FCS_LEN > ESP32_WLAN_MAX_FRAME) {
        return ESP32_WLAN_ERR_TOO_BIG;
    }
    uint8_t *p = put_header(frame, ESP32_WLAN_TYPE_DATA, 0, ESP32_WLAN_FLAG_FROM_DS,
                            eth, bssid, eth + 6, seq);
    memcpy(p, llc_snap, 6);
    p[6] = eth[12];
    p[7] = eth[13];
    memcpy(p + ESP32_WLAN_LLC_LEN, eth + ESP32_ETH_HDR_LEN, payload_len);
    *frame_len = out_len;
    return ESP32_WLAN_OK;
}

static uint8_t *put_ie(uint8_t *p, uint8_t id, const uint8_t *data, uint8_t len)
{
    p[0] = id;
    p[1] = len;
    if (len) {
        memcpy(p + 2, data, len);
    }
    return p + 2 + len;
}

static uint8_t *put_bss_fixed(uint8_t *p, uint64_t tsf_us)
{
    for (int i = 0; i < 8; i++) {
        p[i] = (uint8_t)(tsf_us >> (8 * i));
    }
    put_le16(p + 8, ESP32_WIFI_LINK_BEACON_TU);
    put_le16(p + 10, 0x0001);                      /* ESS, no privacy */
    return p + 12;
}

static uint8_t *put_rates(uint8_t *p)
{
    p = put_ie(p, ESP32_WLAN_IE_RATES, supported_rates, sizeof(supported_rates));
    return put_ie(p, ESP32_WLAN_IE_EXT_RATES, extended_rates, sizeof(extended_rates));
}

#define BSS_FRAME_MAX (ESP32_WLAN_HDR_LEN + 12 + 2 + 32 + 2 + 8 + 2 + 1 + 2 + 4)

size_t esp32_wlan_build_probe_response(uint8_t *out, size_t cap, const uint8_t sta[6],
                                       const uint8_t bssid[6], uint16_t seq,
                                       const uint8_t *ssid, uint8_t ssid_len,
                                       uint8_t channel, uint64_t tsf_us)
{
    if (cap < BSS_FRAME_MAX || ssid_len > 32) {
        return 0;
    }
    uint8_t *p = put_header(out, ESP32_WLAN_TYPE_MGT, ESP32_WLAN_MGT_PROBE_RESP, 0,
                            sta, bssid, bssid, seq);
    p = put_bss_fixed(p, tsf_us);
    p = put_ie(p, ESP32_WLAN_IE_SSID, ssid, ssid_len);
    p = put_rates(p);
    p = put_ie(p, ESP32_WLAN_IE_DS_PARAMS, &channel, 1);
    return (size_t)(p - out);
}

size_t esp32_wlan_build_beacon(uint8_t *out, size_t cap, const uint8_t bssid[6],
                               uint16_t seq, uint8_t channel, uint64_t tsf_us)
{
    static const uint8_t tim[4] = { 0, 1, 0, 0 };
    if (cap < BSS_FRAME_MAX + 6) {
        return 0;
    }
    uint8_t *p = put_header(out, ESP32_WLAN_TYPE_MGT, ESP32_WLAN_MGT_BEACON, 0,
                            broadcast_mac, bssid, bssid, seq);
    p = put_bss_fixed(p, tsf_us);
    /* Hidden SSID: the associated station follows the BSSID, and the link never
     * advertises a network of its own. */
    p = put_ie(p, ESP32_WLAN_IE_SSID, NULL, 0);
    p = put_rates(p);
    p = put_ie(p, ESP32_WLAN_IE_DS_PARAMS, &channel, 1);
    p = put_ie(p, 5, tim, sizeof(tim));
    return (size_t)(p - out);
}

size_t esp32_wlan_build_auth_response(uint8_t *out, size_t cap, const uint8_t sta[6],
                                      const uint8_t bssid[6], uint16_t seq,
                                      uint16_t algorithm, uint16_t status)
{
    if (cap < ESP32_WLAN_HDR_LEN + 6) {
        return 0;
    }
    uint8_t *p = put_header(out, ESP32_WLAN_TYPE_MGT, ESP32_WLAN_MGT_AUTH, 0,
                            sta, bssid, bssid, seq);
    put_le16(p, algorithm);
    put_le16(p + 2, 2);                            /* transaction sequence */
    put_le16(p + 4, status);
    return ESP32_WLAN_HDR_LEN + 6;
}

size_t esp32_wlan_build_assoc_response(uint8_t *out, size_t cap, const uint8_t sta[6],
                                       const uint8_t bssid[6], uint16_t seq,
                                       bool reassoc, uint16_t aid)
{
    if (cap < ESP32_WLAN_HDR_LEN + 6 + 2 + 8 + 2 + 4) {
        return 0;
    }
    uint8_t *p = put_header(out, ESP32_WLAN_TYPE_MGT,
                            reassoc ? ESP32_WLAN_MGT_REASSOC_RESP : ESP32_WLAN_MGT_ASSOC_RESP,
                            0, sta, bssid, bssid, seq);
    put_le16(p, 0x0001);                           /* capability: ESS */
    put_le16(p + 2, 0);                            /* status: success */
    put_le16(p + 4, (uint16_t)(aid | 0xc000));
    p = put_rates(p + 6);
    return (size_t)(p - out);
}

size_t esp32_wlan_build_deauth(uint8_t *out, size_t cap, const uint8_t sta[6],
                               const uint8_t bssid[6], uint16_t seq, uint16_t reason)
{
    if (cap < ESP32_WLAN_HDR_LEN + 2) {
        return 0;
    }
    uint8_t *p = put_header(out, ESP32_WLAN_TYPE_MGT, ESP32_WLAN_MGT_DEAUTH, 0,
                            sta, bssid, bssid, seq);
    put_le16(p, reason);
    return ESP32_WLAN_HDR_LEN + 2;
}

void esp32_wifi_link_derive_bssid(const uint8_t sta[6], uint8_t bssid[6])
{
    /* Locally administered unicast, stable per station and distinct from it. */
    memcpy(bssid, sta, 6);
    bssid[0] = (uint8_t)((sta[0] | 0x02) & ~0x01);
    bssid[5] ^= 0x5a;
    if (!memcmp(bssid, sta, 6)) {
        bssid[4] ^= 0xa5;
    }
}

void esp32_wifi_link_reset(Esp32WifiLink *link, const uint8_t sta[6])
{
    const uint32_t ups = link->link_ups, downs = link->link_downs;
    const bool was_up = link->state == ESP32_WIFI_LINK_ASSOCIATED;
    memset(link, 0, sizeof(*link));
    link->link_ups = ups;
    link->link_downs = downs + (was_up ? 1 : 0);
    memcpy(link->sta, sta, 6);
    esp32_wifi_link_derive_bssid(sta, link->bssid);
}

bool esp32_wifi_link_is_up(const Esp32WifiLink *link)
{
    return link->state == ESP32_WIFI_LINK_ASSOCIATED;
}

static void link_down(Esp32WifiLink *link, Esp32WifiLinkState next)
{
    if (link->state == ESP32_WIFI_LINK_ASSOCIATED) {
        link->link_downs++;
    }
    link->state = next;
    link->aid = 0;
}

static bool addressed_to_link(const Esp32WifiLink *link, const uint8_t *addr)
{
    return !memcmp(addr, link->bssid, 6);
}

Esp32WifiLinkAction esp32_wifi_link_on_tx(Esp32WifiLink *link, const uint8_t *frame,
                                          size_t len)
{
    Esp32WlanHeader h;
    if (esp32_wlan_parse_header(frame, len, &h) != ESP32_WLAN_OK) {
        return ESP32_WIFI_ACT_NONE;
    }
    const uint8_t *body = frame + h.hdr_len;
    const size_t body_len = len - h.hdr_len;

    if (h.type == ESP32_WLAN_TYPE_MGT) {
        switch (h.subtype) {
        case ESP32_WLAN_MGT_PROBE_REQ: {
            const uint8_t *ssid;
            uint8_t ssid_len;
            if (memcmp(h.addr1, broadcast_mac, 6) && !addressed_to_link(link, h.addr1)) {
                return ESP32_WIFI_ACT_NONE;
            }
            /* Only a directed probe (the connection attempt) is answered; a wildcard
             * scan sees an empty list, so no network is ever announced. */
            if (esp32_wlan_find_ie(body, body_len, ESP32_WLAN_IE_SSID, &ssid, &ssid_len) !=
                    ESP32_WLAN_OK || ssid_len == 0 || ssid_len > 32) {
                return ESP32_WIFI_ACT_NONE;
            }
            return ESP32_WIFI_ACT_PROBE_RESPONSE;
        }
        case ESP32_WLAN_MGT_AUTH:
            if (!addressed_to_link(link, h.addr1) || body_len < 6) {
                return ESP32_WIFI_ACT_NONE;
            }
            if (le16(body + 2) != 1) {
                return ESP32_WIFI_ACT_NONE;        /* only the first transaction is a request */
            }
            if (le16(body) != 0) {
                return ESP32_WIFI_ACT_AUTH_REJECT; /* SAE/shared key: not part of the link */
            }
            memcpy(link->sta, h.addr2, 6);
            link_down(link, ESP32_WIFI_LINK_AUTHENTICATED);
            return ESP32_WIFI_ACT_AUTH_RESPONSE;
        case ESP32_WLAN_MGT_ASSOC_REQ:
        case ESP32_WLAN_MGT_REASSOC_REQ:
            if (!addressed_to_link(link, h.addr1)) {
                return ESP32_WIFI_ACT_NONE;
            }
            if (link->state == ESP32_WIFI_LINK_IDLE || memcmp(h.addr2, link->sta, 6)) {
                return ESP32_WIFI_ACT_DEAUTH;      /* class 2 frame before authentication */
            }
            if (link->state != ESP32_WIFI_LINK_ASSOCIATED) {
                link->link_ups++;
            }
            link->state = ESP32_WIFI_LINK_ASSOCIATED;
            link->aid = 1;
            link->reassoc = h.subtype == ESP32_WLAN_MGT_REASSOC_REQ;
            return ESP32_WIFI_ACT_ASSOC_RESPONSE;
        case ESP32_WLAN_MGT_DEAUTH:
            if (addressed_to_link(link, h.addr1)) {
                link_down(link, ESP32_WIFI_LINK_IDLE);
            }
            return ESP32_WIFI_ACT_NONE;
        case ESP32_WLAN_MGT_DISASSOC:
            if (addressed_to_link(link, h.addr1) && link->state == ESP32_WIFI_LINK_ASSOCIATED) {
                link_down(link, ESP32_WIFI_LINK_AUTHENTICATED);
            }
            return ESP32_WIFI_ACT_NONE;
        case ESP32_WLAN_MGT_BEACON:
            return ESP32_WIFI_ACT_SOFTAP_UNSUPPORTED;
        default:
            return ESP32_WIFI_ACT_NONE;
        }
    }

    if (h.type == ESP32_WLAN_TYPE_DATA) {
        if (!addressed_to_link(link, h.addr1) || !(h.flags & ESP32_WLAN_FLAG_TO_DS)) {
            return ESP32_WIFI_ACT_NONE;
        }
        if (link->state != ESP32_WIFI_LINK_ASSOCIATED || memcmp(h.addr2, link->sta, 6)) {
            return ESP32_WIFI_ACT_DEAUTH;          /* class 3 frame from a non-associated STA */
        }
        if (h.subtype & 0x4) {
            return ESP32_WIFI_ACT_NONE;            /* null function (power save keep-alive) */
        }
        return ESP32_WIFI_ACT_FORWARD_DATA;
    }
    return ESP32_WIFI_ACT_NONE;
}

/*
 * Position-independent interior anchor of esp_wifi_set_config() in libnet80211
 * from ESP-IDF 5.5 (also the framework-arduinoespressif32-libs build used by
 * arduino-esp32 3.3.x). It is the field-copy sequence storing the config struct
 * (s8i/s32i at struct offsets 8/4/9/10/12, memw, addi a10,a10,20): only
 * immediates and struct offsets, so it survives the linker's l32r relaxation
 * that rewrites the prologue's literal loads. Verified byte-identical between
 * the archive and a linked firmware image, and unique in the app image.
 */
static const uint8_t anchor_idf55_set_config[] = {
    0x22, 0x4a, 0x08, 0x0c, 0x02, 0x89, 0x1a, 0x0c, 0x08, 0x82, 0x4a, 0x09, 0x82,
    0x4a, 0x0a, 0xc0, 0x20, 0x00, 0x29, 0x3a, 0xa2, 0xca, 0x14, 0xc0, 0x20, 0x00,
};

const Esp32WifiDriverProfile esp32_wifi_driver_profiles[] = {
    {
        .name = "esp-idf 5.5 libnet80211 (arduino-esp32 3.3)",
        .anchor = anchor_idf55_set_config,
        .anchor_len = sizeof(anchor_idf55_set_config),
        .entry_search_back = 200,
        .config_size = 184,
        .off_password = 32,
        .password_len = 64,
        .off_bssid_set = 100,
        .off_authmode = 120,
        .off_pmf_required = 129,
    },
};
const size_t esp32_wifi_driver_profile_count = ARRAY_SIZE(esp32_wifi_driver_profiles);

int esp32_wifi_sta_config_ranges(const Esp32WifiDriverProfile *profile,
                                 Esp32WifiConfigRange ranges[ESP32_WIFI_CONFIG_RANGES])
{
    ranges[0] = (Esp32WifiConfigRange){ profile->off_password, profile->password_len };
    ranges[1] = (Esp32WifiConfigRange){ profile->off_bssid_set, 1 };
    ranges[2] = (Esp32WifiConfigRange){ profile->off_authmode, 4 };
    ranges[3] = (Esp32WifiConfigRange){ profile->off_pmf_required, 1 };
    return ESP32_WIFI_CONFIG_RANGES;
}

void esp32_wifi_neutralize_sta_config(const Esp32WifiDriverProfile *profile,
                                      uint8_t *config)
{
    Esp32WifiConfigRange ranges[ESP32_WIFI_CONFIG_RANGES];
    const int n = esp32_wifi_sta_config_ranges(profile, ranges);
    for (int i = 0; i < n; i++) {
        memset(config + ranges[i].offset, 0, ranges[i].len);
    }
}

static inline uint32_t le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

#define ESP_IMAGE_MAGIC        0xe9
#define ESP_IMAGE_HEADER_LEN   24
#define ESP_IMAGE_MAX_SEGMENTS 16
#define ESP32_IROM_LOW         0x400c2000u
#define ESP32_IROM_HIGH        0x40c00000u

static const uint8_t *find_bytes(const uint8_t *hay, size_t hay_len,
                                 const uint8_t *needle, size_t needle_len)
{
    if (needle_len == 0 || hay_len < needle_len) {
        return NULL;
    }
    const uint8_t *end = hay + hay_len - needle_len;
    for (const uint8_t *p = hay; p <= end; p++) {
        p = memchr(p, needle[0], (size_t)(end - p) + 1);
        if (!p) {
            return NULL;
        }
        if (!memcmp(p, needle, needle_len)) {
            return p;
        }
    }
    return NULL;
}

static int scan_app_image(const uint8_t *flash, size_t flash_len, uint32_t offset,
                          uint32_t limit, Esp32WifiHookSite *sites, int count,
                          int max_sites, bool *valid)
{
    *valid = false;
    if (offset >= flash_len || flash_len - offset < ESP_IMAGE_HEADER_LEN ||
        flash[offset] != ESP_IMAGE_MAGIC) {
        return count;
    }
    const uint32_t end = MIN((uint64_t)offset + limit, (uint64_t)flash_len);
    const unsigned segments = flash[offset + 1];
    if (segments == 0 || segments > ESP_IMAGE_MAX_SEGMENTS) {
        return count;
    }
    uint32_t pos = offset + ESP_IMAGE_HEADER_LEN;
    for (unsigned i = 0; i < segments; i++) {
        if (pos > end || end - pos < 8) {
            return count;
        }
        const uint32_t load = le32(flash + pos);
        const uint32_t size = le32(flash + pos + 4);
        pos += 8;
        if (size > end - pos) {
            return count;
        }
        *valid = true;
        if (load >= ESP32_IROM_LOW && load < ESP32_IROM_HIGH) {
            for (size_t p = 0; p < esp32_wifi_driver_profile_count; p++) {
                const Esp32WifiDriverProfile *profile = &esp32_wifi_driver_profiles[p];
                const uint8_t *seg = flash + pos;
                size_t search_from = 0;
                const uint8_t *hit;
                while (count < max_sites &&
                       (hit = find_bytes(seg + search_from, size - search_from,
                                         profile->anchor, profile->anchor_len)) != NULL) {
                    const size_t anchor_pos = (size_t)(hit - seg);
                    search_from = anchor_pos + 1;
                    /* Walk back to the function's `entry a1, imm` (opcode 0x36,
                     * s-field a1 -> second byte 0x41). */
                    const size_t back = anchor_pos < profile->entry_search_back
                        ? anchor_pos : profile->entry_search_back;
                    for (size_t d = 2; d <= back; d++) {
                        const uint8_t *e = hit - d;
                        if (e[0] == 0x36 && e[1] == 0x41) {
                            sites[count++] = (Esp32WifiHookSite){
                                .vaddr = load + (uint32_t)(e - seg),
                                .profile = profile,
                                .flash_offset = offset,
                            };
                            break;
                        }
                    }
                }
            }
        }
        pos += size;
    }
    return count;
}

int esp32_wifi_scan_flash_image(const uint8_t *flash, size_t flash_len,
                                Esp32WifiHookSite *sites, int max_sites,
                                int *apps_found)
{
    int count = 0, apps = 0;
    bool valid;
    bool have_table = false;
    const uint32_t table = 0x8000;

    for (uint32_t off = table; flash_len >= 32 && off <= flash_len - 32 &&
                               off < table + 0xc00; off += 32) {
        const uint8_t *e = flash + off;
        if (e[0] != 0xaa || e[1] != 0x50) {
            break;                                 /* 0xffff end marker or MD5 entry */
        }
        have_table = true;
        if (e[2] != 0x00) {
            continue;                              /* not an app partition */
        }
        count = scan_app_image(flash, flash_len, le32(e + 4), le32(e + 8), sites, count,
                               max_sites, &valid);
        apps += valid ? 1 : 0;
    }
    if (!have_table) {
        count = scan_app_image(flash, flash_len, 0x10000, UINT32_MAX, sites, count,
                               max_sites, &valid);
        apps += valid ? 1 : 0;
    }
    if (apps_found) {
        *apps_found = apps;
    }
    return count;
}
