/*
 * ESP32 Wi-Fi transparent uplink: pure frame/link logic.
 *
 * Everything here is free of QEMU device state so it can be unit tested
 * (tests/unit/test-esp32-wifi-link.c). The esp32_wifi device feeds it the raw
 * 802.11 frames the guest driver transmits and turns the returned actions into
 * DMA writes towards the guest or Ethernet frames towards the NIC backend.
 *
 * The emulated link is not a radio network: the station is presented with an
 * already-available open link whose uplink is the QEMU network backend. There
 * is no beacon-driven AP, no scan list, no WPA/EAPOL and no configurable SSID.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */
#ifndef HW_MISC_ESP32_WIFI_LINK_H
#define HW_MISC_ESP32_WIFI_LINK_H

#define ESP32_WLAN_HDR_LEN        24
#define ESP32_WLAN_FCS_LEN        4
#define ESP32_WLAN_LLC_LEN        8
#define ESP32_WLAN_MAX_FRAME      2346
#define ESP32_ETH_HDR_LEN         14
#define ESP32_ETH_MAX_FRAME       1514

#define ESP32_WLAN_TYPE_MGT       0
#define ESP32_WLAN_TYPE_CTL       1
#define ESP32_WLAN_TYPE_DATA      2

#define ESP32_WLAN_MGT_ASSOC_REQ    0x0
#define ESP32_WLAN_MGT_ASSOC_RESP   0x1
#define ESP32_WLAN_MGT_REASSOC_REQ  0x2
#define ESP32_WLAN_MGT_REASSOC_RESP 0x3
#define ESP32_WLAN_MGT_PROBE_REQ    0x4
#define ESP32_WLAN_MGT_PROBE_RESP   0x5
#define ESP32_WLAN_MGT_BEACON       0x8
#define ESP32_WLAN_MGT_DISASSOC     0xa
#define ESP32_WLAN_MGT_AUTH         0xb
#define ESP32_WLAN_MGT_DEAUTH       0xc
#define ESP32_WLAN_MGT_ACTION       0xd

#define ESP32_WLAN_FLAG_TO_DS     0x01
#define ESP32_WLAN_FLAG_FROM_DS   0x02
#define ESP32_WLAN_FLAG_PROTECTED 0x40
#define ESP32_WLAN_FLAG_ORDER     0x80

#define ESP32_WLAN_IE_SSID        0
#define ESP32_WLAN_IE_RATES       1
#define ESP32_WLAN_IE_DS_PARAMS   3
#define ESP32_WLAN_IE_EXT_RATES   50

/* Result codes of the parsing/conversion helpers (0 is success). */
enum {
    ESP32_WLAN_OK = 0,
    ESP32_WLAN_ERR_TRUNCATED = -1,   /* shorter than its own headers say */
    ESP32_WLAN_ERR_TOO_BIG = -2,     /* does not fit the destination; never truncated */
    ESP32_WLAN_ERR_UNSUPPORTED = -3, /* valid 802.11/802.3, but outside the modelled link */
    ESP32_WLAN_ERR_NO_PAYLOAD = -4,  /* null-data frame: nothing to forward */
    ESP32_WLAN_ERR_MALFORMED = -5,   /* inconsistent lengths/fields */
};

typedef struct Esp32WlanHeader {
    uint8_t type;
    uint8_t subtype;
    uint8_t flags;
    const uint8_t *addr1;
    const uint8_t *addr2;
    const uint8_t *addr3;
    uint16_t seq_ctl;
    size_t hdr_len;          /* includes QoS/HT control/addr4 when present */
} Esp32WlanHeader;

/* Parses the MAC header of `frame` (FCS already removed). */
int esp32_wlan_parse_header(const uint8_t *frame, size_t len, Esp32WlanHeader *h);

/*
 * Finds information element `id` inside a tagged-parameter area. Returns
 * ESP32_WLAN_OK and points `value`/`value_len` at it, ESP32_WLAN_ERR_UNSUPPORTED
 * when absent, or ESP32_WLAN_ERR_MALFORMED when an element overruns `len`.
 */
int esp32_wlan_find_ie(const uint8_t *ies, size_t len, uint8_t id,
                       const uint8_t **value, uint8_t *value_len);

/* ToDS data frame from the station -> Ethernet II frame (DA=addr3, SA=addr2). */
int esp32_wlan_data_to_ethernet(const uint8_t *frame, size_t len,
                                uint8_t *eth, size_t cap, size_t *eth_len);

/* Ethernet II frame -> FromDS data frame (addr1=DA, addr2=BSSID, addr3=SA), no FCS. */
int esp32_wlan_ethernet_to_data(const uint8_t *eth, size_t eth_len,
                                const uint8_t bssid[6], uint16_t seq,
                                uint8_t *frame, size_t cap, size_t *frame_len);

/* Management frame builders. Return the frame length without FCS, or 0 if `cap` is too small. */
size_t esp32_wlan_build_probe_response(uint8_t *out, size_t cap, const uint8_t sta[6],
                                       const uint8_t bssid[6], uint16_t seq,
                                       const uint8_t *ssid, uint8_t ssid_len,
                                       uint8_t channel, uint64_t tsf_us);
size_t esp32_wlan_build_beacon(uint8_t *out, size_t cap, const uint8_t bssid[6],
                               uint16_t seq, uint8_t channel, uint64_t tsf_us);
size_t esp32_wlan_build_auth_response(uint8_t *out, size_t cap, const uint8_t sta[6],
                                      const uint8_t bssid[6], uint16_t seq,
                                      uint16_t algorithm, uint16_t status);
size_t esp32_wlan_build_assoc_response(uint8_t *out, size_t cap, const uint8_t sta[6],
                                       const uint8_t bssid[6], uint16_t seq,
                                       bool reassoc, uint16_t aid);
size_t esp32_wlan_build_deauth(uint8_t *out, size_t cap, const uint8_t sta[6],
                               const uint8_t bssid[6], uint16_t seq, uint16_t reason);

/* Beacon/probe-response interval advertised by the synthetic link, in TU. */
#define ESP32_WIFI_LINK_BEACON_TU 100

typedef enum Esp32WifiLinkState {
    ESP32_WIFI_LINK_IDLE = 0,
    ESP32_WIFI_LINK_AUTHENTICATED,
    ESP32_WIFI_LINK_ASSOCIATED,
} Esp32WifiLinkState;

typedef enum Esp32WifiLinkAction {
    ESP32_WIFI_ACT_NONE = 0,
    ESP32_WIFI_ACT_PROBE_RESPONSE,   /* reply echoing the probe's SSID element */
    ESP32_WIFI_ACT_AUTH_RESPONSE,
    ESP32_WIFI_ACT_AUTH_REJECT,      /* non-open authentication algorithm */
    ESP32_WIFI_ACT_ASSOC_RESPONSE,
    ESP32_WIFI_ACT_FORWARD_DATA,     /* convert and send to the NIC backend */
    ESP32_WIFI_ACT_DEAUTH,           /* class-3 frame from a non-associated station */
    ESP32_WIFI_ACT_SOFTAP_UNSUPPORTED,
} Esp32WifiLinkAction;

typedef struct Esp32WifiLink {
    Esp32WifiLinkState state;
    uint8_t sta[6];
    uint8_t bssid[6];
    uint16_t aid;
    bool reassoc;
    uint32_t link_ups;
    uint32_t link_downs;
} Esp32WifiLink;

/* Derives the stable internal BSSID from the station MAC (locally administered, never equal). */
void esp32_wifi_link_derive_bssid(const uint8_t sta[6], uint8_t bssid[6]);
void esp32_wifi_link_reset(Esp32WifiLink *link, const uint8_t sta[6]);
/* Consumes one frame transmitted by the station (FCS removed). */
Esp32WifiLinkAction esp32_wifi_link_on_tx(Esp32WifiLink *link, const uint8_t *frame,
                                          size_t len);
bool esp32_wifi_link_is_up(const Esp32WifiLink *link);

/*
 * Guest driver fingerprints.
 *
 * With a non-empty password the Arduino core configures the station with
 * threshold.authmode = WPA2_PSK, so the closed driver would only join an RSN
 * network and then require a 4-way handshake keyed by the passphrase. The
 * transparent link neither knows nor wants that secret. For homologated driver
 * builds QEMU therefore intercepts esp_wifi_set_config(WIFI_IF_STA, conf) and,
 * before the driver copies `conf`, overwrites (never reads) the security fields
 * so the station joins the synthetic open link. The SSID bytes are left alone
 * and only echoed opaquely in the probe response.
 *
 * A profile is identified by the exact, position-independent body of
 * esp_wifi_set_config (its literal pool lives in the same section, so only the
 * pool contents vary between firmware builds). The body also embeds
 * sizeof(wifi_config_t), guarding the offsets below.
 */
/*
 * A profile is anchored on a position-independent interior byte run of
 * esp_wifi_set_config (the field-copy loop that stores the config struct: it
 * carries only immediates and struct offsets, so the linker's l32r relaxation
 * does not change it, unlike the function prologue). From an anchor match the
 * scanner walks back up to entry_search_back bytes to the function's `entry`
 * opcode and reports that as the site. config_size (== sizeof(wifi_config_t))
 * guards the offsets against a driver revision that moved the fields.
 */
typedef struct Esp32WifiDriverProfile {
    const char *name;
    const uint8_t *anchor;
    size_t anchor_len;
    uint32_t entry_search_back;
    uint32_t config_size;
    uint32_t off_password;
    uint32_t password_len;
    uint32_t off_bssid_set;
    uint32_t off_authmode;
    uint32_t off_pmf_required;
} Esp32WifiDriverProfile;

extern const Esp32WifiDriverProfile esp32_wifi_driver_profiles[];
extern const size_t esp32_wifi_driver_profile_count;

typedef struct Esp32WifiHookSite {
    uint32_t vaddr;                         /* guest vaddr of the `entry` opcode */
    const Esp32WifiDriverProfile *profile;
    uint32_t flash_offset;                  /* app image offset the site was found in */
} Esp32WifiHookSite;

/*
 * Scans a raw SPI flash image (partition table at 0x8000, or a bare app image at
 * 0x10000) for homologated driver builds. Returns the number of sites written
 * (at most `max_sites`). `apps_found` (optional) receives the number of valid app
 * images seen, so callers can tell "no Wi-Fi driver" from "unknown driver".
 */
int esp32_wifi_scan_flash_image(const uint8_t *flash, size_t flash_len,
                                Esp32WifiHookSite *sites, int max_sites,
                                int *apps_found);

/*
 * Byte ranges of wifi_config_t that the hook zeroes: password, bssid_set,
 * threshold.authmode (WIFI_AUTH_OPEN == 0) and pmf_cfg.required. Zeroing is a
 * blind write, so the password is never read. Returns the number of ranges.
 */
typedef struct Esp32WifiConfigRange {
    uint32_t offset;
    uint32_t len;
} Esp32WifiConfigRange;

#define ESP32_WIFI_CONFIG_RANGES 4
int esp32_wifi_sta_config_ranges(const Esp32WifiDriverProfile *profile,
                                 Esp32WifiConfigRange ranges[ESP32_WIFI_CONFIG_RANGES]);
/* Applies the same ranges to a host buffer (used by tests). */
void esp32_wifi_neutralize_sta_config(const Esp32WifiDriverProfile *profile,
                                      uint8_t *config);

#endif
