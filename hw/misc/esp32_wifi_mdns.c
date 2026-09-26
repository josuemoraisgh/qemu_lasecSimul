/*
 * Same-PC mDNS support for the transparent Wi-Fi "isolated" (SLIRP) mode.
 *
 * In isolated mode the guest is behind QEMU's SLIRP NAT, and the host's own mDNS
 * resolver and QEMU's SLIRP multicast are on different network paths -- so a host
 * responder built into QEMU cannot feed the OS resolver (verified). Instead this
 * only *learns* the hostname the ESP32 advertises (from the mDNS records it
 * transmits, which pass through the transparent Wi-Fi device) and hands it to the
 * LasecSimul extension over a loopback UDP datagram. The extension runs the
 * actual mDNS responder on the host's real interface, answering
 * `<hostname>.local -> 127.0.0.1` where the guest's forwarded services live.
 *
 * Enabled only when LASECSIMUL_ESP32_MDNS_REFLECT=1 (the Core sets it for
 * isolated); LASECSIMUL_ESP32_MDNS_FEED_PORT gives the extension's loopback port.
 * lab-router/lab-bridge use the real TAP and never enable this.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/sockets.h"
#include "hw/misc/esp32_wifi.h"
#ifdef _WIN32
#include <ws2tcpip.h>
#else
#include <netinet/in.h>
#include <arpa/inet.h>
#endif

#define MDNS_PORT 5353

static int g_feed_fd = -1;
static uint16_t g_feed_port;
static char g_mdns_host[64];   /* last advertised <host>.local, lowercased */

/* Read a DNS name at msg[off] into `out` (dotted, lowercase, no trailing dot).
 * Follows compression pointers while reading but returns the offset immediately
 * after the name field at this level. Returns -1 on malformed input. */
static int dns_read_name(const uint8_t *msg, size_t len, size_t off,
                         char *out, size_t outsz)
{
    size_t o = off, w = 0, guard = 0;
    bool jumped = false;
    int after = -1;

    while (o < len) {
        uint8_t l = msg[o];
        if ((l & 0xc0) == 0xc0) {
            if (o + 1 >= len) {
                return -1;
            }
            if (!jumped) {
                after = (int)(o + 2);
            }
            o = (size_t)(((l & 0x3f) << 8) | msg[o + 1]);
            jumped = true;
            if (++guard > 128) {
                return -1;
            }
            continue;
        }
        if (l == 0) {
            if (!jumped) {
                after = (int)(o + 1);
            }
            break;
        }
        o++;
        if (o + l > len) {
            return -1;
        }
        if (w && w + 1 < outsz) {
            out[w++] = '.';
        }
        for (unsigned i = 0; i < l && w + 1 < outsz; i++) {
            char c = (char)msg[o + i];
            if (c >= 'A' && c <= 'Z') {
                c = (char)(c + 32);
            }
            out[w++] = c;
        }
        o += l;
        if (++guard > 128) {
            return -1;
        }
    }
    out[w < outsz ? w : outsz - 1] = 0;
    return after;
}

static bool is_dot_local(const char *name)
{
    size_t l = strlen(name);
    return l > 6 && strcmp(name + l - 6, ".local") == 0;
}

static void feed_extension(const char *host)
{
    if (g_feed_fd < 0) {
        return;
    }
    struct sockaddr_in to;
    memset(&to, 0, sizeof(to));
    to.sin_family = AF_INET;
    to.sin_port = htons(g_feed_port);
    to.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    sendto(g_feed_fd, host, (int)strlen(host), 0,
           (struct sockaddr *)&to, sizeof(to));
}

/* Scan an mDNS message the guest transmitted for an A record; its owner name is
 * the advertised hostname. */
static void mdns_learn(const uint8_t *msg, size_t len)
{
    if (len < 12) {
        return;
    }
    unsigned qd = (msg[4] << 8) | msg[5];
    unsigned an = (msg[6] << 8) | msg[7];
    unsigned ns = (msg[8] << 8) | msg[9];
    unsigned ar = (msg[10] << 8) | msg[11];
    size_t off = 12;
    char name[64];

    for (unsigned i = 0; i < qd; i++) {
        int n = dns_read_name(msg, len, off, name, sizeof(name));
        if (n < 0) {
            return;
        }
        off = (size_t)n + 4;
        if (off > len) {
            return;
        }
    }
    for (unsigned i = 0; i < an + ns + ar; i++) {
        int n = dns_read_name(msg, len, off, name, sizeof(name));
        if (n < 0) {
            return;
        }
        off = (size_t)n;
        if (off + 10 > len) {
            return;
        }
        unsigned type = (msg[off] << 8) | msg[off + 1];
        unsigned rdlen = (msg[off + 8] << 8) | msg[off + 9];
        off += 10;
        if (off + rdlen > len) {
            return;
        }
        if (type == 1 && rdlen == 4 && is_dot_local(name) &&
            strlen(name) < sizeof(g_mdns_host)) {
            if (strcmp(g_mdns_host, name) != 0) {
                memcpy(g_mdns_host, name, strlen(name) + 1);
                qemu_log("esp32_wifi: mDNS host %s -> extension (127.0.0.1)\n",
                         g_mdns_host);
            }
            feed_extension(g_mdns_host);     /* resend so a late responder catches it */
            return;
        }
        off += rdlen;
    }
}

void esp32_wifi_mdns_observe(uint8_t *eth, size_t len)
{
    if (g_feed_fd < 0 || len < 14 + 20 + 8) {
        return;
    }
    if (eth[12] != 0x08 || eth[13] != 0x00) {    /* IPv4 */
        return;
    }
    const uint8_t *ip = eth + 14;
    if ((ip[0] >> 4) != 4) {
        return;
    }
    unsigned ihl = (unsigned)(ip[0] & 0xf) * 4;
    if (ip[9] != 17 || 14 + ihl + 8 > len) {     /* UDP */
        return;
    }
    const uint8_t *udp = ip + ihl;
    if (((udp[2] << 8) | udp[3]) != MDNS_PORT) {
        return;
    }
    const uint8_t *dns = udp + 8;
    mdns_learn(dns, len - (size_t)(dns - eth));
}

void esp32_wifi_mdns_start(void)
{
    if (g_feed_fd >= 0) {
        return;
    }
    const char *enabled = getenv("LASECSIMUL_ESP32_MDNS_REFLECT");
    const char *port = getenv("LASECSIMUL_ESP32_MDNS_FEED_PORT");
    if (!enabled || strcmp(enabled, "1") != 0 || !port) {
        return;
    }
    int p = atoi(port);
    if (p <= 0 || p > 65535) {
        return;
    }
    int fd = qemu_socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        return;
    }
    g_feed_port = (uint16_t)p;
    g_feed_fd = fd;
    g_mdns_host[0] = 0;
    qemu_log("esp32_wifi: mDNS host learning active (feeds extension on 127.0.0.1:%d)\n", p);
}
