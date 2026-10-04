/*
 * frame_pair_test.c — host test for src/frame_pair.c: is a CRC-bad capture
 * really two good frames glued together?
 *
 * NOT covered: the decision in modbus.c (ignore the pair only when neither half
 * is addressed to us) needs AVR headers — code review, plus the house bus,
 * where the gateways' crc_error must stop climbing with every relay poll.
 *
 * Build & run: `make test`.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "frame_pair.h"

/* CRC-16/MODBUS, bitwise — the host stand-in for modbus.c's table version. */
static uint16_t crc16(const uint8_t *d, uint8_t n) {
    uint16_t c = 0xFFFF;
    while (n--) {
        c ^= *d++;
        for (int b = 0; b < 8; b++) c = (c & 1u) ? (uint16_t)((c >> 1) ^ 0xA001u) : (uint16_t)(c >> 1);
    }
    return c;
}

static int failures;
#define CHECK(cond, what) do { \
    if (!(cond)) { printf("FAIL: %s (line %d)\n", what, __LINE__); failures++; } \
} while (0)

static uint8_t seal(uint8_t *b, uint8_t n) {
    uint16_t c = crc16(b, n);
    b[n] = (uint8_t)c; b[n + 1] = (uint8_t)(c >> 8);
    return (uint8_t)(n + 2u);
}

int main(void) {
    uint8_t f[64], k = 0;

    /* The house capture, 2026-10-01 (both gateways, last_fail, 16 of 17 bytes):
     *   01 01 00 00 00 20 3d d2 | 01 01 04 00 00 00 00 fb [..]
     * = the controller's FC01 "read 32 coils from 0" to the relay board, glued
     * to the board's 4-byte reply. Rebuild the reply's CRC and check it really
     * starts with the captured 0xfb — i.e. the capture is what we think it is. */
    {
        static const uint8_t req[] = { 0x01, 0x01, 0x00, 0x00, 0x00, 0x20, 0x3d, 0xd2 };
        CHECK(crc16(req, sizeof req) == 0, "captured relay request is CRC-good");
        uint8_t rep[16] = { 0x01, 0x01, 0x04, 0x00, 0x00, 0x00, 0x00 };
        uint8_t rn = seal(rep, 7);
        CHECK(rep[7] == 0xfb, "rebuilt relay reply CRC matches the captured 0xfb");
        memcpy(f, req, sizeof req); memcpy(f + sizeof req, rep, rn);
        uint8_t n = (uint8_t)(sizeof req + rn);
        CHECK(crc16(f, n) != 0, "the glued capture fails its CRC as a whole");
        CHECK(frame_is_pair(f, n, crc16, &k), "the glued capture splits into two good frames");
        CHECK(k == sizeof req, "...at the request/reply boundary");
    }

    /* Two built frames, various lengths. */
    {
        uint8_t n = 0;
        f[n++] = 3; f[n++] = 0x41; f[n++] = 0x00; n = seal(f, n);              /* 5 B poll  */
        uint8_t m = 0, g[16];
        g[m++] = 3; g[m++] = 0x41; g[m++] = 0x21; g[m++] = 0x00; m = seal(g, m); /* 6 B reply */
        memcpy(f + n, g, m);
        CHECK(frame_is_pair(f, (uint8_t)(n + m), crc16, &k) && k == n, "FC41 poll + reply splits");
    }

    /* Corruption is still corruption. */
    {
        static const uint8_t req[] = { 0x01, 0x01, 0x00, 0x00, 0x00, 0x20, 0x3d, 0xd2 };
        uint8_t rep[16] = { 0x01, 0x01, 0x04, 0x12, 0x34, 0x56, 0x78 };
        uint8_t rn = seal(rep, 7);
        memcpy(f, req, sizeof req); memcpy(f + sizeof req, rep, rn);
        uint8_t n = (uint8_t)(sizeof req + rn);
        f[10] ^= 0x04;                                   /* one bit flipped in the reply */
        CHECK(!frame_is_pair(f, n, crc16, &k), "a flipped bit in either half is not a pair");
        f[10] ^= 0x04; f[2] ^= 0x80;                     /* ...or in the request */
        CHECK(!frame_is_pair(f, n, crc16, &k), "a flipped bit in the first half is not a pair");
    }

    /* One good frame plus trailing junk, or junk plus a good frame: not a pair. */
    {
        uint8_t n = 0;
        f[n++] = 2; f[n++] = 0x04; f[n++] = 0x03; f[n++] = 0x10; f[n++] = 0x00; f[n++] = 0x04;
        n = seal(f, n);
        f[n++] = 0x00; f[n++] = 0xFF;                    /* 2 junk bytes: too short to be a frame */
        CHECK(!frame_is_pair(f, n, crc16, &k), "good frame + 2 junk bytes is not a pair");
        f[0] = 0x00;
        uint8_t m = 0, g[16];
        g[m++] = 2; g[m++] = 0x41; g[m++] = 0x00; m = seal(g, m);
        uint8_t h[32] = { 0xA5, 0x5A, 0x00 };            /* 3 junk bytes, then a good frame */
        memcpy(h + 3, g, m);
        CHECK(!frame_is_pair(h, (uint8_t)(3 + m), crc16, &k), "junk + good frame is not a pair");
    }

    /* Too short to hold two frames. */
    {
        uint8_t n = 0;
        f[n++] = 1; f[n++] = 0x01; n = seal(f, n);       /* 4 B */
        CHECK(!frame_is_pair(f, n, crc16, &k), "a single 4-byte frame is not a pair");
        CHECK(!frame_is_pair(f, 0, crc16, &k), "empty is not a pair");
    }

    if (failures) { printf("%d FAILURE(S)\n", failures); return EXIT_FAILURE; }
    printf("PASS: all frame_pair checks passed\n");
    return EXIT_SUCCESS;
}
