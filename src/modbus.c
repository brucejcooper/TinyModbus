#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <util/atomic.h>

#include "modbus.h"
#include "registers.h"
#include "frame_pair.h"
#include "errors.h"
#include "sysinfo.h"
#include "cq.h"
#include "fc41.h"
#include "dali.h"
#include "dali_sync.h"

// ---- Modbus constants -----------------------------------------------

#define USART0_BAUD_RATE(B) ((float)(F_CPU * 64 / (16 * (float)(B))) + 0.5f)

// Precomputed USART BAUD register values for each supported baud rate.
// Indexed by userrow_get_baud_index(): 0=9600, 1=19200, 2=38400.
static const uint16_t s_baud_reg[] = {
    (uint16_t)USART0_BAUD_RATE(9600),
    (uint16_t)USART0_BAUD_RATE(19200),
    (uint16_t)USART0_BAUD_RATE(38400),
};

// Apply the factory-measured oscillator error to the nominal BAUD divisor.
//
// s_baud_reg[] is computed assuming an exact F_CPU, but the internal 20 MHz RC
// oscillator has a per-unit frequency error.  The factory measured it and stored
// it — signed, in units of 1/1024 — in the signature row at two supply points,
// OSC20ERR3V and OSC20ERR5V.  The correction follows the RAIL the oscillator
// runs on, NOT the clock divider or F_CPU: the ÷2 prescaler that turns the
// 20 MHz base into the legacy 10 MHz core clock is exact and error-free, so the
// same OSC20 error applies whether the core sees 10 or 20 MHz — what changes it
// is V_DD.  On the 5 V board rev (BOARD_5V) we pick OSC20ERR5V; on the legacy
// 3.3 V board, OSC20ERR3V.  Scaling the nominal divisor by (1024 + err)/1024
// makes the USART sample at the true bit rate regardless of this die's
// individual error — the fleet-safe equivalent of a hand-tuned per-unit trim
// (this is what e.g. megaTinyCore's UART setup does, and skipping it is what
// leaves a marginal link marginal).
//
// Sign convention: a positive error means the oscillator runs fast, so the
// divisor scales up to compensate.
//
// VERIFIED ON THE BENCH 2026-08-14 (Saleae, 4 MS/s, both directions): the
// gateway's transmitted bit width lands at 52.00 us against a 52.08 us nominal
// at 19200 (-0.16%) and 26.00 us against 26.04 us at 38400. The sign is
// CORRECT — do not negate `sigerr`.
static uint16_t baud_reg_for(uint8_t bidx) {
#if BOARD_5V
    int8_t   sigerr = (int8_t)SIGROW.OSC20ERR5V;
#else
    int8_t   sigerr = (int8_t)SIGROW.OSC20ERR3V;
#endif
    int16_t  scale  = (int16_t)1024 + sigerr;     // 896..1151, always > 0
    uint32_t v = ((uint32_t)s_baud_reg[bidx] * (uint32_t)scale) / 1024u;
    return (uint16_t)v;
}

// Frame timing, in TCA0 ticks (~417 µs each), per baud index {9600, 19200, 38400}.
// Characters are 11 bits in the spec's model (MODBUS over Serial Line V1.02
// §2.5.1.1); above 19200 the spec fixes t1.5 = 750 µs and t3.5 = 1.750 ms.
//
//              t1.5 (max gap INSIDE a frame)   t3.5 (min gap BETWEEN frames)
//   9600       1.72 ms                         4.01 ms
//   19200      0.86 ms                         2.01 ms
//   38400      0.75 ms (fixed)                 1.75 ms (fixed)
//
// A byte arrival reloads a countdown and each tick decrements it, so a reload of
// N ticks declares end-of-frame after (N-1, N] ticks of real silence.
//
// DETECTION (s_silence_threshold) must sit strictly between the two: above t1.5
// so a frame is never split, and at or below t3.5 so a legal inter-frame gap is
// never missed. Missing one glues two frames together, and on a multi-drop bus
// that is exactly what happened: a gap as short as t3.5 separates the master's
// request from the other slave's reply, and the old thresholds (N = 10/6/6,
// windows 3.75-4.17 / 2.08-2.50 / 2.08-2.50 ms) all sat ABOVE t3.5. They were
// sized so the LOWER bound met t3.5 (2026-08-09), which is the wrong bound. The
// first two-gateway bench (2026-09-30) caught it: the idle gateway merged
// request+reply pairs of its neighbour and logged them as CRC errors.
//   9600   N = 8 -> 2.92-3.33 ms    (1.72 < w <= 4.01)
//   19200  N = 4 -> 1.25-1.67 ms    (0.86 < w <= 2.01)
//   38400  N = 4 -> 1.25-1.67 ms    (0.75 < w <= 1.75)
//
// REPLY (s_reply_guard) is the mirror image. A reply may only start once the
// line has been quiet for t3.5 (spec) AND for longer than any listener's
// detection window — else a listener that has not yet declared end-of-request
// swallows our reply into it. Replies are therefore held until G ticks have
// elapsed since the request's last byte, i.e. more than G-1 ticks of silence:
//   9600   G = 11 -> > 4.17 ms      (>= t3.5 4.01, > window max 3.33)
//   19200  G = 6  -> > 2.08 ms      (>= t3.5 2.01, > window max 1.67)
//   38400  G = 6  -> > 2.08 ms      (>= t3.5 1.75, > window max 1.67)
// A listener also only sees our first byte once it is fully received (~0.26 ms
// at 38400), which is extra margin on top of these. At 19200/38400 the reply
// goes out when it did before (the old detection point), so throughput is
// unchanged; the master's next request (controller: its t3.5 + 1.5 char
// deadtime, >= ~2.5 ms at 38400) clears the new detection window with margin.
//
// History: 38400 was N = 4 until 2026-08-09, raised to 6 while chasing 38400
// bad frames. That cause turned out to be the SFDEN erratum (see the block
// above modbus_init), not frame splitting — 0.00% bad over 6609 frames once
// fixed — and the scope showed both ends within 0.8% of nominal baud.
static const uint8_t  s_silence_threshold[] = { 8, 4, 4 };
static const uint8_t  s_reply_guard[]       = { 11, 6, 6 };
_Static_assert(sizeof s_silence_threshold == sizeof s_reply_guard, "one entry per baud");
// Per-baud reload value for the live silence_ticks countdown (set from
// s_silence_threshold[] on init / baud change; copied into silence_ticks each
// time a byte arrives).  Named _reload to disambiguate from the countdown.
static volatile uint8_t  s_silence_reload;
static volatile uint8_t  s_reply_guard_ticks;   // s_reply_guard[] for the live baud
// Ticks since the last received byte (saturating): reset by the RX ISR, advanced
// by modbus_tick. The reply guard is measured against it.
static volatile uint8_t  ticks_since_rx = 0xFF;
// A reply is built and waiting for the guard to expire (start_tx -> modbus_tick).
static volatile bool     tx_held        = false;

// Modbus §2.5.1.1 initial-silence gate.  After reset (power-on or WDT) the slave must
// observe 3.5 char of bus silence before treating bytes as the start of a new frame -
// otherwise we would latch onto whatever byte position the master happens to be sending
// if it was mid-frame at reset, fail CRC, and waste bus time on the recovery cycle.
//
// While clear, the RXC ISR drains incoming bytes (resetting the silence countdown each
// time a byte/SFD arrives) but does not accumulate them into buf.  When the silence
// countdown completes without an intervening byte, the flag flips to true and normal
// frame accumulation resumes.  One-shot for the lifetime of the chip.
static volatile bool initial_silence_observed = false;

#define FC_READ_COILS        0x01
#define FC_READ_HOLDING      0x03
#define FC_READ_INPUT        0x04
#define FC_WRITE_SINGLE_COIL 0x05
#define FC_WRITE_SINGLE_REG  0x06
#define FC_WRITE_COILS       0x0F
#define FC_WRITE_MULTI_REG   0x10

// The queue function code, its request lengths, its wire flag bits and its
// accept-status codes all live in fc41.h — one transcription of the spec's §1,
// shared with the host test that enumerates the validation table.
//
// In steady state FC 0x41 is one ~14-byte transaction per DALI frame, against
// the 1.73 transactions per command FC23 measured on this rig: FC23 made the
// master NAME the slot it wrote to, so it could only submit into one a PREVIOUS
// read had shown free, and that dependency cost a poll per frame. (The draft-1
// FC65/FC66 pair used 0x41/0x42 for a different, never deployed, shape — v4 is
// a clean break on the same code, and 0x42 is retired.)
//
// A_FULL is reported IN BAND rather than as a Modbus exception: an exception
// would discard the head report in the same breath, and the head is exactly what
// the master needs in order to learn that a slot has freed. It also leaves
// nothing for a transport auto-retry to latch onto, which is what makes the
// task-07 DEVICE_BUSY double-execution hazard unconstructible here.

typedef enum {
    MB_ERR_ILLEGAL_FUNCTION    = 0x01,
    MB_ERR_ILLEGAL_ADDRESS     = 0x02,
    MB_ERR_ILLEGAL_VALUE       = 0x03,
    MB_ERR_DEVICE_FAILURE      = 0x04,
    MB_ERR_DEVICE_BUSY         = 0x06,
    MB_ERR_TARGET_FAILED       = 0x0B,
    MB_ERR_DALI_PULSE_INVALID  = 0x30,
} mb_err_t;

// THE translation point: internal cause -> Modbus wire exception. Every other
// module speaks tm_err_t and knows nothing about these byte values; this is the
// only place the two vocabularies meet. Keep it total — a new tm_err_t must get
// a case here, and the default deliberately reports DEVICE_FAILURE rather than
// guessing, so an unmapped cause is visible as a fault instead of a plausible lie.
static mb_err_t tm_err_to_modbus(tm_err_t e) {
    switch (e) {
        case TM_OK:              return 0;
        case TM_ERR_NO_SUCH_REG: return MB_ERR_ILLEGAL_ADDRESS;
        case TM_ERR_BAD_VALUE:   return MB_ERR_ILLEGAL_VALUE;
        case TM_ERR_BAD_RANGE:   return MB_ERR_ILLEGAL_ADDRESS;
        case TM_ERR_BUSY:        return MB_ERR_DEVICE_BUSY;
        case TM_ERR_BAD_SLOT:    return MB_ERR_ILLEGAL_ADDRESS;
        case TM_ERR_HW_FAULT:    return MB_ERR_DEVICE_FAILURE;
    }
    return MB_ERR_DEVICE_FAILURE;
}

// ---- RS-485 hardware ------------------------------------------------

#define RS485_PORT  PORTA
#define RS485_TX_bm PIN1_bm
#define RS485_RX_bm PIN2_bm
#define RS485_DE_bm PIN4_bm

// ---- Buffers and state ----------------------------------------------

#define MAX_PKT_SZ  (16 * 2 + 8)   // max 16 registers per read request
#define MAX_REG_RD  16u

static uint8_t  slave_addr = 1;
static uint8_t  buf[MAX_PKT_SZ];
static volatile uint8_t *rx_ptr   = buf;
static volatile uint8_t  rx_sz    = 0;
static volatile uint16_t rx_crc   = 0xFFFF;
static volatile uint8_t  silence_ticks = 0;
static volatile bool     frame_ready   = false;
// Framing/parity errors seen in the frame being captured (saturating). Written
// by the RX ISR; read in modbus_process only while RX is disabled (frame_ready).
static volatile uint8_t  rx_ferr  = 0;
// tx_sz is written on the main path (append_crc) and decremented in the DRE
// ISR, so it must be volatile.  no_response is main-only today (set in
// modbus_process, read in start_tx), marked volatile for consistency with the
// other TX-path state and to stay correct if start_tx is ever reached from an ISR.
static volatile uint8_t  tx_sz    = 0;
static volatile bool     no_response   = false;

// ---- RX diagnostics (read via FC04 0x0300, 16-register read) --------------
// Counters to pin down why some Modbus requests go unanswered under load:
//   rx_overflow : USART RX FIFO overflowed — a byte was lost (RXC starvation).
//   frame_error : USART framing/parity errors in a frame we could not ignore.
//   crc_error   : a frame's worth of bytes arrived, then 3.5-char silence, but
//                 the CRC was non-zero.
//   frames_ok   : valid frames addressed to us that we processed (denominator).
// last_fail[]   : the bytes of the most recent CRC-failed frame, for forensics.
//
// A CRC-good frame addressed to another device is ignored entirely: on a
// multi-drop bus we hear every other slave's traffic, and it is not our link.
// A CRC-bad frame is always counted — its address byte cannot be trusted, so
// there is no telling whose it was. Framing errors are tallied per frame in
// rx_ferr by the RX ISR and charged at the frame boundary, once we know which
// of the two it was; before 2026-09-30 they were charged per byte in the ISR,
// so the idle gateway on a two-gateway bench logged framing errors on the
// other gateway's clean traffic. rx_overflow is still counted on every byte: a
// lost byte is our own ISR starving, whoever the frame was for.
//
// diag_q_discard is the queue protocol's own discard counter (FC04 0x0326, not
// this block): FC 0x41 frames addressed to us with a GOOD CRC that were dropped
// without a reply for a bad length or a length/flag inconsistency. It is
// separate from crc_error because the two say different things — a CRC error is
// the line, a queue discard is the master's frame builder — and it exists at all
// because the spec makes those frames indistinguishable from a lost frame to the
// master, which would otherwise leave them completely unobservable.
static volatile uint16_t diag_rx_overflow   = 0;
static volatile uint16_t diag_frame_error   = 0;
static volatile uint16_t diag_crc_error     = 0;
static volatile uint16_t diag_frames_ok     = 0;
static volatile uint16_t diag_q_discard     = 0;
static volatile uint8_t  diag_last_fail_len = 0;
static volatile uint8_t  diag_last_fail[16];
volatile bool            modbus_idle   = true;

// Set when a write to MBREG_SLAVE_ADDR or MBREG_BAUD_INDEX has just landed in
// USERROW.  The runtime change (USART0.BAUD, slave_addr local) is deferred
// until the response we are about to send has cleared the wire - see
// apply_pending_config().  USERROW already holds the new value, so we just
// re-read on apply.
#define APPLY_BAUD   0x01u
#define APPLY_SLAVE  0x02u
static volatile uint8_t  pending_apply_mask = 0;

// Latched by a MBREG_SYSTEM_RESET write; fired once the ACK is on the wire so
// the master sees its ACK before the gateway reboots.  RSTCTRL.SWRR is
// CCP-protected, so the write goes through _PROTECTED_WRITE; it never returns.
static volatile bool     pending_reset = false;

static inline void fire_pending_reset(void) {
    if (pending_reset)
        _PROTECTED_WRITE(RSTCTRL.SWRR, RSTCTRL_SWRE_bm);
}


// ---- CRC-16 table ---------------------------------------------------

static const uint16_t crc_table[256] PROGMEM = {
    0x0000,0xC0C1,0xC181,0x0140,0xC301,0x03C0,0x0280,0xC241,0xC601,0x06C0,0x0780,
    0xC741,0x0500,0xC5C1,0xC481,0x0440,0xCC01,0x0CC0,0x0D80,0xCD41,0x0F00,0xCFC1,
    0xCE81,0x0E40,0x0A00,0xCAC1,0xCB81,0x0B40,0xC901,0x09C0,0x0880,0xC841,0xD801,
    0x18C0,0x1980,0xD941,0x1B00,0xDBC1,0xDA81,0x1A40,0x1E00,0xDEC1,0xDF81,0x1F40,
    0xDD01,0x1DC0,0x1C80,0xDC41,0x1400,0xD4C1,0xD581,0x1540,0xD701,0x17C0,0x1680,
    0xD641,0xD201,0x12C0,0x1380,0xD341,0x1100,0xD1C1,0xD081,0x1040,0xF001,0x30C0,
    0x3180,0xF141,0x3300,0xF3C1,0xF281,0x3240,0x3600,0xF6C1,0xF781,0x3740,0xF501,
    0x35C0,0x3480,0xF441,0x3C00,0xFCC1,0xFD81,0x3D40,0xFF01,0x3FC0,0x3E80,0xFE41,
    0xFA01,0x3AC0,0x3B80,0xFB41,0x3900,0xF9C1,0xF881,0x3840,0x2800,0xE8C1,0xE981,
    0x2940,0xEB01,0x2BC0,0x2A80,0xEA41,0xEE01,0x2EC0,0x2F80,0xEF41,0x2D00,0xEDC1,
    0xEC81,0x2C40,0xE401,0x24C0,0x2580,0xE541,0x2700,0xE7C1,0xE681,0x2640,0x2200,
    0xE2C1,0xE381,0x2340,0xE101,0x21C0,0x2080,0xE041,0xA001,0x60C0,0x6180,0xA141,
    0x6300,0xA3C1,0xA281,0x6240,0x6600,0xA6C1,0xA781,0x6740,0xA501,0x65C0,0x6480,
    0xA441,0x6C00,0xACC1,0xAD81,0x6D40,0xAF01,0x6FC0,0x6E80,0xAE41,0xAA01,0x6AC0,
    0x6B80,0xAB41,0x6900,0xA9C1,0xA881,0x6840,0x7800,0xB8C1,0xB981,0x7940,0xBB01,
    0x7BC0,0x7A80,0xBA41,0xBE01,0x7EC0,0x7F80,0xBF41,0x7D00,0xBDC1,0xBC81,0x7C40,
    0xB401,0x74C0,0x7580,0xB541,0x7700,0xB7C1,0xB681,0x7640,0x7200,0xB2C1,0xB381,
    0x7340,0xB101,0x71C0,0x7080,0xB041,0x5000,0x90C1,0x9181,0x5140,0x9301,0x53C0,
    0x5280,0x9241,0x9601,0x56C0,0x5780,0x9741,0x5500,0x95C1,0x9481,0x5440,0x9C01,
    0x5CC0,0x5D80,0x9D41,0x5F00,0x9FC1,0x9E81,0x5E40,0x5A00,0x9AC1,0x9B81,0x5B40,
    0x9901,0x59C0,0x5880,0x9841,0x8801,0x48C0,0x4980,0x8941,0x4B00,0x8BC1,0x8A81,
    0x4A40,0x4E00,0x8EC1,0x8F81,0x4F40,0x8D01,0x4DC0,0x4C80,0x8C41,0x4400,0x84C1,
    0x8581,0x4540,0x8701,0x47C0,0x4680,0x8641,0x8201,0x42C0,0x4380,0x8341,0x4100,
    0x81C1,0x8081,0x4040
};

// ---- CRC helpers -------------------------------------------------------

// CRC-16/IBM over len bytes; initial value 0xFFFF.  Uses PROGMEM lookup table.
static uint16_t crc16(const uint8_t *data, uint8_t len) {
    uint16_t crc = 0xFFFF;
    while (len--) {
        uint8_t idx = *data++ ^ (uint8_t)crc;
        crc = (crc >> 8) ^ pgm_read_word(&crc_table[idx]);
    }
    return crc;
}

// Append the CRC-16 of the first len bytes of buf and set tx_sz = len + 2.
static void append_crc(uint8_t len) {
    uint16_t crc = crc16(buf, len);
    buf[len]     = (uint8_t)crc;
    buf[len + 1] = (uint8_t)(crc >> 8);
    tx_sz        = len + 2;
}

// ---- TX helpers --------------------------------------------------------

// Finalise a response frame in buf (appends CRC) and begin DRE-driven transmission.
// No-op for broadcast frames (no_response set by modbus_process).
static void release_tx(void) {
    tx_held       = false;
    USART0.STATUS = USART_TXCIF_bm;
    USART0.CTRLA  = USART_RS485_EXT_gc | USART_DREIE_bm | USART_TXCIE_bm;
}

static void start_tx(uint8_t payload_len) {
    if (no_response) return;
    append_crc(payload_len);
    rx_ptr = buf;    // reuse buf pointer for TX
    // Hold the reply until the line has been quiet for the reply guard (see
    // s_reply_guard); modbus_tick releases it. Checked and set atomically
    // against that ISR so the release can be neither missed nor doubled.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (ticks_since_rx >= s_reply_guard_ticks) release_tx();
        else                                        tx_held = true;
    }
}

// Set the exception flag in buf[1], write the error code, and transmit.
static void send_error(mb_err_t err) {
    buf[1] |= 0x80;
    buf[2]  = (uint8_t)err;
    start_tx(3);
}

// The command and response queues live in cq.c; their DALI adapter (unpack +
// issue + outcome mapping) is dali_sync.c.  There is no cache to invalidate and
// no fade tracking here — the controller owns both, and decides for itself what
// to re-read after a command (see controller/src/dali_state.c).

// ---- FC04 - read input registers ----------------------------------------

// FC04 input registers (read-only). Every one is special-cased; there is no
// generic range:
//   0x0200-0x020F  version block (count must be 16)
//   0x0300-0x030F  RX diagnostics (count must be 16)
//   0x0310-0x0313  sysinfo: uptime, boot count, reset reason (count must be 4)
//   0x0320-0x0327  queue counters (count must be 8)
//
// Anything else returns ILLEGAL_ADDRESS. Block reads are EXACT-LENGTH: asking
// for a different count is ILLEGAL_ADDRESS, not a short read, so a client built
// against a different register map fails loudly instead of parsing a prefix.
//
// The build this image came from (see FC04_BUILD_BLOCK). FW_VERSION is passed as
// a string literal by the Makefile; the fallback marks a build made without it.
#ifndef FW_VERSION
#define FW_VERSION "unknown"
#endif
static const char s_build[] PROGMEM = FW_VERSION;

// Handle FC04 (Read Input Registers).  Supports the FC04_VERSION_BLOCK special case
// at 0x0200 and atomically clears dirty flags for any dirty registers in the read range.
static void process_fc04(void) {
    uint16_t addr  = ((uint16_t)buf[2] << 8) | buf[3];
    uint16_t count = ((uint16_t)buf[4] << 8) | buf[5];

    // Version string block at FC04_VERSION_BLOCK (16-register read, 32 bytes payload).
    // Format: "DALI v<maj>.<min> <10-byte-serial-as-20-hex-chars>" = 30 chars + null.
    if (addr == FC04_VERSION_BLOCK && count == 0x10u) {
        buf[2] = 32u;
        memset(buf + 3, 0, 32u);
        snprintf((char *)(buf + 3), 32u,
                 "DALI v%d.%d %02x%02x%02x%02x%02x%02x%02x%02x%02x%02x",
                 FW_VER_MAJOR, FW_VER_MINOR,
                 SIGROW_SERNUM0, SIGROW_SERNUM1, SIGROW_SERNUM2, SIGROW_SERNUM3,
                 SIGROW_SERNUM4, SIGROW_SERNUM5, SIGROW_SERNUM6, SIGROW_SERNUM7,
                 SIGROW_SERNUM8, SIGROW_SERNUM9);
        start_tx(35u);
        return;
    }

    // Build string block at FC04_BUILD_BLOCK (16-register read, 32 bytes payload).
    if (addr == FC04_BUILD_BLOCK && count == 0x10u) {
        buf[2] = 32u;
        memset(buf + 3, 0, 32u);
        strncpy_P((char *)(buf + 3), s_build, 32u);
        start_tx(35u);
        return;
    }

    // RX diagnostics block at FC04_DIAG_BLOCK (16-register read, 32 bytes).  All
    // values big-endian.  RX is disabled while we build this response, so the
    // ISR-updated counters can't tear mid-read.
    if (addr == FC04_DIAG_BLOCK && count == 0x10u) {
        buf[2] = 32u;
        memset(buf + 3, 0, 32u);
        uint8_t *p = buf + 3;
        p[0] = (uint8_t)(diag_rx_overflow >> 8); p[1] = (uint8_t)diag_rx_overflow;
        p[2] = (uint8_t)(diag_frame_error >> 8); p[3] = (uint8_t)diag_frame_error;
        p[4] = (uint8_t)(diag_crc_error   >> 8); p[5] = (uint8_t)diag_crc_error;
        p[6] = (uint8_t)(diag_frames_ok   >> 8); p[7] = (uint8_t)diag_frames_ok;
        p[8] = 0;                                p[9] = diag_last_fail_len;
        for (uint8_t i = 0; i < 16u; i++) p[10 + i] = diag_last_fail[i];
        // reg13: internal fault count (saturating). reg14: the FIRST fault's
        // identity, [err | site]. These are the errors that had no caller to
        // return to (see errors.h rule 2) — without them a stranded queue slot or
        // a refused command launch is invisible from the master's side.
        uint16_t fc = tm_fault_count();
        p[26] = (uint8_t)(fc >> 8);          p[27] = (uint8_t)fc;
        p[28] = (uint8_t)tm_fault_last_err(); p[29] = (uint8_t)tm_fault_last_site();
        // p[30..31] left zero (spare)
        start_tx(35u);
        return;
    }

    // System-info block at FC04_SYSINFO_BLOCK (4-register read, 8 bytes).
    // uptime/boot-count/reset-reason — see registers.h for the layout.  The
    // 32-bit uptime read is fenced against the LVL0 PIT ISR so it can't tear.
    if (addr == FC04_SYSINFO_BLOCK && count == 0x04u) {
        buf[2] = 8u;
        uint8_t *p = buf + 3;
        uint32_t up = sysinfo_uptime_s();   // fenced inside
        p[0] = (uint8_t)(up >> 24); p[1] = (uint8_t)(up >> 16);
        p[2] = (uint8_t)(up >> 8);  p[3] = (uint8_t)up;
        uint16_t bc = sysinfo_boot_count();
        p[4] = (uint8_t)(bc >> 8);           p[5] = (uint8_t)bc;
        p[6] = 0;                            p[7] = sysinfo_reset_reason();
        start_tx(11u);
        return;
    }

    // Queue counter block at FC04_QDIAG_BLOCK (8-register read, 16 bytes), the
    // spec §6 cold-path diagnostics.  Big-endian, 16-bit WRAPPING counters — see
    // registers.h for the register-by-register layout, which the controller's
    // `mbq` stats parse against and must match exactly.
    //
    // Read straight out of live storage rather than a snapshot: only `results`
    // is written from ISR context, and a torn 16-bit read of one diagnostic
    // sample is not worth a fence on this path (cq.h says why the queue carries
    // no fences at all).
    if (addr == FC04_QDIAG_BLOCK && count == 0x08u) {
        buf[2] = 16u;
        uint8_t *p = buf + 3;
        const volatile cq_counters_t *c = cq_counters();
        const uint16_t v[8] = { c->accepts, c->full, c->executes, c->results,
                                c->acks, c->stalls, diag_q_discard, 0u };
        for (uint8_t i = 0; i < 8u; i++) {
            p[i * 2u]      = (uint8_t)(v[i] >> 8);
            p[i * 2u + 1u] = (uint8_t)v[i];
        }
        start_tx(19u);
        return;
    }

    // Every FC04 block is a special case handled above; anything else is an
    // ILLEGAL_ADDRESS. Where this now falls through, there used to be a generic
    // tail that set buf[2] = count*2 and transmitted WITHOUT filling the payload
    // — harmless only because fc04_range_allowed() returned false
    // unconditionally, and a stale-buffer leak the moment anyone allow-listed a
    // range. The allow-list hook is gone with it; a new block adds its own case
    // above, next to the code that fills the buffer it promises.
    send_error(MB_ERR_ILLEGAL_ADDRESS);
}

// ---- FC03 - read holding registers --------------------------------------

// Handle FC03 (Read Holding Registers).  Returns an exception if any address in
// the requested range is not in the holding register map.
// Build a holding-register read response body into buf: buf[2] = count*2, then
// `count` big-endian registers from `addr`.  Returns false (buf half-filled) if
// any register in the range is unmapped — caller sends ILLEGAL_ADDRESS.
//
// Iterate by index, not `reg < addr + count`: with addr=0xFFFF, count=2 the
// upper bound wraps to 0x0001 and the loop would never run while buf[2] already
// promised 4 bytes.  Iterating by index keeps the count exact; a wrapped
// reg_addr that maps nowhere fails regs_read_holding and trips ILLEGAL_ADDRESS.
static mb_err_t read_holding_block(uint16_t addr, uint16_t count) {
    buf[2] = (uint8_t)(count * 2u);
    uint8_t *rp = buf + 3;
    for (uint16_t i = 0; i < count; i++) {
        uint16_t v = 0;
        tm_err_t e = regs_read_holding((uint16_t)(addr + i), &v);
        if (e != TM_OK) return tm_err_to_modbus(e);
        *rp++ = (uint8_t)(v >> 8);
        *rp++ = (uint8_t)v;
    }
    return 0;
}

static void process_fc03(void) {
    uint16_t addr  = ((uint16_t)buf[2] << 8) | buf[3];
    uint16_t count = ((uint16_t)buf[4] << 8) | buf[5];

    if (count == 0 || count > MAX_REG_RD) { send_error(MB_ERR_ILLEGAL_VALUE); return; }
    mb_err_t e = read_holding_block(addr, count);
    if (e) { send_error(e); return; }
    start_tx(3 + buf[2]);
}

// ---- FC06 / FC10 - write register(s) -----------------------------------

// Its only remaining job is latching pending_reset; validation already
// happened in regs_write_holding, so there is nothing left here that can
// fail.
//
// DALI commands go through the command queue (submit window 0x1000), not through
// here. After the task-05 strip the ONLY action register left is
// MBREG_SYSTEM_RESET — scan-trigger, rescan-interval and commissioning all went
// with the gateway's state.
static void dispatch_action_reg(uint16_t mb_reg) {
    if (mb_reg == MBREG_SYSTEM_RESET) {
        // regs_write_holding validated the magic and returned TM_ERR_BAD_VALUE if
        // it was wrong, so reaching here means the value was correct — which is
        // why this takes no value parameter.  Latch the
        // reset; fire_pending_reset() reboots once the ACK has left the wire.
        pending_reset = true;
    }
}

// Mark slave-addr / baud-index writes for deferred runtime apply.  The
// USERROW write itself has already happened inside regs_write_holding; this
// just remembers to flip the live USART/slave_addr after the response is
// fully on the wire.
static void note_runtime_config_write(uint16_t mb_reg) {
    if (mb_reg == MBREG_SLAVE_ADDR) pending_apply_mask |= APPLY_SLAVE;
    if (mb_reg == MBREG_BAUD_INDEX) pending_apply_mask |= APPLY_BAUD;
}

// Handle FC06 (Write Single Register): validate via regs_write_holding, then
// dispatch_action_reg for the one register that still triggers a side effect
// (MBREG_SYSTEM_RESET).  There is no group membership or queue state left to
// touch here — see registers.c — so this is a straight validate/apply/echo.
static void process_fc06(void) {
    uint16_t addr = ((uint16_t)buf[2] << 8) | buf[3];
    uint16_t val  = ((uint16_t)buf[4] << 8) | buf[5];

    // regs_write_holding now names its cause, so an out-of-range baud index gets
    // ILLEGAL_VALUE and a failed NVM write gets DEVICE_FAILURE, instead of both
    // being reported as "no such register".
    tm_err_t we = regs_write_holding(addr, val);
    if (we != TM_OK) { send_error(tm_err_to_modbus(we)); return; }
    note_runtime_config_write(addr);
    dispatch_action_reg(addr);
    // Echo the request back (standard FC06 response)
    start_tx(6);
}

// Handle FC10 (Write Multiple Registers): validate, update register map, dispatch DALI actions.
static void process_fc10(void) {
    uint16_t addr  = ((uint16_t)buf[2] << 8) | buf[3];
    uint16_t count = ((uint16_t)buf[4] << 8) | buf[5];
    uint8_t  nbytes = buf[6];

    if (count == 0 || count > 15u || nbytes != count * 2u) {
        send_error(MB_ERR_ILLEGAL_VALUE);
        return;
    }

    // No queue submit window here any more: FC 0x41 is the only way into the
    // command queue, so an FC10 write to the retired 0x1000 range falls through
    // to the per-register loop and comes back ILLEGAL_ADDRESS like any other
    // unmapped address.
    //
    // There used to be a pre-flight loop here refusing more than one register
    // in 0x0110-0x014F (group membership masks) with MB_ERR_DEVICE_BUSY. That
    // range was retired in the task-05 stateless-pipe strip along with the
    // group state it guarded, and regs_write_holding already answers any
    // write in it with ILLEGAL_ADDRESS like every other unmapped register —
    // the pre-flight loop was answering DEVICE_BUSY for addresses that no
    // longer exist. A client whose "busy = retry" policy took that literally
    // would retry forever, since ILLEGAL_ADDRESS from the per-register loop
    // below is the only answer this range can ever give now.

    uint8_t *payload_ptr = buf + 7;
    for (uint16_t i = 0; i < count; i++) {
        uint16_t reg_value = ((uint16_t)payload_ptr[0] << 8) | payload_ptr[1];
        uint16_t r         = addr + i;
        payload_ptr += 2;
        tm_err_t we = regs_write_holding(r, reg_value);
        if (we != TM_OK) { send_error(tm_err_to_modbus(we)); return; }
        note_runtime_config_write(r);
        dispatch_action_reg(r);
    }
    // Standard FC10 response: slave addr, FC, start addr (2), count (2)
    start_tx(6);
}

// ---- FC 0x41 - the queue -----------------------------------------------

// Wire flags (bits 7-5) -> the internal QF_* command flags.
//
// THE FLAG BOUNDARY IS HERE, deliberately. The v4 wire packs the flags into the
// top 3 bits of a byte it shares with the ack sequence; the internal 4-byte
// command has carried them as 0x01/0x02/0x04 in a byte of their own since v2,
// and dali_sync.c tests them there. Normalising at the parser keeps the DALI
// adapter unchanged and keeps the wire layout from leaking into the issuance
// code — the alternative (carry the wire packing all the way down) would mean
// every flag test in dali_sync.c changing meaning silently if this one function
// were ever skipped.
static uint8_t queue_flags_to_qf(uint8_t wire) {
    uint8_t f = 0;
    if (wire & QW_TWICE)  f |= QF_TWICE;
    if (wire & QW_DTR0)   f |= QF_DTR0;
    if (wire & QW_ANSWER) f |= QF_ANSWER;
    return f;
}

// Handle FC 0x41.  Length and flag consistency were settled in frame_is_valid,
// so reaching here means the frame is one of the three legal shapes.
//
// THE ORDER OF THESE THREE STEPS IS THE PROTOCOL (spec §2), not an
// implementation detail:
//
//   1. ack     — first, so the response-queue entry it frees is available to the
//                backpressure rule in THIS transaction. Doing it last costs a
//                whole transaction per DALI frame at saturation, which is the
//                entire reason this design exists.
//   2. enqueue — into the entry step 1 may just have made room for.
//   3. reply   — built from the state as it now stands, so the head reported is
//                the head AFTER the ack, which is what lets the master tell a
//                landed frame from a lost one (rules R1/R1') after a timeout.
static void process_fc_queue(void) {
    const bool has_cmd  = (rx_sz != FCQ_REQ_POLL);
    const uint8_t ack   = (uint8_t)(buf[2] & CQ_SEQ_MAX);
    const uint8_t wflag = (uint8_t)(buf[2] & QW_FLAGS_bm);

    // Copy the command out of the RX buffer BEFORE anything writes the reply
    // over it: the reply's accept byte lands on buf[2] and its head bytes on
    // buf[3..4], which are the request's addr and data.
    uint8_t cmd[CQ_CMD_BYTES];
    if (has_cmd) {
        cmd[0] = buf[3];                                        // DALI address byte
        cmd[1] = buf[4];                                        // DALI data byte
        cmd[2] = queue_flags_to_qf(wflag);
        cmd[3] = (rx_sz == FCQ_REQ_ENQ_DTR0) ? buf[5] : 0u;     // DTR0 preload
    }

    (void)cq_ack(ack);   // mismatch is a silent no-op; the head report re-converges the master

    uint8_t n = 2u;
    if (has_cmd) {
        uint8_t seq = cq_accept(cmd);
        buf[n++] = (seq != CQ_SEQ_NONE) ? (uint8_t)((A_ACCEPTED << 5) | seq)
                                        : (uint8_t)(A_FULL << 5);
    }
    cq_head(buf + n);
    start_tx((uint8_t)(n + 2u));
}

// ---- Frame validation --------------------------------------------------

// Return true if the frame in buf has a correct CRC, valid length for its function code,
// and is addressed to this slave (or broadcast address 0).
static bool frame_is_valid(void) {
    // Deferred CRC: compute it once over the captured frame here, at the silence
    // timeout — not incrementally in the RX ISR.  Stored in rx_crc so the
    // discard path can tell a real CRC failure (rx_crc != 0) from a foreign-slave
    // frame (rx_crc == 0, just not our address).
    rx_crc = crc16(buf, rx_sz);
    if (rx_crc != 0 || rx_sz < 5) return false;
    if (buf[0] != slave_addr) {
        if (buf[0] != 0) return false;
        // Broadcast (address 0): only write functions are valid; no response will be sent
        switch (buf[1]) {
            case FC_WRITE_SINGLE_COIL:
            case FC_WRITE_SINGLE_REG:
            case FC_WRITE_COILS:
            case FC_WRITE_MULTI_REG:
                break;
            default:
                return false;
        }
    }

    uint8_t func_code     = buf[1];
    uint8_t expected_size = 255;
    switch (func_code) {
        case FC_READ_COILS:
        case FC_READ_HOLDING:
        case FC_READ_INPUT:
        case FC_WRITE_SINGLE_COIL:
        case FC_WRITE_SINGLE_REG:
            expected_size = 8; break;
        case FC_WRITE_COILS:
        case FC_WRITE_MULTI_REG: {
            uint16_t es = 9u + buf[6];
            expected_size = (es <= MAX_PKT_SZ) ? (uint8_t)es : 255u;
            break;
        }
        case FC_QUEUE:
            // The one place the length/flag table is not a single constant: the
            // three request shapes are distinguished by length AND flags
            // together, so the decision lives in fc41.h as a pure function that
            // the host test enumerates exhaustively.  buf[2] is safe to read —
            // the rx_sz >= 5 gate above has already passed.
            //
            // A false here leaves expected_size at 255, which no frame can match
            // (MAX_PKT_SZ is 40), so the frame is discarded WITHOUT A REPLY,
            // exactly as a corrupt one would be.
            if (fc41_len_ok(rx_sz, buf[2])) expected_size = rx_sz;
            break;
        default:
            // Unknown function code. Accept it on CRC alone — we cannot know its
            // expected length — so it reaches the dispatcher and gets a proper
            // ILLEGAL FUNCTION reply. Dropping it silently instead made the
            // master time out and burn its whole retry budget (~150 ms of bus)
            // for a question we could answer immediately.
            expected_size = rx_sz;
            break;
    }
    return rx_sz == expected_size;
}

// ---- Deferred slave-addr / baud apply ----------------------------------

// Read the freshly-written USERROW value(s) and update the live USART /
// slave-addr state.  Called after the response we sent at the OLD config
// has cleared the wire (either from the TXC ISR for normal frames, or from
// modbus_process for broadcasts that send no response), and BEFORE
// return_to_rx re-enables RX so the next byte arrives at the new baud and
// is matched against the new slave address.
//
// USART0.BAUD must not be written mid-transmission, but TXC fires after the
// last stop bit has cleared so it is safe here.
static void apply_pending_config(void) {
    uint8_t mask = pending_apply_mask;
    if (!mask) return;
    pending_apply_mask = 0;
    if (mask & APPLY_BAUD) {
        uint8_t bidx = userrow_get_baud_index();
        USART0.BAUD     = baud_reg_for(bidx);
        s_silence_reload = s_silence_threshold[bidx];
        s_reply_guard_ticks = s_reply_guard[bidx];
    }
    if (mask & APPLY_SLAVE) {
        slave_addr = userrow_get_slave_addr();
    }
}

// ---- Return to receive mode ---------------------------------------------

// Reset frame state and re-enable the USART receiver.
// Called after every completed transaction: normal response (TXC ISR),
// broadcast (no response sent), and invalid frames (silence timeout).
static void return_to_rx(void) {
    rx_ptr      = buf;
    rx_crc      = 0xFFFF;
    rx_sz       = 0;
    rx_ferr     = 0;
    frame_ready = false;
    modbus_idle = true;
    USART0.STATUS = USART_RXSIF_bm | USART_TXCIF_bm;
    USART0.CTRLA  = USART_RS485_EXT_gc | USART_RXSIE_bm | USART_RXCIE_bm;
}

// ---- SILICON ERRATUM: USART Start-of-Frame Detection ---------------------
//
// Microchip tinyAVR silicon errata, USART: "Start-of-Frame Detector Can Be
// Unintentionally Triggered" (ATtiny1604 is tinyAVR 0-series; the same erratum
// is published for the 0- and 1-series parts). Quoting the mechanism:
//
//   "If the Receive Data (RXDATA) registers are read while receiving new data,
//    the Receive Complete Interrupt Flag (RXCIF) is cleared. This triggers the
//    Start-of-Frame Detector and falsely detects the following falling edge as
//    a start bit. When the Start-of-Frame Detector detects a start condition,
//    the frame reception is restarted, resulting in corrupt received data."
//
//   "The USART Receive Start Interrupt Flag (RXSIF) always is '0' when in
//    Active mode. No interrupt will be triggered."
//
//   Workaround: "Disable Start-of-Frame Detection by writing '0' to the SFDEN
//   bit in USARTn.CTRLB when the device is in Active mode. Re-enable it by
//   writing the bit to '1' before transitioning to Standby sleep mode. This
//   workaround depends on a protocol preventing a new incoming frame when
//   re-enabling Start-of-Frame Detection."
//
// TWO consequences for this driver, both previously wrong:
//
//   1. SFDEN must NOT be left set in Active mode. It is cleared here and armed
//      only around sleep_cpu() in main.c, where modbus_idle already guarantees
//      no frame is incoming — exactly the caveat the workaround requires.
//
//   2. RXSIE must STAY ENABLED, and the reason is worth recording because it
//      looks removable and is not. There is no USART0_RXS_vect handler, so on
//      the face of it enabling RXSIE risks vectoring to __bad_interrupt (which
//      jumps to reset). It is nonetheless required: it is what arms the wake
//      from Standby, and removing it was MEASURED to break reception — 0.11%
//      bad even with the DALI bus idle, where the fix otherwise gives 0.00%,
//      because the first byte after every sleep is then mangled.
//
//      It never actually vectors, and the erratum is why: RXSIF "always is '0'
//      when in Active mode", so once the CPU is awake the flag cannot latch an
//      interrupt. The enable arms the wake; the byte is delivered by RXC as
//      usual. Do not "clean this up" without re-measuring the DALI-idle case.
//
// This was measured, not inferred. With SFDEN set in Active mode the gateway
// reported framing errors on characters that were ELECTRICALLY PERFECT: a PB3
// marker pulsed from the RXC ISR on FERR, captured alongside both RS-485 lines,
// showed the failing frame decoding as valid Modbus with every stop bit high
// and inter-character gaps of exactly 10.00 bits. Rates over 40-60 s of
// saturated traffic, gateway counters zeroed after the baud had settled:
//
//                  SFDEN in Active     SFDEN only around sleep
//     19200          0.13% bad               0.00%  (0 of 3025 frames)
//     38400          0.85% bad               0.00%  (0 of 6609 frames)
//
// The failure needed concurrent DALI traffic because DALI ISR latency is what
// delays our RXC ISR until the RXDATA read lands INSIDE the next character
// rather than in the gap before it; and it hit 38400 ~6.5x harder because that
// gap is half as wide. Errors therefore never appeared on a frame's first byte.

// ---- Public API ---------------------------------------------------------

void modbus_init(uint8_t addr) {
    slave_addr  = addr;
    rx_ptr      = buf;
    rx_crc      = 0xFFFF;
    rx_sz       = 0;
    rx_ferr     = 0;
    frame_ready = false;
    modbus_idle = true;
    initial_silence_observed = false;

    uint8_t bidx = userrow_get_baud_index();
    s_silence_reload = s_silence_threshold[bidx];
    s_reply_guard_ticks = s_reply_guard[bidx];
    tx_held = false;
    // Arm the initial 3.5-char silence countdown.  modbus_tick will set
    // initial_silence_observed when it expires without an intervening byte.
    silence_ticks = s_silence_reload;

    PORTMUX.CTRLB          = PORTMUX_USART0_ALTERNATE_gc;
    RS485_PORT.PIN1CTRL    = PORT_PULLUPEN_bm;
    RS485_PORT.OUTSET      = RS485_TX_bm;
    RS485_PORT.OUTCLR      = RS485_DE_bm;
    RS485_PORT.DIRSET      = RS485_TX_bm | RS485_DE_bm;
    RS485_PORT.DIRCLR      = RS485_RX_bm;

    USART0.BAUD  = baud_reg_for(bidx);
    USART0.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc |
                   USART_CHSIZE_8BIT_gc         | USART_SBMODE_1BIT_gc;
    USART0.CTRLA = USART_RS485_EXT_gc | USART_RXSIE_bm | USART_RXCIE_bm;
    // SFDEN is deliberately CLEAR here and is armed ONLY around sleep_cpu()
    // in main.c. See the erratum note on modbus_init() below: with SFDEN set in
    // Active mode, reading RXDATA while the next byte is arriving restarts
    // frame reception and corrupts the byte.
    USART0.CTRLB = USART_TXEN_bm | USART_RXEN_bm;
}

void modbus_tick(void) {
    if (ticks_since_rx != 0xFFu) ticks_since_rx++;
    // Release a held reply once the reply guard has elapsed (see s_reply_guard).
    if (tx_held && ticks_since_rx >= s_reply_guard_ticks) release_tx();

    // Silence counter: decremented here, frame_ready set when it hits 0.
    // The silence threshold is set (reset to non-zero) by the RXC ISR and at boot
    // (to arm the initial-silence gate).
    if (silence_ticks) {
        silence_ticks--;
        if (!silence_ticks) {
            if (!initial_silence_observed) {
                // 3.5 char of post-reset silence has elapsed - switch on normal
                // frame accumulation.  No frame to validate yet.
                initial_silence_observed = true;
                return;
            }
            // Frame boundary: bytes were captured and t3.5 of silence has now
            // elapsed.  Hand the frame to the main loop (modbus_process) for CRC,
            // validation and processing — none of that runs in this timebase ISR.
            // Disable RX to freeze the captured buffer until the main loop
            // re-arms it (the master is half-duplex and waiting for our reply, so
            // it is not transmitting during this window).
            frame_ready  = true;
            USART0.CTRLA = USART_RS485_EXT_gc;
        }
    }
}

void modbus_process(void) {
    if (!frame_ready) return;

    // Validate the captured frame here, in the main loop — NOT in the timebase
    // ISR.  modbus_tick only detected the t3.5 frame boundary; we compute the CRC
    // and check length/address now, and drop invalid or foreign-slave frames
    // without a response.  The inter-frame silence (§2.5.1.1) has already elapsed,
    // so once a valid frame is confirmed the response can go straight out.
    if (!frame_is_valid()) {
        // A CRC-good frame for another device: none of our business, including
        // any framing errors it carried.
        if (rx_crc == 0u && buf[0] != slave_addr) {
            return_to_rx();
            return;
        }
        // Two CRC-good frames glued together — a slave that answered faster
        // than our silence window, so its reply joined the master's request
        // (the house relay board, frame_pair.h). Not corruption; if neither
        // half is for us it is just someone else's traffic.
        {
            uint8_t k;
            if (rx_crc != 0u && frame_is_pair(buf, rx_sz, crc16, &k)
                && buf[0] != slave_addr && buf[k] != slave_addr) {
                return_to_rx();
                return;
            }
        }
        diag_frame_error += rx_ferr;
        // A real CRC failure, whoever it was meant for.  Snapshot the bytes for
        // forensics.
        if (rx_sz >= 4u && rx_crc != 0u) {
            diag_crc_error++;
            uint8_t n = (rx_sz < sizeof diag_last_fail) ? rx_sz
                                                        : (uint8_t)sizeof diag_last_fail;
            // Store the CLAMPED length, not raw rx_sz: diag_last_fail is only
            // sizeof(diag_last_fail) (16) bytes, and reg4 is documented (and
            // read by FC04 0x0300) as that capture's length. rx_sz can run up
            // to MAX_PKT_SZ (40) for an oversized frame; reporting it
            // uncapped told a reader to decode up to 40 "frame bytes" out of
            // a 16-byte field, swallowing reg13-14 (fault counters) as if
            // they were frame data.
            diag_last_fail_len = n;
            for (uint8_t i = 0; i < n; i++) diag_last_fail[i] = buf[i];
        }
        // A queue frame addressed to US whose CRC was fine but whose length or
        // flags did not agree.  Counted here rather than inside frame_is_valid
        // so validation stays a predicate with no side effects, and counted
        // separately from crc_error because they accuse different things: the
        // line, versus the master's frame builder.  Without this the discard is
        // wholly unobservable — the protocol requires it to look exactly like a
        // dropped frame from the far end.
        else if (rx_sz >= 4u && buf[0] == slave_addr && buf[1] == FC_QUEUE) {
            diag_q_discard++;
        }
        return_to_rx();   // discard + re-arm RX (clears frame_ready)
        return;
    }
    diag_frames_ok++;   // a valid, CRC-good frame addressed to us
    diag_frame_error += rx_ferr;   // CRC-good despite a framing error: rare, still ours

    no_response = (buf[0] == 0);

    // Anything not handled below gets ILLEGAL_FUNCTION, the standard Modbus
    // reply: coils (FC01/FC05/FC0F), which are deliberately unsupported; the
    // retired FC23 and draft FC66 queue paths, which a pre-v4 controller would
    // still try (and which the version register at FC04 0x0200 is what it should
    // be checking instead); and any unrecognised code, which frame_is_valid lets
    // through on CRC alone so it can be answered rather than left to time out.
    switch (buf[1]) {
        case FC_READ_INPUT:        process_fc04(); break;
        case FC_READ_HOLDING:      process_fc03(); break;
        case FC_WRITE_SINGLE_REG:  process_fc06(); break;
        case FC_WRITE_MULTI_REG:   process_fc10(); break;
        case FC_QUEUE:             process_fc_queue(); break;
        default:                   send_error(MB_ERR_ILLEGAL_FUNCTION); break;
    }

    if (no_response) {
        no_response = false;
        // Broadcast: no response was sent, so apply the new config now (no
        // outgoing frame to wait for).
        apply_pending_config();
        fire_pending_reset();   // broadcast reset: no ACK to wait for
        return_to_rx();
    }

    frame_ready = false;
}

// ---- ISRs ----------------------------------------------------------------

ISR(USART0_DRE_vect) {
    if (tx_sz) {
        USART0.TXDATAL = *rx_ptr++;
        tx_sz--;
    } else {
        USART0.CTRLA = USART_RS485_EXT_gc | USART_TXCIE_bm;
    }
}

ISR(USART0_TXC_vect) {
    // Drain any bytes the master sent during our transmission - they sit in the
    // USART RX FIFO (depth 2) and would otherwise be prepended to the next
    // legitimate packet, corrupting its length/CRC and forcing a timeout-driven
    // recovery.  Reading RXDATAL clears RXCIF.
    while (USART0.STATUS & USART_RXCIF_bm) {
        (void)USART0.RXDATAL;
    }
    // Response is fully on the wire now: safe to flip USART0.BAUD or update
    // slave_addr.  Done before return_to_rx so the next byte is sampled at
    // the new baud rate and matched against the new slave address.
    apply_pending_config();
    fire_pending_reset();   // ACK is on the wire; reboot if one was latched
    return_to_rx();
}

ISR(USART0_RXC_vect) {
    modbus_idle    = false;
    silence_ticks  = s_silence_reload;
    ticks_since_rx = 0;
    USART0.STATUS  = USART_RXSIF_bm;

    if (!initial_silence_observed) {
        // Modbus §2.5.1.1: until 3.5 char of post-reset silence has been observed,
        // any incoming byte is residue from a frame already in progress at reset
        // time.  Drain it (clears RXCIF) but do not accumulate; the silence reset
        // above re-arms the countdown so the gate only fires once the master
        // finishes its in-progress frame.
        if (USART0.STATUS & USART_RXCIF_bm) {
            (void)USART0.RXDATAL;
        }
        return;
    }

    if (USART0.STATUS & USART_RXCIF_bm) {
        // RXDATAH carries BUFOVF/FERR/PERR for this byte and must be read
        // before RXDATAL (which pops the FIFO).  BUFOVF = the 2-deep RX FIFO
        // overflowed and a byte was lost — the smoking gun for RXC starvation.
        uint8_t sh  = USART0.RXDATAH;
        uint8_t val = USART0.RXDATAL;
        if (sh & USART_BUFOVF_bm)                    diag_rx_overflow++;
        if ((sh & (USART_FERR_bm | USART_PERR_bm)) && rx_ferr != 0xFFu) rx_ferr++;
        if (rx_sz < MAX_PKT_SZ) {
            *rx_ptr++ = val;
            rx_sz++;
            // No CRC here.  Frame completion is gated on the 3.5-char silence
            // timeout (Modbus §2.5.1.1, modbus_tick), so the CRC is needed only
            // once — when the frame is declared complete.  It is computed in one
            // pass in frame_is_valid() at the timeout, keeping this hot RX ISR to
            // pure byte capture (best margin against RX-FIFO overrun).  A byte
            // arriving within the silence window resets silence_ticks (above) and
            // keeps the frame open; spec-correct, costs ~one t3.5 (~2 ms @ 19200).
        }
    }
}
