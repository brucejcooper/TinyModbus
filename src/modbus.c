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
#include "mbq_slots.h"
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
// divisor scales up to compensate.  VERIFY ON THE BENCH — scope the slave's TX
// bit width, which should land on the nominal period (e.g. 52.08 µs at 19200)
// once this is applied; if it moves the wrong way, negate `sigerr`.
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

// 3.5-character silence threshold in TCA0 ticks (~417 µs each), per baud rate.
// USART is configured for 1 start + 8 data + 1 stop = 10 bits/char.  The first decrement
// after a byte arrival can land anywhere from 0 to 1 tick later, so silence_ticks = N
// gives an observed silence window of (N-1) to N ticks.  N is sized so the lower bound
// (N-1 ticks) meets Modbus §2.5.1.1's 3.5-char minimum.
//   3.5 char @ 9600  = 3.65 ms = 8.74 ticks -> N = 10 -> 9-10 ticks (3.75-4.17 ms)
//   3.5 char @ 19200 = 1.82 ms = 4.37 ticks -> N = 6  -> 5-6  ticks (2.08-2.50 ms)
//   3.5 char @ 38400 = 0.91 ms = 2.19 ticks -> N = 4  -> 3-4  ticks (1.25-1.67 ms)
static const uint8_t  s_silence_threshold[] = { 10, 6, 4 };
// Per-baud reload value for the live silence_ticks countdown (set from
// s_silence_threshold[] on init / baud change; copied into silence_ticks each
// time a byte arrives).  Named _reload to disambiguate from the countdown.
static volatile uint8_t  s_silence_reload;

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
#define FC_READ_WRITE_MULTI  0x17   // FC23: write submit window, then read status/holding

typedef enum {
    MB_ERR_ILLEGAL_FUNCTION    = 0x01,
    MB_ERR_ILLEGAL_ADDRESS     = 0x02,
    MB_ERR_ILLEGAL_VALUE       = 0x03,
    MB_ERR_DEVICE_FAILURE      = 0x04,
    MB_ERR_DEVICE_BUSY         = 0x06,
    MB_ERR_TARGET_FAILED       = 0x0B,
    MB_ERR_DALI_PULSE_INVALID  = 0x30,
} mb_err_t;

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
// tx_sz is written on the main path (append_crc) and decremented in the DRE
// ISR, so it must be volatile.  no_response is main-only today (set in
// modbus_process, read in start_tx), marked volatile for consistency with the
// other TX-path state and to stay correct if start_tx is ever reached from an ISR.
static volatile uint8_t  tx_sz    = 0;
static volatile bool     no_response   = false;

// ---- RX diagnostics (read via FC04 0x0300, 16-register read) --------------
// Counters to pin down why some Modbus requests go unanswered under load:
//   rx_overflow : USART RX FIFO overflowed — a byte was lost (RXC starvation).
//   frame_error : USART framing/parity error on a received byte.
//   crc_error   : a frame's worth of bytes arrived, then 3.5-char silence, but
//                 the CRC was non-zero (corruption — distinct from a foreign
//                 slave's valid-CRC frame, which is NOT counted).
//   frames_ok   : valid frames addressed to us that we processed (denominator).
// last_fail[]   : the bytes of the most recent CRC-failed frame, for forensics.
static volatile uint16_t diag_rx_overflow   = 0;
static volatile uint16_t diag_frame_error   = 0;
static volatile uint16_t diag_crc_error     = 0;
static volatile uint16_t diag_frames_ok     = 0;
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
static void start_tx(uint8_t payload_len) {
    if (no_response) return;
    append_crc(payload_len);
    rx_ptr = buf;    // reuse buf pointer for TX
    USART0.STATUS = USART_TXCIF_bm;
    USART0.CTRLA  = USART_RS485_EXT_gc | USART_DREIE_bm | USART_TXCIE_bm;
}

// Set the exception flag in buf[1], write the error code, and transmit.
static void send_error(mb_err_t err) {
    buf[1] |= 0x80;
    buf[2]  = (uint8_t)err;
    start_tx(3);
}

// The generic slot command queue lives in mbq_slots.c; its DALI adapter (unpack +
// issue + terminal mapping) is the OP_RAW arms of dali_sync.c.  There is no cache
// to invalidate and no fade tracking here — the controller owns both, and decides
// for itself what to re-read after a command (see controller/src/dali_state.c).

// ---- FC04 - read input registers ----------------------------------------

// FC04 input registers (read-only):
//   0x0100-0x0103  present bitmap                   (4 regs)
//   0x0104-0x0107  dirty bitmap (read-clears)       (4 regs)
//   0x0200         version block (special-cased; count must be 16)
//   0x0300 diag / 0x0310 sysinfo (special-cased)
//   0x1010-0x1019  queue status block (also served under FC03; special-cased)
//
// The per-ballast level bank (0x0000-0x003F) and group membership masks
// (0x0110-0x014F) live in FC03/FC06/FC10 holding space; reads via FC04 return
// ILLEGAL_ADDRESS.  present + dirty are packed into the contiguous 0x0100-0x0107
// ladder so a master can read both in one 8-register FC04 request.
//
// addr and count come off the wire as 16-bit values; promoting the sum to
// uint32_t prevents wrap (e.g. addr=0xFFFF, count=1 -> 0) from sneaking
// through the upper-bound check.
static bool fc04_range_allowed(uint16_t addr, uint8_t count) {
    (void)addr; (void)count;
    // TASK 05: the contiguous FC04 ladder (present[4] | dirty[4] at 0x0100-0x0107)
    // is GONE with the gateway's cache. The only FC04 blocks left are the
    // version, diag and sysinfo special cases, each handled before this check.
    return false;
}

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
        // p[26..31] left zero (spare)
        start_tx(35u);
        return;
    }

    // System-info block at FC04_SYSINFO_BLOCK (4-register read, 8 bytes).
    // uptime/boot-count/reset-reason — see registers.h for the layout.  The
    // 32-bit uptime read is fenced against the LVL0 PIT ISR so it can't tear.
    if (addr == FC04_SYSINFO_BLOCK && count == 0x04u) {
        buf[2] = 8u;
        uint8_t *p = buf + 3;
        uint32_t up;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { up = g_uptime_s; }
        p[0] = (uint8_t)(up >> 24); p[1] = (uint8_t)(up >> 16);
        p[2] = (uint8_t)(up >> 8);  p[3] = (uint8_t)up;
        p[4] = (uint8_t)(g_boot_count >> 8); p[5] = (uint8_t)g_boot_count;
        p[6] = 0;                            p[7] = g_reset_reason;
        start_tx(11u);
        return;
    }

    // Slot status block at MBREG_SLOT_STATUS_BASE (0x1010), 1..8 registers, served
    // identically under FC04 and FC03.  Not backed by register_map_t — served
    // from the generic queue module (mbq_slots.c), which fences each slot's
    // [status|response8] read against the LVL1 completion writer.
    if (addr >= MBREG_SLOT_STATUS_BASE
        && (uint32_t)addr + count <= (uint32_t)MBREG_SLOT_STATUS_BASE + MBQ_SLOTS
        && count >= 1u) {
        buf[2] = (uint8_t)(count * 2u);
        mbq_read_status((uint8_t)(addr - MBREG_SLOT_STATUS_BASE), (uint8_t)count, buf + 3);
        start_tx(3 + buf[2]);
        return;
    }

    if (count == 0 || count > MAX_REG_RD) {
        send_error(MB_ERR_ILLEGAL_VALUE);
        return;
    }
    if (!fc04_range_allowed(addr, (uint8_t)count)) {
        send_error(MB_ERR_ILLEGAL_ADDRESS);
        return;
    }

    buf[2] = (uint8_t)(count * 2u);

    start_tx(3 + buf[2]);
}

// ---- FC03 - read holding registers --------------------------------------

// Handle FC03 (Read Holding Registers).  Returns an exception if any address in
// the requested range is not in the holding register map.
// Build a holding-register read response body into buf: buf[2] = count*2, then
// `count` big-endian registers from `addr`.  Returns false (buf half-filled) if
// any register in the range is unmapped — caller sends ILLEGAL_ADDRESS.  Shared
// by FC03 and the read half of FC23.
//
// Iterate by index, not `reg < addr + count`: with addr=0xFFFF, count=2 the
// upper bound wraps to 0x0001 and the loop would never run while buf[2] already
// promised 4 bytes.  Iterating by index keeps the count exact; a wrapped
// reg_addr that maps nowhere fails regs_read_holding and trips ILLEGAL_ADDRESS.
static bool read_holding_block(uint16_t addr, uint16_t count) {
    buf[2] = (uint8_t)(count * 2u);
    uint8_t *rp = buf + 3;
    for (uint16_t i = 0; i < count; i++) {
        uint16_t v = 0;
        if (!regs_read_holding((uint16_t)(addr + i), &v)) return false;
        *rp++ = (uint8_t)(v >> 8);
        *rp++ = (uint8_t)v;
    }
    return true;
}

// Validate + execute a write to the v2 slot command block (0x1000-0x100F),
// shared by FC10 and the write half of FC23.  A write covers 1..8 CONTIGUOUS
// slots: wr_addr = MBREG_SLOT_CMD_BASE + 2*first_slot, wr_count = 2*n_slots.  `regs`
// points at wr_count*2 bytes of opaque command data in wire order.  Returns a
// Modbus error (0 on success): out-of-range / mis-aligned base → ILLEGAL_ADDRESS;
// odd/zero register count → ILLEGAL_VALUE; any targeted slot active → DEVICE_BUSY
// (all-or-nothing — mbq_submit changes nothing on a reject).
static mb_err_t submit_window(uint16_t wr_addr, uint16_t wr_count, const uint8_t *regs) {
    if (wr_addr < MBREG_SLOT_CMD_BASE || wr_addr > MBREG_SLOT_CMD_END)  return MB_ERR_ILLEGAL_ADDRESS;
    uint16_t off = (uint16_t)(wr_addr - MBREG_SLOT_CMD_BASE);
    if (off & 1u)                                              return MB_ERR_ILLEGAL_ADDRESS;  // slot boundary
    if ((wr_count & 1u) || wr_count == 0u)                     return MB_ERR_ILLEGAL_VALUE;
    if ((uint32_t)wr_addr + wr_count > (uint32_t)MBREG_SLOT_CMD_END + 1u) return MB_ERR_ILLEGAL_ADDRESS;
    uint8_t first_slot = (uint8_t)(off / 2u);
    uint8_t n_slots    = (uint8_t)(wr_count / 2u);
    if (!mbq_submit(first_slot, regs, n_slots))               return MB_ERR_DEVICE_BUSY;
    return 0;
}

static void process_fc03(void) {
    uint16_t addr  = ((uint16_t)buf[2] << 8) | buf[3];
    uint16_t count = ((uint16_t)buf[4] << 8) | buf[5];

    if (count == 0 || count > MAX_REG_RD) { send_error(MB_ERR_ILLEGAL_VALUE); return; }
    if (!read_holding_block(addr, count)) { send_error(MB_ERR_ILLEGAL_ADDRESS); return; }
    start_tx(3 + buf[2]);
}

// ---- FC06 / FC10 - write register(s) -----------------------------------

// Returns 0 on success, or a Modbus error code if the command cannot be dispatched.
//
// DALI commands go through the command queue (submit window 0x1000), not through
// here. After the task-05 strip the ONLY action register left is
// MBREG_SYSTEM_RESET — scan-trigger, rescan-interval and commissioning all went
// with the gateway's state.
static mb_err_t dispatch_action_reg(uint16_t mb_reg, uint16_t val) {
    if (mb_reg == MBREG_SYSTEM_RESET) {
        // regs_write_holding already validated the magic value.  Latch the
        // reset; fire_pending_reset() reboots once the ACK has left the wire.
        pending_reset = true;
        return 0;
    }
    return 0;
}

// Mark slave-addr / baud-index writes for deferred runtime apply.  The
// USERROW write itself has already happened inside regs_write_holding; this
// just remembers to flip the live USART/slave_addr after the response is
// fully on the wire.
static void note_runtime_config_write(uint16_t mb_reg) {
    if (mb_reg == MBREG_SLAVE_ADDR) pending_apply_mask |= APPLY_SLAVE;
    if (mb_reg == MBREG_BAUD_INDEX) pending_apply_mask |= APPLY_BAUD;
}

// Handle FC06 (Write Single Register): validate, update register map, dispatch DALI action.
//
// Group register writes go through regs_write_holding which transparently
// handles three cases: (1) no-change write -> ACK without DALI work; (2)
// actual change with group channel idle -> arm + ACK; (3) actual change
// with group channel busy -> rollback + return false.  The downstream
// busy check distinguishes case (3) from a genuine ILLEGAL_ADDRESS.
static void process_fc06(void) {
    uint16_t addr = ((uint16_t)buf[2] << 8) | buf[3];
    uint16_t val  = ((uint16_t)buf[4] << 8) | buf[5];

    if (!regs_write_holding(addr, val)) {
        // Three failure shapes share the regs_write_holding "false" return:
        //   - group-register write rejected because group channel is busy
        //   - 0x1014 commissioning write with an invalid value (illegal value)
        //   - 0x1014 commissioning write while running (busy)
        //   - everything else is genuine ILLEGAL_ADDRESS.
        /* TASK 05: the commissioning register and the group-mask registers are
         * gone, and with them the only two writes that could fail for a reason
         * other than "no such register". A rejected write is now unambiguous. */
        send_error(MB_ERR_ILLEGAL_ADDRESS);
        return;
    }
    note_runtime_config_write(addr);
    mb_err_t err = dispatch_action_reg(addr, val);
    if (err) { send_error(err); return; }
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

    // Raw command queue submit window (0x1000-0x1007): the WHOLE FC10 write is
    // one all-or-nothing submit of count/2 command records.  Handled here,
    // before the per-register loop, because it is a block operation (the record
    // count and all-or-nothing free-slot check need the whole write at once).
    if (addr == MBREG_SLOT_CMD_BASE) {
        mb_err_t e = submit_window(addr, count, buf + 7);
        if (e) { send_error(e); return; }
        start_tx(6);   // standard FC10 echo (addr, fc, start, count)
        return;
    }

    // Group register write rules: only one DALI-actionable group write
    // per FC10 (matches the spec's "restrict FC10 group writes to a
    // single register at a time" option).  Multi-group bunches get
    // DEVICE_BUSY pre-flight; the in-flight busy check happens inside
    // regs_write_holding (so a no-op write doesn't get rejected when
    // another group write is in flight).
    {
        uint8_t group_targets = 0;
        for (uint16_t i = 0; i < count; i++) {
            uint16_t r = addr + i;
            if (r >= 0x0110u && r <= 0x014Fu) group_targets++;
        }
        if (group_targets > 1u) {
            send_error(MB_ERR_DEVICE_BUSY);
            return;
        }
    }

    uint8_t *payload_ptr = buf + 7;
    for (uint16_t i = 0; i < count; i++) {
        uint16_t reg_value = ((uint16_t)payload_ptr[0] << 8) | payload_ptr[1];
        uint16_t r         = addr + i;
        payload_ptr += 2;
        if (!regs_write_holding(r, reg_value)) {
            // Same BUSY-vs-ILLEGAL distinction as FC06.
            send_error(MB_ERR_ILLEGAL_ADDRESS);   /* see process_fc06 */
            return;
        }
        note_runtime_config_write(r);
        mb_err_t err = dispatch_action_reg(r, reg_value);
        if (err) { send_error(err); return; }
    }
    // Standard FC10 response: slave addr, FC, start addr (2), count (2)
    start_tx(6);
}

// ---- FC23 - read/write multiple registers ------------------------------

// Handle FC23 (0x17).  Per the queue design the WRITE executes before the READ:
// the write range is the command-queue submit window (0x1000), the read range
// is holding space (typically the 0x1010 status block) — so one transaction
// both submits new commands and collects the status/results of earlier ones
// (the submit-N / collect-N-1 pipeline).  The response carries the post-write
// read block, exactly like an FC03 read.
static void process_fc23(void) {
    uint16_t rd_addr  = ((uint16_t)buf[2] << 8) | buf[3];
    uint16_t rd_count = ((uint16_t)buf[4] << 8) | buf[5];
    uint16_t wr_addr  = ((uint16_t)buf[6] << 8) | buf[7];
    uint16_t wr_count = ((uint16_t)buf[8] << 8) | buf[9];
    uint8_t  wr_bytes = buf[10];

    if (rd_count == 0 || rd_count > MAX_REG_RD)   { send_error(MB_ERR_ILLEGAL_VALUE); return; }
    if (wr_bytes != (uint8_t)(wr_count * 2u))     { send_error(MB_ERR_ILLEGAL_VALUE); return; }

    // --- WRITE FIRST ---
    // The only writable FC23 target is the queue submit window.  wr_count == 0
    // is allowed: a read-only FC23 (pure status/collect poll, no new submit).
    // wr_data starts at buf[11]; submit_window (via mbq_submit) copies it out
    // before we build the response into buf, so the later overwrite is safe.
    // An exception here replaces the whole response, so nothing is read.
    if (wr_count > 0u) {
        mb_err_t e = submit_window(wr_addr, wr_count, buf + 11);
        if (e) { send_error(e); return; }
    }

    // --- THEN READ (holding space, incl. the 0x1010 status block) ---
    // The write (if any) already executed; a bad read range is a malformed
    // request (the controller always pairs its submit with a valid status read).
    if (!read_holding_block(rd_addr, rd_count)) { send_error(MB_ERR_ILLEGAL_ADDRESS); return; }
    start_tx(3 + buf[2]);
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
        case FC_READ_WRITE_MULTI: {
            // addr,fc,rd_start(2),rd_cnt(2),wr_start(2),wr_cnt(2),wr_bytecount,
            // wr_data[wr_bytecount],crc(2) = 13 + wr_bytecount.  buf[10] is the
            // write byte count (valid here: a passing CRC means the frame is
            // complete, same reasoning as the FC16 9+buf[6] case above).
            uint16_t es = 13u + buf[10];
            expected_size = (es <= MAX_PKT_SZ) ? (uint8_t)es : 255u;
            break;
        }
        default: break;
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
    frame_ready = false;
    modbus_idle = true;
    USART0.STATUS = USART_RXSIF_bm | USART_TXCIF_bm;
    USART0.CTRLA  = USART_RS485_EXT_gc | USART_RXSIE_bm | USART_RXCIE_bm;
}

// ---- Public API ---------------------------------------------------------

void modbus_init(uint8_t addr) {
    slave_addr  = addr;
    rx_ptr      = buf;
    rx_crc      = 0xFFFF;
    rx_sz       = 0;
    frame_ready = false;
    modbus_idle = true;
    initial_silence_observed = false;

    uint8_t bidx = userrow_get_baud_index();
    s_silence_reload = s_silence_threshold[bidx];
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
    USART0.CTRLB = USART_TXEN_bm | USART_RXEN_bm | USART_SFDEN_bm;
}

void modbus_tick(void) {
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
        // Count a real CRC failure only — a foreign slave's frame has a valid
        // (zero) CRC and just isn't our address, so it must not inflate the
        // counter.  Snapshot the bytes for forensics.
        if (rx_sz >= 4u && rx_crc != 0u) {
            diag_crc_error++;
            diag_last_fail_len = rx_sz;
            uint8_t n = (rx_sz < sizeof diag_last_fail) ? rx_sz
                                                        : (uint8_t)sizeof diag_last_fail;
            for (uint8_t i = 0; i < n; i++) diag_last_fail[i] = buf[i];
        }
        return_to_rx();   // discard + re-arm RX (clears frame_ready)
        return;
    }
    diag_frames_ok++;   // a valid, CRC-good frame addressed to us

    no_response = (buf[0] == 0);

    // FC01/FC05/FC0F (coils) are not supported.  Well-formed coil frames
    // survive frame_is_valid (size check still recognises them) and reach
    // this switch, where the default branch returns ILLEGAL_FUNCTION —
    // the standard Modbus reply for unsupported function codes.
    switch (buf[1]) {
        case FC_READ_INPUT:        process_fc04(); break;
        case FC_READ_HOLDING:      process_fc03(); break;
        case FC_WRITE_SINGLE_REG:  process_fc06(); break;
        case FC_WRITE_MULTI_REG:   process_fc10(); break;
        case FC_READ_WRITE_MULTI:  process_fc23(); break;
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
        if (sh & (USART_FERR_bm | USART_PERR_bm))    diag_frame_error++;
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
