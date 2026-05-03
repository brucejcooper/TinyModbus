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

// 3.5-character silence threshold in TCA0 ticks (~417 µs each), per baud rate.
// USART is configured for 1 start + 8 data + 1 stop = 10 bits/char.  The first decrement
// after a byte arrival can land anywhere from 0 to 1 tick later, so silence_ticks = N
// gives an observed silence window of (N-1) to N ticks.  N is sized so the lower bound
// (N-1 ticks) meets Modbus §2.5.1.1's 3.5-char minimum.
//   3.5 char @ 9600  = 3.65 ms = 8.74 ticks -> N = 10 -> 9-10 ticks (3.75-4.17 ms)
//   3.5 char @ 19200 = 1.82 ms = 4.37 ticks -> N = 6  -> 5-6  ticks (2.08-2.50 ms)
//   3.5 char @ 38400 = 0.91 ms = 2.19 ticks -> N = 4  -> 3-4  ticks (1.25-1.67 ms)
static const uint8_t  s_silence_threshold[] = { 10, 6, 4 };
static volatile uint8_t  s_silence_ticks;

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
static          uint8_t  tx_sz    = 0;
static          bool     no_response   = false;
volatile bool            modbus_idle   = true;

// Set when a write to MBREG_SLAVE_ADDR or MBREG_BAUD_INDEX has just landed in
// USERROW.  The runtime change (USART0.BAUD, slave_addr local) is deferred
// until the response we are about to send has cleared the wire - see
// apply_pending_config().  USERROW already holds the new value, so we just
// re-read on apply.
#define APPLY_BAUD   0x01u
#define APPLY_SLAVE  0x02u
static volatile uint8_t  pending_apply_mask = 0;


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

// Raw-command staging lives in dali_sync.c (raw_cmd_addr / raw_cmd_data and
// the 0x015D state machine in g_regs.raw_cmd_result).  Cache invalidation is
// handled centrally in issue_dali_command for every DALI source — there is
// no separate classify_raw_cmd step.  Fade tracking is a natural consequence
// of SYNC_READ_LEVEL being set by the unified invalidation path.

// ---- FC04 - read input registers ----------------------------------------

// FC04 input registers (read-only) after subtask 2:
//   0x0100-0x010F  group_status                     (16 regs)
//   0x0150         scan_status
//   0x0151-0x0154  present bitmap                   (4 regs)
//   0x0155-0x0158  fading bitmap                    (4 regs)
//   0x0159-0x015C  dirty bitmap (read-clears)       (4 regs)
//   0x015D         async raw command result
//   0x0200         version block (special-cased above; count must be 16)
//
// The per-ballast banks (0x0000-0x00FF) and group membership masks
// (0x0110-0x014F) used to live here but have moved to FC03/FC06/FC10 holding
// registers - this is an intentional breaking change confirmed by the user.
// Reads of those addresses via FC04 now return ILLEGAL_ADDRESS.
//
// addr and count come off the wire as 16-bit values; promoting the sum to
// uint32_t prevents wrap (e.g. addr=0xFFFF, count=1 -> 0) from sneaking
// through the upper-bound check.
static bool fc04_range_allowed(uint16_t addr, uint8_t count) {
    uint32_t end = (uint32_t)addr + count;  // exclusive
    if (addr >= 0x0100u && end <= 0x0110u) return true;  // group_status
    if (addr >= 0x0150u && end <= 0x015Eu) return true;  // scan/present/fading/dirty/raw_result
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

    if (count == 0 || count > MAX_REG_RD) {
        send_error(MB_ERR_ILLEGAL_VALUE);
        return;
    }
    if (!fc04_range_allowed(addr, (uint8_t)count)) {
        send_error(MB_ERR_ILLEGAL_ADDRESS);
        return;
    }

    buf[2] = (uint8_t)(count * 2u);
    regs_read_input_bulk(addr, (uint8_t)count, buf + 3);

    // If the read range covers any dirty-flag words, atomically read-and-clear
    // each one and OVERWRITE the corresponding bytes in buf with the captured
    // value.  Without this overwrite, an ISR setting a new dirty bit between
    // the bulk-read above and the clear below would: (1) be sent to the
    // master as 0 in the bulk-read snapshot, then (2) be wiped by the clear
    // before the next FC04 - silently losing the update.  The atomic
    // read-and-clear inside regs_read_clear_dirty captures the up-to-the-
    // moment state, which we then ship to the master.
    if (addr + count > FC04_DIRTY_BASE && addr <= FC04_DIRTY_END) {
        uint16_t d_start = (addr < FC04_DIRTY_BASE) ? FC04_DIRTY_BASE : addr;
        uint16_t d_end   = (addr + count - 1u > FC04_DIRTY_END) ? FC04_DIRTY_END : (addr + count - 1u);
        for (uint16_t a = d_start; a <= d_end; a++) {
            uint16_t fresh   = regs_read_clear_dirty((uint8_t)(a - FC04_DIRTY_BASE));
            uint16_t buf_off = (uint16_t)(a - addr) * 2u;
            buf[3 + buf_off]      = (uint8_t)(fresh >> 8);
            buf[3 + buf_off + 1u] = (uint8_t)fresh;
        }
    }

    // 0x015D raw-cmd-result: read-clears semantics on TERMINAL values
    // (high byte 0x01 NAK / 0x02 OK / 0x03 ERR).  Pending (0xFF) and idle
    // (0x00) stay put so the master can poll without losing state.  The
    // capture must be atomic w.r.t. the ISR completion handler that writes
    // raw_cmd_result, so disable TCB0 + ATOMIC_BLOCK during the snapshot.
    if (addr <= FC04_RAW_RESULT && addr + count > FC04_RAW_RESULT) {
        uint16_t snap;
        uint8_t old_tcb = TCB0.INTCTRL;
        TCB0.INTCTRL = 0;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            snap = g_regs.raw_cmd_result;
            uint8_t hi = (uint8_t)(snap >> 8);
            if (hi == 0x01u || hi == 0x02u || hi == 0x03u)
                g_regs.raw_cmd_result = RAW_RESULT_IDLE;
        }
        TCB0.INTCTRL = old_tcb;
        uint16_t buf_off = (uint16_t)(FC04_RAW_RESULT - addr) * 2u;
        buf[3 + buf_off]      = (uint8_t)(snap >> 8);
        buf[3 + buf_off + 1u] = (uint8_t)snap;
    }

    start_tx(3 + buf[2]);
}

// ---- FC03 - read holding registers --------------------------------------

// Handle FC03 (Read Holding Registers).  Returns an exception if any address in
// the requested range is not in the holding register map.
static void process_fc03(void) {
    uint16_t addr  = ((uint16_t)buf[2] << 8) | buf[3];
    uint16_t count = ((uint16_t)buf[4] << 8) | buf[5];

    if (count == 0 || count > MAX_REG_RD) { send_error(MB_ERR_ILLEGAL_VALUE); return; }

    buf[2] = (uint8_t)(count * 2u);
    uint8_t *response_ptr = buf + 3;
    // Iterate by offset, not `reg_addr < addr + count`: with addr=0xFFFF and count=2 the
    // upper bound wraps to 0x0001 in uint16_t and the loop body never runs, but buf[2] has
    // already been set to 4 - start_tx would then ship 4 bytes of stale buffer.  Iterating
    // by index keeps the iteration count exact, and any wrapped reg_addr that doesn't map
    // to a real holding register fails regs_read_holding and triggers ILLEGAL_ADDRESS.
    for (uint16_t i = 0; i < count; i++) {
        uint16_t reg_addr = (uint16_t)(addr + i);
        uint16_t reg_value = 0;
        if (!regs_read_holding(reg_addr, &reg_value)) {
            send_error(MB_ERR_ILLEGAL_ADDRESS);
            return;
        }
        *response_ptr++ = (uint8_t)(reg_value >> 8);
        *response_ptr++ = (uint8_t)reg_value;
    }
    start_tx(3 + buf[2]);
}

// ---- FC06 / FC10 - write register(s) -----------------------------------

// Returns 0 on success, or a Modbus error code if the command cannot be dispatched.
//
// After subtask 5 the only registers that map to a DALI command issued from
// here are: MBREG_RAW_CMD / MBREG_RAW_CMD_TWICE (the raw command channel),
// MBREG_SET_DTR0/1/2 (control regs that load DTR0..2 with master-supplied
// values), and MBREG_SCAN_TRIGGER (sets SYNC_READ_FULL on all ballasts).
// Per-ballast bank writes (0x0000-0x00FF) and group register writes
// (0x0110-0x014F) are handled directly by regs_write_holding via the
// SYNC_WRITE_* / group_write paths and do not reach this function.
static mb_err_t dispatch_action_reg(uint16_t mb_reg, uint16_t val) {
    if (mb_reg == MBREG_RAW_CMD || mb_reg == MBREG_RAW_CMD_TWICE) {
        // Raw DALI: [15:8]=first frame byte, [7:0]=second frame byte.
        // The two bytes are sent as-is; they can encode any valid DALI forward
        // frame (addressed, group, broadcast, or special command).  The
        // unified issue_dali_command performs cache invalidation as a
        // property of the command bytes — no separate classification here.
        uint8_t sends = (mb_reg == MBREG_RAW_CMD_TWICE) ? 2u : 1u;
        if (!dali_sync_arm_raw((uint8_t)(val >> 8), (uint8_t)val, sends))
            return MB_ERR_DEVICE_BUSY;
        return 0;
    }
    if (mb_reg == MBREG_SCAN_TRIGGER) {
        // Set SYNC_READ_FULL on all 64 ballasts.  The work selector
        // services them one at a time via issue_dali_command.
        if (val == 0x0001) {
            for (uint8_t i = 0; i < DALI_MAX_DEVICES; i++)
                sync_flags_set(i, SYNC_READ_FULL);
        }
        return 0;
    }
    if (mb_reg == MBREG_SET_DTR0 || mb_reg == MBREG_SET_DTR1 || mb_reg == MBREG_SET_DTR2) {
        // SET_DTR0/1/2 = special DALI commands (no target ballast).
        // Address byte is the DTR opcode; data byte is the value.
        uint8_t dtr_addr = (mb_reg == MBREG_SET_DTR0) ? DALI_ADDR_SET_DTR0 :
                           (mb_reg == MBREG_SET_DTR1) ? DALI_ADDR_SET_DTR1 :
                                                        DALI_ADDR_SET_DTR2;
        if (!dali_sync_arm_raw(dtr_addr, (uint8_t)val, 1u))
            return MB_ERR_DEVICE_BUSY;
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
        // BUSY race only for group-register writes where the arming
        // failed against an in-flight write; everything else is genuine
        // ILLEGAL_ADDRESS.
        if (addr >= 0x0110u && addr <= 0x014Fu && dali_sync_group_busy())
            send_error(MB_ERR_DEVICE_BUSY);
        else
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

    // Pre-flight: refuse if more than one register in the FC10 range
    // is a raw-arming target (raw / DAPC / action / DTR / coil) and the
    // raw channel is busy.  dali_sync_arm_raw enforces single-command
    // semantics, so we just need to refuse outright if the master
    // bunched several action-style registers into one FC10 — only the
    // first would be staged.
    {
        uint8_t arm_targets = 0;
        for (uint16_t i = 0; i < count; i++) {
            uint16_t r = addr + i;
            if (r >= MBREG_RAW_CMD && r <= MBREG_SET_DTR2)
                arm_targets++;
        }
        if (arm_targets > 1u) {
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
            if (r >= 0x0110u && r <= 0x014Fu && dali_sync_group_busy())
                send_error(MB_ERR_DEVICE_BUSY);
            else
                send_error(MB_ERR_ILLEGAL_ADDRESS);
            return;
        }
        note_runtime_config_write(r);
        mb_err_t err = dispatch_action_reg(r, reg_value);
        if (err) { send_error(err); return; }
    }
    // Standard FC10 response: slave addr, FC, start addr (2), count (2)
    start_tx(6);
}

// ---- Frame validation --------------------------------------------------

// Return true if the frame in buf has a correct CRC, valid length for its function code,
// and is addressed to this slave (or broadcast address 0).
static bool frame_is_valid(void) {
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
        USART0.BAUD     = s_baud_reg[bidx];
        s_silence_ticks = s_silence_threshold[bidx];
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
    s_silence_ticks = s_silence_threshold[bidx];
    // Arm the initial 3.5-char silence countdown.  modbus_tick will set
    // initial_silence_observed when it expires without an intervening byte.
    silence_ticks = s_silence_ticks;

    PORTMUX.CTRLB          = PORTMUX_USART0_ALTERNATE_gc;
    RS485_PORT.PIN1CTRL    = PORT_PULLUPEN_bm;
    RS485_PORT.OUTSET      = RS485_TX_bm;
    RS485_PORT.OUTCLR      = RS485_DE_bm;
    RS485_PORT.DIRSET      = RS485_TX_bm | RS485_DE_bm;
    RS485_PORT.DIRCLR      = RS485_RX_bm;

    USART0.BAUD  = s_baud_reg[bidx];
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
            if (frame_is_valid()) {
                frame_ready = true;
                USART0.CTRLA = USART_RS485_EXT_gc;  // Disable RX ISRs during processing
            } else {
                return_to_rx();
            }
        }
    }
}

void modbus_process(void) {
    if (!frame_ready) return;

    // No explicit response delay needed.  modbus_tick gates frame_ready on
    // silence_ticks reaching 0, which is sized to ≥3.5 char-times per
    // Modbus §2.5.1.1.  By the time we get here the inter-frame silence
    // has already elapsed since the master's last byte; the response can
    // go on the wire immediately.

    no_response = (buf[0] == 0);

    // FC01/FC05/FC0F (coils) were retired in subtask 5.  Frames using those
    // function codes survive frame_is_valid (size check still recognises
    // them) and reach this switch, where the default branch returns
    // ILLEGAL_FUNCTION — the standard Modbus reply for unsupported FCs.
    switch (buf[1]) {
        case FC_READ_INPUT:        process_fc04(); break;
        case FC_READ_HOLDING:      process_fc03(); break;
        case FC_WRITE_SINGLE_REG:  process_fc06(); break;
        case FC_WRITE_MULTI_REG:   process_fc10(); break;
        default:                   send_error(MB_ERR_ILLEGAL_FUNCTION); break;
    }

    if (no_response) {
        no_response = false;
        // Broadcast: no response was sent, so apply the new config now (no
        // outgoing frame to wait for).
        apply_pending_config();
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
    return_to_rx();
}

ISR(USART0_RXC_vect) {
    modbus_idle    = false;
    silence_ticks  = s_silence_ticks;
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
        uint8_t val = USART0.RXDATAL;
        if (rx_sz < MAX_PKT_SZ) {
            *rx_ptr++ = val;
            uint8_t idx = val ^ (uint8_t)rx_crc;
            rx_crc = (rx_crc >> 8) ^ pgm_read_word(&crc_table[idx]);
            rx_sz++;

            if (frame_is_valid()) {
                silence_ticks = 0;
                frame_ready   = true;
                USART0.CTRLA  = USART_RS485_EXT_gc;
            }
        }
    }
}
