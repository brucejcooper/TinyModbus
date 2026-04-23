#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <util/atomic.h>
#include <util/delay.h>

#include "modbus.h"
#include "registers.h"
#include "cmd_queue.h"
#include "dali.h"
#include "bg_tasks.h"

// ---- Modbus constants -----------------------------------------------

#define USART0_BAUD_RATE(B) ((float)(F_CPU * 64 / (16 * (float)(B))) + 0.5f)

// Precomputed USART BAUD register values for each supported baud rate.
// Indexed by g_regs.baud_index: 0=9600, 1=19200, 2=38400.
static const uint16_t s_baud_reg[] = {
    (uint16_t)USART0_BAUD_RATE(9600),
    (uint16_t)USART0_BAUD_RATE(19200),
    (uint16_t)USART0_BAUD_RATE(38400),
};

// 3.5-character silence threshold in TCA0 ticks (~416 µs each), per baud rate.
// 9600: 3.5×10/9600 = 3.65 ms → 9 ticks; 19200 → 5; 38400 → 3.
static const uint8_t  s_silence_threshold[] = { 9, 5, 3 };
static          uint8_t  s_silence_ticks;

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

// ---- Dispatch table for action registers 0x1003–0x100A -----------------
// Write value low byte = DALI address byte; high byte ignored.

typedef struct {
    uint16_t mb_reg;
    uint8_t  dali_cmd;
    bool     marks_fading;
} action_reg_t;

static const action_reg_t action_regs[] PROGMEM = {
    { MBREG_GOTO_LAST_ACTIVE, DALI_CMD_GOTO_LAST_ACTIVE, true },
    { MBREG_OFF,              DALI_CMD_OFF,              true },
    { MBREG_UP,               DALI_CMD_UP,               true },
    { MBREG_DOWN,             DALI_CMD_DOWN,             true },
    { MBREG_STEP_UP,          DALI_CMD_STEP_UP,          true },
    { MBREG_STEP_DOWN,        DALI_CMD_STEP_DOWN,        true },
    { MBREG_RECALL_MAX,       DALI_CMD_RECALL_MAX,       true },
    { MBREG_RECALL_MIN,       DALI_CMD_RECALL_MIN,       true },
};
#define NUM_ACTION_REGS (sizeof(action_regs) / sizeof(action_regs[0]))

// ---- Raw command post-action (register update driven by command bytes) -------
//
// Decoded once when the raw command is queued; consumed by the completion callback.
// Safe to use as a static because only one raw command can be in flight at a time
// (enforced by the RAW_RESULT_PENDING busy-check in dispatch_action_reg).

static struct {
    uint8_t      ballast_idx;   // 0xFF = no follow-up needed
    uint8_t      query_cmd;     // DALI query to issue (0 when fade poll)
    reg_target_t query_target;  // REG_NONE when fade poll is the action
} s_raw_post;

// Decode the forward frame (addr_byte, cmd_byte) and populate s_raw_post.
// Group/broadcast addresses: no per-ballast update possible → ballast_idx 0xFF.
// Short-address DAPC or modification cmds (0x00-0x0A): fade poll.
// Short-address setting cmds (0x2A-0x2F): queue a follow-up query.
static void classify_raw_cmd(uint8_t addr_byte, uint8_t cmd_byte) {
    s_raw_post.ballast_idx  = 0xFF;
    s_raw_post.query_cmd    = 0;
    s_raw_post.query_target = REG_NONE;

    if (addr_byte & 0x80u) return;  // group or broadcast

    uint8_t bidx = (addr_byte >> 1) & 0x3Fu;

    if (!(addr_byte & DALI_CMD_FLAG)) {
        // DAPC: arc level change / fade
        s_raw_post.ballast_idx = bidx;
        return;
    }

    if (cmd_byte <= DALI_CMD_GOTO_LAST_ACTIVE) {
        // Modification commands 0x00-0x0A: affect brightness/status
        s_raw_post.ballast_idx = bidx;
        return;
    }

    // Setting commands: queue a read-back query after completion
    uint8_t      qcmd    = 0;
    reg_target_t qtarget = REG_NONE;
    switch (cmd_byte) {
        case DALI_CMD_SET_MAX_LEVEL:      qcmd = DALI_CMD_QUERY_MAX_LEVEL;      qtarget = REG_MAX;       break;
        case DALI_CMD_SET_MIN_LEVEL:      qcmd = DALI_CMD_QUERY_MIN_LEVEL;      qtarget = REG_MIN;       break;
        case DALI_CMD_SET_SYS_FAIL_LEVEL: qcmd = DALI_CMD_QUERY_FAIL_LEVEL;     qtarget = REG_SYS_FAIL;  break;
        case DALI_CMD_SET_POWER_ON_LEVEL: qcmd = DALI_CMD_QUERY_POWER_ON_LEVEL; qtarget = REG_POWER_ON;  break;
        case DALI_CMD_SET_FADE_TIME:      qcmd = DALI_CMD_QUERY_FADE_TIME;      qtarget = REG_FADE_TIME; break;
        case DALI_CMD_SET_FADE_RATE:      qcmd = DALI_CMD_QUERY_FADE_RATE;      qtarget = REG_FADE_RATE; break;
        default: return;
    }
    s_raw_post.ballast_idx  = bidx;
    s_raw_post.query_cmd    = qcmd;
    s_raw_post.query_target = qtarget;
}

// Execute the follow-up action determined by classify_raw_cmd().
// Called from both ISR callback context (cmdq_kick is a no-op; state machine picks
// up queued commands naturally) and main-loop context (cmdq_kick starts transmission).
static void raw_do_post_action(void) {
    if (s_raw_post.ballast_idx == 0xFF) return;
    uint8_t ballast = s_raw_post.ballast_idx;
    if (s_raw_post.query_target == REG_NONE) {
        bgtasks_start_fade_poll(ballast);
    } else {
        uint8_t dali_addr = DALI_SHORT_ADDR(ballast) | DALI_CMD_FLAG;
        if (cmdq_push_query(dali_addr, s_raw_post.query_cmd, ballast, s_raw_post.query_target) == SYS_OK)
            cmdq_kick();
    }
}

// ---- Callbacks for async raw DALI result (registers 0x1000 / 0x1001) ----

static void raw_cmd_single_done(dali_response_type_t type, uint8_t ballast_idx, uint8_t response) {
    (void)ballast_idx;
    if (type == DALI_RESPONSE_OK)
        g_regs.raw_cmd_result = RAW_RESULT_OK(response);
    else if (type == DALI_RESPONSE_NAK)
        g_regs.raw_cmd_result = RAW_RESULT_NAK;
    else
        g_regs.raw_cmd_result = RAW_RESULT_ERR((uint8_t)(type - DALI_RESPONSE_MALFORMED) + 1u);

    if (type == DALI_RESPONSE_OK || type == DALI_RESPONSE_NAK)
        raw_do_post_action();
}

static void raw_cmd_twice_done(dali_response_type_t type, uint8_t ballast_idx, uint8_t response) {
    (void)ballast_idx; (void)response;
    // Send-twice commands don't return a response; NAK on completion is normal
    if (type == DALI_RESPONSE_NAK)
        g_regs.raw_cmd_result = RAW_RESULT_NAK;
    else
        g_regs.raw_cmd_result = RAW_RESULT_ERR((uint8_t)(type - DALI_RESPONSE_MALFORMED) + 1u);

    if (type == DALI_RESPONSE_NAK)
        raw_do_post_action();
}

// ---- FC04 — read input registers ----------------------------------------

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

    if (count == 0 || count > MAX_REG_RD || addr + count > FC04_MAX_ADDR + 1u) {
        send_error(MB_ERR_ILLEGAL_ADDRESS);
        return;
    }

    buf[2] = (uint8_t)(count * 2u);
    regs_read_input_bulk(addr, (uint8_t)count, buf + 3);

    // Clear dirty flags after copying so the master sees the actual bits, not zeros
    if (addr + count > FC04_DIRTY_BASE && addr <= FC04_DIRTY_END) {
        uint16_t d_start = (addr < FC04_DIRTY_BASE) ? FC04_DIRTY_BASE : addr;
        uint16_t d_end   = (addr + count - 1u > FC04_DIRTY_END) ? FC04_DIRTY_END : (addr + count - 1u);
        for (uint16_t a = d_start; a <= d_end; a++)
            regs_read_clear_dirty((uint8_t)(a - FC04_DIRTY_BASE));
    }

    start_tx(3 + buf[2]);
}

// ---- FC03 — read holding registers --------------------------------------

// Handle FC03 (Read Holding Registers).  Returns an exception if any address in
// the requested range is not in the holding register map.
static void process_fc03(void) {
    uint16_t addr  = ((uint16_t)buf[2] << 8) | buf[3];
    uint16_t count = ((uint16_t)buf[4] << 8) | buf[5];

    if (count == 0 || count > MAX_REG_RD) { send_error(MB_ERR_ILLEGAL_ADDRESS); return; }

    buf[2] = (uint8_t)(count * 2u);
    uint8_t *response_ptr = buf + 3;
    for (uint16_t reg_addr = addr; reg_addr < addr + count; reg_addr++) {
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

// ---- FC06 / FC10 — write register(s) -----------------------------------

// Returns 0 on success, or a Modbus error code if the command cannot be dispatched.
static mb_err_t dispatch_action_reg(uint16_t mb_reg, uint16_t val) {
    if (mb_reg == MBREG_RAW_CMD || mb_reg == MBREG_RAW_CMD_TWICE) {
        // Raw DALI: [15:8]=first frame byte, [7:0]=second frame byte.
        // The two bytes are sent as-is; they can encode any valid DALI forward frame
        // (addressed, group, broadcast, or special command).
        // MBREG_RAW_CMD = single send (waits for response);
        // MBREG_RAW_CMD_TWICE = send twice (no response expected).
        // Reject if a previous raw command is still pending.
        bool pending;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            pending = (g_regs.raw_cmd_result == RAW_RESULT_PENDING);
            if (!pending) g_regs.raw_cmd_result = RAW_RESULT_PENDING;
        }
        if (pending) return MB_ERR_DEVICE_BUSY;

        bool is_twice = (mb_reg == MBREG_RAW_CMD_TWICE);
        classify_raw_cmd((uint8_t)(val >> 8), (uint8_t)val);
        cmd_entry_t entry = {
            .type        = is_twice ? CMD_SEND_TWICE : CMD_QUERY,
            .dali_addr   = (uint8_t)(val >> 8),
            .dali_cmd    = (uint8_t)val,
            .ballast_idx = 0xFF,
            .reg_target  = REG_NONE,
            .repeat      = is_twice ? 2 : 1,
            .on_result   = is_twice ? raw_cmd_twice_done : raw_cmd_single_done,
        };
        if (cmdq_push(&entry) != SYS_OK) {
            g_regs.raw_cmd_result = RAW_RESULT_IDLE;
            return MB_ERR_DEVICE_BUSY;
        }
        cmdq_kick();
        return 0;
    }
    if (mb_reg == MBREG_DAPC) {
        // DAPC: [15:8]=DALI addr byte, [7:0]=arc level
        uint8_t dali_addr_byte = (uint8_t)(val >> 8);
        uint8_t arc_level      = (uint8_t)val;
        uint8_t ballast_idx    = (dali_addr_byte >> 1) & 0x3F;
        cmdq_push_arc(dali_addr_byte, arc_level, (dali_addr_byte & 0x80) ? 0xFF : ballast_idx);
        if (!(dali_addr_byte & 0x80))
            bgtasks_start_fade_poll(ballast_idx);
        cmdq_kick();
        return 0;
    }
    if (mb_reg == MBREG_SCAN_TRIGGER) {
        if (val == 0x0001) bgtasks_trigger_scan();
        return 0;
    }
    if (mb_reg == MBREG_SET_DTR0 || mb_reg == MBREG_SET_DTR1 || mb_reg == MBREG_SET_DTR2) {
        // Set DTR0/DTR1/DTR2: low byte of val is the DTR data byte.
        uint8_t dtr_addr = (mb_reg == MBREG_SET_DTR0) ? DALI_ADDR_SET_DTR0 :
                           (mb_reg == MBREG_SET_DTR1) ? DALI_ADDR_SET_DTR1 : DALI_ADDR_SET_DTR2;
        cmdq_push_cmd(dtr_addr, (uint8_t)val, 0xFF);
        cmdq_kick();
        return 0;
    }
    for (uint8_t i = 0; i < NUM_ACTION_REGS; i++) {
        uint16_t reg = pgm_read_word(&action_regs[i].mb_reg);
        if (reg == mb_reg) {
            uint8_t dali_cmd_byte  = pgm_read_byte(&action_regs[i].dali_cmd);
            uint8_t dali_addr_byte = (uint8_t)val;  // low byte = DALI addr byte
            uint8_t ballast_idx    = (dali_addr_byte >> 1) & 0x3F;
            bool    marks_fading   = pgm_read_byte(&action_regs[i].marks_fading);
            cmdq_push_cmd(dali_addr_byte, dali_cmd_byte, ballast_idx);
            if (marks_fading && !(dali_addr_byte & 0x80))
                bgtasks_start_fade_poll(ballast_idx);
            cmdq_kick();
            return 0;
        }
    }
    return 0;
}

// Handle FC06 (Write Single Register): validate, update register map, dispatch DALI action.
static void process_fc06(void) {
    uint16_t addr = ((uint16_t)buf[2] << 8) | buf[3];
    uint16_t val  = ((uint16_t)buf[4] << 8) | buf[5];

    if (!regs_write_holding(addr, val)) {
        send_error(MB_ERR_ILLEGAL_ADDRESS);
        return;
    }
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

    uint8_t *payload_ptr = buf + 7;
    for (uint16_t i = 0; i < count; i++) {
        uint16_t reg_value = ((uint16_t)payload_ptr[0] << 8) | payload_ptr[1];
        payload_ptr += 2;
        if (!regs_write_holding(addr + i, reg_value)) {
            send_error(MB_ERR_ILLEGAL_ADDRESS);
            return;
        }
        mb_err_t err = dispatch_action_reg(addr + i, reg_value);
        if (err) { send_error(err); return; }
    }
    // Standard FC10 response: slave addr, FC, start addr (2), count (2)
    start_tx(6);
}

// ---- FC01 / FC05 / FC0F — coils (ballast on/off state) ------------------
//
// Coil N maps to ballast N (0-63).  A coil reads as 1 when brightness > 0 and
// the ballast is present (brightness != 0xFF absent sentinel).
// Write ON  → GOTO_LAST_ACTIVE; Write OFF → OFF; both start fade polling.
// Write 0x5500 (non-normative toggle) checks the current brightness and
// inverts the state.

static void process_fc01(void) {
    uint16_t addr  = ((uint16_t)buf[2] << 8) | buf[3];
    uint16_t count = ((uint16_t)buf[4] << 8) | buf[5];

    if (count == 0 || addr >= 64u || addr + count > 64u) {
        send_error(MB_ERR_ILLEGAL_ADDRESS);
        return;
    }

    uint8_t nbytes = (uint8_t)((count + 7u) >> 3);
    buf[2] = nbytes;
    memset(buf + 3, 0, nbytes);
    for (uint16_t i = 0; i < count; i++) {
        uint8_t bidx   = (uint8_t)(addr + i);
        bool    present = (g_regs.present[bidx >> 4] >> (bidx & 0x0Fu)) & 1u;
        if (present && g_regs.block0[bidx].brightness > 0)
            buf[3 + (i >> 3)] |= (uint8_t)(1u << (i & 7u));
    }
    start_tx(3 + nbytes);
}

static void process_fc05(void) {
    uint16_t addr = ((uint16_t)buf[2] << 8) | buf[3];
    uint16_t val  = ((uint16_t)buf[4] << 8) | buf[5];

    if (addr >= 64u) { send_error(MB_ERR_ILLEGAL_ADDRESS); return; }
    if (val != 0xFF00u && val != 0x0000u && val != 0x5500u) {
        send_error(MB_ERR_ILLEGAL_VALUE);
        return;
    }

    bool on;
    if (val == 0x5500u) {
        uint8_t bright = g_regs.block0[addr].brightness;
        on = (bright == 0);  // currently off → turn on
    } else {
        on = (val == 0xFF00u);
    }

    uint8_t dali_addr = DALI_SHORT_ADDR((uint8_t)addr) | DALI_CMD_FLAG;
    cmdq_push_cmd(dali_addr, on ? DALI_CMD_GOTO_LAST_ACTIVE : DALI_CMD_OFF, (uint8_t)addr);
    cmdq_kick();
    bgtasks_start_fade_poll((uint8_t)addr);
    start_tx(6);  // echo request
}

static void process_fc0f(void) {
    uint16_t addr   = ((uint16_t)buf[2] << 8) | buf[3];
    uint16_t count  = ((uint16_t)buf[4] << 8) | buf[5];
    uint8_t  nbytes = buf[6];

    if (count == 0 || addr >= 64u || addr + count > 64u ||
            nbytes != (uint8_t)((count + 7u) >> 3)) {
        send_error(MB_ERR_ILLEGAL_VALUE);
        return;
    }

    // Verify the queue has room for all commands before pushing any.
    // ISRs can only consume queue entries (not produce), so available space
    // can only increase from this point — checking once is sufficient.
    uint8_t avail;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { avail = (CMD_QUEUE_SIZE - 1u) - cmdq_count(); }
    if (count > avail) {
        send_error(MB_ERR_DEVICE_BUSY);
        return;
    }

    // Push all action commands first so fade-poll STATUS queries trail behind them.
    uint8_t *data = buf + 7;
    for (uint16_t i = 0; i < count; i++) {
        uint8_t bidx = (uint8_t)(addr + i);
        bool    on   = (data[i >> 3] >> (i & 7u)) & 1u;
        cmdq_push_cmd(DALI_SHORT_ADDR(bidx) | DALI_CMD_FLAG,
                      on ? DALI_CMD_GOTO_LAST_ACTIVE : DALI_CMD_OFF, bidx);
    }
    cmdq_kick();
    for (uint16_t i = 0; i < count; i++)
        bgtasks_start_fade_poll((uint8_t)(addr + i));

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
        case FC_WRITE_MULTI_REG:
            expected_size = (uint8_t)(9u + buf[6]); break;
        default: break;
    }
    return rx_sz == expected_size;
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

    uint8_t bidx = (g_regs.baud_index <= 2) ? g_regs.baud_index : 0;
    s_silence_ticks = s_silence_threshold[bidx];

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
    // The silence threshold is set (reset to non-zero) by the RXC ISR.
    if (silence_ticks && (--silence_ticks == 0)) {
        if (frame_is_valid()) {
            frame_ready = true;
            USART0.CTRLA = USART_RS485_EXT_gc;  // Disable RX ISRs during processing
        } else {
            return_to_rx();
        }
    }
}

void modbus_process(void) {
    if (!frame_ready) return;

    _delay_us(1500);  // Modbus turnaround guard

    no_response = (buf[0] == 0);

    switch (buf[1]) {
        case FC_READ_COILS:        process_fc01(); break;
        case FC_READ_INPUT:        process_fc04(); break;
        case FC_READ_HOLDING:      process_fc03(); break;
        case FC_WRITE_SINGLE_COIL: process_fc05(); break;
        case FC_WRITE_SINGLE_REG:  process_fc06(); break;
        case FC_WRITE_COILS:       process_fc0f(); break;
        case FC_WRITE_MULTI_REG:   process_fc10(); break;
        default:                   send_error(MB_ERR_ILLEGAL_FUNCTION); break;
    }

    if (no_response) {
        no_response = false;
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
    return_to_rx();
}

ISR(USART0_RXC_vect) {
    modbus_idle    = false;
    silence_ticks  = s_silence_ticks;
    USART0.STATUS  = USART_RXSIF_bm;

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
