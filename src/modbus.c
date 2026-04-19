#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
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

#define MODBUS_BAUD  9600
#define USART0_BAUD_RATE(B) ((float)(F_CPU * 64 / (16 * (float)(B))) + 0.5f)

#define FC_READ_COILS        0x01
#define FC_READ_HOLDING      0x03
#define FC_READ_INPUT        0x04
#define FC_WRITE_SINGLE_COIL 0x05
#define FC_WRITE_SINGLE_REG  0x06
#define FC_WRITE_COILS       0x0F
#define FC_WRITE_MULTI_REG   0x10
#define FC_CUSTOM_DALI       0x44   // synchronous raw DALI command (legacy)

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

#define MAX_PKT_SZ  (64 * 2 + 8)

static uint8_t  slave_addr = 1;
static uint8_t  buf[MAX_PKT_SZ];
static volatile uint8_t *rx_ptr   = buf;
static volatile uint8_t  rx_sz    = 0;
static volatile uint16_t rx_crc   = 0xFFFF;
static volatile uint8_t  silence_ticks = 0;
static volatile bool     frame_ready   = false;
static          uint8_t  tx_sz    = 0;
volatile bool            modbus_idle   = true;

// Version string for FC04 reads at addr 0 (legacy behaviour)
static char ver_str[64];

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

static uint16_t crc16(const uint8_t *data, uint8_t len) {
    uint16_t crc = 0xFFFF;
    while (len--) {
        uint8_t idx = *data++ ^ (uint8_t)crc;
        crc = (crc >> 8) ^ pgm_read_word(&crc_table[idx]);
    }
    return crc;
}

static void append_crc(uint8_t len) {
    uint16_t crc = crc16(buf, len);
    buf[len]     = (uint8_t)crc;
    buf[len + 1] = (uint8_t)(crc >> 8);
    tx_sz        = len + 2;
}

// ---- TX helpers --------------------------------------------------------

static void start_tx(uint8_t payload_len) {
    append_crc(payload_len);
    rx_ptr = buf;    // reuse buf pointer for TX
    USART0.STATUS = USART_TXCIF_bm;
    USART0.CTRLA  = USART_RS485_EXT_gc | USART_DREIE_bm | USART_TXCIE_bm;
}

static void send_error(mb_err_t err) {
    buf[1] |= 0x80;
    buf[2]  = (uint8_t)err;
    start_tx(3);
}

// ---- Dispatch table for action registers 0x1002–0x1009 -----------------

typedef struct {
    uint16_t mb_reg;
    uint8_t  dali_cmd;
    bool     marks_fading;
} action_reg_t;

static const action_reg_t action_regs[] PROGMEM = {
    { 0x1002, DALI_CMD_GOTO_LAST_ACTIVE, true  },
    { 0x1003, DALI_CMD_OFF,              false },
    { 0x1004, DALI_CMD_UP,               false },
    { 0x1005, DALI_CMD_DOWN,             false },
    { 0x1006, DALI_CMD_STEP_UP,          false },
    { 0x1007, DALI_CMD_STEP_DOWN,        false },
    { 0x1008, DALI_CMD_RECALL_MAX,       false },
    { 0x1009, DALI_CMD_RECALL_MIN,       false },
};
#define NUM_ACTION_REGS (sizeof(action_regs) / sizeof(action_regs[0]))

// ---- FC04 — read input registers ----------------------------------------

static void process_fc04(void) {
    uint16_t addr  = ((uint16_t)buf[2] << 8) | buf[3];
    uint16_t count = ((uint16_t)buf[4] << 8) | buf[5];

    // Legacy: version string at 0x0200-0x021F (32 registers = 64 bytes)
    if (addr >= 0x0200 && addr + count <= 0x0220u) {
        buf[2] = (uint8_t)(count * 2u);
        memcpy(buf + 3, ver_str + (addr - 0x0200u) * 2u, count * 2u);
        start_tx(3 + buf[2]);
        return;
    }

    if (count == 0 || addr + count > FC04_MAX_ADDR + 1u) {
        send_error(MB_ERR_ILLEGAL_ADDRESS);
        return;
    }

    // Atomically read-and-clear dirty flags if the read spans the dirty range
    if (addr + count > 0x0159u && addr <= 0x015Cu) {
        uint16_t d_start = (addr    < 0x0159u) ? 0x0159u : addr;
        uint16_t d_end   = (addr + count - 1u > 0x015Cu) ? 0x015Cu : (addr + count - 1u);
        for (uint16_t a = d_start; a <= d_end; a++)
            regs_read_clear_dirty((uint8_t)(a - 0x0159u));
    }

    buf[2] = (uint8_t)(count * 2u);
    regs_read_input_bulk(addr, (uint8_t)count, buf + 3);
    start_tx(3 + buf[2]);
}

// ---- FC03 — read holding registers --------------------------------------

static void process_fc03(void) {
    uint16_t addr  = ((uint16_t)buf[2] << 8) | buf[3];
    uint16_t count = ((uint16_t)buf[4] << 8) | buf[5];

    if (count == 0 || count > 64) { send_error(MB_ERR_ILLEGAL_ADDRESS); return; }

    buf[2] = (uint8_t)(count * 2u);
    uint8_t *dst = buf + 3;
    for (uint16_t a = addr; a < addr + count; a++) {
        uint16_t val = 0;
        if (!regs_read_holding(a, &val)) {
            send_error(MB_ERR_ILLEGAL_ADDRESS);
            return;
        }
        *dst++ = (uint8_t)(val >> 8);
        *dst++ = (uint8_t)val;
    }
    start_tx(3 + buf[2]);
}

// ---- FC06 / FC10 — write register(s) -----------------------------------

static void dispatch_action_reg(uint16_t mb_reg, uint16_t val) {
    if (mb_reg == 0x1000) {
        // Raw DALI command: [15:8]=addr byte, [7:0]=cmd byte
        uint8_t da = (uint8_t)(val >> 8);
        uint8_t dc = (uint8_t)val;
        cmdq_push_cmd(da, dc, 0xFF);
        cmdq_kick();
        return;
    }
    if (mb_reg == 0x1001) {
        // DAPC: [15:8]=DALI addr byte, [7:0]=level
        uint8_t da  = (uint8_t)(val >> 8);
        uint8_t lvl = (uint8_t)val;
        uint8_t idx = (da >> 1) & 0x3F;
        cmdq_push_arc(da, lvl, (da & 0x80) ? 0xFF : idx);
        // Mark fading so the fade poller tracks it
        if (idx < DALI_MAX_DEVICES)
            g_regs.fading[idx >> 4] |= (uint16_t)1u << (idx & 0x0F);
        cmdq_kick();
        return;
    }
    if (mb_reg == 0x1010) {
        if (val == 0x0001) bgtasks_trigger_scan();
        return;
    }
    for (uint8_t i = 0; i < NUM_ACTION_REGS; i++) {
        uint16_t reg = pgm_read_word(&action_regs[i].mb_reg);
        if (reg == mb_reg) {
            uint8_t dc    = pgm_read_byte(&action_regs[i].dali_cmd);
            uint8_t da    = (uint8_t)val;  // addr byte in low byte for these regs
            uint8_t idx   = (da >> 1) & 0x3F;
            bool    fade  = pgm_read_byte(&action_regs[i].marks_fading);
            cmdq_push_cmd(da, dc, idx);
            if (fade && idx < DALI_MAX_DEVICES)
                g_regs.fading[idx >> 4] |= (uint16_t)1u << (idx & 0x0F);
            cmdq_kick();
            return;
        }
    }
}

static void process_fc06(void) {
    uint16_t addr = ((uint16_t)buf[2] << 8) | buf[3];
    uint16_t val  = ((uint16_t)buf[4] << 8) | buf[5];

    if (!regs_write_holding(addr, val)) {
        send_error(MB_ERR_ILLEGAL_ADDRESS);
        return;
    }
    dispatch_action_reg(addr, val);
    // Echo the request back (standard FC06 response)
    start_tx(6);
}

static void process_fc10(void) {
    uint16_t addr  = ((uint16_t)buf[2] << 8) | buf[3];
    uint16_t count = ((uint16_t)buf[4] << 8) | buf[5];
    uint8_t  nbytes = buf[6];

    if (count == 0 || nbytes != count * 2u) {
        send_error(MB_ERR_ILLEGAL_VALUE);
        return;
    }

    uint8_t *src = buf + 7;
    for (uint16_t i = 0; i < count; i++) {
        uint16_t val = ((uint16_t)src[0] << 8) | src[1];
        src += 2;
        if (!regs_write_holding(addr + i, val)) {
            send_error(MB_ERR_ILLEGAL_ADDRESS);
            return;
        }
        dispatch_action_reg(addr + i, val);
    }
    // Standard FC10 response: slave addr, FC, start addr (2), count (2)
    buf[4] = buf[4]; buf[5] = buf[5];  // already in place
    start_tx(6);
}

// ---- FC44 — synchronous raw DALI command (legacy, blocking) ------------

static mb_err_t dali_err_to_mb(dali_response_type_t t) {
    switch (t) {
        case DALI_RESPONSE_NAK:             return MB_ERR_TARGET_FAILED;
        case DALI_RESPONSE_MALFORMED:
        case DALI_RESPONSE_PULSE_TOO_SHORT:
        case DALI_RESPONSE_PULSE_IN_MIDDLE:
        case DALI_RESPONSE_PULSE_TOO_LONG:
            return (mb_err_t)(MB_ERR_DALI_PULSE_INVALID +
                              (uint8_t)(t - DALI_RESPONSE_MALFORMED));
        default:                            return MB_ERR_DEVICE_FAILURE;
    }
}

static void process_fc44(void) {
    uint16_t dali_cmd = ((uint16_t)buf[2] << 8) | buf[3];
    uint8_t  repeat   = buf[4];
    if (repeat > 2) { send_error(MB_ERR_ILLEGAL_VALUE); return; }
    if (repeat == 0) repeat = 1;

    cmd_entry_t e = {
        .type        = (repeat == 2) ? CMD_SEND_TWICE : CMD_QUERY,
        .dali_addr   = (uint8_t)(dali_cmd >> 8),
        .dali_cmd    = (uint8_t)dali_cmd,
        .ballast_idx = 0xFF,
        .reg_target  = REG_RAW_RESPONSE,
        .repeat      = repeat,
        .on_result   = NULL,
    };

    g_raw_resp_type = DALI_RESPONSE_PENDING;

    if (cmdq_push(&e) != SYS_OK) {
        send_error(MB_ERR_DEVICE_BUSY);
        return;
    }
    cmdq_kick();

    // Busy-wait — TCA0 ISRs continue firing so the DALI state machine runs.
    for (uint16_t t = 300u; t > 0u && g_raw_resp_type == DALI_RESPONSE_PENDING; t--)
        _delay_ms(1);

    if (g_raw_resp_type == DALI_RESPONSE_PENDING)
        g_raw_resp_type = DALI_RESPONSE_NAK;

    if (g_raw_resp_type == DALI_RESPONSE_OK) {
        buf[2] = g_raw_resp_byte;
        start_tx(3);
    } else {
        send_error(dali_err_to_mb(g_raw_resp_type));
    }
}

// ---- Frame validation --------------------------------------------------

static bool frame_is_valid(void) {
    if (rx_crc != 0 || rx_sz < 5) return false;
    if (buf[0] != slave_addr && buf[0] != 0) return false;

    uint8_t fc  = buf[1];
    uint8_t exp = 255;
    switch (fc) {
        case FC_READ_COILS:
        case FC_READ_HOLDING:
        case FC_READ_INPUT:
        case FC_WRITE_SINGLE_COIL:
        case FC_WRITE_SINGLE_REG:
            exp = 8; break;
        case FC_WRITE_COILS:
        case FC_WRITE_MULTI_REG:
            exp = (uint8_t)(9u + buf[6]); break;
        case FC_CUSTOM_DALI:
            exp = 7; break;
        default: break;
    }
    return rx_sz == exp;
}

// ---- RX ring-buffer reset -----------------------------------------------

static void rx_reset(void) {
    rx_ptr  = buf;
    rx_crc  = 0xFFFF;
    rx_sz   = 0;
    frame_ready = false;
}

// ---- Public API ---------------------------------------------------------

void modbus_init(uint8_t addr) {
    slave_addr = addr;
    rx_reset();
    modbus_idle = true;

    memset(ver_str, 0, sizeof(ver_str));
    snprintf(ver_str, sizeof(ver_str),
             "Mechination DALI v%d.%d ID:%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x",
             FW_VER_MAJOR, FW_VER_MINOR,
             SIGROW_SERNUM0, SIGROW_SERNUM1, SIGROW_SERNUM2, SIGROW_SERNUM3,
             SIGROW_SERNUM4, SIGROW_SERNUM5, SIGROW_SERNUM6, SIGROW_SERNUM7,
             SIGROW_SERNUM8, SIGROW_SERNUM9);

    PORTMUX.CTRLB          = PORTMUX_USART0_ALTERNATE_gc;
    RS485_PORT.PIN7CTRL    = PORT_PULLUPEN_bm;
    RS485_PORT.PIN1CTRL    = PORT_PULLUPEN_bm;
    RS485_PORT.OUTSET      = RS485_TX_bm;
    RS485_PORT.OUTCLR      = RS485_DE_bm;
    RS485_PORT.DIRSET      = RS485_TX_bm | RS485_DE_bm;
    RS485_PORT.DIRCLR      = RS485_RX_bm;

    USART0.BAUD  = (uint16_t)USART0_BAUD_RATE(MODBUS_BAUD);
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
            rx_reset();
            modbus_idle = true;
        }
    }
}

void modbus_process(void) {
    if (!frame_ready) return;

    _delay_us(1500);  // Modbus turnaround guard

    switch (buf[1]) {
        case FC_READ_INPUT:        process_fc04(); break;
        case FC_READ_HOLDING:      process_fc03(); break;
        case FC_WRITE_SINGLE_REG:  process_fc06(); break;
        case FC_WRITE_MULTI_REG:   process_fc10(); break;
        case FC_CUSTOM_DALI:       process_fc44(); break;
        default:                   send_error(MB_ERR_ILLEGAL_FUNCTION); break;
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
    rx_reset();
    modbus_idle = true;
    USART0.STATUS = USART_RXSIF_bm | USART_TXCIF_bm;
    USART0.CTRLA  = USART_RS485_EXT_gc | USART_RXSIE_bm | USART_RXCIE_bm;
}

ISR(USART0_RXC_vect) {
    modbus_idle    = false;
    silence_ticks  = 4;  // ≈1.6 ms inter-character silence threshold
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
