#include "rs485.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <util/atomic.h>
#include <util/delay.h>

#include "dali.h"
#include "timers.h"

// Stringify the version macros set by the Makefile so the reported version tracks the build.
#define _STRINGIFY(x) #x
#define STRINGIFY(x) _STRINGIFY(x)
static const char verstr[] = "MechDALI v" STRINGIFY(FW_VER_MAJOR) "." STRINGIFY(FW_VER_MINOR) ":";
#define VERSTR_LEN (sizeof(verstr) - 1)


#define MODBUS_READ_COILS_CMD 0x01
#define MODBUS_READ_DISCRETE_INPUTS_CMD 0x02
#define MODBUS_READ_HOLDING_REGISTERS_CMD 0x03
#define MODBUS_READ_INPUT_REGISTERS_CMD 0x04
#define MODBUS_WRITE_SINGLE_COIL_CMD 0x05
#define MODBUS_WRITE_SINGLE_HOLDING_REGISTER_CMD 0x06
#define MODBUS_WRITE_COILS_CMD 0x0F
#define MODBUS_WRITE_REGISTERS_CMD 0x10
#define MODBUS_CUSTOM_EXEC_DALI_CMD 0x44
#define MODBUS_CUSTOM_START_PROCESS_CMD 0x45

typedef enum {
    MODBUS_ERR_ILLEGAL_FUNCTION = 0x01,
    MODBUS_ERR_ILLEGAL_ADDRESS = 0x02,
    MODBUS_ERR_ILLEGAL_VALUE = 0x03,
    MODBUS_ERR_SLAVE_DEVICE_FAILURE = 0x04,
    MODBUS_ERR_ACKNOWLEDGE = 0x05,
    MODBUS_ERR_SLAVE_DEVICE_BUSY = 0x06,
    MODBUS_ERR_NEGATIVE_ACKNOWLEDGE = 0x07,
    MODBUS_ERR_MEMORY_PARITY_ERROR = 0x08,
    MODBUS_ERR_GATEWAY_PATH_UNAVAILABLE = 0x0a,
    MODBUS_ERR_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND = 0x0b,
    MODBUS_ERR_DALI_RESPONSE_SHORT = 0x20,
    MODBUS_ERR_DALI_PULSE_WIDTH_INVALID = 0x30,
} modbus_err_t;

#define MODBUS_BAUD 9600
#define USART0_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5)

#define RS485_PORT PORTA
#define RS485_TX_bm PIN1_bm
#define RS485_RX_bm PIN2_bm
#define RS485_DE_bm PIN4_bm

// buf only needs to hold the largest non-streamed response. Read-holding-regs uses a streaming
// path that pulls data directly from dali_holding_regs in the DRE ISR, so it isn't bounded by buf.
// 70 bytes covers every other handler comfortably (sendInfo writes ~37 bytes; everything else is shorter).
#define MAX_PKT_SZ 70

static uint8_t deviceId = 1;

static volatile uint16_t crc = 0xFFFF;
static uint8_t buf[MAX_PKT_SZ];
static volatile uint8_t *ptr = buf;
static volatile uint8_t sz = 0;

// Streaming TX state for read-holding-regs responses. While streamWords > 0, the DRE ISR reads
// each register fresh from dali_holding_regs (one read per word, used for both bytes) and updates
// the CRC on the fly so the transmitted bytes always match the CRC even if a register is updated
// mid-stream. After the data drains, the final CRC bytes are parked in buf and sent from there.
static volatile uint16_t *streamSrc;
static volatile uint8_t streamWords = 0;
static volatile bool streamHighByte;
static volatile uint8_t streamLowByte;   // pending low byte of the current word
static volatile uint16_t streamCrc;      // running CRC over header + already-streamed data bytes

volatile bool modbus_idle = true;
static bool modbus_cmd_available = false;
// Set when inputIsValidModbusCommand accepts a broadcast (deviceId 0) write. sendResponse and
// sendError check this flag and skip the actual transmission — Modbus spec forbids slaves from
// responding to broadcasts; multiple slaves replying would collide on the bus.
static bool was_broadcast = false;

static inline char hexDigit(uint8_t n) {
    return n < 10 ? '0' + n : 'A' + (n - 10);
}

static const uint16_t crc_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241, 0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440, 0xCC01, 0x0CC0, 0x0D80,
    0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40, 0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841, 0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1,
    0xDA81, 0x1A40, 0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41, 0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641, 0xD201,
    0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040, 0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240, 0x3600, 0xF6C1, 0xF781, 0x3740,
    0xF501, 0x35C0, 0x3480, 0xF441, 0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41, 0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881,
    0x3840, 0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41, 0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40, 0xE401, 0x24C0,
    0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640, 0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041, 0xA001, 0x60C0, 0x6180, 0xA141, 0x6300,
    0xA3C1, 0xA281, 0x6240, 0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441, 0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840, 0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41, 0xBE01, 0x7EC0, 0x7F80,
    0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40, 0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640, 0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0,
    0x7080, 0xB041, 0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241, 0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440, 0x9C01,
    0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40, 0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841, 0x8801, 0x48C0, 0x4980, 0x8941,
    0x4B00, 0x8BC1, 0x8A81, 0x4A40, 0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41, 0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680,
    0x8641, 0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040};

// Computes CRC-16/Modbus over pkt[0..len-1], appends 2 CRC bytes, and sets the global sz to
// the total transmit length. Caller must ensure the response (len + 2 + any further trailing
// bytes) fits inside buf — sz is uint8_t and would wrap if len ≥ 254.
static inline void appendCrc(uint8_t *pkt, uint8_t len) {
    if (len > MAX_PKT_SZ - 2) len = MAX_PKT_SZ - 2;
    uint16_t crc = 0xFFFF;
    sz = len + 2;
    while (len--) {
        uint8_t xor = (*pkt++) ^ crc;
        crc >>= 8;
        crc ^= crc_table[xor];
    }
    *pkt++ = crc;
    *pkt++ = crc >> 8;
}

// Single-byte CRC update step used by the streaming TX path (header bytes in sendHoldingRegs
// and data bytes in the DRE ISR). Folds one byte into a running CRC-16/Modbus.
static inline uint16_t crc_step(uint16_t c, uint8_t b) {
    uint8_t x = b ^ c;
    return (c >> 8) ^ crc_table[x];
}

static inline bool inputIsValidModbusCommand() {
    // We know we have a valid command when:
    // a) the CRC is valid
    // b) the DeviceID matches ours, or is a broadcast
    // c) we recognise and can process the command
    // d) the length is valid for the command.  The smallest possible modbus packet size is 8 bytes,
    //    but some a variable length

    if (crc != 0 || sz < 5) {
        return false;
    }
    uint8_t cmdDeviceId = buf[0];
    uint8_t cmd = buf[1];
    uint8_t expectedSz = 255;

    if (cmdDeviceId == deviceId || cmdDeviceId == 0) {
        // Per Modbus spec, broadcasts (deviceId 0) are write-only — multiple slaves replying
        // to a broadcast read would collide. Only accept broadcasts for write functions.
        bool acceptBroadcast = false;
        switch (cmd) {
            case MODBUS_WRITE_COILS_CMD:
                expectedSz = 9 + buf[6];
                acceptBroadcast = true;
                break;
            case MODBUS_WRITE_SINGLE_COIL_CMD:
            case MODBUS_WRITE_SINGLE_HOLDING_REGISTER_CMD:
                expectedSz = 8;
                acceptBroadcast = true;
                break;
            case MODBUS_READ_COILS_CMD:
            case MODBUS_READ_INPUT_REGISTERS_CMD:
            case MODBUS_READ_HOLDING_REGISTERS_CMD:
                expectedSz = 8;
                break;
            case MODBUS_CUSTOM_EXEC_DALI_CMD:
                expectedSz = 7;
                break;
        }
        if (cmdDeviceId == 0 && !acceptBroadcast) {
            return false;
        }
        if (sz == expectedSz) {
            was_broadcast = (cmdDeviceId == 0);
            return true;
        }
        return false;
    }
    return false;
}

static void modbus_reset_buffer();

// Skip the TX entirely (broadcast path) but still re-arm RX so we can receive the next request.
// Used by the broadcast-suppression branches in sendResponse/sendError.
static inline void abandon_tx_re_enable_rx() {
    modbus_reset_buffer();
    modbus_idle = true;
    USART0.STATUS = USART_RXSIF_bm | USART_TXCIF_bm;
    USART0.CTRLA = USART_RS485_EXT_gc | USART_RXSIE_bm | USART_RXCIE_bm;
}

static void sendResponse(uint8_t *buf, uint8_t sz) {
    if (was_broadcast) {
        // Modbus §2.4.1: slaves must not respond to broadcasts. Just clear state and listen again.
        abandon_tx_re_enable_rx();
        return;
    }
    appendCrc(buf, sz);
    ptr = buf;

    // We are interested in Data ready and transmisson complete.
    USART0.STATUS = USART_TXCIF_bm;  // Clear any existing transmission complete flag set.
    USART0.CTRLA = USART_RS485_EXT_gc | USART_DREIE_bm | USART_TXCIE_bm;
}

static void sendError(modbus_err_t err) {
    if (was_broadcast) {
        // Same as sendResponse: don't transmit on a broadcast — even an exception. Just rearm RX.
        abandon_tx_re_enable_rx();
        return;
    }
    buf[0] = deviceId;
    buf[1] |= 0x80;  // Set the high bit to indicate an error.
    buf[2] = err;
    sendResponse(buf, 3);
}

modbus_err_t dali_response_to_err(dali_response_type_t type) {
    switch (type) {
        case DALI_RESPONSE_NAK:
            return MODBUS_ERR_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND;
        case DALI_RESPONSE_MALFORMED:
        case DALI_RESPONSE_PULSE_TOO_SHORT:
        case DALI_RESPONSE_PULSE_IN_MIDDLE:
        case DALI_RESPONSE_PULSE_TOO_LONG:
            return MODBUS_ERR_DALI_PULSE_WIDTH_INVALID + (type - DALI_RESPONSE_MALFORMED);
        case DALI_RESPONSE_COLLISION:
            return MODBUS_ERR_MEMORY_PARITY_ERROR;
        default:
            if (type >= DALI_RESPONSE_SHORT && type < DALI_RESPONSE_SHORT+16) {
                return MODBUS_ERR_DALI_RESPONSE_SHORT + (type - DALI_RESPONSE_SHORT);
            }
            return MODBUS_ERR_ILLEGAL_FUNCTION;
    }
}


void got_dali_highreg(dali_response_type_t err, uint16_t cmd, uint8_t res) {
    switch (err) {
        case DALI_RESPONSE_OK:
            buf[3] = res;
            break;
        default:
            buf[3] = 0x00;
            break;
    }

}
void got_dali_lowreg(dali_response_type_t err, uint16_t cmd, uint8_t res) {
    switch (err) {
        case DALI_RESPONSE_OK:
            buf[4] = res;
            sendResponse(buf, 5);
            break;
        default:
            sendError(dali_response_to_err(err));
    }
}





static void processGetCoilsCommand() {
    uint16_t addr = buf[2] << 8 | buf[3];
    uint16_t count = buf[4] << 8 | buf[5];

    // (uint32_t) cast prevents the addr+count comparison from wrapping in uint16_t when the
    // master sends a large addr (e.g. 65535) — without this, a wrap to a small value would pass
    // the bound check and cause out-of-bounds reads of dali_holding_regs.
    if (count == 0) {
        sendError(MODBUS_ERR_ILLEGAL_VALUE);  // Modbus §7: count out of range → ILLEGAL_VALUE
        return;
    }
    if ((uint32_t)addr + count > DALI_MAX_DEVICES) {
        sendError(MODBUS_ERR_ILLEGAL_ADDRESS); // addr range invalid → ILLEGAL_ADDRESS
        return;
    }

    // Pointer arithmetic must come after the bound check — forming a pointer more than one past
    // an array's end is UB even if it's never dereferenced.
    uint16_t *sptr = (uint16_t *)dali_holding_regs.banks[DALI_STATUS_BANK] + addr;
    uint8_t numBytes = (count + 7) / 8;
    buf[2] = numBytes;
    // Modbus 0x01 packs coils LSB-first: bit 0 of byte N holds the lowest-numbered coil in that byte.
    // Zero the data bytes first so we don't OR into uninitialised memory (which would leak prior buf contents).
    memset(buf + 3, 0, numBytes);
    for (uint16_t i = 0; i < count; i++) {
        if (*sptr++) {
            buf[3 + (i >> 3)] |= 1 << (i & 7);
        }
    }
    sendResponse(buf, numBytes + 3);
}

// Reads holding registers and sends the response by streaming data directly from
// dali_holding_regs in the DRE ISR. The DRE ISR reads each register fresh and folds the byte
// into the CRC as it goes, so the CRC always matches the bytes actually sent — even if a TCA0
// ISR updates a register mid-response. Modbus RTU caps a read at 125 registers.
static void sendHoldingRegs() {
    uint16_t addr = buf[2] << 8 | buf[3];
    uint16_t count = buf[4] << 8 | buf[5];

    // (uint32_t) cast prevents the addr+count comparison from wrapping in uint16_t — see
    // processGetCoilsCommand for the equivalent justification. streamSrc would otherwise be
    // pointed past the end of dali_holding_regs and the streaming TX would read out-of-bounds.
    if (count == 0 || count > 125) {
        sendError(MODBUS_ERR_ILLEGAL_VALUE);  // Modbus §7: count range error → ILLEGAL_VALUE
        return;
    }
    if ((uint32_t)addr + count > DALI_MAX_HOLDING_REGISTER) {
        sendError(MODBUS_ERR_ILLEGAL_ADDRESS); // addr range error → ILLEGAL_ADDRESS
        return;
    }

    // Header: buf[0] (device id) and buf[1] (func code 0x03) are already correct from the request.
    buf[2] = 2 * count;

    // Seed the running CRC with the 3 header bytes; the DRE ISR will continue folding in each
    // data byte as it's transmitted. After the data drains, the final CRC is written to buf[3..4].
    uint16_t c = 0xFFFF;
    c = crc_step(c, buf[0]);
    c = crc_step(c, buf[1]);
    c = crc_step(c, buf[2]);
    streamCrc = c;

    streamSrc = (uint16_t *)&dali_holding_regs + addr;
    streamWords = count;
    streamHighByte = true;

    ptr = buf;
    sz = 3;  // 3 header bytes; ISR will then stream data, then send 2 CRC bytes from buf+3.

    USART0.STATUS = USART_TXCIF_bm;
    USART0.CTRLA = USART_RS485_EXT_gc | USART_DREIE_bm | USART_TXCIE_bm;
}

static inline void tohexstr(uint8_t num, char *hexStr) {
    hexStr[0] = hexDigit(num >> 4);
    hexStr[1] = hexDigit(num & 0x0F);
}

/**
 * Sends device info, as a String, with each register holding two bytes
 */
static void sendInfo() {
    uint16_t addr = buf[2] << 8 | buf[3];
    uint16_t count = buf[4] << 8 | buf[5];

    strcpy((char *) buf+3, verstr);
    char *ptr = (char *) (buf+3 + VERSTR_LEN);
    tohexstr(SIGROW_SERNUM0, ptr);
    ptr+= 2;
    tohexstr(SIGROW_SERNUM1, ptr);
    ptr+= 2;
    tohexstr(SIGROW_SERNUM2, ptr);
    ptr+= 2;
    tohexstr(SIGROW_SERNUM3, ptr);
    ptr+= 2;
    tohexstr(SIGROW_SERNUM4, ptr);
    ptr+= 2;
    tohexstr(SIGROW_SERNUM5, ptr);
    ptr+= 2;
    tohexstr(SIGROW_SERNUM6, ptr);
    ptr+= 2;
    tohexstr(SIGROW_SERNUM7, ptr);
    ptr+= 2;
    tohexstr(SIGROW_SERNUM8, ptr);
    ptr+= 2;
    tohexstr(SIGROW_SERNUM9, ptr);
    *ptr++ = 0; // Just in case the resulting string length is odd.
    uint8_t strRegs = (VERSTR_LEN + 20 + 1) / 2;  // VERSTR + 20 hex chars, rounded up to whole registers

    // (uint32_t) cast: with addr/count both uint16_t, addr+count can wrap to a small value
    // when addr is large; without the cast the bound check would pass and memmove(buf+3,
    // buf+3+addr*2, count*2) would read out of bounds.
    if (count == 0 || (uint32_t)addr + count > strRegs) {
        sendError(MODBUS_ERR_ILLEGAL_ADDRESS);
        return;
    }
    buf[2] = count * 2;
    if (addr > 0)
        memmove(buf + 3, buf + 3 + addr * 2, count * 2);
    sendResponse(buf, count * 2 + 3);
}

// Stored as uint8_t (not dali_response_type_t) so reads and writes are single-instruction
// atomic on AVR — both main and ISR contexts write here, and the original 16-bit storage
// could tear between PENDING (0x00FF) and any value with a non-zero high byte. The enum is
// numbered to fit in 8 bits, so no information is lost.
static volatile uint8_t daliResponseType;
static volatile uint8_t daliResponse;

static void waitForDaliResponse() {
    // Bound the wait to ~600 ms. Any DALI batch that has not completed within this window is
    // reported to the Modbus master as a NAK (well before the 8.2 s WDT). The commands themselves
    // remain queued and continue executing on the DALI bus regardless of the Modbus response.
    // TCA0 ISRs continue to fire during _delay_ms, so the DALI state machine keeps running.
    for (uint16_t t = 600; t > 0 && daliResponseType == DALI_RESPONSE_PENDING; t--) {
        _delay_ms(1);
    }
    if (daliResponseType == DALI_RESPONSE_PENDING) {
        daliResponseType = DALI_RESPONSE_NAK;
    }
}

void dali_cmd_completed(dali_response_type_t responseType, uint16_t cmd, uint8_t response) {
    daliResponse = response;
    daliResponseType = responseType;
}

static void processCustomDaliCommand() {
    uint16_t cmd = buf[2] << 8 | buf[3];
    uint8_t repeat = buf[4];
    if (repeat > 2) {
        sendError(MODBUS_ERR_ILLEGAL_VALUE);
        return;
    }
    daliResponseType = DALI_RESPONSE_PENDING;

    dali_custom_command(cmd, dali_cmd_completed, repeat);
    waitForDaliResponse();
    // dali_cmd_callback() will perform the response
    if (daliResponseType == DALI_RESPONSE_OK) {
        buf[2] = daliResponse;
        sendResponse(buf, 3);
    } else {
        sendError(dali_response_to_err(daliResponseType));
    }

}

static void processSetSingleHoldingRegisterCommand() {
    uint16_t groupChanges;
    uint16_t addr = buf[2] << 8 | buf[3];
    uint16_t val = buf[4] << 8 | buf[5];

    uint8_t bank = addr / 64;
    uint8_t daliAddr = addr % 64;

    daliResponseType = DALI_RESPONSE_PENDING;
    switch (bank) {
        case DALI_STATUS_BANK:
            // Its power level;
            if (val <= 254) {
                dali_set_level(daliAddr, val, dali_cmd_completed);
                waitForDaliResponse();
            } else {
                sendError(MODBUS_ERR_ILLEGAL_VALUE);
                return;
            }
            break;
        case DALI_MINMAX_BANK:
            dali_set_min_level(daliAddr, val & 0xFF, NULL);
            dali_set_max_level(daliAddr, val >> 8 , dali_cmd_completed);
            // The 16-bit cache write is two STS instructions; wrap so a TCA0 ISR doing a byte
            // write to the same register can't interleave and produce a torn shortVal.
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                dali_holding_regs.banks[DALI_MINMAX_BANK][daliAddr].shortVal = val;
            }
            waitForDaliResponse();
            break;
        case DALI_FADE_BANK:
            dali_set_fade_rate(daliAddr, val & 0xFF, NULL);
            dali_set_fade_time(daliAddr, val >> 8, dali_cmd_completed);
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                dali_holding_regs.banks[DALI_FADE_BANK][daliAddr].shortVal = val;
            }
            waitForDaliResponse();
            break;
        case DALI_POWER_BANK:
            dali_set_power_on_level(daliAddr, val & 0xFF, NULL);
            dali_set_power_fail_level(daliAddr, val >> 8, dali_cmd_completed);
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                dali_holding_regs.banks[DALI_POWER_BANK][daliAddr].shortVal = val;
            }
            waitForDaliResponse();
            break;
        case DALI_GROUPS_BANK: {
            uint16_t cached;
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                cached = dali_holding_regs.banks[DALI_GROUPS_BANK][daliAddr].shortVal;
            }
            groupChanges = cached ^ val;
            if (groupChanges > 0) {
                ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                    dali_holding_regs.banks[DALI_GROUPS_BANK][daliAddr].shortVal = val;
                }
                for (uint8_t bit = 0; bit < 16; bit++) {
                    if (groupChanges & 0x01) {
                        // We only want the callback for the very last change.
                        dali_callback_t cb = groupChanges == 1 ? dali_cmd_completed : NULL;
                        if (val & 0x01) {
                            dali_add_to_group(daliAddr, bit, cb);
                        } else {
                            dali_remove_from_group(daliAddr, bit, cb);
                        }
                    }
                    groupChanges >>= 1;
                    val >>= 1;
                }
                waitForDaliResponse();
            } else {
                // No bits actually changed — treat as a no-op success.
                daliResponseType = DALI_RESPONSE_OK;
            }
            break;
        }
        default:
            sendError(MODBUS_ERR_ILLEGAL_ADDRESS);
            return;
    }
    if (daliResponseType == DALI_RESPONSE_OK) {
        sendResponse(buf, sz - 2);
    } else {
        sendError(dali_response_to_err(daliResponseType));
    }
}

// Forward declaration — defined in dali_state_idle.c. We call this directly (instead of
// dali_off/dali_goto_last_active or dali_custom_command) because those helpers route through
// track_fade and the singleton post_cmd_callback, which would race across iterations of the
// per-coil loop below.
extern dali_result_t dali_queue_command(uint16_t cmd, uint8_t repeat, dali_callback_t callback);

// Modbus 0x0F (Write Multiple Coils): maps each bit to a DALI device on/off, mirroring the
// semantics of processSetSingleCoilCommand (bit set → goto-last-active, bit clear → off). Coils
// are addressed as DALI short addresses 0..63. We process one coil at a time so the DALI command
// queue depth never exceeds 1 per iteration regardless of count, and the cached holding registers
// are refreshed by the next background scan rather than being kept in lockstep here.
//
// MAX_MULTI_COIL_COUNT is bounded so the worst-case batch (every device NAKing) completes well
// within the WDT window: count × waitForDaliResponse ≤ MAX_MULTI_COIL_COUNT × 600 ms.
#define MAX_MULTI_COIL_COUNT 8
static void processSetCoilsCommand() {
    uint16_t addr = buf[2] << 8 | buf[3];
    uint16_t count = buf[4] << 8 | buf[5];
    uint8_t numBytes = buf[6];

    // (uint32_t) cast prevents addr+count from wrapping in uint16_t. Without it, a large addr
    // would alias to a small DALI address (after the << 9 shift in ENCODE_CMD's caller below),
    // potentially even matching the broadcast address byte 0xFF — turning a stray write into
    // a bus-wide GOTO_LAST_ACTIVE/OFF.
    if (count == 0 || count > MAX_MULTI_COIL_COUNT) {
        sendError(MODBUS_ERR_ILLEGAL_VALUE);  // Modbus §7: count range error → ILLEGAL_VALUE
        return;
    }
    if ((uint32_t)addr + count > DALI_MAX_DEVICES) {
        sendError(MODBUS_ERR_ILLEGAL_ADDRESS);
        return;
    }
    if (numBytes != (uint8_t)((count + 7) / 8)) {
        sendError(MODBUS_ERR_ILLEGAL_VALUE);
        return;
    }

    // Modbus packs coils LSB-first: bit 0 of byte N holds the lowest-numbered coil in that byte.
    uint8_t *coilData = buf + 7;
    for (uint16_t i = 0; i < count; i++) {
        bool on = (coilData[i >> 3] >> (i & 7)) & 1;
        uint16_t cmd = (on ? DALI_CMD_GOTO_LAST_ACTIVE : DALI_CMD_OFF) | ((addr + i) << 9);
        daliResponseType = DALI_RESPONSE_PENDING;
        dali_queue_command(cmd, 1, dali_cmd_completed);
        waitForDaliResponse();
        if (daliResponseType != DALI_RESPONSE_OK) {
            sendError(dali_response_to_err(daliResponseType));
            return;
        }
    }
    // Modbus 0x0F success response echoes the request's first 6 bytes (addr + func + start + count).
    sendResponse(buf, 6);
}

static void processSetSingleCoilCommand() {
    uint16_t addr = buf[2] << 8 | buf[3];
    uint16_t val = buf[4] << 8 | buf[5];

    if (addr >= DALI_MAX_DEVICES) {
        sendError(MODBUS_ERR_ILLEGAL_ADDRESS);
        return;
    }

    // Modbus §6.5: data must be 0xFF00 (ON) or 0x0000 (OFF). We additionally accept 0x55xx
    // (any low byte) as a non-standard toggle extension — masters that don't know about it
    // simply won't use it. Anything else is an illegal value per spec.
    daliResponseType = DALI_RESPONSE_PENDING;
    if (buf[4] == 0x55) {
        dali_toggle(addr, dali_cmd_completed);
    } else if (val == 0xFF00) {
        dali_goto_last_active(addr, dali_cmd_completed);
    } else if (val == 0x0000) {
        dali_off(addr, dali_cmd_completed);
    } else {
        sendError(MODBUS_ERR_ILLEGAL_VALUE);
        return;
    }
    waitForDaliResponse();
    if (daliResponseType == DALI_RESPONSE_OK) {
        // Mirror the request back to the caller as the success response.
        sendResponse(buf, sz - 2);
    } else {
        sendError(dali_response_to_err(daliResponseType));
    }
}

static void modbus_reset_buffer() {
    ptr = buf;
    crc = 0xFFFF;
    sz = 0;
    streamWords = 0;
    modbus_cmd_available = false;
    was_broadcast = false;
}

void rs485Timer_timeout() {
    // cli();
    modbus_reset_buffer();
    modbus_idle = true;
    // sei();
}

extern void rs485_init(uint8_t rs485_device_id) {
    modbus_reset_buffer();

    deviceId = rs485_device_id;

    // Set up USART to operate at 9600, running on alternate pins.
    PORTMUX.CTRLB = PORTMUX_USART0_ALTERNATE_gc;
    RS485_PORT.PIN7CTRL = PORT_PULLUPEN_bm;
    RS485_PORT.PIN1CTRL = PORT_PULLUPEN_bm;
    RS485_PORT.OUTSET = RS485_TX_bm;                // PA1 (TXD) idles high.
    RS485_PORT.OUTCLR = RS485_DE_bm;                // PA4 (DE) low for receive — pre-USART safety.
    RS485_PORT.DIRSET = RS485_TX_bm | RS485_DE_bm;  // PA1 (TXD) and PA4 (DE/XDIR) are outputs.
    RS485_PORT.DIRCLR = RS485_RX_bm;                // PA2 (RXD) is an input.
    // Note: once USART0 is enabled below with USART_RS485_EXT_gc, PA4 is taken over as the
    // hardware-driven XDIR signal. The USART asserts XDIR around active transmissions and
    // deasserts it at TXC automatically — firmware should NOT manually toggle PA4 after this.

    USART0.BAUD = USART0_BAUD_RATE(MODBUS_BAUD);  // Set Baud rate to 9600
    // N81 Async Sserial
    USART0.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc | USART_SBMODE_1BIT_gc;
    USART0.CTRLA = USART_RS485_EXT_gc | USART_RXSIE_bm | USART_RXCIE_bm;  // ISR for Start of Frmae or reception of data.
    USART0.CTRLB = USART_TXEN_bm | USART_RXEN_bm | USART_SFDEN_bm;        // enable reception, transmission, and wake from sleep

    rs485Timer = 0;
}

void rs485_process_command() {
    // We decouple receiving data in an ISR from processing the command through the
    // modbus_cmd_available variable.  When it is set, there is a valid command in
    // the incoming buffer, and RX has been temporarily turned off. 
    if (modbus_cmd_available) {
        // Modbus RTU requires at least 3.5 character times of silence (≈ 3.65 ms at 9600 baud)
        // between request and response so masters relying on inter-frame timing can separate the two.
        _delay_ms(4);

        switch (buf[1]) {
            case MODBUS_READ_COILS_CMD:
                processGetCoilsCommand();
                break;
            case MODBUS_READ_HOLDING_REGISTERS_CMD:
                sendHoldingRegs();
                break;
            case MODBUS_READ_INPUT_REGISTERS_CMD:
                sendInfo();
                break;
            case MODBUS_WRITE_COILS_CMD:
                processSetCoilsCommand();
                break;
            case MODBUS_WRITE_SINGLE_COIL_CMD:
                processSetSingleCoilCommand();
                break;
            case MODBUS_WRITE_SINGLE_HOLDING_REGISTER_CMD:
                processSetSingleHoldingRegisterCommand();
                break;

            case MODBUS_CUSTOM_EXEC_DALI_CMD:
                processCustomDaliCommand();
                break;
            default:
                sendError(MODBUS_ERR_ILLEGAL_FUNCTION);
                break;
        }
        modbus_cmd_available = false;
    }
}

/**
 * Transmits data via RS485, until we run out
 */
ISR(USART0_DRE_vect) {
    if (sz) {
        // Header bytes (CRC was seeded over them in sendHoldingRegs) or, after the stream
        // drains, the trailing CRC bytes (no further CRC update for these).
        USART0.TXDATAL = *ptr++;
        sz--;
    } else if (streamWords) {
        uint8_t b;
        if (streamHighByte) {
            // Read the register fresh, atomically. TCA0 (LVL1) can preempt this ISR, so the
            // 16-bit load needs to be uninterruptible to avoid a torn high/low pair.
            uint16_t w;
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                w = *streamSrc;
            }
            b = w >> 8;
            streamLowByte = w & 0xFF;
            streamHighByte = false;
        } else {
            b = streamLowByte;
            streamSrc++;
            streamWords--;
            streamHighByte = true;
        }
        USART0.TXDATAL = b;
        streamCrc = crc_step(streamCrc, b);

        if (streamWords == 0 && streamHighByte) {
            // Just sent the final low byte. Park the running CRC and switch to buf-send mode
            // for the trailing 2 CRC bytes (CRC low first, then CRC high — Modbus RTU).
            buf[3] = streamCrc & 0xFF;
            buf[4] = streamCrc >> 8;
            ptr = buf + 3;
            sz = 2;
        }
    } else {
        // Nothing left to send.
        USART0.CTRLA = USART_RS485_EXT_gc | USART_TXCIE_bm;
    }
}

/**
 * Once transmission is complete, this ISR is fired, to move us back into receiving state
 */
ISR(USART0_TXC_vect) {
    modbus_reset_buffer();
    modbus_idle = true;

    // Drain any bytes the master sent during our transmission — they sit in the USART RX FIFO
    // (depth 2) and would otherwise be prepended to the next legitimate packet, corrupting its
    // length/CRC and forcing a timeout-driven recovery. Reading RXDATAL clears RXCIF.
    while (USART0.STATUS & USART_RXCIF_bm) {
        (void)USART0.RXDATAL;
    }

    // Clear any pending flags.
    USART0.STATUS = USART_RXSIF_bm | USART_TXCIF_bm;
    // ISR on reception.
    USART0.CTRLA = USART_RS485_EXT_gc | USART_RXSIE_bm | USART_RXCIE_bm;
}

ISR(USART0_RXC_vect) {
    // Reset the countdown Timer.
    modbus_idle = false;
    // Modbus RTU inter-character timeout is 1.5 char times (≈ 1.56 ms at 9600 baud). The TCA0
    // OVF tick is 416.67 µs and rs485Timer decrement is asynchronous to byte arrival, so a value
    // of N gives an effective timeout window of (N-1) to N ticks. rs485Timer = 5 → 4 to 5 ticks
    // = 1.667 to 2.083 ms, which keeps the lower bound at or above the 1.5-char minimum.
    rs485Timer = 5;
    USART0.STATUS = USART_RXSIF_bm;  // clear the interrupt flag.
    // }

    if (USART0.STATUS & USART_RXCIF_bm) {
        uint8_t val = USART0.RXDATAL;  // this will clear the CIF flag.
        if (sz < MAX_PKT_SZ) {
            *ptr++ = val;
            uint8_t xor = val ^ crc;
            crc >>= 8;
            crc ^= crc_table[xor];
            sz++;

            if (inputIsValidModbusCommand()) {
                rs485Timer = 0;
                modbus_cmd_available = true;
                USART0.CTRLA = USART_RS485_EXT_gc;  // Remove the Receive ISRs.
            }
        }
    }
}