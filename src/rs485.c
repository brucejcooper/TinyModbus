#include "rs485.h"

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

#include "dali.h"
#include "timers.h"

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
    MODBUS_ERR_GATEWAY_PATH_UNAVAILABLE = 0x09,
    MODBUS_ERR_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND = 0x0a,
} modbus_err_t;

#define MODBUS_BAUD 9600
#define USART0_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5)

#define RS485_PORT PORTA
#define RS485_TX_bm PIN1_bm
#define RS485_RX_bm PIN2_bm
#define RS485_DE_bm PIN4_bm

#define MAX_PKT_SZ 64 * 2 + 8

static uint8_t deviceId = 1;

static volatile uint16_t crc = 0xFFFF;
static uint8_t buf[MAX_PKT_SZ];  // Allow for a lot of input (but not the full 256 that MODBUS allows for)
static volatile uint8_t *ptr = buf;
static volatile uint8_t sz = 0;
volatile bool modbus_idle = false;
volatile uint8_t readCountDown;
static bool modbus_cmd_available = false;

/**
 * Stores an indentifier string for this device.  Clients can fetch 32 registers worth of Input registers at address 0 to get the string.
 */
static char verStr[64];

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

static inline void appendCrc(uint8_t *pkt, uint8_t len) {
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
        switch (cmd) {
            case MODBUS_WRITE_COILS_CMD:
                expectedSz = 9 + buf[6];
                break;
            case MODBUS_READ_COILS_CMD:
            case MODBUS_WRITE_SINGLE_COIL_CMD:
            case MODBUS_READ_INPUT_REGISTERS_CMD:
            case MODBUS_READ_HOLDING_REGISTERS_CMD:
            case MODBUS_WRITE_SINGLE_HOLDING_REGISTER_CMD:
                expectedSz = 8;
                break;

            case MODBUS_CUSTOM_EXEC_DALI_CMD:
                expectedSz = 7;
                break;
        }
        return sz == expectedSz;
    }
    return false;
}

static void sendResponse(uint8_t *buf, uint8_t sz) {
    appendCrc(buf, sz);
    ptr = buf;

    // We are interested in Data ready and transmisson complete.
    USART0.STATUS = USART_TXCIF_bm;  // Clear any existing transmission complete flag set.
    USART0.CTRLA = USART_RS485_EXT_gc | USART_DREIE_bm | USART_TXCIE_bm;
}

static void sendError(modbus_err_t err) {
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
            return MODBUS_ERR_SLAVE_DEVICE_FAILURE;
        case DALI_RESPONSE_COLLISION:
            return MODBUS_ERR_MEMORY_PARITY_ERROR;
        default:
            return MODBUS_ERR_ILLEGAL_FUNCTION;
    }
}

static void processGetCoilsCommand() {
    uint16_t addr = buf[2] << 8 | buf[3];
    uint16_t count = buf[4] << 8 | buf[5];
    uint8_t *dptr = buf + 2;
    uint16_t *sptr = (uint16_t *)dali_holding_regs.power + addr;

    if (count > 0 && addr + count <= DALI_MAX_DEVICES) {
        *dptr = count / 8 + (count % 8 ? 1 : 0);
        for (int i = 0; i < count; i++) {
            if (i % 8 == 0) {
                dptr++;
            }
            *dptr = *dptr << 1 | (*sptr++ ? 1 : 0);
        }
        sendResponse(buf, buf[2] + 3);
    } else {
        // We can't query downstream sources, so this should return an illegal address error.
        sendError(MODBUS_ERR_ILLEGAL_ADDRESS);
    }
}

static void sendModbusRegs(uint16_t *sptr, uint16_t max) {
    uint16_t addr = buf[2] << 8 | buf[3];
    uint16_t count = buf[4] << 8 | buf[5];

    if (count > 0 && count < 128 && addr + count <= max) {
        buf[2] = 2 * count;
        sptr += addr;
        uint16_t *end = sptr + count;
        uint8_t *dptr = buf + 3;

        while (sptr < end) {
            *dptr++ = *sptr >> 8;
            *dptr++ = *sptr++ & 0xFF;
        }
        sendResponse(buf, buf[2] + 3);
    } else {
        sendError(MODBUS_ERR_ILLEGAL_ADDRESS);
    }
}

/**
 * Sends device info, as a String, with each register holding two bytes
 */
static void sendInfo() {
    uint16_t addr = buf[2] << 8 | buf[3];
    uint16_t count = buf[4] << 8 | buf[5];

    if (count > 0 && addr + count <= (sizeof(verStr) / 2)) {
        buf[2] = count * 2;
        memcpy(buf + 3, verStr + addr * 2, count * 2);

        sendResponse(buf, buf[2] + 3);
    } else {
        sendError(MODBUS_ERR_ILLEGAL_ADDRESS);
    }
}

static void dali_custom_cmd_callback(dali_response_type_t type, uint16_t cmd, uint8_t response) {
    switch (type) {
        case DALI_RESPONSE_OK:
            buf[2] = response;
            sendResponse(buf, 3);
            break;
        case DALI_RESPONSE_NAK:
            sendError(MODBUS_ERR_GATEWAY_TARGET_DEVICE_FAILED_TO_RESPOND);
            break;
        case DALI_RESPONSE_MALFORMED:
            sendError(MODBUS_ERR_SLAVE_DEVICE_FAILURE);
            break;
        case DALI_RESPONSE_COLLISION:
            sendError(MODBUS_ERR_MEMORY_PARITY_ERROR);
            break;
        default:
            sendError(MODBUS_ERR_ILLEGAL_FUNCTION);
            break;
    }
}

static volatile dali_response_type_t daliResponseType;
static volatile uint8_t daliResponse;

void waitForDaliResponse() {
    while (daliResponseType == DALI_RESPONSE_PENDING) {
        // TODO WDT or something.
        ;
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
    dali_custom_command(cmd, dali_custom_cmd_callback, repeat);
    // dali_cmd_callback() will perform the response
}

static void processSetSingleHoldingRegisterCommand() {
    uint16_t addr = buf[2] << 8 | buf[3];
    uint16_t val = buf[4] << 8 | buf[5];

    uint8_t bank = addr / 64;
    uint8_t bankAddr = addr % 64;

    switch (bank) {
        case 0:
            // Its power level;
            if (val <= 254) {
                daliResponseType = DALI_RESPONSE_PENDING;
                dali_set_level(bankAddr, val, dali_cmd_completed);
                waitForDaliResponse();
                sendResponse(buf, sz - 2);
            } else {
                sendError(MODBUS_ERR_ILLEGAL_VALUE);
            }
            break;
        default:
            sendError(MODBUS_ERR_ILLEGAL_ADDRESS);
            break;
    }
}

static void processSetCoilsCommand() {
    uint16_t addr = buf[2] << 8 | buf[3];
    uint16_t count = buf[4] << 8 | buf[5];
    uint8_t numBytes = buf[6];

    uint8_t *coilData = buf + 7;
    if (addr < 8) {
        // At least some of the coil values are for this unit
        uint8_t bitsForThisUnit = 8 - addr;
        if (bitsForThisUnit > count) {
            bitsForThisUnit = count;
        }
        // Set coil values.
        // for (uint8_t i = 0; i < bitsForThisUnit; i++) {
        //     setCoil(addr + i, val & 0x01 ? 0xFF : 0x00);
        //     val >>= 1;
        // }
        // latchOutputs();

        sendResponse(buf, 6);
    } else {
        sendError(MODBUS_ERR_ILLEGAL_ADDRESS);
    }
}

static void processSetSingleCoilCommand() {
    uint16_t addr = buf[2] << 8 | buf[3];
    uint8_t val = buf[4];

    if (addr < DALI_MAX_DEVICES) {
        daliResponseType = DALI_RESPONSE_PENDING;
        if (val == 0x55) {
            dali_toggle(addr, dali_cmd_completed);
        } else if (val == 0xFF) {
            dali_goto_last_active(addr, dali_cmd_completed);
        } else {
            dali_off(addr, dali_cmd_completed);
        }
        waitForDaliResponse();
        // Simply mirror the request back to the caller
        sendResponse(buf, sz - 2);
    } else {
        sendError(MODBUS_ERR_ILLEGAL_ADDRESS);
    }
}

static void modbus_reset_buffer() {
    ptr = buf;
    crc = 0xFFFF;
    sz = 0;
    modbus_cmd_available = false;
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
    memset(verStr, 0, sizeof(verStr));
    snprintf(verStr, sizeof(verStr), "Mechination DALI v%d.%d ID:%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x", FW_VER_MAJOR, FW_VER_MINOR, SIGROW_SERNUM0,
             SIGROW_SERNUM1, SIGROW_SERNUM2, SIGROW_SERNUM3, SIGROW_SERNUM4, SIGROW_SERNUM5, SIGROW_SERNUM6, SIGROW_SERNUM7, SIGROW_SERNUM8, SIGROW_SERNUM9);

    // Set up USART to operate at 9600, running on alternate pins.
    PORTMUX.CTRLB = PORTMUX_USART0_ALTERNATE_gc;
    RS485_PORT.PIN7CTRL = PORT_PULLUPEN_bm;
    RS485_PORT.PIN1CTRL = PORT_PULLUPEN_bm;
    RS485_PORT.OUTSET = RS485_TX_bm;                // PB2 defaults high, because its UART out.
    RS485_PORT.OUTCLR = RS485_DE_bm;                // PB2 defaults high, because its UART out.
    RS485_PORT.DIRSET = RS485_TX_bm | RS485_DE_bm;  // PB0 (XDIR) and PB2 (TXD) are outputs
    RS485_PORT.DIRCLR = RS485_RX_bm;                // PB3 (RXD) is an input

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
        uint8_t cmd_addr = buf[0];
        switch (buf[1]) {
            case MODBUS_READ_COILS_CMD:
                processGetCoilsCommand();
                break;
            case MODBUS_READ_HOLDING_REGISTERS_CMD:
                sendModbusRegs((uint16_t *)&dali_holding_regs, DALI_MAX_HOLDING_REGISTER);
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
        USART0.TXDATAL = *ptr++;
        sz--;
    } else {
        // Disable DRE ISR
        USART0.CTRLA = USART_RS485_EXT_gc | USART_TXCIE_bm;
    }
}

/**
 * Once transmission is complete, this ISR is fired, to move us back into receiving state
 */
ISR(USART0_TXC_vect) {
    modbus_reset_buffer();
    modbus_idle = true;

    // Clear any pending flags.
    USART0.STATUS = USART_RXSIE_bm | USART_RXCIE_bm | USART_TXCIF_bm;
    // ISR on reception.
    USART0.CTRLA = USART_RS485_EXT_gc | USART_RXSIE_bm | USART_RXCIE_bm;
}

ISR(USART0_RXC_vect) {
    // Reset the countdown Timer.
    modbus_idle = false;
    // if (USART0.STATUS & USART_RXSIF_bm) {
    // This timeout will be somewhere between 3 ticks (1.25 millis) and 4 ticks (1.666 millis).
    // Strictly the limit is 1.5msec, but this will be fine.

    rs485Timer = 4;
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