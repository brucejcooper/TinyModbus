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
#include "timers.h"
#include "dali.h"

#define MODBUS_READ_COILS_CMD 0x01
#define MODBUS_READ_DISCRETE_INPUTS_CMD 0x02
#define MODBUS_READ_HOLDING_REGISTERS_CMD 0x03
#define MODBUS_READ_INPUT_REGISTERS_CMD 0x04
#define MODBUS_WRITE_SINGLE_COIL_CMD 0x05
#define MODBUS_WRITE_SINGLE_REGISTER_CMD 0x06
#define MODBUS_WRITE_COILS_CMD 0x0F
#define MODBUS_WRITE_REGISTERS_CMD 0x10

typedef enum {
    MODBUS_ERR_ILLEGAL_FUNCTION = 0x01,
    MODBUS_ERR_ILLEGAL_ADDRESS,
    MODBUS_ERR_ILLEGAL_VALUE,
} modbus_err_t;

#define MODBUS_BAUD 9600
#define USART0_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5)

#define RS485_PORT PORTA
#define RS485_TX_bm PIN1_bm
#define RS485_RX_bm PIN2_bm
#define RS485_DE_bm PIN4_bm

#define MAX_PKT_SZ 64*2+8

static uint8_t deviceId = 1;

static uint16_t modbuscrc = 0xFFFF;
static uint8_t modbusbuf[MAX_PKT_SZ];  // Allow for a lot of input (but not the full 256 that MODBUS allows for)
static uint8_t *modbusptr = modbusbuf;
static uint8_t modbus_sz = 0;



static const uint16_t crc_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241, 0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481,
    0x0440, 0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40, 0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0,
    0x0880, 0xC841, 0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40, 0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01,
    0x1DC0, 0x1C80, 0xDC41, 0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641, 0xD201, 0x12C0, 0x1380, 0xD341,
    0x1100, 0xD1C1, 0xD081, 0x1040, 0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240, 0x3600, 0xF6C1, 0xF781,
    0x3740, 0xF501, 0x35C0, 0x3480, 0xF441, 0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41, 0xFA01, 0x3AC0,
    0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840, 0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41, 0xEE01,
    0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40, 0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041, 0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281,
    0x6240, 0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441, 0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0,
    0x6E80, 0xAE41, 0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840, 0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01,
    0x7BC0, 0x7A80, 0xBA41, 0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40, 0xB401, 0x74C0, 0x7580, 0xB541,
    0x7700, 0xB7C1, 0xB681, 0x7640, 0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041, 0x5000, 0x90C1, 0x9181,
    0x5140, 0x9301, 0x53C0, 0x5280, 0x9241, 0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440, 0x9C01, 0x5CC0,
    0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40, 0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841, 0x8801,
    0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40, 0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641, 0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081,
    0x4040};


static void appendCrc(uint8_t *pkt, uint8_t len) {
    uint16_t crc = 0xFFFF;
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

    if (modbus_sz < 8) {
        return false;
    }
    uint8_t cmdDeviceId = modbusbuf[0];
    uint8_t cmd = modbusbuf[1];
    if ((cmdDeviceId == deviceId || cmdDeviceId == 0) && modbuscrc == 0) {
        switch (cmd) {
            case MODBUS_WRITE_COILS_CMD:
                return modbus_sz == 9 + modbusbuf[6];
            case MODBUS_READ_COILS_CMD:
            case MODBUS_WRITE_SINGLE_COIL_CMD:
            case MODBUS_READ_INPUT_REGISTERS_CMD:
            case MODBUS_READ_HOLDING_REGISTERS_CMD:
                return modbus_sz == 8;
        }
    }
    return false;
}

static void setErrorResponse(modbus_err_t err) {
    modbusbuf[1] |= 0x80;  // Set the high bit to indicate an error.
    modbusbuf[2] = err;
    appendCrc(modbusbuf, 3);
    modbus_sz = 5;
}

static void processGetCoilsCommand() {
    uint16_t addr = modbusbuf[2] << 8 | modbusbuf[3];
    uint16_t count = modbusbuf[4] << 8 | modbusbuf[5];
    uint8_t *dptr = modbusbuf + 2;
    uint16_t *sptr = dali_holding_regs.power + addr;

    if (count > 0 && addr + count <= DALI_MAX_DEVICES) {
        *dptr = count/8 + (count % 8 ? 1 : 0);
        for (int i = 0; i < count; i++) {
            if (i % 8 == 0) {
                dptr++;
            }
            *dptr = *dptr << 1 | (*sptr++ ? 1 : 0);
        }
        appendCrc(modbusbuf, modbusbuf[2] + 3);
        modbus_sz = modbusbuf[2] + 5;
    } else {
        // We can't query downstream sources, so this should return an illegal address error.
        setErrorResponse(MODBUS_ERR_ILLEGAL_ADDRESS);
    }
}


static void sendModbusRegs(uint16_t *sptr, uint16_t max) {
    uint16_t addr = modbusbuf[2] << 8 | modbusbuf[3];
    uint16_t count = modbusbuf[4] << 8 | modbusbuf[5];
        
    if (count > 0 && count < 128 && addr + count <= max) {
        modbusbuf[2] = 2 * count;
        sptr += addr;
        uint16_t *end = sptr + count;
        uint8_t *dptr = modbusbuf + 3;

        while (sptr < end) {
            *dptr++ = *sptr >> 8;
            *dptr++ = *sptr++ & 0xFF;
        }
        appendCrc(modbusbuf, modbusbuf[2] + 3);
        modbus_sz = modbusbuf[2] + 5;
    } else {
        setErrorResponse(MODBUS_ERR_ILLEGAL_ADDRESS);
    }
}


static void processSetCoilsCommand() {
    uint16_t addr = modbusbuf[2] << 8 | modbusbuf[3];
    uint16_t count = modbusbuf[4] << 8 | modbusbuf[5];
    uint8_t numBytes = modbusbuf[6];

    uint8_t *coilData = modbusbuf + 7;
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

    } else {
        setErrorResponse(MODBUS_ERR_ILLEGAL_ADDRESS);
    }
    // The respnose is simply the input bytes, but only up to the address (6 bytes).
    // We need to change the CRC.
    appendCrc(modbusbuf, 6);
    modbus_sz = 8;
}

static void processSetSingleCoilCommand() {
    uint16_t addr = modbusbuf[2] << 8 | modbusbuf[3];
    uint8_t val = modbusbuf[4];

    if (addr < DALI_MAX_DEVICES) {
        // setCoil(addr, val);
        // latchOutputs();
    }
}



extern void rs485_init(uint8_t rs485_device_id) {
    deviceId = rs485_device_id;

    // Set up USART to operate at 9600, running on alternate pins.
    PORTMUX.CTRLB = PORTMUX_USART0_ALTERNATE_gc;
    RS485_PORT.PIN7CTRL = PORT_PULLUPEN_bm;
    RS485_PORT.PIN1CTRL = PORT_PULLUPEN_bm;
    RS485_PORT.OUTSET = RS485_TX_bm;            // PB2 defaults high, because its UART out.
    RS485_PORT.OUTCLR = RS485_DE_bm;            // PB2 defaults high, because its UART out.
    RS485_PORT.DIRSET = RS485_TX_bm | RS485_DE_bm;  // PB0 (XDIR) and PB2 (TXD) are outputs
    RS485_PORT.DIRCLR = RS485_RX_bm;            // PB3 (RXD) is an input

    USART0.BAUD = USART0_BAUD_RATE(MODBUS_BAUD);  // Set Baud rate to 9600
    // USART0.CTRLA = USART_RS485_EXT_gc;
    USART0.CTRLB = USART_RXEN_bm | USART_SFDEN_bm;  // enable reception, and wake from sleep

}


/**
 * Reads a single character in from the UART, with a rough timeout.
 */
static int readch() {
    // Both timers are already in use by DALI, so we have to poll the CPU clock for timeout.
    // If we assume that each check is 9 cycles (to make the maths easy), It'll do one cycle in 3 microseconds .
    // So, to get 1.5 ms, we would wait for 500 cycles.
    // If DALI is doing stuff, it will extend the timeout, but the timer need only be rough.
    int countDown = USEC_TO_TICKS(1500)/9;
    while (!(USART0.STATUS & USART_RXCIF_bm)) {
        if (--countDown == 0) {
            return -1;
        }
    }
    uint8_t val = USART0.RXDATAL; // this will clear the CIF flag. 
    return val;
}


static inline void send(uint8_t *buf, size_t sz) {
    USART0.CTRLB = USART_TXEN_bm;
    // Enabling and disabling the UART seems to turn the pin back to an input.  Make sure they're all outputs.
    RS485_PORT.DIRSET = RS485_TX_bm | RS485_DE_bm; 
    USART0.STATUS = USART_TXCIF_bm;
    RS485_PORT.OUTSET = RS485_DE_bm;
    _delay_us(100);



    while (sz--) {
        // wait for TX buffer to be empty.
        while (!(USART0.STATUS & USART_DREIF_bm)) {
        }
        USART0.TXDATAL = *buf++;
    }
    // Wait all output to be flushed
    while (!(USART0.STATUS & USART_TXCIF_bm)) {
    }
    USART0.STATUS = USART_TXCIF_bm;

    // Switch the RS485 transciever back into receive mode.
    RS485_PORT.OUTCLR = RS485_DE_bm;

    USART0.CTRLB = USART_RXEN_bm | USART_SFDEN_bm;  // enable reception, and wake from sleep

}



extern bool rs485_process_command() {
    modbusptr = modbusbuf;
    modbuscrc = 0xFFFF;
    modbus_sz = 0;
    int ch;

    // Reset the Start of Frame flag, if it was set
    USART0.STATUS = USART_RXSIF_bm;

    do {
        ch = readch();

        if (ch == -1 || modbus_sz == MAX_PKT_SZ) {
            return false;
        }

        *modbusptr++ = ch;
        modbus_sz++;
        uint8_t xor = ch ^ modbuscrc;
        modbuscrc >>= 8;
        modbuscrc ^= crc_table[xor];
    } while (!inputIsValidModbusCommand());

    uint8_t cmd_addr = modbusbuf[0];

    if (cmd_addr == deviceId || cmd_addr == 0) {
        switch (modbusbuf[1]) {
            case MODBUS_READ_COILS_CMD:

                processGetCoilsCommand();
                break;
            case MODBUS_READ_HOLDING_REGISTERS_CMD:
                sendModbusRegs((uint16_t *) &dali_holding_regs, DALI_MAX_HOLDING_REGISTER);
                break;
            case MODBUS_READ_INPUT_REGISTERS_CMD:
                sendModbusRegs(dali_input_regs, DALI_MAX_INPUT_REGISTER);
                break;
            case MODBUS_WRITE_COILS_CMD:
                processSetCoilsCommand();
                break;
            case MODBUS_WRITE_SINGLE_COIL_CMD:
                processSetSingleCoilCommand();
                break;
            default:
                setErrorResponse(MODBUS_ERR_ILLEGAL_FUNCTION);
                break;
        }

        // Respond to the caller, unless it was a broadcast
        if (cmd_addr) {
            send(modbusbuf, modbus_sz);
        }
    }
    return true;
}
