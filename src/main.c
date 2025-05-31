#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <stdbool.h>
#include <stdint.h>
#include <util/atomic.h>
#include <util/delay.h>

#define MODBUS_GET_COILS_CMD 0x01
#define MODBUS_SET_SINGLE_COIL_CMD 0x05
#define MODBUS_SET_COILS_CMD 0x0F

typedef enum {
    MODBUS_ERR_ILLEGAL_FUNCTION = 0x01,
    MODBUS_ERR_ILLEGAL_ADDRESS,
    MODBUS_ERR_ILLEGAL_VALUE,
} modbus_err_t;

#define MODBUS_BAUD 9600
#define USART0_BAUD_RATE(BAUD_RATE) ((float)(F_CPU * 64 / (16 * (float)BAUD_RATE)) + 0.5)
#define USEC_TO_TICKS(x) ((unsigned)(((float)F_CPU * (x) / (float)1000000) + 0.5))
#define SOFTSERIAL_PER USEC_TO_TICKS(104.2)
#define SOFTSERIAL_CMPVAL_ONE (SOFTSERIAL_PER + 1)

#define MAX_PKT_SZ 200

#define TXFLAG_UART (1 << 0)
#define TXFLAG_SOFT (1 << 1)

// When we're idle we're only interested in reading a new byte in.
#define USART_CTRLA_OUTPUT_BUFFER_EMPTY (USART_RXCIF_bm | USART_TXCIF_bm | USART_RS485_EXT_gc)
// When we're Active, we are also interested in Transmission completion and keeping the buffer fed.
#define USART_CTRLA_TRANSMITTING (USART_CTRLA_OUTPUT_BUFFER_EMPTY | USART_DREIF_bm)

static uint8_t deviceId = 1;
static uint8_t coilValues = 0;

static uint16_t modbuscrc = 0xFFFF;
static uint8_t modbusbuf[MAX_PKT_SZ];  // Allow for a lot of input (but not the full 256 that MODBUS allows for)
static uint8_t *modbusptr = modbusbuf;
static uint8_t modbus_sz = 0;

static uint8_t outbuf[10];  // We never output very much back over MODBUS (maximum 8 bytes)
static uint8_t *outhead = outbuf;
static uint8_t *outtail = outbuf;

static uint8_t downstreambuf[MAX_PKT_SZ];
static uint8_t *downstreamptr = downstreambuf;

static volatile uint8_t *softserial_ptr;
static volatile uint8_t softserial_byte = 0;
static volatile uint8_t softserial_remaining_bytes = 0;
static volatile uint8_t softserial_remaining_bits = 0;

static volatile uint8_t transmissionFlags = 0;

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

/**
 * Transmits at most 256 bytes of data out via timer.
 */
static void softserial_tx(uint8_t *data, uint8_t num) {
    // wait for any previous transmission to complete
    softserial_ptr = data;
    softserial_byte = *softserial_ptr++;
    softserial_remaining_bytes = num - 1;  // -1 because we've already popped one off.
    softserial_remaining_bits = 10;

    transmissionFlags |= TXFLAG_SOFT;

    TCA0.SINGLE.CMP1 = SOFTSERIAL_CMPVAL_ONE;  // No toggling on first cycle
    TCA0.SINGLE.CNT = 4;                       // We've got some interrupt delay and setup instructions
                                               // which add latency, so we start a bit higher for the first bit
    TCA0.SINGLE.CTRLC = 0;                     // Drive output low
    TCA0.SINGLE.CTRLA = TCA_SINGLE_ENABLE_bm | TCA_SINGLE_CLKSEL_DIV1_gc;        // Start Timer.
    TCA0.SINGLE.CMP1BUF = (softserial_byte & 0x01) ? SOFTSERIAL_CMPVAL_ONE : 0;  // Buffer next Period which will be the LSB
}

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

static inline void send(uint8_t byte) {
    transmissionFlags |= TXFLAG_UART;
    *outtail++ = byte;
    // Enable the output buffer ready ISR.  If we were previously idle, this will fire immediately.
    USART0.CTRLA = USART_CTRLA_TRANSMITTING;
}

static inline void resetModbusBuf() {
    modbusptr = modbusbuf;
    modbus_sz = 0;
    modbuscrc = 0xFFFF;
}

static void latchOutputs() {
    // Bit 0 -> PB5
    if (coilValues & 0x01) {
        PORTB.OUTSET = PIN5_bm;
    } else {
        PORTB.OUTCLR = PIN5_bm;
    }
    // Bit 1 -> PB4
    if (coilValues & 0x02) {
        PORTB.OUTSET = PIN4_bm;
    } else {
        PORTB.OUTCLR = PIN4_bm;
    }

    // Bit 2->7 match to PORTA directly, by pure chance.
    PORTA.OUT = coilValues & 0xFC;
}

static void setCoil(uint8_t addr, uint8_t val) {
    uint8_t bm = 1 << addr;
    if (val == 0xFF) {
        // Set
        coilValues |= bm;
    } else if (val == 0x55) {
        // Toggle
        coilValues ^= bm;
    } else {
        // Clear
        coilValues &= ~bm;
    }
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
            case MODBUS_SET_COILS_CMD:
                return modbus_sz == 9 + modbusbuf[6];
            case MODBUS_GET_COILS_CMD:
            case MODBUS_SET_SINGLE_COIL_CMD:
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

    if (count > 0 && addr + count <= 8) {
        modbusbuf[2] = 1;  // there can only be one bytes as we are a 1 byte coil holder
        modbusbuf[3] = coilValues >> addr;
        appendCrc(modbusbuf, 4);
        modbus_sz = 6;
    } else {
        // We can't query downstream sources, so this should return an illegal address error.
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

        uint8_t val = modbusbuf[7];
        // Set coil values.
        for (uint8_t i = 0; i < bitsForThisUnit; i++) {
            setCoil(addr + i, val & 0x01 ? 0xFF : 0x00);
            val >>= 1;
        }
        latchOutputs();

        // All remaining values get shunted downstream, shifted by bitsForThisUnit bits,
        // and any remaining bits from the next byte.
        count -= bitsForThisUnit;
        if (count > 0) {
            addr -= bitsForThisUnit;
            numBytes = count / 8;
            if (bitsForThisUnit < 8) {
                numBytes++;
            }

            *downstreamptr++ = modbusbuf[0];          // Device ID
            *downstreamptr++ = MODBUS_SET_COILS_CMD;  // CMD
            *downstreamptr++ = addr << 8;
            *downstreamptr++ = addr;
            *downstreamptr++ = count << 8;
            *downstreamptr++ = count;
            *downstreamptr++ = numBytes;
            // Then copy in the bytes from the upstream.
            if (bitsForThisUnit == 8) {
                // Degenerate use case that aligns on byte boundaries.
                coilData++;  // Skip the first byte, as we consumed it all
                while (numBytes--) {
                    *downstreamptr++ = *coilData++;
                }
            } else {
                // We need part from the current byte, and part from the next byte
                while (numBytes--) {
                    // If its the last byte, we don't have a next byte, so use 0.
                    uint8_t next = numBytes == 0 ? 0 : coilData[1];
                    *downstreamptr++ = (*coilData >> bitsForThisUnit) | (next << (8 - bitsForThisUnit));
                    coilData++;
                }
            }
        }
    } else {
        // All of the bytes are for downstream. Copy them through but with a lower address.
        addr -= 8;
        // We need to pass all but the first 8 coils on to the next controller
        *downstreamptr++ = modbusbuf[0];  // Device ID
        *downstreamptr++ = MODBUS_SET_COILS_CMD;
        *downstreamptr++ = addr << 8;
        *downstreamptr++ = addr;
        *downstreamptr++ = count << 8;
        *downstreamptr++ = count;
        *downstreamptr++ = numBytes;

        // Then copy in the bytes from the upstream.
        for (int i = 0; i < numBytes; i++) {
            *downstreamptr++ = *coilData++;
        }
    }
    // The respnose is simply the input bytes, but only up to the address (6 bytes).
    // We need to change the CRC.
    appendCrc(modbusbuf, 6);
    modbus_sz = 8;
}

static void processSetSingleCoilCommand() {
    uint16_t addr = modbusbuf[2] << 8 | modbusbuf[3];
    uint8_t val = modbusbuf[4];

    if (addr < 8) {
        setCoil(addr, val);
        latchOutputs();
    } else {
        // Its for a downstream unit. Pass the command on.
        addr -= 8;
        *downstreamptr++ = deviceId;  // Device ID
        *downstreamptr++ = MODBUS_SET_SINGLE_COIL_CMD;
        *downstreamptr++ = addr << 8;
        *downstreamptr++ = addr;
        *downstreamptr++ = val;  // The value
        *downstreamptr++ = 0;
    }
}

static void processModbusCommand() {
    downstreamptr = downstreambuf;

    switch (modbusbuf[1]) {
        case MODBUS_GET_COILS_CMD:
            processGetCoilsCommand();
            break;

        case MODBUS_SET_COILS_CMD:
            processSetCoilsCommand();
            break;

        case MODBUS_SET_SINGLE_COIL_CMD:
            processSetSingleCoilCommand();
            break;
    }

    // Buffer the downstream command, if there is one.
    uint8_t sz = downstreamptr - downstreambuf;
    if (sz) {
        appendCrc(downstreambuf, sz);
        softserial_tx(downstreambuf, sz + 2);
    }
}

int main(void) {
    // Set up Relay outputs, which are split amongst PA and PB
    PORTA.OUTCLR = PIN2_bm | PIN3_bm | PIN4_bm | PIN5_bm | PIN6_bm | PIN7_bm;
    PORTA.DIRSET = PIN2_bm | PIN3_bm | PIN4_bm | PIN5_bm | PIN6_bm | PIN7_bm;
    PORTB.OUTCLR = PIN4_bm | PIN5_bm;
    PORTB.DIRSET = PIN4_bm | PIN5_bm;

    // Set up USART to operate at 9600
    PORTB.OUTSET = PIN2_bm;            // PB2 defaults high, because its UART out.
    PORTB.DIRSET = PIN2_bm | PIN0_bm;  // PB0 (XDIR) and PB2 (TXD) are outputs
    PORTB.DIRCLR = PIN3_bm;            // PB3 (RXD) is an input

    USART0.BAUD = USART0_BAUD_RATE(MODBUS_BAUD);  // Set Baud rate to 9600
    USART0.CTRLA = USART_CTRLA_OUTPUT_BUFFER_EMPTY;
    USART0.CTRLB = USART_RXEN_bm | USART_TXEN_bm | USART_SFDEN_bm;  // enable reception, and wake from sleep

    // Software Serial Setup, @9600 baud using TCA0
    PORTB.OUTSET = PIN1_bm;  // PB1 defaults to high
    PORTB.DIRSET = PIN1_bm;  // PB1 is an output
    TCA0.SINGLE.PER = SOFTSERIAL_PER;
    TCA0.SINGLE.CTRLC = TCA_SINGLE_CMP1OV_bm;  // Default to high
    TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc | TCA_SINGLE_CMP1EN_bm;
    TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;  // Interrupt on OVF

    // Set up TCB0 to handle timeout duties for us.
    // The standard says that if there is a gap of 1.5 characters or more between two the end of one byte and the start of
    // another, it is considered invalid. We receive notification at the end of each character, so we use 2.5 characters as our
    // timeout.
    TCB0.CCMP = USEC_TO_TICKS(25000000.0 / MODBUS_BAUD) / 2;
    TCB0.INTCTRL = TCB_CAPT_bm;  // Enable overflow interrupt.

    set_sleep_mode(SLEEP_MODE_STANDBY);
    sei();  // Enable Interrupts

    // Device ID is stored in the first byte of the USERRROW EEPROM.  It is offset by 2, so that when set to the
    // default value (0xFF), it comes out as Device no 1.  If you want device 2, write 0x00
    deviceId = USERROW.USERROW0 + 2;

    for (;;) {
        // If we're not transmitting anything, then go to sleep
        if (!transmissionFlags) {
            sleep_mode();
        }

        // Process the input buffer, if it is valid
        if (modbus_sz == 1) {
            // We just read our first character. Turn on the read timeout timer.
            TCB0.CTRLA = TCB_RUNSTDBY_bm | TCB_CLKSEL_CLKDIV2_gc | TCB_CNTMODE_INT_gc | TCB_ENABLE_bm;
        }
        // Check to see if we have received a valid MODBUS packet
        if (inputIsValidModbusCommand()) {
            cli();  // Turn off interrupts while we process the packet.
            processModbusCommand();

            // Respond to the caller, unless it was a broadcast
            if (modbusbuf[0]) {
                for (uint8_t i = 0; i < modbus_sz; i++) {
                    send(modbusbuf[i]);
                }
            }
            // Reset the input buffer to allow for a new command.
            resetModbusBuf();
            sei();
        }
    }
}

ISR(TCB0_INT_vect) {
    // We haven't received a character in a while.  Reset the input buffer
    TCB0.CTRLA = 0;               // Turn off the timer.
    TCB0.INTFLAGS = TCB_CAPT_bm;  // Clear the interrupt flag.
    TCB0.CNT = 0;                 // Set the timer back to 0 so that it starts there when we next turn the timer on
    resetModbusBuf();
}

// USART output shift register is empty.  If we have further bytes in the output buffer, buffer the next one, otherwise return
// to idle. This ISR is turned on whenever a character is added to the output buffer.
ISR(USART0_DRE_vect) {
    if (outhead < outtail) {
        USART0.TXDATAL = *outhead++;
    } else {
        // Disable the Data Ready Interrupt until there is more data to transmit.
        USART0.CTRLA = USART_CTRLA_OUTPUT_BUFFER_EMPTY;
        // Start from the start of the buffer again
        outhead = outtail = outbuf;
    }
}

// There's no more data to be shifted out, so we return to the idle state.
ISR(USART0_TXC_vect) {
    transmissionFlags &= ~TXFLAG_UART;
    // Clear the flag. The Datasheet says  this happens automatically, but this appears to be untrue
    USART0.STATUS = USART_TXCIF_bm;
}

// There is a new character to read from the input buffer.
ISR(USART0_RXC_vect) {
    if (USART0.STATUS & USART_RXCIF_bm) {
        // Read the byte directly into the output buffer.
        uint8_t val = USART0.RXDATAL;
        if (modbus_sz < MAX_PKT_SZ) {
            *modbusptr++ = val;  // This will clear the Interrupt flag;
            modbus_sz++;
            uint8_t xor = val ^ modbuscrc;
            modbuscrc >>= 8;
            modbuscrc ^= crc_table[xor];
        }
        TCB0.CNT = 0;  // Reset the read timeout timer.
    }
    if (USART0.STATUS & USART_RXSIF_bm) {
        // Clear the flag
        USART0.STATUS = USART_RXSIF_bm;
    }
}

ISR(TCA0_OVF_vect) {
    switch (--softserial_remaining_bits) {
        case 2:
            // B7 has been set as CMP1. Set next bit to a 1 (Stop bit)
            TCA0.SINGLE.CMP1BUF = SOFTSERIAL_CMPVAL_ONE;
            break;
        case 1:
            // STOP bit has been set as CMP1.  Check to see if there's another byte still to send.
            // Set to 0 (a start bit) if there is, or 1 otherwise (leave it high).
            TCA0.SINGLE.CMP1BUF = softserial_remaining_bytes ? 0 : SOFTSERIAL_CMPVAL_ONE;
            break;
        case 0:
            // START bit has been set as CMP1. Set next bit to Bit 0 of the new byte
            if (softserial_remaining_bytes) {
                softserial_byte = *softserial_ptr++;
                softserial_remaining_bytes--;
                softserial_remaining_bits = 10;
                TCA0.SINGLE.CMP1BUF = softserial_byte & 0x01 ? SOFTSERIAL_CMPVAL_ONE : 0;  // Buffer next Period.
            } else {
                TCA0.SINGLE.CTRLA = 0;              // Stop the timer.
                transmissionFlags &= ~TXFLAG_SOFT;  // Mark transmission as complete.
            }
            break;
        default:
            softserial_byte >>= 1;
            TCA0.SINGLE.CMP1BUF = softserial_byte & 0x01 ? SOFTSERIAL_CMPVAL_ONE : 0;  // Buffer next Period.
            break;
    }
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;  // Clear interrupt flag
}