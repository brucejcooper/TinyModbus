#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <stdbool.h>
#include <string.h>
#include <util/atomic.h>
#include <util/delay.h>

#include "dali.h"

#define CMD_Q_SIZE 20
static dali_command_t commandQ[CMD_Q_SIZE];
static volatile uint8_t q_size = 0;
volatile bool dali_idle = false;

static void cleanup() {
    dali_idle = false;
    dali_cancel_edge_detect();
}

static const uint16_t command_bank_mapping[] = {
    DALI_CMD_GET_LEVEL,
    DALI_CMD_QUERY_STATUS,
    DALI_CMD_GET_MIN,
    DALI_CMD_GET_MAX,
    DALI_CMD_GET_FADE,
    DALI_CMD_GET_EXT_FADE,
    DALI_CMD_GET_POWER_ON_LEVEL,
    DALI_CMD_GET_POWER_FAIL_LEVEL,
    DALI_CMD_GET_GROUPS0_7,
    DALI_CMD_GET_GROUPS8_15,
};


#define NUM_COMMAND_MAPPINGS (sizeof(command_bank_mapping) / sizeof(uint16_t))

void process_command_response(dali_response_type_t err, uint8_t res) {
    if (q_size > 0) {
        // We don't set the callback when we are repeating ourselves, because we need to send the command again (once we re-enter the idle state)
        if (commandQ->repeat == 0 || --(commandQ->repeat) == 0) {
            // In addition to calling the callback, we automatically mirror the output of some DALI commands to our holding registers.
            // This makes it easier to process when also calling Custom Dali commands.
            uint16_t masked = commandQ -> cmd & 0x81FF;
            // It was a device command
            // The results of some commands map to holding registers.  We update them.
            int i;
            for (i = 0; i < NUM_COMMAND_MAPPINGS; i++) {
                if (masked == command_bank_mapping[i]) {
                    uint8_t addr = DALI_DECODE_ADDR(commandQ->cmd);
                    uint8_t *ptr = ((uint8_t *) &dali_holding_regs.banks[i/2][addr]) + (i%2);
                    *ptr = res;
                    break;
                }
            }

            // Call the callback handler, if there is one.
            if (commandQ->callback) {
                commandQ->callback(err, commandQ->cmd, res);
            }
            // Delete the entry from the queue. The source and destination overlap, so use
            // memmove rather than memcpy — avr-libc's memcpy happens to do a forward copy
            // (which is safe in this direction) but the C standard treats overlapping memcpy
            // as undefined.
            q_size--;
            if (q_size > 0) {
                memmove(commandQ, commandQ + 1, q_size * sizeof(dali_command_t));
            }
        }
    }
}

/**
 * Enqueues a command to be sent on the DALI bus, when it becomes available.
 *
 * @param data
 * @param numBits can be 16 or 24 - 16 bits are gear commands, while 24 is a control command.
 * @param responseHandler called once we're done, with the response (if any).  Also called if there is a collision
 * @return dali_result_t
 */

dali_result_t dali_queue_command(uint16_t cmd, uint8_t repeat, dali_callback_t callback) {
    bool startTx = false;
    dali_result_t result;

    // Queue manipulation must be atomic w.r.t. ISR contexts: this function is called from main
    // (Modbus handlers) and from TCA0/TCB0 ISR callbacks (e.g. scan_result_received via
    // process_command_response). Without protection, main could read q_size, an ISR could fire
    // and shrink+compact the queue, then main would write to a freed slot. We also claim the
    // "start TX" role atomically by clearing dali_idle in the same block, so concurrent callers
    // can't both decide to start a transmission.
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (q_size >= CMD_Q_SIZE) {
            result = DALI_ERR_QUEUEFULL;
        } else {
            commandQ[q_size].cmd = cmd;
            commandQ[q_size].callback = callback;
            commandQ[q_size++].repeat = repeat < 2 ? 1 : 2;  // Min 1 times, max 2.
            result = DALI_OK;
            if (dali_idle) {
                dali_idle = false;
                startTx = true;
            }
        }
    }

    if (startTx) {
        // dali_idle was already cleared inside the atomic block; just disable the rising-edge
        // detector and start the transmission.
        dali_cancel_edge_detect();
        dali_transmitting_state_enter(commandQ);
    }
    return result;
}

void externalDaliMessageStarted() {
    // We've started receiving something transmitted by another controller.  we don't care about anything that
    // anyone else sends, so simply wait for it to stop. However, we don't want to start transmitting while the
    // other master is doign something, so we simply read in the command (and its response) and ignore it.
    cleanup();
    dali_wait_for_idle_state_enter();
}

void dali_idle_state_enter() {
    // pulse_led(5);
    // If we've something to send, send it immediately
    if (q_size > 0) {
        // Immediately start transmitting the next command.
        dali_transmitting_state_enter(commandQ);
    } else {
        dali_idle = true;
        // Otherwise, wait for a transmission to be received, or button to be pressed.
        dali_state_receiving_prepare();
        // Configure RX to interrupt on Rising Edge, notifying us - which we then use to transition to reading_state.
        dali_call_on_rising_once(externalDaliMessageStarted);
    }
}