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


void process_command_response(dali_response_type_t err, uint8_t res) {
    uint8_t addr;
    if (q_size > 0) {
        // We don't set the callback when we are repeating ourselves, because we need to send the command again (once we re-enter the idle state)
        if (commandQ->repeat == 0 || --(commandQ->repeat) == 0) {
            // In addition to calling the callback, we automatically mirror the output of some DALI commands to our holding registers.  
            // This makes it easier to process when also calling Custom Dali commands. 
            addr = DALI_DECODE_ADDR(commandQ->cmd);
            switch (commandQ->cmd & 0x81FF) {
                // Query Actual Level
                case 0x1a0:
                    dali_holding_regs.power[addr].low = err != DALI_RESPONSE_OK ? 255 : res;
                    break;
                // Query Status
                case 0x190:
                    dali_holding_regs.power[addr].high = err != DALI_RESPONSE_OK ? 0 : res;
                    break;
            }

            // Call the callback handler, if there is one.
            if (commandQ->callback) {
                commandQ->callback(err, commandQ->cmd, res);
            }
            // Delete the entry from the queue.
            q_size--;
            if (q_size > 0) {
                memcpy(commandQ, commandQ + 1, q_size * sizeof(dali_command_t));
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
    if (q_size >= CMD_Q_SIZE) {
        return DALI_ERR_QUEUEFULL;
    }
    commandQ[q_size].cmd = cmd;
    commandQ[q_size].callback = callback;
    commandQ[q_size++].repeat = repeat < 2 ? 1 : 2;     // Min 1 times, max 2.

    if (dali_idle) {
        cleanup();
        dali_transmitting_state_enter(commandQ);
    }
    return DALI_OK;
}

void externalDaliMessageStarted() {
    // We've started receiving something transmitted by another controller.  we don't care about anything that
    // anyone else sends, so simply wait for it to stop. However, we don't want to start transmitting while the
    // other master is doign something, so we simply read in the command (and its response) and ignore it.
    cleanup();
    dali_wait_for_idle_state_enter();
}

void dali_idle_state_enter() {
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