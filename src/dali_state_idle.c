#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <stdbool.h>
#include <string.h>
#include <util/atomic.h>
#include <util/delay.h>
#include "timers.h"

#include "dali.h"


#define RESPONSE_DELAY USEC_TO_TICKS(10000)
#define CMD_Q_SIZE 5
static dali_command_t commandQ[CMD_Q_SIZE];
static int q_size = 0;
static bool idle = false;

static void cleanup() {
    dali_call_on_rising_once(NULL);
    idle = false;
}

bool dali_idle() { return idle; }


void send_command_response(dali_response_type_t err, uint8_t res) {
    if (q_size > 0) {
        commandQ[0].responseHandler(err, commandQ[0].cmd, res);
        if (q_size >= 1) {
            memcpy(commandQ, commandQ+1, (--q_size)*sizeof(dali_command_t));
        }
    }
}

static void check_for_commands_to_send() {
    if (q_size > 0) {
        // A command has been enqueued.
        cleanup();
        dali_transmitting_state_enter(commandQ);
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
dali_result_t dali_queue_command(uint16_t cmd, dali_response_callback_t responseHandler) {

    // TODO make the queue static
    if (q_size >= CMD_Q_SIZE) {
        return DALI_ERR_QUEUEFULL;
    }
    commandQ[q_size].cmd = cmd;
    commandQ[q_size++].responseHandler = responseHandler;

    if (idle) {
        check_for_commands_to_send();
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
        idle = false;
        check_for_commands_to_send();
    } else {
        // Otherwise, wait for a transmission to be received, or button to be pressed.
        idle = true;
        dali_state_receiving_prepare();

        // Configure RX to interrupt on Rising Edge, notifying us - which we then use to transition to reading_state.
        dali_call_on_rising_once(externalDaliMessageStarted);
    }
}