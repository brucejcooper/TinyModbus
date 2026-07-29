#ifndef DALI_H
#define DALI_H

#include <inttypes.h>
#include <stdbool.h>
#include <avr/io.h>
#include "types.h"
#include "tick.h"

// ---- Protocol constants ------------------------------------------------

#define DALI_BAUD             1200
#define DALI_BIT_USECS        (1000000.0 / DALI_BAUD)
#define DALI_HALF_BIT_USECS   (DALI_BIT_USECS / 2.0)

// ---- What the gateway knows about DALI: almost nothing ---------------------
//
// Everything that used to live here — the address-encoding macros, the ~30
// command opcodes (OFF/UP/DOWN/RECALL/QUERY */SET *), the commissioning
// specials (INITIALISE, RANDOMISE, COMPARE, WITHDRAW, SEARCHADDRH/M/L,
// PROGRAM/VERIFY SHORT ADDRESS, TERMINATE) and their data-byte conventions —
// was deleted in this pass. Not one of them had a reference left.
//
// That is the intended end state of task 05, not an oversight. The gateway is a
// dumb pipe: the controller composes complete 16-bit forward frames and submits
// them as opaque queue commands, and this firmware shifts the bytes onto the
// wire without interpreting them. Keeping an opcode table here implied a
// vocabulary the gateway does not have, and the commissioning block documented
// itself as "driven by the OP_COMMISSION state machine in dali_sync.c" — a state
// machine that no longer exists. Anything needed again belongs on the controller
// side, which is where the frames are built.
//
// DTR0 is the single exception, and only because the QF_DTR0 flag makes the
// preload a GATEWAY-side two-step sequence (0xA3<value>, then the frame) inside
// one slot lifecycle. DTR1/DTR2 (0xC3/0xC5) need no constant here — the master
// submits them as ordinary command records.
#define DALI_ADDR_SET_DTR0  0xA3u

// ---- Hardware pins -----------------------------------------------------

// DALI_TX (PB0) polarity convention: OUTSET (PB0 HIGH) asserts the bus;
// OUTCLR (PB0 LOW) and high-Z release.  The external transistor inverts
// polarity between the MCU pin and the bus line, so PB0 LOW = bus released
// = idle/safe state.  Invariant maintained by every TX callback in the
// bit-level state machine (see dali_state_transmitting.c): PB0 is released
// within one TCA0 tick (~417 µs) of every OUTSET, by pairing each OUTSET
// with `dali_timer = 1` and a callback (half_bit_toggle / start_stop_bits)
// that OUTTGLs or OUTCLRs the line.  Any new OUTSET site MUST preserve
// this invariant or the TX-asserted watchdog in TCA0_OVF_vect (tick.c) is
// the only firmware-side backstop before the WDT (~8.2 s).
#define DALI_PORT         PORTB
#define DALI_TX_bm        PIN0_bm
#define DALI_RX_bm        PIN1_bm
#define DALI_RX_PINCTRL   PIN1CTRL

// ---- Public interface --------------------------------------------------

// True while the state machine is idle and able to accept new commands.
extern volatile bool dali_idle;

// Initialise TX/RX pins, TCB0 edge-capture, RTC PIT for periodic rescan,
// and enter wait-for-idle to let the bus settle after reset.
void    dali_init(void);

// Returns non-zero when another master is driving the bus (RX pin HIGH after optocoupler inversion).
uint8_t dali_is_bus_active(void);

// Arm a one-shot PORTB rising-edge interrupt; fires callback once then self-disarms.
void dali_call_on_rising_once(void (*callback)(void));

// Arm a one-shot PORTB low-level interrupt; fires callback once then self-disarms.
void dali_call_on_low_once(void (*callback)(void));

// Disarm the PORTB edge/level interrupt and clear the pending callback pointer.
void dali_cancel_edge_detect(void);

// If the queue is non-empty, begin transmitting the head command immediately.
// Otherwise arm the bus-monitor to detect external traffic.
void dali_idle_state_enter(void);

// Begin transmitting cmd as a DALI forward frame (Manchester-encoded, 1200 baud).
// Returns immediately; the timer ISR drives the rest of the transmission.
void dali_transmitting_state_enter(const dali_step_t *step);

// Enforce the mandatory 22 TE (~9 ms) inter-frame gap before the next TX.
// Handles bus-shorted (another master active) by waiting for it to release first.
// Has a 200 TE safety escape in case the bus never becomes idle.
void dali_wait_for_idle_state_enter(void);

// Arm TCB0 and a no-response timeout for the backward frame window that follows TX.
// Must be called immediately after the last stop bit is sent.
void dali_wait_for_response_state_enter(void);

// Configure TCB0 and the Manchester decoder without enabling the timer.
// Call this before the rising edge that starts the backward frame.
void dali_state_receiving_prepare(void);

// Enable TCB0 and begin decoding the backward frame that has already started.
// Assumes dali_state_receiving_prepare() was called first.
void dali_state_receiving_enter(void);

// Single authority for post-command state transitions - called from ISR context.
// Updates the target register (if any), fires the optional callback, decrements
// repeat count for DALI_KIND_TWICE, pops the queue, and enters the next state.
void dali_result_received(dali_response_type_t type, uint8_t value);

#endif
