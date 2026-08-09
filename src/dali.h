#ifndef DALI_H
#define DALI_H

#include <inttypes.h>
#include <stdbool.h>
#include <avr/io.h>
#include "tick.h"

// How the bit-level receiver classified a transaction. Lives here, in the
// bit-level layer that PRODUCES it (dali_result_received below takes it), not in
// the issuance layer — dali_sync.h already includes this header, so the reverse
// would be circular.
typedef enum {
    DALI_RESPONSE_OK = 0,
    DALI_RESPONSE_NAK,
    DALI_RESPONSE_MALFORMED,
    DALI_RESPONSE_PULSE_TOO_SHORT,
    DALI_RESPONSE_PULSE_IN_MIDDLE,
    DALI_RESPONSE_PULSE_TOO_LONG,
    DALI_RESPONSE_PENDING = 0xFF,
} dali_response_type_t;

// ---- Protocol constants ------------------------------------------------

#define DALI_BAUD             1200
#define DALI_BIT_USECS        (1000000.0 / DALI_BAUD)
#define DALI_HALF_BIT_USECS   (DALI_BIT_USECS / 2.0)

// ---- What the gateway knows about DALI: almost nothing ---------------------
//
// There is no opcode table here — no address-encoding macros, no command
// opcodes, no commissioning specials. That is deliberate. The gateway is a
// dumb pipe: the controller composes complete 16-bit forward frames and submits
// them as opaque queue commands, and this firmware shifts the bytes onto the
// wire without interpreting them. Keeping an opcode table here implied a
// vocabulary the gateway does not have, and the commissioning block documented
// itself as "driven by the OP_COMMISSION state machine in dali_sync.c" — a state
// wire without interpreting them. Anything needed belongs on the controller
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

// IS SOMETHING GOING ON ON THE DALI BUS? That is the whole question this
// answers, and the only thing outside the bit-level machine needs to know about
// the wire. True while we are transmitting, while a response window is open,
// while the mandatory 22 TE quiet period is counting down, and while another
// master or a fault holds the line. False only when parked: TX released,
// receiver armed, monitor watching, nothing outstanding.
//
// IT STAYS TRUE THROUGH THE POST-COMMAND SETTLE, and that is the point. DALI
// requires a quiet gap before the next frame, so the bus is not free the instant
// a transaction ends. This flag — not the queue's state — is what stops the next
// command starting too early.
//
// Deliberately decoupled from when the MASTER learns the outcome. The slot goes
// terminal in dali_sync_completion the moment the transaction ends (right after
// the last stop bit for an answerless frame, or when the response window
// closes), so a master can collect its result and submit the next command while
// this is still true for our settle. Publishing the result and freeing the wire
// are two different events and must not be tied together — tying them would add
// the whole 22 TE to every round trip the master sees.
//
// It says nothing about whether a command is in flight; the queue owns that, as
// CQ_EXECUTING, via dali_sync_owns_channel(). Between the frames of a
// send-twice the bus IS busy and a command IS in flight; while another master
// sends, the bus is busy and no command is. Both facts are needed and neither
// implies the other.
//
// LOAD-BEARING for sleep: false means no TCA0 deadline is outstanding. TCA0 has
// no RUNSTDBY bit and so STOPS in standby — see the sleep gate in main().
extern volatile bool dali_bus_busy;

// Initialise TX/RX pins and TCB0 edge-capture, then enter wait-for-idle to let
// the bus settle after reset. (The RTC PIT lives in sysinfo.c — it drives the
// uptime counter, not anything DALI.)
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
// Shift one forward frame onto the wire. `hold_window` keeps the response
// window open after the last stop bit (a QUERY awaiting a backward frame, or a
// send-twice borrowing it as the 22 TE inter-send gap). The caller derives all
// three from the queued command — this layer holds no copy of them.
void dali_transmitting_state_enter(uint8_t addr, uint8_t data, bool hold_window);

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

// Single authority for post-command state transitions — called from ISR context.
// Offers the issuance layer a chance to re-fire (the second send of a
// send-twice), otherwise completes the command and enters the next state. The
// register-update / callback / queue-pop duties are not this layer's.
void dali_result_received(dali_response_type_t type, uint8_t value);

#endif
