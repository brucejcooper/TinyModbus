#ifndef TYPES_H
#define TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef enum {
    DALI_RESPONSE_OK = 0,
    DALI_RESPONSE_NAK,
    DALI_RESPONSE_MALFORMED,
    DALI_RESPONSE_PULSE_TOO_SHORT,
    DALI_RESPONSE_PULSE_IN_MIDDLE,
    DALI_RESPONSE_PULSE_TOO_LONG,
    DALI_RESPONSE_PENDING = 0xFF,
} dali_response_type_t;

// How one forward frame is transmitted. This is the ONLY classification of a
// send; there used to be a second, identical enum (cmd_type_t: CMD_SEND,
// CMD_SEND_TWICE, CMD_QUERY, CMD_DIRECT_ARC) that issue_dali_command converted
// to one-for-one, so every send was classified twice under two sets of names.
//
// The bit-level machine asks exactly one question of this: does the response
// window stay open after the last stop bit (QUERY awaits a backward frame,
// TWICE borrows the same window as its 22 TE inter-send gap)? A DAPC value also
// existed and is gone with the duplicate enum — no caller ever produced it, and
// a direct-arc frame is transmitted identically to any other answerless frame.
typedef enum __attribute__((packed)) {
    DALI_KIND_CMD = 0,   // single send, no backward frame
    DALI_KIND_TWICE,     // two sends within ~50 ms (DALI SET-class commands)
    DALI_KIND_QUERY,     // single send, backward frame expected
} dali_kind_t;

// ONE STEP of the DALI channel — the unit the bit-level state machine actually
// transmits. Named cmd_entry_t until this pass, a fossil of the v1 cmd_queue
// that was deleted in task 05; the name invited the reasonable question of why
// it duplicates a queue entry.
//
// It is NOT a duplicate of the queue's command. A queued command carrying
// QF_DTR0 is TWO steps — a DTR0 preload, then the frame — sharing one slot
// lifecycle, and the two steps have different kinds (the preload never awaits an
// answer even when the frame does). So the queue entry describes the command and
// this describes the send currently on the wire. `addr`/`data` match the names in
// dali_frame_t deliberately: for a single-step command they hold exactly those
// bytes.
typedef struct {
    dali_kind_t kind;
    uint8_t     addr;    // encoded DALI address byte
    uint8_t     data;    // command byte or arc level
    uint8_t     repeat;  // remaining sends: 1 normally, 2 for DALI_KIND_TWICE.
                         // A live countdown (dali_result_received decrements it
                         // to re-fire), so it is state — but its INITIAL value is
                         // implied by `kind`, which is why issue_dali_command no
                         // longer takes it as a parameter.
} dali_step_t;

#endif
