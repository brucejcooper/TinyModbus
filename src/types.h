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

typedef enum __attribute__((packed)) {
    CMD_DIRECT_ARC,   // DAPC - no backward frame expected
    CMD_SEND,         // single send, no backward frame expected
    CMD_SEND_TWICE,   // two sends within ~50ms (DALI config commands)
    CMD_QUERY,        // single send, backward frame expected
} cmd_type_t;

// Which config-field a full-scan query response should update.  Used only
// by the OP_FULL_SCAN completion path in dali_sync.c.
//
// Note: there is intentionally no REG_FADE_TIME.  In DALI 1.0, fade time
// and fade rate are returned as a single byte from QUERY_FADE_TIME_FADE_RATE
// (0xA5): high nibble = fade time, low nibble = fade rate.  REG_FADE_RATE
// stores the response byte and regs_apply_result splits the nibbles into
// block3.fade_time and block3.fade_rate.  (DALI opcode 0xA6 is reserved.)
typedef enum __attribute__((packed)) {
    REG_NONE,
    REG_MIN,
    REG_MAX,
    REG_POWER_ON,
    REG_SYS_FAIL,
    REG_FADE_RATE,
    REG_GROUP_0_7,
    REG_GROUP_8_15,
} reg_target_t;

// In-flight DALI command stash.  Populated by issue_dali_command and read by
// the bit-level state machine (dali_state_transmitting reads dali_addr /
// dali_cmd / type; dali_state_idle reads repeat for SEND_TWICE re-fire).
typedef struct {
    cmd_type_t      type;
    uint8_t         dali_addr;    // encoded DALI address byte
    uint8_t         dali_cmd;     // command byte or arc level
    uint8_t         repeat;       // remaining sends: 1 normally, 2 for CMD_SEND_TWICE
} cmd_entry_t;

#endif
