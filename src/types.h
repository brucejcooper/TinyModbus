#ifndef TYPES_H
#define TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef enum {
    DALI_RESPONSE_OK = 0,
    DALI_RESPONSE_NAK,
    DALI_RESPONSE_COLLISION,
    DALI_RESPONSE_MALFORMED,
    DALI_RESPONSE_PULSE_TOO_SHORT,
    DALI_RESPONSE_PULSE_IN_MIDDLE,
    DALI_RESPONSE_PULSE_TOO_LONG,
    DALI_RESPONSE_PENDING = 0xFF,
} dali_response_type_t;

typedef enum __attribute__((packed)) {
    CMD_DIRECT_ARC,   // DAPC — no backward frame expected
    CMD_SEND,         // single send, no backward frame expected
    CMD_SEND_TWICE,   // two sends within ~50ms (DALI config commands)
    CMD_QUERY,        // single send, backward frame expected
} cmd_type_t;

// Which register field a query response should update
typedef enum __attribute__((packed)) {
    REG_NONE,
    REG_BRIGHTNESS,
    REG_STATUS,
    REG_MIN,
    REG_MAX,
    REG_POWER_ON,
    REG_SYS_FAIL,
    REG_FADE_RATE,
    REG_FADE_TIME,
    REG_GROUP_0_7,
    REG_GROUP_8_15,
} reg_target_t;

// Callback for complex multi-step operations (scan, fade tracking).
// Called from ISR context; must be short.
typedef void (*cmd_result_cb_t)(dali_response_type_t type, uint8_t ballast_idx, uint8_t response);

typedef struct {
    cmd_type_t      type;
    uint8_t         dali_addr;    // encoded DALI address byte
    uint8_t         dali_cmd;     // command byte or arc level
    uint8_t         ballast_idx;  // 0–63; 0xFF for non-ballast cmds
    reg_target_t    reg_target;   // where to store query response
    uint8_t         repeat;       // remaining sends: 1 normally, 2 for CMD_SEND_TWICE
    cmd_result_cb_t on_result;    // NULL for most commands; used by scan/fade
} cmd_entry_t;

typedef enum {
    SYS_OK = 0,
    SYS_ERR_QUEUE_FULL,
    SYS_ERR_BAD_ADDR,
    SYS_ERR_BAD_VALUE,
} sys_result_t;

#endif
