#ifndef CMD_QUEUE_H
#define CMD_QUEUE_H

#include <stdint.h>
#include <stdbool.h>
#include "types.h"

#define CMD_QUEUE_SIZE 8   // must be a power of 2

typedef struct {
    cmd_entry_t entries[CMD_QUEUE_SIZE];
    uint8_t     head;
    uint8_t     tail;
} cmd_queue_t;

extern cmd_queue_t g_cmd_queue;

// Initialise the queue to empty.
void         cmdq_init(void);

// Returns true when no entries are pending.
bool         cmdq_empty(void);

// Returns true when the queue has no room for another entry.
bool         cmdq_full(void);

// Returns the number of entries currently queued.
uint8_t      cmdq_count(void);

// Copy *entry onto the tail; returns SYS_ERR_QUEUE_FULL if full.
sys_result_t cmdq_push(const cmd_entry_t *entry);

// Copy the head entry into *entry without removing it; returns false if empty.
bool         cmdq_peek(cmd_entry_t *entry);

// Remove the head entry.  No-op if empty.
void         cmdq_pop(void);

// Push a Direct Arc Power Control (DAPC) command - sets arc level directly.
// dali_addr: encoded DALI address byte (use DALI_SHORT_ADDR / DALI_GROUP_ADDR / DALI_BROADCAST).
// ballast_idx: register map index (0-63), or 0xFF for group/broadcast commands.
sys_result_t cmdq_push_arc(uint8_t dali_addr, uint8_t level, uint8_t ballast_idx);

// Push a single-send command with no register update on completion.
sys_result_t cmdq_push_cmd(uint8_t dali_addr, uint8_t dali_cmd, uint8_t ballast_idx);

// Push a command that must be sent twice (required by DALI for configuration commands).
sys_result_t cmdq_push_twice(uint8_t dali_addr, uint8_t dali_cmd, uint8_t ballast_idx);

// Push a query command; on DALI_RESPONSE_OK the response byte is stored in reg[ballast_idx][target].
sys_result_t cmdq_push_query(uint8_t dali_addr, uint8_t dali_cmd,
                              uint8_t ballast_idx, reg_target_t target);

// Push a query command with an additional completion callback (e.g. for scan sequencing).
// The callback fires from ISR context; keep it short.
sys_result_t cmdq_push_query_cb(uint8_t dali_addr, uint8_t dali_cmd,
                                 uint8_t ballast_idx, reg_target_t target,
                                 cmd_result_cb_t cb);

// If the DALI state machine is idle and the queue is non-empty, begin transmission.
// Call from main-loop context after pushing one or more commands.
void cmdq_kick(void);

#endif
