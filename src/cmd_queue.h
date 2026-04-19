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

void         cmdq_init(void);
bool         cmdq_empty(void);
bool         cmdq_full(void);
uint8_t      cmdq_count(void);

sys_result_t cmdq_push(const cmd_entry_t *e);
bool         cmdq_peek(cmd_entry_t *e);   // false if empty
void         cmdq_pop(void);

// Convenience push helpers
sys_result_t cmdq_push_arc(uint8_t dali_addr, uint8_t level, uint8_t ballast_idx);
sys_result_t cmdq_push_cmd(uint8_t dali_addr, uint8_t dali_cmd, uint8_t ballast_idx);
sys_result_t cmdq_push_twice(uint8_t dali_addr, uint8_t dali_cmd, uint8_t ballast_idx);
sys_result_t cmdq_push_query(uint8_t dali_addr, uint8_t dali_cmd,
                              uint8_t ballast_idx, reg_target_t target);
sys_result_t cmdq_push_query_cb(uint8_t dali_addr, uint8_t dali_cmd,
                                 uint8_t ballast_idx, reg_target_t target,
                                 cmd_result_cb_t cb);

// After pushing, kick the DALI state machine if it is idle.
// Safe to call from main-loop context.
void cmdq_kick(void);

#endif
