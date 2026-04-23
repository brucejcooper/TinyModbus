#include <avr/io.h>
#include <util/atomic.h>
#include "cmd_queue.h"
#include "dali.h"

cmd_queue_t g_cmd_queue;

void cmdq_init(void) {
    g_cmd_queue.head = 0;
    g_cmd_queue.tail = 0;
}

bool cmdq_empty(void) {
    return g_cmd_queue.head == g_cmd_queue.tail;
}

bool cmdq_full(void) {
    return ((g_cmd_queue.tail + 1u) & (CMD_QUEUE_SIZE - 1u)) == g_cmd_queue.head;
}

uint8_t cmdq_count(void) {
    return (uint8_t)((g_cmd_queue.tail - g_cmd_queue.head) & (CMD_QUEUE_SIZE - 1u));
}

sys_result_t cmdq_push(const cmd_entry_t *entry) {
    // Protect tail update from concurrent pushes from any interrupt level.
    // Saving/clearing TCB0.INTCTRL blocks level-1 preemption; ATOMIC_BLOCK blocks level-0.
    // Both are safe from ISR context: TCB0.INTCTRL restore is a no-op if already 0,
    // and ATOMIC_BLOCK restores I-flag to whatever it was on entry.
    uint8_t old_tcb_intctrl = TCB0.INTCTRL;
    TCB0.INTCTRL = 0;
    sys_result_t result;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (cmdq_full()) {
            result = SYS_ERR_QUEUE_FULL;
        } else {
            g_cmd_queue.entries[g_cmd_queue.tail] = *entry;
            g_cmd_queue.tail = (g_cmd_queue.tail + 1u) & (CMD_QUEUE_SIZE - 1u);
            result = SYS_OK;
        }
    }
    TCB0.INTCTRL = old_tcb_intctrl;
    return result;
}

bool cmdq_peek(cmd_entry_t *entry) {
    if (cmdq_empty()) return false;
    *entry = g_cmd_queue.entries[g_cmd_queue.head];
    return true;
}

void cmdq_pop(void) {
    if (!cmdq_empty())
        g_cmd_queue.head = (g_cmd_queue.head + 1u) & (CMD_QUEUE_SIZE - 1u);
}

sys_result_t cmdq_push_arc(uint8_t dali_addr, uint8_t level, uint8_t ballast_idx) {
    cmd_entry_t entry = {
        .type        = CMD_DIRECT_ARC,
        .dali_addr   = dali_addr,
        .dali_cmd    = level,
        .ballast_idx = ballast_idx,
        .reg_target  = REG_NONE,
        .repeat      = 1,
        .on_result   = NULL,
    };
    return cmdq_push(&entry);
}

sys_result_t cmdq_push_cmd(uint8_t dali_addr, uint8_t dali_cmd, uint8_t ballast_idx) {
    cmd_entry_t entry = {
        .type        = CMD_SEND,
        .dali_addr   = dali_addr,
        .dali_cmd    = dali_cmd,
        .ballast_idx = ballast_idx,
        .reg_target  = REG_NONE,
        .repeat      = 1,
        .on_result   = NULL,
    };
    return cmdq_push(&entry);
}

sys_result_t cmdq_push_twice(uint8_t dali_addr, uint8_t dali_cmd, uint8_t ballast_idx) {
    cmd_entry_t entry = {
        .type        = CMD_SEND_TWICE,
        .dali_addr   = dali_addr,
        .dali_cmd    = dali_cmd,
        .ballast_idx = ballast_idx,
        .reg_target  = REG_NONE,
        .repeat      = 2,
        .on_result   = NULL,
    };
    return cmdq_push(&entry);
}

sys_result_t cmdq_push_query(uint8_t dali_addr, uint8_t dali_cmd,
                              uint8_t ballast_idx, reg_target_t target) {
    cmd_entry_t entry = {
        .type        = CMD_QUERY,
        .dali_addr   = dali_addr,
        .dali_cmd    = dali_cmd,
        .ballast_idx = ballast_idx,
        .reg_target  = target,
        .repeat      = 1,
        .on_result   = NULL,
    };
    return cmdq_push(&entry);
}

sys_result_t cmdq_push_query_cb(uint8_t dali_addr, uint8_t dali_cmd,
                                 uint8_t ballast_idx, reg_target_t target,
                                 cmd_result_cb_t cb) {
    cmd_entry_t entry = {
        .type        = CMD_QUERY,
        .dali_addr   = dali_addr,
        .dali_cmd    = dali_cmd,
        .ballast_idx = ballast_idx,
        .reg_target  = target,
        .repeat      = 1,
        .on_result   = cb,
    };
    return cmdq_push(&entry);
}

void cmdq_kick(void) {
    if (dali_idle && !cmdq_empty())
        dali_idle_state_enter();
}
