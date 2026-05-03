// dali_sync.c — sync-flag-driven DALI work selector + unified command
// issuance + completion dispatch.
//
// Architectural model:
//   - Every DALI command leaves the firmware through issue_dali_command.
//     That function performs cache invalidation as a property of the
//     command bytes (not its source), then hands a stack-local
//     cmd_entry_t to dali_transmitting_state_enter.
//   - start_next_dali_work picks the highest-priority pending work
//     (raw -> group bit -> SYNC_WRITE_* -> SYNC_READ_LEVEL ->
//     full-scan continue -> full-scan new) and kicks off the first send.
//   - The bit-level state machine drives the send to completion.  When
//     it returns to idle, cmdq_result_received fires.  If dali_busy is
//     true, that routes to dali_sync_completion (here), which advances
//     the multi-step sequence or releases the channel.
//   - Multi-step writes (DTR0 -> SET_X x2 -> DTR0 -> SET_Y x2) run as
//     one atomic unit because DTR0 is shared.  Other multi-step ops
//     (level read, full-scan) yield between steps.
//
// Threading model:
//   - sync_flags[]: read by main (start_next_dali_work) and ISR (PIT,
//     completion handler); RMW only via sync_flags_set/clear.
//   - dali_busy / current_op / scan_full_*: written by start_next_dali_work
//     (main, under TCB0+ATOMIC) and by dali_sync_completion (ISR);
//     reads from main are atomic (1 byte) but compound checks need ATOMIC.
//   - raw_cmd_addr/data: written by dali_sync_arm_raw (main, ATOMIC),
//     read by dispatch (main when starting work).
//   - group_write: same pattern as raw_cmd_*.
//   - g_regs.raw_cmd_result: 16-bit; ALL RMW under ATOMIC + TCB0 disable
//     (cmdq_result_received runs at LVL1).
//   - g_regs.dirty[]: ISR-set, main-cleared via regs_read_clear_dirty
//     (already atomic).

#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdbool.h>
#include <stdint.h>
#include <util/atomic.h>

#include "dali.h"
#include "dali_sync.h"
#include "registers.h"

// ---- Globals ---------------------------------------------------------

volatile uint8_t           sync_flags[DALI_MAX_DEVICES];
volatile bool              dali_busy;
volatile dali_current_op_t current_op;
volatile dali_group_write_t group_write;
volatile cmd_entry_t       s_active;

volatile uint8_t raw_cmd_addr;
volatile uint8_t raw_cmd_data;

volatile uint8_t scan_full_addr;
volatile uint8_t scan_full_step;

// PIT round-robin cursor — advances at most one slot per fired tick.
static volatile uint8_t scan_cursor;

// Rotating cursor for the work-selector ballast walk.  Same cursor for
// SYNC_WRITE_* (priority 3) and SYNC_READ_LEVEL (priority 4) — they
// share the fairness property: each issued command (write or read)
// advances the cursor so high-numbered ballasts can't be starved by
// continuous low-numbered work.
static uint8_t work_cursor;

// ---- Init ------------------------------------------------------------

void dali_sync_init(void) {
    for (uint8_t i = 0; i < DALI_MAX_DEVICES; i++) {
        sync_flags[i] = 0;
    }
    dali_busy        = false;
    current_op.addr  = 0xFF;
    current_op.op    = OP_NONE;
    current_op.step  = 0;
    group_write.reg_addr        = 0xFFFFu;
    group_write.pending_adds    = 0;
    group_write.pending_removes = 0;
    raw_cmd_addr   = 0;
    raw_cmd_data   = 0;
    scan_full_addr = 0xFF;
    scan_full_step = 0;
    scan_cursor    = 0;
    work_cursor    = 0;
}

// ---- Cache-invalidation classification ------------------------------
//
// Every DALI command leaving issue_dali_command runs through this.
// Returns the SYNC_READ_* mask to set on the affected ballasts.
//
//  - DAPC (bit0 of addr clear)              -> SYNC_READ_LEVEL
//  - 0x00..0x08, 0x0A, 0x10..0x1F           -> SYNC_READ_LEVEL
//  - 0x20 RESET                             -> SYNC_READ_LEVEL | SYNC_READ_FULL
//  - 0x21 STORE_CURRENT_LEVEL_DTR0          -> 0
//  - 0x2A..0x2F SET_*                       -> SYNC_READ_FULL
//  - 0x40..0x7F SET_SCENE/REMOVE_SCENE/
//    ADD_TO_GROUP/REMOVE_FROM_GROUP         -> SYNC_READ_FULL
//  - 0x80 STORE_DTR_AS_SHORT_ADDRESS        -> SYNC_READ_FULL
//  - 0x90..0xFF queries / specials          -> 0
static uint8_t classify_invalidation(uint8_t addr_byte, uint8_t data_byte) {
    // DAPC: address bit 0 clear -> arc level command.
    if ((addr_byte & DALI_CMD_FLAG) == 0) return SYNC_READ_LEVEL;

    if (data_byte <= 0x08u)                 return SYNC_READ_LEVEL;
    if (data_byte == 0x0Au)                 return SYNC_READ_LEVEL;
    if (data_byte >= 0x10u && data_byte <= 0x1Fu) return SYNC_READ_LEVEL;
    if (data_byte == 0x20u)                 return SYNC_READ_LEVEL | SYNC_READ_FULL;
    if (data_byte == 0x21u)                 return 0;
    if (data_byte >= 0x2Au && data_byte <= 0x2Fu) return SYNC_READ_FULL;
    if (data_byte >= 0x40u && data_byte <= 0x7Fu) return SYNC_READ_FULL;
    if (data_byte == 0x80u)                 return SYNC_READ_FULL;
    return 0;
}

// Apply inv_mask to all ballasts targeted by addr_byte.
//   - bit 7 = 0          : short address, one ballast.
//   - 0x80..0x9F         : group (DAPC/cmd encoded in bit 0).  Walk the
//                          cached group_mask to find members.
//   - 0xA0..0xCF         : special command (DTR setters, INITIALISE,
//                          RANDOMISE, ...) — no per-ballast target.
//   - 0xFE / 0xFF        : broadcast, all 64 ballasts.
//   - other (0xD0..0xFD) : reserved; no-op.
static void apply_invalidation(uint8_t addr_byte, uint8_t inv_mask) {
    if (inv_mask == 0) return;

    if ((addr_byte & 0x80u) == 0) {
        sync_flags_set((uint8_t)((addr_byte >> 1) & 0x3Fu), inv_mask);
        return;
    }
    if (addr_byte < 0xA0u) {
        // Group.  group_mask[group][0..7] is a 64-bit per-group bitmap.
        uint8_t group = (uint8_t)((addr_byte >> 1) & 0x0Fu);
        for (uint8_t byte_idx = 0; byte_idx < 8; byte_idx++) {
            uint8_t mb = g_regs.group_mask[group][byte_idx];
            for (uint8_t bit = 0; bit < 8; bit++) {
                if (mb & (1u << bit))
                    sync_flags_set((uint8_t)((byte_idx << 3) | bit), inv_mask);
            }
        }
        return;
    }
    if (addr_byte == 0xFEu || addr_byte == 0xFFu) {
        for (uint8_t i = 0; i < DALI_MAX_DEVICES; i++)
            sync_flags_set(i, inv_mask);
        return;
    }
    // 0xA0-0xCF special command, or 0xD0-0xFD reserved: no-op.
}

// ---- Unified DALI issuance ------------------------------------------

__attribute__((noinline, noclone))
void issue_dali_command(uint8_t addr_byte, uint8_t data_byte,
                         dali_kind_t kind, uint8_t repeat) {
    // Cache invalidation runs BEFORE the command bytes hit the wire.
    uint8_t inv = classify_invalidation(addr_byte, data_byte);
    apply_invalidation(addr_byte, inv);

    cmd_type_t t;
    switch (kind) {
        case DALI_KIND_DAPC:  t = CMD_DIRECT_ARC; break;
        case DALI_KIND_CMD:   t = CMD_SEND;       break;
        case DALI_KIND_TWICE: t = CMD_SEND_TWICE; break;
        case DALI_KIND_QUERY: t = CMD_QUERY;      break;
        default:              t = CMD_SEND;       break;
    }
    s_active.type        = t;
    s_active.dali_addr   = addr_byte;
    s_active.dali_cmd    = data_byte;
    s_active.repeat      = repeat;
    s_active.ballast_idx = 0xFF;
    s_active.reg_target  = REG_NONE;
    s_active.on_result   = NULL;

    dali_transmitting_state_enter((const cmd_entry_t *)&s_active);
}

// ---- Multi-step write helpers ---------------------------------------
//
// Each multi-step write is 4 LOGICAL steps (= 6 hardware sends, with
// the SET twice-sends handled by the repeat-decrement re-fire in
// cmdq_result_received — same mechanism the legacy queue used):
//   step 0: DTR0 = low_byte               (CMD_SEND, repeat=1)
//   step 1: SET_X                          (CMD_SEND_TWICE, repeat=2)
//   step 2: DTR0 = high_byte               (CMD_SEND, repeat=1)
//   step 3: SET_Y                          (CMD_SEND_TWICE, repeat=2)
//
// The whole 4-step sequence runs without yielding because DTR0 is
// shared across all ballasts (an interleaving raw command or another
// write could clobber DTR0).

__attribute__((noinline, noclone))
static void issue_write_step(uint8_t op, uint8_t addr, uint8_t step) {
    uint8_t bidx = addr & 0x3Fu;
    uint8_t set_x, set_y, low_val, high_val;

    switch (op) {
        case OP_WRITE_MINMAX:
            set_x   = DALI_CMD_SET_MIN_LEVEL;
            set_y   = DALI_CMD_SET_MAX_LEVEL;
            low_val  = g_regs.block1[bidx].min;
            high_val = g_regs.block1[bidx].max;
            break;
        case OP_WRITE_POWER:
            set_x   = DALI_CMD_SET_POWER_ON_LEVEL;
            set_y   = DALI_CMD_SET_SYS_FAIL_LEVEL;
            low_val  = g_regs.block2[bidx].power_on;
            high_val = g_regs.block2[bidx].sys_fail;
            break;
        case OP_WRITE_FADE:
        default:
            set_x   = DALI_CMD_SET_FADE_RATE;
            set_y   = DALI_CMD_SET_FADE_TIME;
            low_val  = g_regs.block3[bidx].fade_rate;
            high_val = g_regs.block3[bidx].fade_time;
            break;
    }

    uint8_t short_addr = DALI_SHORT_ADDR(bidx) | DALI_CMD_FLAG;
    switch (step) {
        case 0: issue_dali_command(DALI_ADDR_SET_DTR0, low_val,  DALI_KIND_CMD,   1u); break;
        case 1: issue_dali_command(short_addr,         set_x,    DALI_KIND_TWICE, 2u); break;
        case 2: issue_dali_command(DALI_ADDR_SET_DTR0, high_val, DALI_KIND_CMD,   1u); break;
        case 3: issue_dali_command(short_addr,         set_y,    DALI_KIND_TWICE, 2u); break;
        default: break;
    }
}

// ---- Full-scan step table -------------------------------------------

typedef struct { uint8_t cmd; reg_target_t target; } scan_step_t;

static const scan_step_t scan_steps[7] = {
    { DALI_CMD_QUERY_MIN_LEVEL,            REG_MIN        },
    { DALI_CMD_QUERY_MAX_LEVEL,            REG_MAX        },
    { DALI_CMD_QUERY_FADE_TIME_FADE_RATE,  REG_FADE_RATE  },
    { DALI_CMD_QUERY_POWER_ON_LEVEL,       REG_POWER_ON   },
    { DALI_CMD_QUERY_FAIL_LEVEL,           REG_SYS_FAIL   },
    { DALI_CMD_QUERY_GROUP_0_7,            REG_GROUP_0_7  },
    { DALI_CMD_QUERY_GROUP_8_15,           REG_GROUP_8_15 },
};

// ---- Dispatch current op (called from idle / start_next_dali_work) --

void dali_sync_dispatch_current(void) {
    uint8_t op   = current_op.op;
    uint8_t addr = current_op.addr;
    uint8_t step = current_op.step;
    uint8_t short_addr = DALI_SHORT_ADDR(addr & 0x3Fu) | DALI_CMD_FLAG;

    switch (op) {
        case OP_WRITE_LEVEL: {
            // Single DAPC.  Address byte: short addr without bit0 (DAPC).
            uint8_t dapc_addr = DALI_SHORT_ADDR(addr & 0x3Fu);
            uint8_t level     = g_regs.block0[addr & 0x3Fu].brightness;
            issue_dali_command(dapc_addr, level, DALI_KIND_DAPC, 1u);
            break;
        }
        case OP_WRITE_MINMAX:
        case OP_WRITE_POWER:
        case OP_WRITE_FADE:
            issue_write_step(op, addr, step);
            break;
        case OP_LEVEL_READ:
            if (step == 0)
                issue_dali_command(short_addr, DALI_CMD_QUERY_STATUS,       DALI_KIND_QUERY, 1u);
            else
                issue_dali_command(short_addr, DALI_CMD_QUERY_ACTUAL_LEVEL, DALI_KIND_QUERY, 1u);
            break;
        case OP_FULL_SCAN: {
            uint8_t s = scan_full_step;
            issue_dali_command(short_addr, scan_steps[s].cmd, DALI_KIND_QUERY, 1u);
            break;
        }
        case OP_GROUP_BIT:
            // current_op.addr = ballast; current_op.step = data byte (0x60+g or 0x70+g).
            // ADD_TO_GROUP / REMOVE_FROM_GROUP are SET-class commands: send twice.
            issue_dali_command(short_addr, step, DALI_KIND_TWICE, 2u);
            break;
        case OP_RAW: {
            // raw_cmd_addr / raw_cmd_data hold the staged bytes.
            // sends_remaining encoded in the low byte of raw_cmd_result.
            // For a single send: KIND_QUERY (backward frame may arrive).
            // For twice: KIND_TWICE with repeat=2 (cmdq_result_received's
            // re-fire handles the second send within the 22 TE window).
            uint8_t  rc_lo;
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { rc_lo = (uint8_t)g_regs.raw_cmd_result; }
            if (rc_lo >= 2u)
                issue_dali_command(raw_cmd_addr, raw_cmd_data, DALI_KIND_TWICE, 2u);
            else
                issue_dali_command(raw_cmd_addr, raw_cmd_data, DALI_KIND_QUERY, 1u);
            break;
        }
        default:
            break;
    }
}

// ---- Work selection --------------------------------------------------

// Find the lowest set bit of mask (0..15); returns 16 if mask == 0.
static uint8_t lowest_bit_set(uint16_t mask) {
    for (uint8_t b = 0; b < 16; b++)
        if (mask & (1u << b)) return b;
    return 16;
}

// Map sync_flags write bit -> dali_op_t.
static dali_op_t write_op_from_flags(uint8_t flags) {
    if (flags & SYNC_WRITE_LEVEL)  return OP_WRITE_LEVEL;
    if (flags & SYNC_WRITE_MINMAX) return OP_WRITE_MINMAX;
    if (flags & SYNC_WRITE_POWER)  return OP_WRITE_POWER;
    if (flags & SYNC_WRITE_FADE)   return OP_WRITE_FADE;
    return OP_NONE;
}

// Map dali_op_t back to its corresponding SYNC_WRITE_* bit.
static uint8_t write_flag_for_op(dali_op_t op) {
    switch (op) {
        case OP_WRITE_LEVEL:  return SYNC_WRITE_LEVEL;
        case OP_WRITE_MINMAX: return SYNC_WRITE_MINMAX;
        case OP_WRITE_POWER:  return SYNC_WRITE_POWER;
        case OP_WRITE_FADE:   return SYNC_WRITE_FADE;
        default: return 0;
    }
}

// pick_work() — runs under TCB0-disabled + ATOMIC_BLOCK in
// start_next_dali_work.  Inspects the priority chain and, if work is
// found, populates current_op and returns true.  Does NOT touch
// dali_busy/dali_idle (the caller does).
static bool pick_work(void) {
    // 1. Raw command (high byte of raw_cmd_result == 0xFF).
    if ((uint8_t)(g_regs.raw_cmd_result >> 8) == 0xFFu) {
        current_op.op   = OP_RAW;
        current_op.addr = 0xFFu;
        current_op.step = 0;
        return true;
    }

    // 2. Group bit-change pending.
    if (group_write.reg_addr != 0xFFFFu) {
        // pending_adds first, then pending_removes.
        uint16_t adds    = group_write.pending_adds;
        uint16_t removes = group_write.pending_removes;
        uint8_t bit;
        uint8_t group = (uint8_t)((group_write.reg_addr - 0x0110u) >> 2);
        uint8_t reg_off = (uint8_t)((group_write.reg_addr - 0x0110u) & 0x03u);
        uint8_t data_byte;
        if (adds) {
            bit = lowest_bit_set(adds);
            data_byte = 0x60u | (group & 0x0Fu);  // ADD_TO_GROUP_X
        } else if (removes) {
            bit = lowest_bit_set(removes);
            data_byte = 0x70u | (group & 0x0Fu);  // REMOVE_FROM_GROUP_X
        } else {
            // Empty: clear arming and move on.
            group_write.reg_addr = 0xFFFFu;
            goto try_writes;
        }
        // Affected ballast: reg_off (0..3) gives 16-ballast-wide block,
        // bit (0..15) gives the ballast within that block.
        uint8_t ballast_idx = (uint8_t)(reg_off * 16u + bit);
        current_op.op   = OP_GROUP_BIT;
        current_op.addr = ballast_idx;
        current_op.step = data_byte;     // stash data byte in step
        return true;
    }
try_writes:

    // 3. Pending write on any ballast (rotating cursor).
    for (uint8_t i = 0; i < DALI_MAX_DEVICES; i++) {
        uint8_t idx = (uint8_t)((work_cursor + i) & 0x3Fu);
        uint8_t f = sync_flags[idx];
        dali_op_t op = write_op_from_flags(f);
        if (op != OP_NONE) {
            current_op.op   = op;
            current_op.addr = idx;
            current_op.step = 0;
            work_cursor = (uint8_t)((idx + 1u) & 0x3Fu);
            return true;
        }
    }

    // 4. Pending level read (same rotating cursor — fairness across
    // both writes and reads on every issued command).
    for (uint8_t i = 0; i < DALI_MAX_DEVICES; i++) {
        uint8_t idx = (uint8_t)((work_cursor + i) & 0x3Fu);
        if (sync_flags[idx] & SYNC_READ_LEVEL) {
            current_op.op   = OP_LEVEL_READ;
            current_op.addr = idx;
            current_op.step = 0;
            work_cursor = (uint8_t)((idx + 1u) & 0x3Fu);
            return true;
        }
    }

    // 5. Continue in-progress full scan.
    if (scan_full_addr != 0xFFu) {
        current_op.op   = OP_FULL_SCAN;
        current_op.addr = scan_full_addr;
        current_op.step = 0;
        return true;
    }

    // 6. Start a new full scan.
    for (uint8_t i = 0; i < DALI_MAX_DEVICES; i++) {
        if (sync_flags[i] & SYNC_READ_FULL) {
            scan_full_addr  = i;
            scan_full_step  = 0;
            current_op.op   = OP_FULL_SCAN;
            current_op.addr = i;
            current_op.step = 0;
            return true;
        }
    }

    return false;
}

bool start_next_dali_work(void) {
    bool launch;
    uint8_t old_tcb = TCB0.INTCTRL;
    TCB0.INTCTRL = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        launch = dali_idle && !dali_busy && pick_work();
        if (launch) {
            dali_busy = true;
            dali_idle = false;
            dali_cancel_edge_detect();
        }
    }
    TCB0.INTCTRL = old_tcb;

    if (launch) {
        // We are now committed to a sync-driven send.  If the bus is
        // currently active (another master), wait for it to clear; the
        // wait-for-idle -> idle-state-enter path will re-enter and
        // dispatch via dali_sync_dispatch_current (because dali_busy is
        // true).
        if (dali_is_bus_active()) {
            dali_wait_for_idle_state_enter();
        } else {
            dali_sync_dispatch_current();
        }
    }
    return launch;
}

// ---- Raw / group arming ---------------------------------------------

bool dali_sync_arm_raw(uint8_t addr_byte, uint8_t data_byte, uint8_t sends) {
    bool armed = false;
    uint8_t old_tcb = TCB0.INTCTRL;
    TCB0.INTCTRL = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (g_regs.raw_cmd_result == RAW_RESULT_IDLE) {
            raw_cmd_addr = addr_byte;
            raw_cmd_data = data_byte;
            // 0xFF02 (twice) or 0xFF01 (single).
            g_regs.raw_cmd_result = (uint16_t)(0xFF00u | (sends & 0xFFu));
            armed = true;
        }
    }
    TCB0.INTCTRL = old_tcb;
    return armed;
}

bool dali_sync_arm_group_write(uint16_t reg_addr,
                                uint16_t pending_adds,
                                uint16_t pending_removes) {
    bool armed = false;
    uint8_t old_tcb = TCB0.INTCTRL;
    TCB0.INTCTRL = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (group_write.reg_addr == 0xFFFFu) {
            group_write.reg_addr        = reg_addr;
            group_write.pending_adds    = pending_adds;
            group_write.pending_removes = pending_removes;
            armed = true;
        }
    }
    TCB0.INTCTRL = old_tcb;
    return armed;
}

// ---- Completion handler ---------------------------------------------
//
// Runs in ISR context (called from cmdq_result_received in
// dali_state_idle.c when dali_busy is true).  Updates cache, advances
// step, decides whether to release dali_busy.  Either path tail-calls
// dali_wait_for_idle_state_enter which falls through to
// dali_idle_state_enter — the latter dispatches the next step
// (dali_busy still true) or arms idle (dali_busy cleared).

// On level/status change, set dirty bit.
static void mark_dirty(uint8_t idx) {
    uint8_t  word = idx >> 4;
    uint16_t bit  = (uint16_t)1u << (idx & 0x0Fu);
    g_regs.dirty[word] |= bit;
}

// Helper for level read / write-final completion: apply retry-pending
// logic on NAK.  retry_flag is the SYNC_READ_* flag to leave set on
// first NAK.
static void apply_retry_pending(uint8_t idx, uint8_t retry_flag) {
    uint8_t f = sync_flags[idx];
    if (f & SYNC_RETRY_PENDING) {
        // Second consecutive NAK -> mark absent.
        regs_clear_ballast(idx);
        sync_flags[idx] = 0;
    } else {
        // First NAK -> set retry pending, leave the read flag set.
        sync_flags[idx] = (uint8_t)((f | SYNC_RETRY_PENDING | retry_flag) & 0xFFu);
    }
}

// On any successful response: transition to present + set
// SYNC_READ_FULL if the ballast was absent.  Returns true if a
// presence transition occurred.
static bool note_present_on_ok(uint8_t idx) {
    uint16_t bm   = g_regs.present[idx >> 4];
    uint16_t bit  = (uint16_t)1u << (idx & 0x0Fu);
    if (!(bm & bit)) {
        regs_set_present(idx, true);
        // Re-fetch all config on presence transition.
        sync_flags[idx] |= SYNC_READ_FULL;
        mark_dirty(idx);
        return true;
    }
    // Clear retry-pending on any successful response.
    sync_flags[idx] &= (uint8_t)~SYNC_RETRY_PENDING;
    return false;
}

void dali_sync_completion(dali_response_type_t type, uint8_t value) {
    dali_op_t op   = current_op.op;
    uint8_t   addr = current_op.addr;
    uint8_t   step = current_op.step;
    bool more = false;   // multi-step continuation?

    switch (op) {
        case OP_WRITE_LEVEL: {
            // DAPC has no backward frame.  Cache was updated predictively.
            uint8_t idx = addr & 0x3Fu;
            if (type == DALI_RESPONSE_OK || type == DALI_RESPONSE_NAK) {
                // Both treated as success: DAPC produces no response.
                sync_flags_clear(idx, SYNC_WRITE_LEVEL);
                mark_dirty(idx);
            } else {
                // Bus error: leave bit set so we retry.
            }
            break;
        }

        case OP_WRITE_MINMAX:
        case OP_WRITE_POWER:
        case OP_WRITE_FADE: {
            uint8_t idx  = addr & 0x3Fu;
            uint8_t flag = write_flag_for_op(op);
            // 4 logical steps (0..3): DTR0_low / SET_X(twice) / DTR0_high
            // / SET_Y(twice).  SET commands produce no backward frame, so
            // NAK is the expected outcome and is treated as "step done".
            // Bus errors (anything other than OK/NAK) on a SET step abort
            // the sequence; the cache will be reconfirmed by a follow-up
            // SYNC_READ_FULL.  DTR0 steps (0, 2) are broadcast-style and
            // cannot meaningfully NAK, but a bus error there also aborts.
            if (type != DALI_RESPONSE_OK && type != DALI_RESPONSE_NAK) {
                // Bus error: abort sequence.
                sync_flags_clear(idx, flag);
                apply_retry_pending(idx, SYNC_READ_FULL);
            } else if (step >= 3u) {
                // Final SET twice-pair done.
                sync_flags_clear(idx, flag);
                mark_dirty(idx);
            } else {
                // Advance to next logical step.
                current_op.step = step + 1u;
                more = true;
            }
            break;
        }

        case OP_LEVEL_READ: {
            uint8_t idx = addr & 0x3Fu;
            if (step == 0) {
                // QUERY_STATUS response.
                if (type == DALI_RESPONSE_OK) {
                    note_present_on_ok(idx);
                    regs_apply_status(idx, value);
                    current_op.step = 1u;
                    more = true;
                } else if (type == DALI_RESPONSE_NAK) {
                    apply_retry_pending(idx, SYNC_READ_LEVEL);
                }
                // else (bus error): leave SYNC_READ_LEVEL set, no
                // retry-pending change; next cursor pass will retry.
            } else {
                // QUERY_ACTUAL_LEVEL response.
                if (type == DALI_RESPONSE_OK) {
                    uint8_t old_b = g_regs.block0[idx].brightness;
                    g_regs.block0[idx].brightness = value;
                    if (old_b != value) mark_dirty(idx);
                    sync_flags_clear(idx, SYNC_READ_LEVEL | SYNC_RETRY_PENDING);
                } else if (type == DALI_RESPONSE_NAK) {
                    apply_retry_pending(idx, SYNC_READ_LEVEL);
                }
                // else (bus error): leave SYNC_READ_LEVEL set.
            }
            break;
        }

        case OP_FULL_SCAN: {
            uint8_t idx = addr & 0x3Fu;
            uint8_t s   = scan_full_step;
            uint8_t recv = (type == DALI_RESPONSE_OK) ? value : 0;
            // Presence transition: SCAN_TRIGGER seeds SYNC_READ_FULL on
            // every ballast, so the FIRST OK in a scan of a previously-
            // absent ballast must flip its present bit.  This complements
            // the level-read presence transition (OP_LEVEL_READ step 0).
            if (type == DALI_RESPONSE_OK)
                note_present_on_ok(idx);
            // Spec: "cache result (zero on NAK)".  Bus error also zeros
            // and advances — the loss is bounded to one config field.
            regs_apply_result(idx, scan_steps[s].target, recv);
            scan_full_step = s + 1u;
            if (scan_full_step >= 7u) {
                sync_flags_clear(idx, SYNC_READ_FULL | SYNC_RETRY_PENDING);
                scan_full_addr = 0xFFu;
                scan_full_step = 0;
            }
            // Yield between steps: dali_busy cleared below.  Main picks
            // the next step via priority 5 (continue in-progress full
            // scan) unless higher-priority work arrives.
            break;
        }

        case OP_GROUP_BIT: {
            uint8_t idx = addr & 0x3Fu;
            uint8_t data = step;  // we stashed data byte in step
            uint16_t reg_off = (group_write.reg_addr - 0x0110u) & 0x03u;
            uint16_t bit_in_reg = (uint16_t)1u << (idx % 16u);
            (void)reg_off;
            // SET commands produce no backward frame; treat OK and NAK
            // as success.  Bus error: leave bit set for retry next pass.
            if (type == DALI_RESPONSE_OK || type == DALI_RESPONSE_NAK) {
                if (data & 0x10u) {
                    // 0x70-0x7F: REMOVE_FROM_GROUP_X
                    group_write.pending_removes &= (uint16_t)~bit_in_reg;
                } else {
                    // 0x60-0x6F: ADD_TO_GROUP_X
                    group_write.pending_adds &= (uint16_t)~bit_in_reg;
                }
                if (group_write.pending_adds == 0
                    && group_write.pending_removes == 0) {
                    group_write.reg_addr = 0xFFFFu;
                }
            }
            // If bus error, leave the bits set; retry on next pass.
            break;
        }

        case OP_RAW: {
            // Each call here is one terminated logical command (twice-
            // sends are coalesced by cmdq_result_received's repeat
            // re-fire).  So this path always writes a terminal value to
            // 0x015D and releases the channel.
            uint16_t terminal;
            if (type == DALI_RESPONSE_OK) {
                terminal = RAW_RESULT_OK(value);
            } else if (type == DALI_RESPONSE_NAK) {
                terminal = RAW_RESULT_NAK;
            } else {
                uint8_t sub = (uint8_t)(type - DALI_RESPONSE_MALFORMED) + 1u;
                terminal = RAW_RESULT_ERR(sub);
            }
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                g_regs.raw_cmd_result = terminal;
            }
            break;
        }

        default:
            break;
    }

    if (!more) {
        // Op fully done (or aborted): release dali_busy.
        dali_busy = false;
        current_op.op = OP_NONE;
    }

    // Always go through the wait-for-idle gap.  When dali_busy stays
    // true, dali_idle_state_enter will dispatch the next step via
    // dali_sync_dispatch_current.  When false, it arms idle.
    dali_wait_for_idle_state_enter();
}

// ---- Periodic re-poll (PIT) — round-robin ----------------------------
//
// The RTC PIT fires every ~1 s (configured by dali_init via CYC1024 on
// the 1 kHz internal oscillator).  Each fire we increment a tick counter;
// once it reaches MBREG_RESCAN_INTERVAL, we set SYNC_READ_LEVEL on the
// ballast at scan_cursor and advance the cursor (mod 64).
//
// rescan_interval == 0 disables the cursor entirely (no automatic polls).
ISR(RTC_PIT_vect) {
    static uint8_t tick = 0;

    uint16_t interval = userrow_get_rescan_interval();
    if (interval > 0) {
        tick++;
        if (tick >= interval) {
            tick = 0;
            // ISR-context: sync_flags[i] |= ... is non-atomic on AVR
            // (LDS/OR/STS) but we are already at the highest-priority
            // context that touches this byte; only main can race with us,
            // and main's writes are wrapped in ATOMIC_BLOCK via
            // sync_flags_set.  Disabling interrupts here would be redundant.
            sync_flags[scan_cursor] |= SYNC_READ_LEVEL;
            scan_cursor++;
            if (scan_cursor >= DALI_MAX_DEVICES) scan_cursor = 0;
        }
    } else {
        tick = 0;
    }

    RTC.PITINTFLAGS = RTC_PI_bm;
}
