// dali_sync.c — sync-flag-driven DALI work selector + unified command
// issuance + completion dispatch.
//
// Architectural model:
//   - Every DALI command leaves the firmware through issue_dali_command.
//     That function performs cache invalidation as a property of the
//     command bytes (not its source), then hands a stack-local
//     cmd_entry_t to dali_transmitting_state_enter.
//   - start_next_dali_work picks the highest-priority pending work
//     (commissioning -> raw -> group bit -> SYNC_WRITE_* ->
//     SYNC_READ_LEVEL -> full-scan continue -> full-scan new)
//     and kicks off the first send.
//   - The bit-level state machine drives the send to completion.  When
//     it returns to idle, dali_result_received fires.  If dali_busy is
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
//   - the generic slot queue (mbq_slots.c): submit/status served in main; the
//     terminal write (mbq_complete — response8 BEFORE status) runs in the LVL1
//     completion handler; mbq_read_status fences each slot's read (ATOMIC_BLOCK)
//     so it never sees a terminal status with a stale response8.
//   - group_write: same TCB0+ATOMIC pattern.
//   - g_regs.dirty[]: ISR-set, main-cleared via regs_read_clear_dirty
//     (already atomic).
//   - commission_state: written from main only inside
//     dali_sync_arm_commission and regs_read_holding's terminal-state
//     clear (both under TCB0+ATOMIC); all other updates run from ISR
//     context (dali_sync_completion / dispatch_current).  16-bit status
//     writes from ISR are LVL1, fenced against main reads by main's
//     TCB0=0 around its ATOMIC blocks.

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdbool.h>
#include <stdint.h>
#include <util/atomic.h>

#include "dali.h"
#include "dali_sync.h"
#include "mbq_slots.h"
#include "registers.h"

// ---- Globals ---------------------------------------------------------

volatile uint8_t           sync_flags[DALI_MAX_DEVICES];
volatile bool              dali_busy;
volatile dali_current_op_t current_op;
volatile dali_group_write_t group_write;
volatile cmd_entry_t       s_active;
volatile commission_state_t commission_state;

// ---- Raw command queue (task 06, queue v2) ---------------------------
//
// The generic 8-slot lifecycle queue lives in mbq_slots.c (DALI-agnostic:
// opaque 32-bit command in, [status|response8] out).  This module is the DALI
// ADAPTER: it picks the next pending slot, unpacks the 32-bit command into a
// DALI frame (issue_dali_command), and maps the DALI terminal back to a
// (status, response8) via mbq_complete.  The only state kept here is the slot +
// unpacked command of the raw op currently on the bus, held across the DTR0
// two-step and into completion.  OP_RAW is single-in-flight: dali_busy gates the
// arbiter, so these are only rewritten when the next slot begins.
static uint8_t s_raw_slot;                 // mbq slot index of the executing OP_RAW
static uint8_t s_raw_cmd[MBQ_CMD_BYTES];   // its opaque command: [addr,data,flags,dtr0]

volatile uint8_t scan_full_addr;
volatile uint8_t scan_full_step;

// PIT round-robin cursor — advances at most one slot per fired tick.
static volatile uint8_t scan_cursor;

// Seconds since boot.  Incremented once per RTC PIT fire (~1 Hz); exposed via
// the FC04 sysinfo block so the master can detect a gateway reboot (the value
// drops back toward 0).  Pure RAM — no persistent storage, no wear.
volatile uint32_t g_uptime_s = 0;

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
    // Generic slot queue: all slots idle (a reboot's "idle means gone" signal
    // to the controller — no cross-reboot queue state is kept).
    mbq_init();
    s_raw_slot = 0;
    scan_full_addr = 0xFF;
    scan_full_step = 0;
    scan_cursor    = 0;
    work_cursor    = 0;
    // Commissioning state — idle.  All other fields populated by
    // dali_sync_arm_commission when the master triggers a run.
    commission_state.phase           = COM_IDLE;
    commission_state.status          = COMM_STATUS_IDLE;
    commission_state.abort_requested = false;
    commission_state.resend_pending  = false;
}

// ---- Cache-invalidation classification ------------------------------
//
// Every DALI command leaving issue_dali_command runs through this.
// Returns the SYNC_READ_* mask to set on the affected ballasts.  The
// table is collapsed to four coarse ranges; the user accepted that a
// few reserved opcodes (notably 0x21 STORE_CURRENT_LEVEL_DTR0 and the
// gaps in the 0x22..0x29 / 0x30..0x3F / 0x81..0x8F bands) now over-
// invalidate by triggering a SYNC_READ_SETTINGS.  The cost is one extra
// 7-step background scan per spurious match, which is well within the
// existing rescan budget — and the simpler classifier is easier to
// maintain than an exhaustive opcode table.
//
//  - DAPC (bit0 of addr clear)              -> SYNC_READ_LEVEL
//  - 0x00..0x1F (level / scene / step / etc) -> SYNC_READ_LEVEL
//  - 0x20 RESET                             -> SYNC_READ_LEVEL | SYNC_READ_SETTINGS
//  - 0x21..0x8F (config + special)          -> SYNC_READ_SETTINGS  [over-invalidates a handful of reserved opcodes]
//  - 0x90..0xFF queries / specials          -> 0
static uint8_t classify_invalidation(uint8_t addr_byte, uint8_t data_byte) {
    // DAPC: address bit 0 clear -> arc level command.
    if ((addr_byte & DALI_CMD_FLAG) == 0) return SYNC_READ_LEVEL;
    if (data_byte <  0x20u)               return SYNC_READ_LEVEL;
    if (data_byte == 0x20u)               return SYNC_READ_LEVEL | SYNC_READ_SETTINGS;
    if (data_byte <  0x90u)               return SYNC_READ_SETTINGS;
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

// ---- Commissioning helpers ------------------------------------------
//
// 24-bit search address held as a 3-byte array (search[0]=L, [1]=M, [2]=H)
// so `bit >> 3` indexes the byte directly — no per-range branching, and
// avoids the big-endian shuffle if it were a packed uint32_t.
//
// com_set_bit / com_clear_bit each have a single effective call site
// inside the COM_COMPARE handler — leave them inlinable.
// com_reset_bisect / com_lowest_unused both have multiple call sites
// (arm() and OP_COMMISSION completion), so they're tagged noinline +
// noclone to keep them at one copy and avoid the avr-gcc constprop
// cloning bloat that subtask 7 measured for the unified issue path.
static void com_clear_bit(uint8_t bit) {
    commission_state.search[bit >> 3] &= (uint8_t)~(1u << (bit & 7u));
}

static void com_set_bit(uint8_t bit) {
    commission_state.search[bit >> 3] |= (uint8_t)(1u << (bit & 7u));
}

// Reset bisect state for the next ballast: search ← 0xFFFFFF, bisect_bit
// ← 24 (sentinel "first COMPARE — any ballast?").  err_streak reset is
// handled by the caller (only on VERIFY-yes; NAK keeps the streak).
__attribute__((noinline, noclone))
static void com_reset_bisect(void) {
    commission_state.search[0]  = 0xFFu;
    commission_state.search[1]  = 0xFFu;
    commission_state.search[2]  = 0xFFu;
    commission_state.bisect_bit = 24u;
}

// Find the lowest short address with `present` bit clear.  Used by
// incremental mode and for advancing next_short_addr after a successful
// VERIFY.  Returns DALI_MAX_DEVICES (64) when all 64 slots are taken.
__attribute__((noinline, noclone))
static uint8_t com_lowest_unused(void) {
    for (uint8_t i = 0; i < DALI_MAX_DEVICES; i++) {
        uint16_t bm  = g_regs.present[i >> 4];
        uint16_t bit = (uint16_t)1u << (i & 0x0Fu);
        if (!(bm & bit)) return i;
    }
    return DALI_MAX_DEVICES;
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
    s_active.type      = t;
    s_active.dali_addr = addr_byte;
    s_active.dali_cmd  = data_byte;
    s_active.repeat    = repeat;

    dali_transmitting_state_enter((const cmd_entry_t *)&s_active);
}

// ---- Full-scan step table -------------------------------------------
//
// (The per-ballast config writes — DTR0/SET_MIN/MAX/POWER/SYS-FAIL/FADE
// multi-step sequences — were retired with the config banks; the controller
// now issues those as raw DTR0+SET pairs through the command queue.)

typedef struct { uint8_t cmd; reg_target_t target; } scan_step_t;

// After the config-bank retirement the settings scan reads ONLY group
// membership.  Presence is established by the first OK response (here or on the
// SYNC_READ_LEVEL QUERY_STATUS), so dropping the 5 config queries does not
// affect presence detection — it only removes cache fields the gateway no
// longer keeps.  Idle DALI budget freed: 5 of the old 7 settings queries per
// ballast per settings-scan (~71% fewer settings-scan frames).
#define SCAN_STEPS_N 2u
static const scan_step_t scan_steps[SCAN_STEPS_N] = {
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
            // DALI adapter: unpack the executing slot's opaque command
            // [addr, data, flags, dtr0] and issue it.  A QF_DTR0 command is a
            // two-step atomic sequence — step 0 preloads DTR0 (0xA3<dtr0>), step 1
            // sends the frame — back-to-back with nothing interleaved because
            // dali_busy stays held across the pair (one slot's lifecycle).
            uint8_t flags = s_raw_cmd[2];

            if ((flags & QF_DTR0) && step == 0u) {
                issue_dali_command(DALI_ADDR_SET_DTR0, s_raw_cmd[3],
                                   DALI_KIND_CMD, 1u);
                break;
            }
            // Terminal frame.  KIND selection from the flags byte:
            //   QF_TWICE  → send twice (SET-class), no answer.
            //   QF_ANSWER → KIND_QUERY, open the backward-frame window.
            //   else      → KIND_CMD, completes immediately (no 22 TE wait).
            uint8_t a = s_raw_cmd[0];
            uint8_t d = s_raw_cmd[1];
            if (flags & QF_TWICE)
                issue_dali_command(a, d, DALI_KIND_TWICE, 2u);
            else if (flags & QF_ANSWER)
                issue_dali_command(a, d, DALI_KIND_QUERY, 1u);
            else
                issue_dali_command(a, d, DALI_KIND_CMD, 1u);
            break;
        }
        case OP_COMMISSION: {
            // Abort flag is checked at every dispatch boundary — the
            // master can write 0x0000 to MBREG_COMMISSION at any time
            // and this is where the request takes effect.  Force phase
            // to COM_TERMINATE and stamp the aborted-status word; the
            // dispatch table below issues TERMINATE.
            commission_phase_t p = commission_state.phase;
            if (commission_state.abort_requested && p != COM_TERMINATE) {
                commission_state.status = COMM_STATUS_ERR_ABORTED;
                commission_state.phase  = COM_TERMINATE;
                commission_state.abort_requested = false;
                p = COM_TERMINATE;
            }
            // Per-phase dispatch table indexed by `phase - COM_INITIALISE`.
            // PROGMEM keeps the 20 bytes out of RAM.  `repeat` is derived
            // from `kind` (TWICE → 2, otherwise 1) so we don't pay for a
            // third column.
            //                          addr_byte                          kind
            static const uint8_t dispatch_tbl[][2] PROGMEM = {
                /*COM_INITIALISE  */ { DALI_ADDR_INITIALISE,            DALI_KIND_TWICE },
                /*COM_RANDOMISE   */ { DALI_ADDR_RANDOMISE,             DALI_KIND_TWICE },
                /*COM_SEARCH_H    */ { DALI_ADDR_SEARCHADDRH,           DALI_KIND_CMD   },
                /*COM_SEARCH_M    */ { DALI_ADDR_SEARCHADDRM,           DALI_KIND_CMD   },
                /*COM_SEARCH_L    */ { DALI_ADDR_SEARCHADDRL,           DALI_KIND_CMD   },
                /*COM_COMPARE     */ { DALI_ADDR_COMPARE,               DALI_KIND_QUERY },
                /*COM_PROGRAM_ADDR*/ { DALI_ADDR_PROGRAM_SHORT_ADDRESS, DALI_KIND_CMD   },
                /*COM_VERIFY      */ { DALI_ADDR_VERIFY_SHORT_ADDRESS,  DALI_KIND_QUERY },
                /*COM_WITHDRAW    */ { DALI_ADDR_WITHDRAW,              DALI_KIND_CMD   },
                /*COM_TERMINATE   */ { DALI_ADDR_TERMINATE,             DALI_KIND_CMD   },
            };
            uint8_t idx       = (uint8_t)(p - COM_INITIALISE);
            uint8_t addr_byte = pgm_read_byte(&dispatch_tbl[idx][0]);
            uint8_t kind      = pgm_read_byte(&dispatch_tbl[idx][1]);
            uint8_t repeat    = (kind == DALI_KIND_TWICE) ? 2u : 1u;
            // Per-phase data byte.  Default 0x00 covers RANDOMISE /
            // COMPARE / WITHDRAW / TERMINATE.  SEARCH_H/M/L collapsed
            // to one indexed load: search[] is laid out [0]=L, [1]=M,
            // [2]=H, so the byte for COM_SEARCH_x is search[COM_SEARCH_L - x].
            uint8_t data_byte = 0x00u;
            switch (p) {
                case COM_INITIALISE:
                    // mode 1: all gear; mode 3: only the gear currently at the
                    // collided short address (0AAAAAA1); mode 2: no-short gear.
                    if (commission_state.mode == 1u)
                        data_byte = DALI_INIT_DATA_ALL;
                    else if (commission_state.mode == 3u)
                        data_byte = DALI_PROG_SHORT(commission_state.reprov_addr);
                    else
                        data_byte = DALI_INIT_DATA_NO_SHORT;
                    break;
                case COM_SEARCH_H:
                case COM_SEARCH_M:
                case COM_SEARCH_L:
                    data_byte = commission_state.search[COM_SEARCH_L - p];
                    break;
                case COM_PROGRAM_ADDR:
                case COM_VERIFY:
                    data_byte = DALI_PROG_SHORT(commission_state.next_short_addr);
                    break;
                default: break;
            }
            issue_dali_command(addr_byte, data_byte,
                               (dali_kind_t)kind, repeat);
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

// Map sync_flags write bit -> dali_op_t.  Only SYNC_WRITE_LEVEL (DAPC via the
// per-ballast brightness bank) remains after the config-bank retirement.
static dali_op_t write_op_from_flags(uint8_t flags) {
    if (flags & SYNC_WRITE_LEVEL)  return OP_WRITE_LEVEL;
    return OP_NONE;
}

// pick_work() — runs under TCB0-disabled + ATOMIC_BLOCK in
// start_next_dali_work.  Inspects the priority chain and, if work is
// found, populates current_op and returns true.  Does NOT touch
// dali_busy/dali_idle (the caller does).
//
// Priority order:
//   1. OP_COMMISSION  — master is actively polling 0x1014, and the spec
//                       (task 2 §"Edge cases") explicitly puts raw commands
//                       behind commissioning so a master `0xA3VV` raw can't
//                       clobber DTR0 mid-bisect.
//   2. OP_RAW         — single-shot raw command (queues behind commissioning).
//   3. OP_GROUP_BIT   — pending ADD/REMOVE_FROM_GROUP from a group-mask write.
//   4. SYNC_WRITE_*   — per-ballast bank writes (rotating cursor for fairness).
//   5. SYNC_READ_LEVEL- per-ballast level polls (same cursor).
//   6. continue full-scan in progress.
//   7. start a new full-scan from SYNC_READ_SETTINGS.
static bool pick_work(void) {
    // 1. Commissioning state machine (highest priority).  phase != IDLE
    //    means a run is armed or mid-flight; OP_COMMISSION is sticky for
    //    the entire run (each completion sets `more = true` until COM_TERMINATE).
    if (commission_state.phase != COM_IDLE) {
        current_op.op   = OP_COMMISSION;
        current_op.addr = 0xFFu;
        current_op.step = 0;
        return true;
    }

    // 2. Generic slot queue has a pending command.  It preempts all background
    //    polling (priorities 4-7) — the gateway's own idle work always yields to
    //    the queue, so a submitted toggle runs within ~1-2 frame times.  Begin the
    //    lowest accept-sequence pending slot: mbq_begin marks it EXECUTING and
    //    copies its opaque command for the DALI adapter (dispatch) to unpack.
    {
        uint8_t slot = mbq_next_pending();
        if (slot != MBQ_NONE) {
            mbq_begin(slot, s_raw_cmd);
            s_raw_slot      = slot;
            current_op.op   = OP_RAW;
            current_op.addr = 0xFFu;
            current_op.step = 0;
            return true;
        }
    }

    // 3. Group bit-change pending.
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
        // Affected ballast: reg_off (0..3) gives the 16-ballast-wide block,
        // bit (0..15) the ballast within it.  pending_adds/removes are held in
        // natural ballast-bit order (byte-swapped from the wire at arm time),
        // so this is a direct mapping — and it matches the completion's
        // `bit_in_reg = 1 << (idx % 16)` clear.
        uint8_t ballast_idx = (uint8_t)(reg_off * 16u + bit);
        current_op.op   = OP_GROUP_BIT;
        current_op.addr = ballast_idx;
        current_op.step = data_byte;     // stash data byte in step
        return true;
    }
try_writes:

    // 4. Pending write on any ballast (rotating cursor).
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

    // 5. Pending level read (same rotating cursor — fairness across
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

    // 6. Continue in-progress full scan.
    if (scan_full_addr != 0xFFu) {
        current_op.op   = OP_FULL_SCAN;
        current_op.addr = scan_full_addr;
        current_op.step = 0;
        return true;
    }

    // 7. Start a new full scan.
    for (uint8_t i = 0; i < DALI_MAX_DEVICES; i++) {
        if (sync_flags[i] & SYNC_READ_SETTINGS) {
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
    bool launch = false;
    uint8_t old_tcb = TCB0.INTCTRL;
    TCB0.INTCTRL = 0;   // keep the LVL1 completion handler out for the whole selection

    // pick_work() scans up to ~192 ballast-flag entries.  It MUST NOT run under a
    // global interrupt-disable: doing so (the previous ATOMIC_BLOCK here) held all
    // interrupts off for hundreds of µs on every idle main-loop pass, overrunning
    // the USART's 2-byte RX FIFO and silently dropping ~25% of incoming Modbus
    // bytes — so every frame failed CRC and the gateway went deaf whenever DALI
    // was idle.  The scan only needs the TCB0 fence above (sync_flags vs the LVL1
    // completion writer); single-byte flag reads are atomic and a flag changing
    // mid-scan is benign (picked next pass).  Only the short commit needs a
    // global-atomic re-check against the LVL0 idle-timer / PIT / edge callbacks.
    if (dali_idle && !dali_busy && pick_work()) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            if (dali_idle && !dali_busy) {   // re-check: state may have moved during the scan
                dali_busy = true;
                dali_idle = false;
                dali_cancel_edge_detect();
                launch = true;
            }
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

// ---- Raw command queue ----------------------------------------------
//
// The generic slot queue (submit / status / accept-sequence / lifecycle) moved
// to mbq_slots.c for queue v2.  The DALI adapter for it is the OP_RAW arms of
// pick_work / dali_sync_dispatch_current / dali_sync_completion above, which
// unpack a slot's opaque command into DALI frames and map the DALI terminal to
// (status, response8) via mbq_complete.

bool dali_sync_arm_commission(uint8_t mode, uint8_t target) {
    bool armed   = false;
    bool do_wipe = false;

    // Header ATOMIC: refuse if a run is already in progress (status high
    // byte 0x01), then commit the initial commission_state in one atomic
    // burst.  TCB0 fence guards against an in-flight LVL1 OP_COMMISSION
    // completion writing status while we read it.  mode out-of-range
    // never reaches here — regs_write_holding rejects val > 2 first.
    uint8_t old_tcb = TCB0.INTCTRL;
    TCB0.INTCTRL = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if ((uint8_t)(commission_state.status >> 8) == 0x01u) {
            // Already running — caller surfaces DEVICE_BUSY.
            goto out;
        }
        // Incremental (2) and re-provision-one-address (3) both assign onto the
        // lowest free short address; full (1) renumbers from 0.
        bool lowest_free = (mode == 2u || mode == 3u);
        uint8_t initial_addr = lowest_free ? com_lowest_unused() : 0u;
        if (lowest_free && initial_addr >= DALI_MAX_DEVICES) {
            // Bus already fully addressed: no free slot to assign into.
            commission_state.status = COMM_STATUS_ERR_ADDR_EXHAUST;
            commission_state.phase  = COM_IDLE;
            armed = true;
            goto out;
        }
        com_reset_bisect();
        commission_state.next_short_addr = initial_addr;
        commission_state.reprov_addr     = target;   // only consulted in mode 3
        commission_state.count           = 0;
        commission_state.err_streak      = 0;
        commission_state.mode            = mode;
        commission_state.abort_requested = false;
        commission_state.resend_pending  = false;
        commission_state.status          = COMM_STATUS_RUNNING(0);
        commission_state.phase           = COM_INITIALISE;
        do_wipe = (mode == 1u);   // never wipe in modes 2/3 — preserve other gear
        armed   = true;
    out: ;
    }
    TCB0.INTCTRL = old_tcb;

    // Per-ballast cache wipe for full mode.  TCB0.INTCTRL=0 fences the LVL1
    // DALI completion ISR (the only other writer of the ballast regs/sync_flags)
    // for the whole sweep, so the wipe is serialised against it.  But global
    // interrupts must NOT stay off for all 64 iterations: at 10 MHz that window
    // is long enough to overrun the USART's 2-byte RX FIFO and drop an inbound
    // Modbus byte — the same class as the 2026-06-05 RX-starvation bug.  So we
    // wrap only each single clear in ATOMIC and re-enable interrupts between
    // iterations, letting the LVL0 USART RX ISR drain the FIFO.  A partially
    // wiped cache is never observable as "complete": status=0x01XX ("running")
    // is already published from the header ATOMIC above.
    //
    // Wipe-vs-in-flight-completion race (intentionally benign): when arm() is
    // called, dali_busy may still be true for a previously armed op (e.g., a
    // SCAN_TRIGGER full-scan in progress).  Its completion ISR's writes are
    // serialised against the wipe by TCB0.INTCTRL=0 (LVL1 fenced).  Either
    // outcome — wipe-then-ISR or ISR-then-wipe — is harmless: each newly-
    // programmed short address gets SYNC_READ_SETTINGS after VERIFY-yes (re-fills
    // the cache), and the master observes status=0x01XX while the run is in
    // flight so transient gaps don't leak into a "completed" snapshot.
    if (do_wipe) {
        uint8_t ot = TCB0.INTCTRL;
        TCB0.INTCTRL = 0;
        for (uint8_t i = 0; i < DALI_MAX_DEVICES; i++) {
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                regs_clear_ballast(i);
                sync_flags[i] = 0;
            }
        }
        TCB0.INTCTRL = ot;
    }

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
            /* The wire register is big-endian (group_mask byte[reg*2] = ballasts
             * 0-7 of the block sit in the HIGH byte), so byte-swap the pending
             * masks into natural ballast-bit order here — the issuance and
             * completion below then index ballasts directly (bit = ballast). */
            group_write.pending_adds    = (uint16_t)((pending_adds    >> 8) | (pending_adds    << 8));
            group_write.pending_removes = (uint16_t)((pending_removes >> 8) | (pending_removes << 8));
            armed = true;
        }
    }
    TCB0.INTCTRL = old_tcb;
    return armed;
}

// ---- Completion handler ---------------------------------------------
//
// Runs in ISR context (called from dali_result_received in
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
// first NAK.  Both branches go through the atomic helpers so the
// safety against an interleaving sync_flags update is obvious from the
// call site rather than relying on the caller's ISR-context invariants.
static void apply_retry_pending(uint8_t idx, uint8_t retry_flag) {
    if (sync_flags[idx] & SYNC_RETRY_PENDING) {
        // Second consecutive NAK -> mark absent.
        regs_clear_ballast(idx);
        sync_flags_clear(idx, 0xFFu);
    } else {
        // First NAK -> set retry pending, leave the read flag set.
        sync_flags_set(idx, SYNC_RETRY_PENDING | retry_flag);
    }
}

// On any successful response: transition to present + set
// SYNC_READ_SETTINGS if the ballast was absent.  Returns true if a
// presence transition occurred.
static bool note_present_on_ok(uint8_t idx) {
    uint16_t bm   = g_regs.present[idx >> 4];
    uint16_t bit  = (uint16_t)1u << (idx & 0x0Fu);
    if (!(bm & bit)) {
        regs_set_present(idx, true);
        // Re-fetch all config on presence transition.
        sync_flags_set(idx, SYNC_READ_SETTINGS);
        mark_dirty(idx);
        return true;
    }
    // Clear retry-pending on any successful response.
    sync_flags_clear(idx, SYNC_RETRY_PENDING);
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
                    // 0xFF = MASK: NOT a valid arc level (max is 254). Never store
                    // it as a brightness — keep the last good value and re-read.
                    bool valid  = (value != 0xFFu);
                    bool fading = (g_regs.block0[idx].status & DALI_STATUS_FADE_RUNNING) != 0;
                    if (valid) {
                        uint8_t old_b = g_regs.block0[idx].brightness;
                        g_regs.block0[idx].brightness = value;
                        if (old_b != value) mark_dirty(idx);
                    }
                    // Keep re-reading until the level has settled to a valid value:
                    // while still fading, or while the answer is invalid (0xFF).
                    if (fading || !valid)
                        sync_flags_clear(idx, SYNC_RETRY_PENDING);
                    else
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
            // Short-circuit on any non-OK response: abort this ballast's
            // scan and let the work selector pick the next ballast on the
            // next pass.  Absent ballasts cost 1 query per scan cycle
            // instead of 7, dropping the SCAN_TRIGGER all-absent worst
            // case from ~22 s to ~3 s.
            //
            // User-accepted trade: a transient NAK at step k of 7 leaves
            // fields k..6 holding their previous value (or zero from
            // regs_init).  The next SCAN_TRIGGER, presence transition, or
            // a subsequent OK on the level-poll cursor will retry.  Net
            // simpler completion semantics — no more "advance through
            // junk on NAK" walk that the queued path used to do.
            if (type == DALI_RESPONSE_OK) {
                note_present_on_ok(idx);
                regs_apply_result(idx, scan_steps[s].target, value);
                scan_full_step = s + 1u;
                if (scan_full_step >= SCAN_STEPS_N) {
                    sync_flags_clear(idx, SYNC_READ_SETTINGS | SYNC_RETRY_PENDING);
                    scan_full_addr = 0xFFu;
                    scan_full_step = 0;
                }
                // Yield between steps: dali_busy cleared below.  Main
                // picks the next step via priority 5 (continue in-progress
                // full scan) unless higher-priority work arrives.
            } else {
                // NAK or bus error: abort this ballast's scan.
                sync_flags_clear(idx, SYNC_READ_SETTINGS | SYNC_RETRY_PENDING);
                scan_full_addr = 0xFFu;
                scan_full_step = 0;
            }
            break;
        }

        case OP_GROUP_BIT: {
            uint8_t idx = addr & 0x3Fu;
            uint8_t data = step;  // we stashed data byte in step
            uint16_t bit_in_reg = (uint16_t)1u << (idx % 16u);
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

        case OP_COMMISSION: {
            // The phase machine: each completion advances exactly one
            // step.  Bisect bookkeeping (search bytes, bisect_bit) lives
            // in commission_state, NOT in current_op.step.  `more = true`
            // keeps dali_busy held for the full run; the only release
            // is through COM_TERMINATE → COM_IDLE.
            //
            // Trivial linear advances (INITIALISE → RANDOMISE, RANDOMISE
            // → SEARCH_H, SEARCH_H → SEARCH_M, SEARCH_M → SEARCH_L,
            // SEARCH_L → COMPARE, PROGRAM_ADDR → VERIFY) all collapse
            // into the default branch via `phase + 1` thanks to the
            // consecutive enum ordering.  Only COMPARE / VERIFY /
            // WITHDRAW / TERMINATE need explicit cases.
            commission_phase_t p = commission_state.phase;
            switch (p) {
                case COM_COMPARE: {
                    // bisect_bit==24 is the "first COMPARE at 0xFFFFFF"
                    // probe (search is still 0xFF/0xFF/0xFF).  Otherwise
                    // search has bit `bb` cleared from the prior
                    // SEARCHADDR round; "yes" keeps it cleared, "no"
                    // restores.  COMPARE is anyone-pulls-low — any
                    // decoded edges (OK / MALFORMED / pulse-*) mean
                    // "match"; only DALI_RESPONSE_NAK means "no".
                    bool yes = (type != DALI_RESPONSE_NAK);
                    uint8_t bb = commission_state.bisect_bit;
                    if (!yes) {
                        if (bb == 24u) {
                            // First-COMPARE "no": no responders left in
                            // the comparison set.  If count==0 this is
                            // an empty bus → 0x0302; otherwise it's
                            // natural completion (we've programmed
                            // everything).  COM_TERMINATE's stamp logic
                            // handles natural-complete (high byte 0x01
                            // → 0x02XX with count) so we don't override
                            // status here in that branch.
                            if (commission_state.count == 0u)
                                commission_state.status = COMM_STATUS_ERR_NO_BALLASTS;
                            commission_state.phase = COM_TERMINATE;
                            more = true;
                            break;
                        }
                        com_set_bit(bb);
                    } else if (bb == 24u
                               && commission_state.next_short_addr >= DALI_MAX_DEVICES) {
                        commission_state.status = COMM_STATUS_ERR_ADDR_EXHAUST;
                        commission_state.phase  = COM_TERMINATE;
                        more = true;
                        break;
                    }
                    // Common bisect advance: bb=24 → 23, bb>0 → bb-1,
                    // bb==0 → bisect done (search holds the random_addr).
                    if (bb == 0u) {
                        if (!yes) {
                            // bb==0 NO: prior SEARCH_L pushed a bit-0-cleared
                            // search to the gears; com_set_bit(0) above
                            // restored bit 0 in our local copy but the gears'
                            // SEARCHADDR registers are now stale.  Re-send
                            // SEARCH_H/M/L before PROGRAM, otherwise the
                            // isolated gear (random has bit 0 = 1) won't
                            // match the gear-side search and PROGRAM is a
                            // no-op -> VERIFY NAKs -> err_streak trips.
                            // SEARCH_L's completion handler sees this flag
                            // and routes to PROGRAM_ADDR (skipping the
                            // wasted COMPARE round).
                            commission_state.resend_pending = true;
                            commission_state.phase = COM_SEARCH_H;
                        } else {
                            // bb==0 YES: gears already hold the matching
                            // search (the bit-0-cleared value still matches
                            // the gear's random with bit 0 = 0).  PROGRAM
                            // directly.
                            commission_state.phase = COM_PROGRAM_ADDR;
                        }
                    } else {
                        commission_state.bisect_bit = bb - 1u;
                        com_clear_bit(bb - 1u);
                        commission_state.phase = COM_SEARCH_H;
                    }
                    more = true;
                    break;
                }
                case COM_SEARCH_L:
                    // SEARCH_L's normal advance is the default (-> COMPARE).
                    // Only divergent path: a bb==0 NO COMPARE outcome set
                    // resend_pending so that, after we just emitted the
                    // corrected SEARCHADDR, we go straight to PROGRAM_ADDR
                    // instead of wasting another COMPARE round.  Clear the
                    // flag here so a subsequent verify-fail re-bisect cycle
                    // can't accidentally inherit it.
                    if (commission_state.resend_pending) {
                        commission_state.resend_pending = false;
                        commission_state.phase = COM_PROGRAM_ADDR;
                    } else {
                        commission_state.phase = (commission_phase_t)(p + 1u);
                    }
                    more = true;
                    break;
                case COM_VERIFY: {
                    // Strict OK + 0xFF: only the just-programmed gear
                    // should respond.  Anything else (NAK / OK-wrong-
                    // value / decoder errors including MALFORMED) is
                    // verify-fail.  A MALFORMED here would mean two
                    // gears collided at the same address — definitely
                    // not OK.
                    bool ok = (type == DALI_RESPONSE_OK && value == 0xFFu);
                    if (ok) {
                        // Mark presence + queue background SYNC_READ_SETTINGS
                        // so the cache fills without manual SCAN_TRIGGER.
                        // next_short_addr advance MUST happen here (not
                        // after WITHDRAW) — otherwise incremental's
                        // lowest-unused re-picks this slot.
                        uint8_t sa = commission_state.next_short_addr;
                        regs_set_present(sa, true);
                        sync_flags_set(sa, SYNC_READ_SETTINGS);
                        commission_state.count++;
                        commission_state.err_streak = 0;
                        commission_state.status     = COMM_STATUS_RUNNING(commission_state.count);
                        commission_state.next_short_addr =
                            (commission_state.mode == 1u)
                                ? (uint8_t)(sa + 1u)
                                : com_lowest_unused();
                    } else {
                        commission_state.err_streak++;
                        if (commission_state.err_streak > 3u) {
                            commission_state.status = COMM_STATUS_ERR_VERIFY_FAIL;
                            commission_state.phase  = COM_TERMINATE;
                            more = true;
                            break;
                        }
                    }
                    // Reset bisect for next ballast (or no-more probe).
                    com_reset_bisect();
                    commission_state.phase = COM_WITHDRAW;
                    more = true;
                    break;
                }
                case COM_WITHDRAW:
                    // Just-isolated ballast no longer responds; loop
                    // back to a fresh bisect.  Next first-COMPARE
                    // either finds another ballast or returns "no",
                    // which the COM_COMPARE handler turns into natural
                    // completion (status 0x02xx).
                    commission_state.phase = COM_SEARCH_H;
                    more = true;
                    break;
                case COM_TERMINATE:
                    // Stamp natural-completion status iff no terminal
                    // value was already latched (aborted / no-ballasts /
                    // verify-fail / addr-exhaust all set their status
                    // BEFORE TERMINATE issues).
                    if ((uint8_t)(commission_state.status >> 8) == 0x01u)
                        commission_state.status = COMM_STATUS_COMPLETE(commission_state.count);
                    commission_state.phase = COM_IDLE;
                    // more = false → release dali_busy below.
                    break;
                default:
                    // Trivial linear advance via consecutive enum order.
                    commission_state.phase = (commission_phase_t)(p + 1u);
                    more = true;
                    break;
            }
            break;
        }

        case OP_RAW: {
            uint8_t flags = s_raw_cmd[2];

            // DTR0 preload step just finished (no backward frame → OK/NAK is
            // "done").  Advance to the frame within the SAME slot lifecycle; a real
            // bus error falls through to terminal handling below.
            if ((flags & QF_DTR0) && step == 0u
                && (type == DALI_RESPONSE_OK || type == DALI_RESPONSE_NAK)) {
                current_op.step = 1u;
                more = true;
                break;
            }

            // Terminal.  Map the DALI outcome to the slot's (status, response8):
            //   OK        → COMPLETE, response8 = backward-frame answer (0 if none)
            //   NAK       → FAILED,   DALI_RESP_NAK       (no/short backward frame)
            //   MALFORMED → FAILED,   DALI_RESP_COLLISION (garbled backward frame)
            //   other     → FAILED,   DALI_RESP_BUSFAULT  (transport-level fault)
            // mbq_complete writes response8 BEFORE status under the read fence, so a
            // fenced status read never sees a terminal status with a stale value.
            uint8_t st, r8;
            if      (type == DALI_RESPONSE_OK)        { st = MBQ_S_COMPLETE; r8 = value; }
            else if (type == DALI_RESPONSE_NAK)       { st = MBQ_S_FAILED;   r8 = DALI_RESP_NAK; }
            else if (type == DALI_RESPONSE_MALFORMED) { st = MBQ_S_FAILED;   r8 = DALI_RESP_COLLISION; }
            else                                      { st = MBQ_S_FAILED;   r8 = DALI_RESP_BUSFAULT; }
            mbq_complete(s_raw_slot, st, r8);
            // more stays false → release dali_busy; the arbiter picks the next
            // pending slot (if any) on the following main-loop pass.
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

    g_uptime_s++;   // ~1 Hz wall-clock tick (independent of the rescan cursor)

    uint16_t interval = userrow_get_rescan_interval();
    if (interval > 0) {
        tick++;
        if (tick >= interval) {
            tick = 0;
            // RTC PIT runs at LVL0; TCB0 (LVL1) preempts it and
            // dali_sync_completion (called from TCB0 context via
            // dali_result_received) writes sync_flags[idx] for any
            // ballast — including possibly the same index this PIT
            // tick is targeting.  A naked sync_flags[i] |= ... here
            // is a non-atomic LDS/OR/STS triple, so a TCB0 preemption
            // between the load and the store can lose the LVL1 update.
            // Use sync_flags_set (ATOMIC_RESTORESTATE) — it disables
            // I for a few cycles and restores SREG; safe from any
            // context.
            sync_flags_set(scan_cursor, SYNC_READ_LEVEL);
            scan_cursor++;
            if (scan_cursor >= DALI_MAX_DEVICES) scan_cursor = 0;
        }
    } else {
        tick = 0;
    }

    RTC.PITINTFLAGS = RTC_PI_bm;
}
