#include <string.h>
#include <avr/io.h>
#include <util/atomic.h>
#include "registers.h"
#include "dali_sync.h"
#include "mbq_slots.h"

register_map_t g_regs;

// ---- USERROW layout -------------------------------------------------------
// byte 0:    slave address (encoded as addr - 2; 0xFF -> address 1)
// byte 1:    baud index (0=9600, 1=19200, 2=38400; 0xFF -> 9600)
// bytes 2-3: rescan interval, big-endian (0xFFFF -> 64 s)
//
// 0xFF is the post-erase / unconfigured value at every offset; each getter
// maps it back to the documented default (address 1 / 9600 / 64 s).

#define USERROW_OFFSET_SLAVE_ADDR  0u
#define USERROW_OFFSET_BAUD_INDEX  1u
#define USERROW_OFFSET_RESCAN_HI   2u
#define USERROW_OFFSET_RESCAN_LO   3u

#define DEFAULT_RESCAN_INTERVAL    64u

// SPM-CCP unlock + PAGEERASEWRITE.  The store to NVMCTRL.CTRLA must happen
// within 4 cycles of the CCP write, so cli/out/sts are kept adjacent in one
// inline-asm block.  CPU is halted by NVMCTRL for the duration of the
// erase+write (~3.4 ms); ISRs do not fire during the halt.  This avr-libc
// version ships ccp_write_io but not ccp_write_spm, so the unlock is open-coded.
static void userrow_commit(void) {
    uint8_t sreg = SREG;
    __asm__ __volatile__(
        "cli"                       "\n\t"
        "out %[ccp], %[key]"        "\n\t"
        "sts %[reg], %[cmd]"
        :
        : [ccp] "I" (_SFR_IO_ADDR(CCP)),
          [key] "r" ((uint8_t)CCP_SPM_gc),
          [reg] "n" (_SFR_MEM_ADDR(NVMCTRL.CTRLA)),
          [cmd] "r" ((uint8_t)NVMCTRL_CMD_PAGEERASEWRITE_gc)
    );
    SREG = sreg;
    while (NVMCTRL.STATUS & NVMCTRL_FBUSY_bm) {}
}

// Update one byte of USERROW.  Reloads every byte of the page buffer (reads
// hit flash; writes land in the buffer) so that the unmodified bytes are
// preserved through the page-erase that PAGEERASEWRITE performs first.
// No-op (and no flash-cycle cost) if the byte already matches.
static bool userrow_write_byte(uint8_t offset, uint8_t value) {
    if (offset >= sizeof(USERROW)) return false;

    volatile uint8_t *userrow_bytes = (volatile uint8_t *)&USERROW;
    while (NVMCTRL.STATUS & NVMCTRL_FBUSY_bm) {}
    if (userrow_bytes[offset] == value) return true;

    for (uint8_t i = 0; i < sizeof(USERROW); i++) {
        uint8_t b = (i == offset) ? value : userrow_bytes[i];
        userrow_bytes[i] = b;
    }
    userrow_commit();
    return true;
}

// Update two adjacent bytes of USERROW in a single page-erase-write cycle.
// Used for the 16-bit rescan interval so a config write costs one 3.4 ms
// stall, not two.
static bool userrow_write_word(uint8_t offset_hi, uint8_t hi, uint8_t lo) {
    if ((uint16_t)offset_hi + 1u >= sizeof(USERROW)) return false;

    volatile uint8_t *userrow_bytes = (volatile uint8_t *)&USERROW;
    while (NVMCTRL.STATUS & NVMCTRL_FBUSY_bm) {}
    if (userrow_bytes[offset_hi] == hi && userrow_bytes[offset_hi + 1u] == lo)
        return true;

    for (uint8_t i = 0; i < sizeof(USERROW); i++) {
        uint8_t b;
        if      (i == offset_hi)        b = hi;
        else if (i == offset_hi + 1u)   b = lo;
        else                            b = userrow_bytes[i];
        userrow_bytes[i] = b;
    }
    userrow_commit();
    return true;
}

uint8_t userrow_get_slave_addr(void) {
    // 0xFF (post-erase) -> 1 (default).  0xF6..0xFD would map to reserved IDs
    // 248..255; 0xFE would wrap to 0 (broadcast); both clamped to 1.
    uint8_t v = USERROW.USERROW0;
    return (v >= 0xF6 && v != 0xFF) ? 1 : (uint8_t)(v + 2);
}

bool userrow_set_slave_addr(uint8_t addr) {
    if (addr < 1 || addr > 247) return false;
    // addr == 1 stores 0xFF so the default (post-erase) chip continues to
    // come up at address 1.  All other addresses store (addr - 2).
    uint8_t v = (addr == 1) ? 0xFF : (uint8_t)(addr - 2);
    return userrow_write_byte(USERROW_OFFSET_SLAVE_ADDR, v);
}

uint8_t userrow_get_baud_index(void) {
    uint8_t v = USERROW.USERROW1;
    return (v <= 2) ? v : 0;
}

bool userrow_set_baud_index(uint8_t idx) {
    if (idx > 2) return false;
    return userrow_write_byte(USERROW_OFFSET_BAUD_INDEX, idx);
}

uint16_t userrow_get_rescan_interval(void) {
    uint16_t v = ((uint16_t)USERROW.USERROW2 << 8) | USERROW.USERROW3;
    return (v == 0xFFFFu) ? DEFAULT_RESCAN_INTERVAL : v;
}

bool userrow_set_rescan_interval(uint16_t val) {
    return userrow_write_word(USERROW_OFFSET_RESCAN_HI,
                              (uint8_t)(val >> 8), (uint8_t)val);
}

void regs_init(void) {
    memset(&g_regs, 0, sizeof(g_regs));
}

// ---- bitmask helpers ---------------------------------------------------

static void bm_set(uint16_t *words, uint8_t idx, bool val) {
    uint8_t  word_idx = idx >> 4;
    uint16_t bit_mask = (uint16_t)1u << (idx & 0x0F);
    if (val) words[word_idx] |=  bit_mask;
    else     words[word_idx] &= ~bit_mask;
}

// ---- FC04 bulk read ----------------------------------------------------

void regs_read_input_bulk(uint16_t addr, uint8_t count, uint8_t *buf) {
    // FC04 input ladder 0x0100-0x0107 = present[4] | dirty[4], a contiguous run
    // of big-endian uint16_t in register_map_t.  With the config banks retired
    // the old "struct offset == addr*2" identity no longer holds, so index from
    // the named field base instead.  Caller (process_fc04) only reaches here for
    // FC04_PRESENT_BASE..FC04_INPUT_END.
    const uint8_t *source_ptr = (const uint8_t *)&g_regs.present[0]
                              + (uint16_t)(addr - FC04_PRESENT_BASE) * 2u;
    memcpy(buf, source_ptr, (uint16_t)count * 2u);
}

// ---- FC03 holding read -------------------------------------------------

bool regs_read_holding(uint16_t addr, uint16_t *val) {
    // Live arc-level bank 0x0000-0x003F (status | brightness).  Read by name;
    // block0 is the only per-ballast bank left after the config-bank retirement.
    if (addr <= 0x003Fu) {
        const ballast_bs_t *b = &g_regs.block0[addr];
        *val = ((uint16_t)b->status << 8) | b->brightness;
        return true;
    }
    // 0x0040-0x00FF: retired config banks (min/max/power-on/sys-fail/fade).
    // The controller owns config over raw DALI now (task-03 Phase D), so these
    // fall through to ILLEGAL DATA ADDRESS below rather than returning stale
    // zeros that a legacy bank-reading client would mistake for real values.
    // Group membership masks 0x0110-0x014F: 16 groups × 4 registers (8 bytes
    // each).  Indexed from the named field base — the old offset-trick anchor
    // (struct offset 0x0220) is gone with the config banks.
    if (addr >= 0x0110u && addr <= 0x014Fu) {
        const uint8_t *p = &g_regs.group_mask[0][0] + (uint16_t)(addr - 0x0110u) * 2u;
        *val = ((uint16_t)p[0] << 8) | p[1];
        return true;
    }
    // Slot status block at 0x1010-0x1017 (served identically under FC03 and FC04;
    // FC23's read range is holding-only, so it targets this).  Served one register
    // (one slot's [status|response8]) at a time from the generic queue module; each
    // slot register is independently fenced and self-describing, so a tear across
    // slots is benign (slots reconcile independently).  0x1018-0x1019 fall through
    // to ILLEGAL DATA ADDRESS (retired v1 QRES[6/7]).
    if (addr >= MBREG_QSTAT_BASE && addr <= MBREG_QSTAT_END) {
        uint8_t out[2];
        mbq_read_status((uint8_t)(addr - MBREG_QSTAT_BASE), 1u, out);
        *val = ((uint16_t)out[0] << 8) | out[1];
        return true;
    }

    switch (addr) {
        // Action / control registers read as 0 (write-only by convention)
        case MBREG_SCAN_TRIGGER:
            *val = 0;
            return true;
        case MBREG_RESCAN_INTERVAL: *val = userrow_get_rescan_interval(); return true;
        case MBREG_SLAVE_ADDR:      *val = userrow_get_slave_addr();      return true;
        case MBREG_BAUD_INDEX:      *val = userrow_get_baud_index();      return true;
        case MBREG_COMMISSION: {
            // Read-clears semantics on TERMINAL high bytes (0x02 complete,
            // 0x03 error/aborted).  Idle (0x00) and running (0x01) values stay
            // put so the master can poll progress without losing state.
            // Capture must be atomic w.r.t. the OP_COMMISSION ISR that
            // writes commission_state.status; TCB0 fence guards LVL1,
            // ATOMIC_BLOCK guards LVL0.
            uint16_t snap;
            uint8_t old_tcb = TCB0.INTCTRL;
            TCB0.INTCTRL = 0;
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                snap = commission_state.status;
                uint8_t hi = (uint8_t)(snap >> 8);
                if (hi == 0x02u || hi == 0x03u)
                    commission_state.status = COMM_STATUS_IDLE;
            }
            TCB0.INTCTRL = old_tcb;
            *val = snap;
            return true;
        }
        default:     return false;
    }
}

// ---- FC06/FC10 holding write -------------------------------------------

bool regs_write_holding(uint16_t addr, uint16_t val) {
    // Per-ballast bank writes: predictive cache update + SYNC_WRITE_*
    // arming.  The work selector services the flag and calls
    // issue_dali_command, which in turn handles cache-invalidation
    // uniformly for all DALI command sources.
    if (addr <= 0x003Fu) {
        // Brightness bank: low byte = DAPC level; high byte (status) ignored.
        // Status is owned by DALI query responses, not master writes.
        // 8-bit store is atomic on AVR; no ATOMIC needed for this byte.
        g_regs.block0[addr].brightness = (uint8_t)val;
        sync_flags_set((uint8_t)addr, SYNC_WRITE_LEVEL);
        return true;
    }
    // 0x0040-0x00FF: retired config banks — writes are no longer accepted (the
    // controller stores config via raw DTR0+SET pairs).  Fall through to the
    // default ILLEGAL DATA ADDRESS return.
    // Group masks 0x0110-0x014F: validate, snapshot old value, compute
    // pending_adds = new & ~old, pending_removes = old & ~new, update
    // cache, arm group_write state machine.  Master writes that don't
    // change any bits (changed == 0) need no DALI work and return
    // success WITHOUT consulting the group_write state — even if a
    // group write is in flight.  Only a write that actually requires
    // ADD/REMOVE commands needs to fence against an existing in-flight.
    if (addr >= 0x0110u && addr <= 0x014Fu) {
        uint8_t group_idx = (uint8_t)((addr - 0x0110u) >> 2);
        uint8_t word_idx  = (uint8_t)((addr - 0x0110u) & 0x03u);
        uint16_t old_val, new_val = val;
        old_val  = ((uint16_t)g_regs.group_mask[group_idx][word_idx * 2u]      << 8)
                 |  (uint16_t)g_regs.group_mask[group_idx][word_idx * 2u + 1u];
        uint16_t changed = (uint16_t)(old_val ^ new_val);
        if (changed == 0) {
            // No-op write: nothing to send to ballasts, just ACK.
            return true;
        }
        uint16_t pending_adds    = (uint16_t)(new_val & changed);
        uint16_t pending_removes = (uint16_t)(old_val & changed);
        // Predictive cache update.
        g_regs.group_mask[group_idx][word_idx * 2u]      = (uint8_t)(new_val >> 8);
        g_regs.group_mask[group_idx][word_idx * 2u + 1u] = (uint8_t)new_val;
        if (!dali_sync_arm_group_write(addr, pending_adds, pending_removes)) {
            // Race: another group write became armed between caller's
            // pre-check and now (or no pre-check happened at all).
            // Rollback cache and refuse.
            g_regs.group_mask[group_idx][word_idx * 2u]      = (uint8_t)(old_val >> 8);
            g_regs.group_mask[group_idx][word_idx * 2u + 1u] = (uint8_t)old_val;
            return false;
        }
        return true;
    }

    switch (addr) {
        case MBREG_RESCAN_INTERVAL:
            return userrow_set_rescan_interval(val);
        case MBREG_SLAVE_ADDR:
            return userrow_set_slave_addr((uint8_t)val);
        case MBREG_BAUD_INDEX:
            return userrow_set_baud_index((uint8_t)val);
        // Scan-trigger: caller (dispatch_action_reg) handles the DALI dispatch.
        case MBREG_SCAN_TRIGGER:
            return true;
        case MBREG_SYSTEM_RESET:
            // Validate the magic here; dispatch_action_reg latches the deferred
            // reset so the FC06/FC10 ACK can still go out before we reboot.  A
            // wrong value falls through to ILLEGAL_ADDRESS.
            return (val == MBREG_SYSTEM_RESET_MAGIC);
        case MBREG_COMMISSION: {
            // 0x0000 abort, 0x0001 full, 0x0002 incremental, 0x03YY re-provision
            // the gear at short address YY.  Anything else is ILLEGAL_VALUE; an
            // attempt to start while a run is already in progress is DEVICE_BUSY.
            // We return false in both cases — process_fc06 / process_fc10
            // disambiguate via MBREG_COMM_VALID.
            if (!MBREG_COMM_VALID(val)) return false;
            if (val == 0u) {
                // Abort: latch the request iff a run is actually in
                // progress.  No-op when idle or in a terminal state
                // (master must read 0x1014 to clear those).  phase is a
                // 1-byte packed enum so the read is atomic on AVR.
                if (commission_state.phase != COM_IDLE)
                    commission_state.abort_requested = true;
                return true;
            }
            if ((val & 0xFF00u) == MBREG_COMM_REPROV_BASE)
                return dali_sync_arm_commission(3u, (uint8_t)(val & 0x00FFu));
            return dali_sync_arm_commission((uint8_t)val, 0u);
        }
        default:
            return false;
    }
}

// ---- result application ------------------------------------------------

// Apply a single settings-scan query response to the group-membership cache.
// Only called from dali_sync.c's OP_FULL_SCAN completion path.  After the
// config-bank retirement the settings scan reads only group membership
// (REG_GROUP_0_7 / REG_GROUP_8_15) — the min/max/power/sys-fail/fade fields are
// no longer cached on the gateway, so their targets are inert here.
void regs_apply_result(uint8_t idx, reg_target_t target, uint8_t value) {
    switch (target) {
        case REG_GROUP_0_7:
            regs_update_group_mask(idx, value, false);
            break;
        case REG_GROUP_8_15:
            regs_update_group_mask(idx, value, true);
            break;
        default:
            break;
    }
}

void regs_apply_status(uint8_t idx, uint8_t dali_status) {
    g_regs.block0[idx].status = dali_status;
    bm_set(g_regs.dirty,  idx, true);
}

void regs_set_present(uint8_t idx, bool present) {
    bm_set(g_regs.present, idx, present);
}

void regs_clear_ballast(uint8_t idx) {
    // Only block0 (live level/status) remains a per-ballast cache; the config
    // banks were retired.  Group membership is cleared separately when a
    // ballast is re-commissioned; a plain absence transition leaves the group
    // masks alone (they are re-synced by the settings scan on re-appearance).
    g_regs.block0[idx].status     = 0;
    g_regs.block0[idx].brightness = 0;
    regs_set_present(idx, false);
}

uint16_t regs_read_clear_dirty(uint8_t word_idx) {
    uint16_t val;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        val = g_regs.dirty[word_idx];
        g_regs.dirty[word_idx] = 0;
    }
    return val;
}

void regs_update_group_mask(uint8_t ballast_idx, uint8_t bits, bool high_groups) {
    uint8_t base     = high_groups ? 8 : 0;
    uint8_t byte_idx = ballast_idx >> 3;
    uint8_t bit_mask = (uint8_t)(1u << (ballast_idx & 7u));

    for (uint8_t group_idx = 0; group_idx < 8; group_idx++) {
        if (bits & (1u << group_idx))
            g_regs.group_mask[base + group_idx][byte_idx] |=  bit_mask;
        else
            g_regs.group_mask[base + group_idx][byte_idx] &= ~bit_mask;
    }
}
