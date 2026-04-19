#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include "bg_tasks.h"
#include "registers.h"
#include "cmd_queue.h"
#include "dali.h"
#include "tick.h"

// ---- RTC / periodic rescan ----------------------------------------

#define RESCAN_PERIOD_S    64u
#define RTC_PIT_HZ         1u    // CYC1024 on 1 kHz internal osc

static volatile uint8_t  rescan_counter  = RESCAN_PERIOD_S;
static volatile bool     scan_trigger    = false;

// ---- Fade poll timer -----------------------------------------------
// Decremented by bgtasks_tick() at one tick (≈416 µs) per count.
// Reloaded from g_regs.poll_interval × TICKS_PER_100MS each run.

static volatile uint16_t fade_poll_timer = 1;  // fire shortly after boot
static volatile bool     fade_poll_due   = false;

// ---- Bus scan state ------------------------------------------------

static volatile bool    scan_active     = false;
static volatile uint8_t scan_addr       = 0;
static volatile uint8_t scan_err_count  = 0;

static void enqueue_scan_for_addr(uint8_t addr);  // forward decl

// ---- Scan callbacks (run in ISR context) ---------------------------

// ISR callback: advance the bus scan to ballast_idx+1, or mark scan complete at addr 63.
static void scan_advance(dali_response_type_t type, uint8_t ballast_idx, uint8_t response) {
    (void)type; (void)response;
    uint8_t next = ballast_idx + 1u;
    if (next < DALI_MAX_DEVICES) {
        enqueue_scan_for_addr(next);
    } else {
        g_regs.scan_status = SCAN_COMPLETE;
        scan_active = false;
    }
}

// ISR callback: handle the QUERY_STATUS result for one ballast during a bus scan.
// OK  → mark present and queue data reads (brightness, min/max, fade, groups).
// NAK → mark absent and advance.
// Other → retry up to 3 times, then treat as absent.
static void scan_status_received(dali_response_type_t type, uint8_t ballast_idx, uint8_t response) {
    if (type == DALI_RESPONSE_OK) {
        scan_err_count = 0;
        regs_set_present(ballast_idx, true);

        // Queue remaining data queries for this ballast.
        // scan_advance callback fires on the last one.
        uint8_t dali_addr_byte = DALI_SHORT_ADDR(ballast_idx) | DALI_CMD_FLAG;
        cmdq_push_query(dali_addr_byte, DALI_CMD_QUERY_ACTUAL_LEVEL,   ballast_idx, REG_BRIGHTNESS);
        cmdq_push_query(dali_addr_byte, DALI_CMD_QUERY_MIN_LEVEL,      ballast_idx, REG_MIN);
        cmdq_push_query(dali_addr_byte, DALI_CMD_QUERY_MAX_LEVEL,      ballast_idx, REG_MAX);
        cmdq_push_query(dali_addr_byte, DALI_CMD_QUERY_POWER_ON_LEVEL, ballast_idx, REG_POWER_ON);
        cmdq_push_query(dali_addr_byte, DALI_CMD_QUERY_FAIL_LEVEL,     ballast_idx, REG_SYS_FAIL);
        cmdq_push_query(dali_addr_byte, DALI_CMD_QUERY_FADE_RATE,      ballast_idx, REG_FADE_RATE);
        cmdq_push_query(dali_addr_byte, DALI_CMD_QUERY_FADE_TIME,      ballast_idx, REG_FADE_TIME);
        cmdq_push_query(dali_addr_byte, DALI_CMD_QUERY_GROUP_0_7,      ballast_idx, REG_GROUP_0_7);
        cmdq_push_query_cb(dali_addr_byte, DALI_CMD_QUERY_GROUP_8_15,  ballast_idx, REG_GROUP_8_15,
                           scan_advance);
    } else if (type == DALI_RESPONSE_NAK) {
        scan_err_count = 0;
        regs_clear_ballast(ballast_idx);
        scan_advance(type, ballast_idx, 0);
    } else {
        // Transient error: retry up to 3 times, then treat as absent
        if (++scan_err_count < 3) {
            uint8_t dali_addr_byte = DALI_SHORT_ADDR(ballast_idx) | DALI_CMD_FLAG;
            cmdq_push_query_cb(dali_addr_byte, DALI_CMD_QUERY_STATUS, ballast_idx,
                               REG_STATUS, scan_status_received);
        } else {
            scan_err_count = 0;
            regs_clear_ballast(ballast_idx);
            scan_advance(type, ballast_idx, 0);
        }
    }
}

// Push QUERY_STATUS for one ballast address and kick the DALI state machine.
static void enqueue_scan_for_addr(uint8_t addr) {
    scan_addr = addr;
    uint8_t dali_addr_byte = DALI_SHORT_ADDR(addr) | DALI_CMD_FLAG;
    cmdq_push_query_cb(dali_addr_byte, DALI_CMD_QUERY_STATUS, addr, REG_STATUS,
                       scan_status_received);
    cmdq_kick();
}

// ---- Public API ----------------------------------------------------

void bgtasks_trigger_scan(void) {
    scan_trigger = true;
}

void bgtasks_tick(void) {
    // Fade poll timer (TCA0 ticks ≈ 416 µs each)
    if (fade_poll_timer && (--fade_poll_timer == 0))
        fade_poll_due = true;
}

void bgtasks_run(void) {
    // Fade poller
    if (fade_poll_due) {
        fade_poll_due = false;
        // Reload timer before pushing so polls can't pile up
        fade_poll_timer = (uint16_t)g_regs.poll_interval * (uint16_t)TICKS_PER_100MS;
        if (fade_poll_timer == 0) fade_poll_timer = 1;

        for (uint8_t word_idx = 0; word_idx < 4; word_idx++) {
            uint16_t fading_mask;
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { fading_mask = g_regs.fading[word_idx]; }
            uint16_t pending_bits = fading_mask;
            while (pending_bits) {
                uint8_t  bit_pos    = 0;
                uint16_t scan_word  = pending_bits;
                while (!(scan_word & 1u)) { bit_pos++; scan_word >>= 1u; }
                uint8_t ballast_idx    = (uint8_t)(word_idx * 16u + bit_pos);
                uint8_t dali_addr_byte = DALI_SHORT_ADDR(ballast_idx) | DALI_CMD_FLAG;
                cmdq_push_query(dali_addr_byte, DALI_CMD_QUERY_STATUS,       ballast_idx, REG_STATUS);
                cmdq_push_query(dali_addr_byte, DALI_CMD_QUERY_ACTUAL_LEVEL, ballast_idx, REG_BRIGHTNESS);
                pending_bits &= pending_bits - 1u;
            }
        }
        cmdq_kick();
    }

    // Bus scan
    if (scan_trigger) {
        scan_trigger = false;
        if (!scan_active) {
            scan_active = true;
            scan_err_count = 0;
            g_regs.scan_status = SCAN_SCANNING;
            enqueue_scan_for_addr(0);
        }
    }
}

// ---- RTC PIT — periodic rescan (same cadence as original) ----------

ISR(RTC_PIT_vect) {
    if (--rescan_counter == 0) {
        rescan_counter = RESCAN_PERIOD_S;
        scan_trigger   = true;
    }
    RTC.PITINTFLAGS = RTC_PI_bm;
}
