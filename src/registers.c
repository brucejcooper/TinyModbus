#include <string.h>
#include <avr/io.h>
#include <util/atomic.h>
#include "registers.h"
#include "dali_sync.h"

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

// sizeof(USERROW) is the full 32-byte hardware user row, but no setter ever
// stages past offset 3 (above) — shadowing the whole row for a deferred
// commit would cost 28 bytes of SRAM nothing uses. The commit loop below
// still reloads all 32 real bytes from flash (required to survive the page
// erase); only this much smaller prefix needs an SRAM shadow.
#define USERROW_PENDING_BYTES      4u

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

// ---- Deferred commit -------------------------------------------------------
//
// A register write stages the new byte here — pure SRAM, no flash access, so
// it never halts the CPU — and userrow_service() performs the actual
// PAGEERASEWRITE later, from the main loop, only when start_next_dali_work()
// (dali_sync.h) has just reported nothing outstanding: no command executing,
// none queued, and the bus not settling. That is the same "genuinely nothing
// to do" fact main.c already computes once per iteration to decide whether to
// sleep. Before this, the commit ran synchronously inside modbus_process() on
// every slave-address/baud-index write with no check at all — if a DALI
// command was mid-flight, the ~3.4 ms hardware halt (ISRs frozen, comment on
// userrow_commit() below) froze TCA0_OVF bit timing mid-frame, potentially
// leaving PB0 asserted and corrupting a forward frame the ballast was
// receiving — a double-apply risk for a non-idempotent command.
//
// A reset (any cause) arriving before the deferred commit fires loses the
// staged write, same as a reset landing a cycle before the old synchronous
// commit used to run — s_pending_mask is ordinary .bss, zeroed on every boot.
static uint8_t s_pending_bytes[USERROW_PENDING_BYTES];
static uint8_t s_pending_mask = 0;   // bit i set -> s_pending_bytes[i] awaits commit

// Stage one byte of USERROW for deferred commit. Folds to a no-op if `value`
// already matches whatever is currently effective — the pending byte if one
// is staged for this offset, otherwise the committed flash byte — so a
// register write that doesn't actually change anything costs nothing, same
// as the old synchronous path.
static bool userrow_stage_byte(uint8_t offset, uint8_t value) {
    if (offset >= USERROW_PENDING_BYTES) return false;
    volatile uint8_t *userrow_bytes = (volatile uint8_t *)&USERROW;
    uint8_t effective = (s_pending_mask & (uint8_t)(1u << offset))
                             ? s_pending_bytes[offset] : userrow_bytes[offset];
    if (effective == value) return true;
    s_pending_bytes[offset] = value;
    s_pending_mask |= (uint8_t)(1u << offset);
    return true;
}

// Commit any staged USERROW bytes to flash. Call from the main loop only when
// start_next_dali_work() reports nothing outstanding (see dali_sync.h). No-op
// when nothing is staged, so it's cheap to call unconditionally from that
// gate every iteration.
//
// RACE-FREE AGAINST THE DALI COMPLETION ISR. Nothing outside
// start_next_dali_work() ever moves a command from CQ_NONE to executing, and
// this function runs from main.c strictly BETWEEN two calls to
// start_next_dali_work() rather than concurrently with one, so the caller's
// dali_sync_owns_channel() check cannot go stale before we get here. The one
// thing that CAN flip asynchronously is dali_bus_busy: the rising-edge
// monitor (ISR(PORTB_PORT_vect) in dali_init.c, default priority = LVL0) sets
// it when external DALI traffic starts while we're parked. That is the exact
// hazard dali_sync.c's own launch commit guards against with a plain
// ATOMIC_BLOCK, and for the same reason it's enough here: that ISR is LVL0,
// and a global cli() excludes LVL0. TCB0's completion path runs at LVL1 (see
// tick.c's CPUINT.LVL1VEC) and dali_sync.c fences it SEPARATELY for its
// queue-slot test-and-set — but that fence exists because LVL1 can preempt a
// plain cli(); it is not needed here because TCB0's path never sets
// dali_bus_busy true, only false (dali_state_idle.c, releasing the channel).
// Once PAGEERASEWRITE actually issues below, the NVM controller hardware-
// halts the CPU outright (userrow_commit()'s own comment), so there is
// nothing left to race from that point on.
void userrow_service(void) {
    if (!s_pending_mask) return;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (dali_sync_owns_channel() || dali_bus_busy) return;   // try again next pass
        while (NVMCTRL.STATUS & NVMCTRL_FBUSY_bm) {}
        volatile uint8_t *userrow_bytes = (volatile uint8_t *)&USERROW;
        // Reload every real byte of the page (all 32 survive the erase this
        // way) but only the first USERROW_PENDING_BYTES have an SRAM shadow
        // to consult — the rest are never staged, so their current flash
        // value is already the effective one.
        for (uint8_t i = 0; i < sizeof(USERROW); i++) {
            uint8_t b = (i < USERROW_PENDING_BYTES && (s_pending_mask & (uint8_t)(1u << i)))
                            ? s_pending_bytes[i] : userrow_bytes[i];
            userrow_bytes[i] = b;
        }
        s_pending_mask = 0;
        userrow_commit();
    }
}

uint8_t userrow_get_slave_addr(void) {
    // A staged-but-not-yet-committed write is the effective value: this is
    // what lets a client read back its own FC06 write immediately, without
    // waiting for the deferred flash commit to actually land.
    uint8_t v = (s_pending_mask & (uint8_t)(1u << USERROW_OFFSET_SLAVE_ADDR))
                    ? s_pending_bytes[USERROW_OFFSET_SLAVE_ADDR] : USERROW.USERROW0;
    // 0xFF (post-erase) -> 1 (default).  0xF6..0xFD would map to reserved IDs
    // 248..255; 0xFE would wrap to 0 (broadcast); both clamped to 1.
    return (v >= 0xF6 && v != 0xFF) ? 1 : (uint8_t)(v + 2);
}

tm_err_t userrow_set_slave_addr(uint8_t addr) {
    if (addr < 1 || addr > 247) return TM_ERR_BAD_VALUE;
    // addr == 1 stores 0xFF so the default (post-erase) chip continues to
    // come up at address 1.  All other addresses store (addr - 2).
    uint8_t v = (addr == 1) ? 0xFF : (uint8_t)(addr - 2);
    return userrow_stage_byte(USERROW_OFFSET_SLAVE_ADDR, v) ? TM_OK : TM_ERR_HW_FAULT;
}

uint8_t userrow_get_baud_index(void) {
    uint8_t v = (s_pending_mask & (uint8_t)(1u << USERROW_OFFSET_BAUD_INDEX))
                    ? s_pending_bytes[USERROW_OFFSET_BAUD_INDEX] : USERROW.USERROW1;
    return (v <= 2) ? v : 0;
}

tm_err_t userrow_set_baud_index(uint8_t idx) {
    if (idx > 2) return TM_ERR_BAD_VALUE;
    return userrow_stage_byte(USERROW_OFFSET_BAUD_INDEX, idx) ? TM_OK : TM_ERR_HW_FAULT;
}

void regs_init(void) {
    // Nothing to initialise: the gateway holds no register map any more. Kept as
    // a call site so the boot sequence reads the same and a future addition has
    // an obvious home.
}

// ---- FC03 holding read -------------------------------------------------

tm_err_t regs_read_holding(uint16_t addr, uint16_t *val) {
    // Holding space serves exactly two things now: the slave address and the
    // baud index. Anything else falls through to ILLEGAL DATA ADDRESS — the
    // gateway caches nothing about the bus, so there is no ballast, presence or
    // group state to read here, and a client asking for it should fail loudly
    // rather than receive plausible zeros.
    //
    // The v2 slot status block (0x1010-0x1011) went with the slot model: under
    // v4 the master learns results from the FC 0x41 reply's head field, so there
    // is no queue state to expose as registers at all, and the range is retired
    // to ILLEGAL DATA ADDRESS.
    switch (addr) {
        // Action registers read as 0 (write-only by convention).
        case MBREG_SLAVE_ADDR:      *val = userrow_get_slave_addr();      return TM_OK;
        case MBREG_BAUD_INDEX:      *val = userrow_get_baud_index();      return TM_OK;
        default:                                              return TM_ERR_NO_SUCH_REG;
    }
}

// ---- FC06/FC10 holding write -------------------------------------------

tm_err_t regs_write_holding(uint16_t addr, uint16_t val) {
    // Writable holding space is config only: slave address, baud index and the
    // system-reset magic. No group masks — the controller diffs membership
    // against its own cache and emits ADD/REMOVE FROM GROUP itself, so a gateway
    // reboot cannot lose it.

    switch (addr) {
        // The setters distinguish BAD_VALUE from HW_FAULT themselves, so this
        // forwards rather than re-deriving the ranges in a second place.
        // A 16-bit val above 255 would truncate into range, so check width here.
        case MBREG_SLAVE_ADDR:
            if (val > 0xFFu) return TM_ERR_BAD_VALUE;
            return userrow_set_slave_addr((uint8_t)val);
        case MBREG_BAUD_INDEX:
            if (val > 0xFFu) return TM_ERR_BAD_VALUE;
            return userrow_set_baud_index((uint8_t)val);
        case MBREG_SYSTEM_RESET:
            // Validate the magic here; dispatch_action_reg latches the deferred
            // reset so the FC06/FC10 ACK can still go out before we reboot.
            return (val == MBREG_SYSTEM_RESET_MAGIC) ? TM_OK : TM_ERR_BAD_VALUE;
        default:
            return TM_ERR_NO_SUCH_REG;
    }
}
