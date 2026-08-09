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

uint8_t userrow_get_slave_addr(void) {
    // 0xFF (post-erase) -> 1 (default).  0xF6..0xFD would map to reserved IDs
    // 248..255; 0xFE would wrap to 0 (broadcast); both clamped to 1.
    uint8_t v = USERROW.USERROW0;
    return (v >= 0xF6 && v != 0xFF) ? 1 : (uint8_t)(v + 2);
}

tm_err_t userrow_set_slave_addr(uint8_t addr) {
    if (addr < 1 || addr > 247) return TM_ERR_BAD_VALUE;
    // addr == 1 stores 0xFF so the default (post-erase) chip continues to
    // come up at address 1.  All other addresses store (addr - 2).
    uint8_t v = (addr == 1) ? 0xFF : (uint8_t)(addr - 2);
    return userrow_write_byte(USERROW_OFFSET_SLAVE_ADDR, v) ? TM_OK : TM_ERR_HW_FAULT;
}

uint8_t userrow_get_baud_index(void) {
    uint8_t v = USERROW.USERROW1;
    return (v <= 2) ? v : 0;
}

tm_err_t userrow_set_baud_index(uint8_t idx) {
    if (idx > 2) return TM_ERR_BAD_VALUE;
    return userrow_write_byte(USERROW_OFFSET_BAUD_INDEX, idx) ? TM_OK : TM_ERR_HW_FAULT;
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
