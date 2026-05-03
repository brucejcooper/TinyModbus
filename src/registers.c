#include <string.h>
#include <avr/io.h>
#include <util/atomic.h>
#include "registers.h"

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
    // The register_map_t is laid out so that each FC04 word occupies exactly 2 bytes
    // in the same order as the Modbus wire, so memcpy is safe for aligned regions.
    const uint8_t *source_ptr = (const uint8_t *)&g_regs + (uint16_t)addr * 2u;
    memcpy(buf, source_ptr, (uint16_t)count * 2u);
}

// ---- FC03 holding read -------------------------------------------------

bool regs_read_holding(uint16_t addr, uint16_t *val) {
    switch (addr) {
        // Action registers read as 0 (write-only by convention)
        case MBREG_RAW_CMD:       case MBREG_RAW_CMD_TWICE:
        case MBREG_DAPC:
        case MBREG_GOTO_LAST_ACTIVE: case MBREG_OFF:          case MBREG_UP:
        case MBREG_DOWN:          case MBREG_STEP_UP:         case MBREG_STEP_DOWN:
        case MBREG_RECALL_MAX:    case MBREG_RECALL_MIN:
        case MBREG_SET_DTR0:      case MBREG_SET_DTR1:        case MBREG_SET_DTR2:
        case MBREG_SCAN_TRIGGER:
            *val = 0;
            return true;
        case MBREG_RESCAN_INTERVAL: *val = userrow_get_rescan_interval(); return true;
        case MBREG_SLAVE_ADDR:      *val = userrow_get_slave_addr();      return true;
        case MBREG_BAUD_INDEX:      *val = userrow_get_baud_index();      return true;
        default:     return false;
    }
}

// ---- FC06/FC10 holding write -------------------------------------------

bool regs_write_holding(uint16_t addr, uint16_t val) {
    switch (addr) {
        case MBREG_RESCAN_INTERVAL:
            return userrow_set_rescan_interval(val);
        case MBREG_SLAVE_ADDR:
            return userrow_set_slave_addr((uint8_t)val);
        case MBREG_BAUD_INDEX:
            return userrow_set_baud_index((uint8_t)val);
        // Action registers: caller handles DALI dispatch
        case MBREG_RAW_CMD:       case MBREG_RAW_CMD_TWICE:
        case MBREG_DAPC:
        case MBREG_GOTO_LAST_ACTIVE: case MBREG_OFF:          case MBREG_UP:
        case MBREG_DOWN:          case MBREG_STEP_UP:         case MBREG_STEP_DOWN:
        case MBREG_RECALL_MAX:    case MBREG_RECALL_MIN:
        case MBREG_SET_DTR0:      case MBREG_SET_DTR1:        case MBREG_SET_DTR2:
        case MBREG_SCAN_TRIGGER:
            return true;
        default:
            return false;
    }
}

// ---- result application ------------------------------------------------

void regs_apply_result(uint8_t idx, reg_target_t target, uint8_t value) {
    switch (target) {
        case REG_BRIGHTNESS:
            g_regs.block0[idx].brightness = value;
            bm_set(g_regs.dirty, idx, true);
            break;
        case REG_STATUS:
            regs_apply_status(idx, value);
            break;
        case REG_MIN:
            g_regs.block1[idx].min = value;
            break;
        case REG_MAX:
            g_regs.block1[idx].max = value;
            break;
        case REG_POWER_ON:
            g_regs.block2[idx].power_on = value;
            break;
        case REG_SYS_FAIL:
            g_regs.block2[idx].sys_fail = value;
            break;
        case REG_FADE_RATE:
            // QUERY_FADE_TIME_FADE_RATE returns one byte: high nibble = fade
            // time (0..15), low nibble = fade rate (0..15).  Split into the
            // two cached fields so a single query refreshes both.
            g_regs.block3[idx].fade_rate = value & 0x0Fu;
            g_regs.block3[idx].fade_time = (uint8_t)(value >> 4) & 0x0Fu;
            break;
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
    bm_set(g_regs.fading, idx, (dali_status & DALI_STATUS_FADE_RUNNING) != 0);
    bm_set(g_regs.dirty,  idx, true);
}

void regs_set_present(uint8_t idx, bool present) {
    bm_set(g_regs.present, idx, present);
}

void regs_clear_ballast(uint8_t idx) {
    g_regs.block0[idx].status     = 0;
    g_regs.block0[idx].brightness = 0;
    memset(&g_regs.block1[idx], 0, sizeof(ballast_mm_t));
    memset(&g_regs.block2[idx], 0, sizeof(ballast_pf_t));
    memset(&g_regs.block3[idx], 0, sizeof(ballast_fade_t));
    regs_set_present(idx, false);
    bm_set(g_regs.fading, idx, false);
    uint8_t byte_idx = idx >> 3;
    uint8_t bit_mask = (uint8_t)(1u << (idx & 7u));
    for (uint8_t g = 0; g < 16; g++) {
        if (g_regs.group_mask[g][byte_idx] & bit_mask)
            regs_rebuild_group_status(g);
    }
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

    for (uint8_t group_idx = 0; group_idx < 8; group_idx++)
        regs_rebuild_group_status(base + group_idx);
}

void regs_rebuild_group_status(uint8_t group_idx) {
    uint8_t merged_status = 0;
    for (uint8_t ballast_idx = 0; ballast_idx < 64; ballast_idx++) {
        if (g_regs.group_mask[group_idx][ballast_idx >> 3] & (1u << (ballast_idx & 7u)))
            merged_status |= g_regs.block0[ballast_idx].status;
    }
    g_regs.group_status[group_idx].status = merged_status;
}
