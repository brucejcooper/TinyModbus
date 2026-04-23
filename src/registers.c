#include <string.h>
#include <avr/eeprom.h>
#include <avr/io.h>
#include <util/atomic.h>
#include "registers.h"

register_map_t g_regs;

static uint16_t s_rescan_interval = 64u;

// EEPROM layout: byte 0 = slave address, byte 1 = baud index, bytes 2-3 = rescan interval
static uint8_t  EEMEM ee_slave_addr      = 0xFF;
static uint8_t  EEMEM ee_baud_index      = 0xFF;
static uint16_t EEMEM ee_rescan_interval = 0xFFFF;

void regs_init(void) {
    memset(&g_regs, 0, sizeof(g_regs));

    uint16_t rescan_val = eeprom_read_word(&ee_rescan_interval);
    s_rescan_interval = (rescan_val == 0xFFFFu) ? 64u : rescan_val;

    // Slave address: EEPROM first, fall back to USERROW formula
    uint8_t ee_addr = eeprom_read_byte(&ee_slave_addr);
    if (ee_addr >= 1 && ee_addr <= 247) {
        g_regs.mb_slave_addr = ee_addr;
    } else {
        // Legacy: USERROW.USERROW0 + 2; wraps to 1 when USERROW[0]=0xFF
        g_regs.mb_slave_addr = (uint8_t)(USERROW.USERROW0 + 2u);
        if (g_regs.mb_slave_addr == 0 || g_regs.mb_slave_addr > 247)
            g_regs.mb_slave_addr = 1;
    }

    uint8_t baud_idx = eeprom_read_byte(&ee_baud_index);
    g_regs.baud_index = (baud_idx <= 2) ? baud_idx : 0;
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
        case MBREG_RESCAN_INTERVAL: *val = s_rescan_interval; return true;
        case MBREG_SLAVE_ADDR:    *val = g_regs.mb_slave_addr;  return true;
        case MBREG_BAUD_INDEX:    *val = g_regs.baud_index;     return true;
        default:     return false;
    }
}

// ---- FC06/FC10 holding write -------------------------------------------

bool regs_write_holding(uint16_t addr, uint16_t val) {
    switch (addr) {
        case MBREG_RESCAN_INTERVAL:
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { s_rescan_interval = val; }
            eeprom_update_word(&ee_rescan_interval, val);
            return true;
        case MBREG_SLAVE_ADDR:
            if (val < 1 || val > 247) return false;
            g_regs.mb_slave_addr = (uint8_t)val;
            eeprom_update_byte(&ee_slave_addr, g_regs.mb_slave_addr);
            return true;
        case MBREG_BAUD_INDEX:
            if (val > 2) return false;
            g_regs.baud_index = (uint8_t)val;
            eeprom_update_byte(&ee_baud_index, g_regs.baud_index);
            return true;
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
            g_regs.block3[idx].fade_rate = value;
            break;
        case REG_FADE_TIME:
            g_regs.block3[idx].fade_time = value;
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

uint16_t regs_get_rescan_interval(void) {
    return s_rescan_interval;
}
