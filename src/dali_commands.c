#include "dali.h"

#define DALI_CMD_GET_LEVEL 0x1a0
#define DALI_CMD_QUERY_STATUS 0x190
#define DALI_CMD_GOTO_LAST_ACTIVE 0x10a
#define DALI_CMD_OFF 0x100

#define ENCODE_CMD(cmd, addr) ((cmd) | ((addr) << 9))
#define DECODE_ADDR(cmd) (((cmd) >> 9) & 0x3F)

static void scan_result_received(dali_response_type_t err, uint16_t cmd, uint8_t response) {
    uint8_t addr = DECODE_ADDR(cmd);
    if (addr < 64) {
        switch (err) {
            case DALI_RESPONSE_OK:
                dali_holding_regs.power[addr] = response;
                break;
            default:
                dali_holding_regs.power[addr] = err;
        }
        if (addr < 63) {
            dali_start_scan(addr + 1);
        }
    }
}

static void track_fade(dali_response_type_t err, uint16_t cmd, uint8_t response);

static void fade_status_received(dali_response_type_t err, uint16_t cmd, uint8_t deviceStatus) {
    if (err == DALI_RESPONSE_OK) {
        if (deviceStatus & 0x10) {
            // Still fading.
            track_fade(err, cmd, deviceStatus);
        }
    }
}

static void fade_level_received(dali_response_type_t err, uint16_t cmd, uint8_t currentLevel) {
    uint8_t addr = DECODE_ADDR(cmd);
    if (err == DALI_RESPONSE_OK) {
        dali_holding_regs.power[addr] = currentLevel;
    }
    // We continue to fetch the level until the fade status bit is no longer set.
    dali_queue_command(ENCODE_CMD(DALI_CMD_QUERY_STATUS, addr), fade_status_received);
}

static void track_fade(dali_response_type_t err, uint16_t cmd, uint8_t _ignored) {
    // We expect it to have NAKed.  Ignore the response.
    // Fetch current level.
    dali_queue_command(ENCODE_CMD(DALI_CMD_GET_LEVEL, DECODE_ADDR(cmd)), fade_level_received);
}

static void toggle_based_on_level(dali_response_type_t err, uint16_t cmd, uint8_t currentLevel) {
    uint8_t addr = DECODE_ADDR(cmd);
    if (err == DALI_RESPONSE_OK) {
        if (currentLevel) {
            dali_off(addr);
        } else {
            dali_goto_last_active(addr);
        }
    }
}

void dali_start_scan(uint8_t addr) { dali_queue_command(ENCODE_CMD(DALI_CMD_GET_LEVEL, addr), scan_result_received); }

void dali_set_level(uint8_t addr, uint8_t level) { dali_queue_command(ENCODE_CMD(level, addr), track_fade); }

void dali_goto_last_active(uint8_t addr) { dali_queue_command(ENCODE_CMD(DALI_CMD_GOTO_LAST_ACTIVE, addr), track_fade); }

void dali_off(uint8_t addr) { dali_queue_command(ENCODE_CMD(DALI_CMD_OFF, addr), track_fade); }

void dali_toggle(uint8_t addr) {
    // Start by fetching the current level
    dali_queue_command(ENCODE_CMD(DALI_CMD_GET_LEVEL, addr), toggle_based_on_level);
}
