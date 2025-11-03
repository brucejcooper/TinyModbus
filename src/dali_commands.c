#include <string.h>
#include "dali.h"

#define DALI_CMD_GET_LEVEL 0x1a0
#define DALI_CMD_QUERY_STATUS 0x190
#define DALI_CMD_GOTO_LAST_ACTIVE 0x10a
#define DALI_CMD_OFF 0x100

#define ENCODE_CMD(cmd, addr) ((cmd) | ((addr) << 9))


static dali_callback_t post_cmd_callback = NULL;

static void track_fade(uint8_t addr);
dali_result_t dali_queue_command(uint16_t data, uint8_t repeat, dali_callback_t final);


static inline bool isFading(uint8_t addr) {
    return dali_holding_regs.power[addr].high & 0x10;
}


static void scan_level_received(dali_response_type_t err, uint16_t cmd, uint8_t response) {
    // Once we've scanned the level, we also want to scan the status.
    uint8_t addr = DALI_DECODE_ADDR(cmd);
    modbus_register_t *reg = dali_holding_regs.power + addr;

    if (err == DALI_RESPONSE_OK) {
       reg->low = response;
    } else {
        reg->low = 0xFF;
    }
    if (addr < 64) {
        if (addr < 63) {
            dali_start_background_scan(addr + 1);
        }
    }
    if (reg->high & 0x10) {
        // Still fading. poll again.
        // TODO this could call the wrong post_cmd_callback if it interleaves a scan with a command.
        track_fade(addr);
    }
}

static void scan_status_received(dali_response_type_t err, uint16_t cmd, uint8_t response) {
    uint8_t addr = DALI_DECODE_ADDR(cmd);
    if (err == DALI_RESPONSE_OK) {
        dali_holding_regs.power[addr].high = response;
    } else {
        dali_holding_regs.power[addr].high = 0x00;
    }
    dali_queue_command(ENCODE_CMD(DALI_CMD_GET_LEVEL, addr), 1, scan_level_received); 
}


static void toggle_based_on_level(dali_response_type_t err, uint16_t cmd, uint8_t currentLevel) {
    uint8_t addr = DALI_DECODE_ADDR(cmd);
    if (err == DALI_RESPONSE_OK) {
        dali_queue_command(ENCODE_CMD(currentLevel ? DALI_CMD_OFF: DALI_CMD_GOTO_LAST_ACTIVE, addr), 1, NULL); 
    }
    track_fade(addr);
}

void dali_start_background_scan(uint8_t addr) { 
    dali_queue_command(ENCODE_CMD(DALI_CMD_QUERY_STATUS, addr), 1, scan_status_received); 
}

void dali_set_level(uint8_t addr, uint8_t level, dali_callback_t final) { 
    post_cmd_callback = final;
    dali_queue_command(ENCODE_CMD(level, addr), 1, NULL); 
    track_fade(addr);
}

void dali_goto_last_active(uint8_t addr, dali_callback_t cb) { 
    post_cmd_callback = cb;
    dali_queue_command(ENCODE_CMD(DALI_CMD_GOTO_LAST_ACTIVE, addr), 1, NULL); 
    track_fade(addr);
}

void dali_off(uint8_t addr, dali_callback_t cb) { 
    post_cmd_callback = cb;
    dali_queue_command(ENCODE_CMD(DALI_CMD_OFF, addr), 1, NULL); 
    track_fade(addr);
}

void dali_toggle(uint8_t addr, dali_callback_t cb) {
    // Start by fetching the current level
    post_cmd_callback = cb;
    dali_queue_command(ENCODE_CMD(DALI_CMD_GET_LEVEL, addr), 1, toggle_based_on_level);
}



void dali_custom_command(uint16_t cmd, dali_callback_t responseCallback, uint8_t repeat) {
    bool isLevelChanging = (cmd & 0x81FF) < 0x120;
    dali_queue_command(cmd, repeat, isLevelChanging ? NULL : responseCallback);
    if (isLevelChanging) {
        uint8_t addr = DALI_DECODE_ADDR(cmd);
        post_cmd_callback = responseCallback;
        track_fade(DALI_DECODE_ADDR(addr));
    }
}


static void check_fade_completed(dali_response_type_t err, uint16_t cmd, uint8_t response) {
    uint8_t addr = DALI_DECODE_ADDR(cmd);
    if (post_cmd_callback != NULL) {
        post_cmd_callback(err, cmd, response);
        post_cmd_callback = NULL;
    }
    if (isFading(addr)) {
        // Still fading.  poll again.
        track_fade(addr);
    }
}


static void track_fade(uint8_t addr) {
    dali_queue_command(ENCODE_CMD(DALI_CMD_QUERY_STATUS, addr), 1, NULL);
    dali_queue_command(ENCODE_CMD(DALI_CMD_GET_LEVEL, addr), 1, check_fade_completed);
}
