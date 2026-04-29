#include <string.h>
#include <util/atomic.h>
#include "dali.h"


#define ENCODE_CMD(cmd, addr) ((cmd) | ((addr) << 9))


// Singleton "fade completed" callback - written from main-thread Modbus handlers and
// read+cleared by check_fade_completed running in ISR context. The 16-bit pointer assignment
// is two non-atomic STS instructions; if an ISR fires between them, check_fade_completed
// would call a torn (partly-old / partly-new) pointer. set_post_cmd_callback wraps the write
// in ATOMIC_BLOCK so the writer can't be split by an interrupt.
static dali_callback_t post_cmd_callback = NULL;
static inline void set_post_cmd_callback(dali_callback_t cb) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        post_cmd_callback = cb;
    }
}
static const uint16_t scan_command_sequence[] = {
    DALI_CMD_QUERY_STATUS,
    DALI_CMD_GET_LEVEL,
    DALI_CMD_GET_MAX,
    DALI_CMD_GET_MIN,
    DALI_CMD_GET_EXT_FADE,
    DALI_CMD_GET_FADE,
    DALI_CMD_GET_POWER_FAIL_LEVEL,
    DALI_CMD_GET_POWER_ON_LEVEL,
    DALI_CMD_GET_GROUPS0_7,
    DALI_CMD_GET_GROUPS8_15,
};

static uint8_t scan_index = 0;
static uint8_t scan_err_count = 0;


static void track_fade(uint8_t addr);
dali_result_t dali_queue_command(uint16_t data, uint8_t repeat, dali_callback_t final);


static inline bool isFading(uint8_t addr) {
    return dali_holding_regs.banks[DALI_STATUS_BANK][addr].high & 0x10;
}


#define REG_BANK_SZ (DALI_MAX_DEVICES*sizeof(modbus_register_t))



static void scan_result_received(dali_response_type_t err, uint16_t cmd, uint8_t response) {
    // Once we've scanned the level, we also want to scan the status.
    uint8_t addr = DALI_DECODE_ADDR(cmd);

    switch (err) {
        case DALI_RESPONSE_OK: {
            // Capture the index that *produced* this response before we advance to the next
            // command; otherwise the "fading device" check below can never see scan_index == 0.
            uint8_t completedIndex = scan_index;
            scan_err_count = 0;
            if (scan_index < 9) {
                // Send the next command
                dali_queue_command(ENCODE_CMD(scan_command_sequence[++scan_index], addr), 1, scan_result_received);
            } else {
                // We are done with this address. Move onto the next one.
                dali_start_background_scan(addr + 1);
            }
            if (completedIndex == 0 && (response & 0x10)) {
                // We scanned a device that is currently fading... schedule an ongoing poll
                track_fade(addr);
            }
            break;
        }
        default:
            // We try again for a maximum of three times, then we treat it as a device that is not found.
            if (scan_err_count++ < 3) {
                // Send the same command again!
                dali_queue_command(ENCODE_CMD(scan_command_sequence[scan_index], addr), 1, scan_result_received); 
                break;
            }
            // otherwise, fall through to the not found situation.
        case DALI_RESPONSE_NAK:
            // Set the current and remaining registers that we haven't fetched to their default values.
            for (uint8_t i = scan_index ; i < 10; i++) {
                // We set level to a special marker, but set everything else to 0
                if (i%2 == 0) {
                    modbus_register_t *reg = &dali_holding_regs.banks[i/2][addr];
                    reg->low = i == 0 ? 0xFF : 0;
                    reg->high = 0;

                }
            }
            // We treat a single NAK as the device not being present.
            scan_err_count = 0;
            dali_start_background_scan(addr + 1);
            break;
    }
}


static void toggle_based_on_level(dali_response_type_t err, uint16_t cmd, uint8_t currentLevel) {
    uint8_t addr = DALI_DECODE_ADDR(cmd);
    if (err == DALI_RESPONSE_OK) {
        dali_queue_command(ENCODE_CMD(currentLevel ? DALI_CMD_OFF: DALI_CMD_GOTO_LAST_ACTIVE, addr), 1, NULL); 
    }
    track_fade(addr);
}

void dali_start_background_scan(uint8_t addr) {
    if (addr < 64) {
        // Both the RTC_PIT ISR and TCA0 (via scan_result_received's chain) can call this. TCA0
        // has higher priority and can preempt RTC_PIT, leaving scan_index/scan_err_count
        // inconsistent with what's queued. Atomic block serialises the reset+queue.
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            scan_index = 0;
            scan_err_count = 0;
            dali_queue_command(ENCODE_CMD(scan_command_sequence[scan_index], addr), 1, scan_result_received);
        }
    }
}

void dali_set_level(uint8_t addr, uint8_t level, dali_callback_t final) {
    set_post_cmd_callback(final);
    dali_queue_command(ENCODE_CMD(level, addr), 1, NULL);
    track_fade(addr);
}

void dali_goto_last_active(uint8_t addr, dali_callback_t cb) {
    set_post_cmd_callback(cb);
    dali_queue_command(ENCODE_CMD(DALI_CMD_GOTO_LAST_ACTIVE, addr), 1, NULL);
    track_fade(addr);
}

void dali_off(uint8_t addr, dali_callback_t cb) {
    set_post_cmd_callback(cb);
    dali_queue_command(ENCODE_CMD(DALI_CMD_OFF, addr), 1, NULL);
    track_fade(addr);
}

void dali_set_dtr0(uint8_t val, dali_callback_t callback) {
    dali_queue_command(DALI_CMD_SET_DTR0 | val, 1, callback); 
}



void dali_set_min_level(uint8_t addr, uint8_t val, dali_callback_t callback) {
    dali_set_dtr0(val, NULL);
    dali_queue_command(ENCODE_CMD(DALI_CMD_SET_MIN, addr), 2, NULL); 
    dali_queue_command(ENCODE_CMD(DALI_CMD_GET_MIN, addr), 1, callback); 
    track_fade(addr);
}

void dali_set_max_level(uint8_t addr, uint8_t val, dali_callback_t callback) {
    dali_set_dtr0(val, NULL);
    dali_queue_command(ENCODE_CMD(DALI_CMD_SET_MAX, addr), 2, NULL); 
    dali_queue_command(ENCODE_CMD(DALI_CMD_GET_MAX, addr), 1, callback); 
    track_fade(addr);
}

void dali_set_fade_rate(uint8_t addr, uint8_t val, dali_callback_t callback) {
    dali_set_dtr0(val, NULL);
    dali_queue_command(ENCODE_CMD(DALI_CMD_SET_FADE_RATE, addr), 2, NULL); 
    dali_queue_command(ENCODE_CMD(DALI_CMD_GET_FADE, addr), 1, callback); 
}

void dali_set_fade_time(uint8_t addr, uint8_t val, dali_callback_t callback) {
    dali_set_dtr0(val, NULL);
    dali_queue_command(ENCODE_CMD(DALI_CMD_SET_FADE_TIME, addr), 2, NULL);
    // Read back via GET_FADE (the basic-fade-time/rate combined query) since SET_FADE_TIME
    // writes the basic fade-time register. The previous code queried GET_EXT_FADE, which reads
    // the extended-fade-time register that nothing in this firmware ever sets - so the readback
    // never reflected what we just wrote.
    dali_queue_command(ENCODE_CMD(DALI_CMD_GET_FADE, addr), 1, callback);
}

void dali_set_power_on_level(uint8_t addr, uint8_t val, dali_callback_t callback) {
    dali_set_dtr0(val, NULL);
    dali_queue_command(ENCODE_CMD(DALI_CMD_SET_POWER_ON_LEVEL, addr), 2, NULL);
    dali_queue_command(ENCODE_CMD(DALI_CMD_GET_POWER_ON_LEVEL, addr), 1, callback);
}

void dali_set_power_fail_level(uint8_t addr, uint8_t val, dali_callback_t callback) {
    dali_set_dtr0(val, NULL);
    dali_queue_command(ENCODE_CMD(DALI_CMD_SET_POWER_FAIL_LEVEL, addr), 2, NULL); 
    // Read back the power fail level after writing it.
    dali_queue_command(ENCODE_CMD(DALI_CMD_GET_POWER_FAIL_LEVEL, addr), 1, callback); 
}

void dali_add_to_group(uint8_t addr, uint8_t group, dali_callback_t callback) {
    dali_queue_command(ENCODE_CMD(DALI_CMD_ADD_TO_GROUP + group, addr), 2, NULL); 
    dali_queue_command(ENCODE_CMD(DALI_CMD_GET_GROUPS0_7, addr), 1, callback); 
}

void dali_remove_from_group(uint8_t addr, uint8_t group, dali_callback_t callback) {
    dali_queue_command(ENCODE_CMD(DALI_CMD_REMOVE_FROM_GROUP + group, addr), 2, NULL); 
    dali_queue_command(ENCODE_CMD(DALI_CMD_GET_GROUPS8_15, addr), 1, callback); 
}



void dali_toggle(uint8_t addr, dali_callback_t cb) {
    // Start by fetching the current level
    set_post_cmd_callback(cb);
    dali_queue_command(ENCODE_CMD(DALI_CMD_GET_LEVEL, addr), 1, toggle_based_on_level);
}



void dali_custom_command(uint16_t cmd, dali_callback_t responseCallback, uint8_t repeat) {
    bool isLevelChanging = (cmd & 0x81FF) < 0x120;
    dali_queue_command(cmd, repeat, isLevelChanging ? NULL : responseCallback);
    if (isLevelChanging) {
        uint8_t addr = DALI_DECODE_ADDR(cmd);
        set_post_cmd_callback(responseCallback);
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
