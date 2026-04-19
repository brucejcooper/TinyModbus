#ifndef REGISTERS_H
#define REGISTERS_H

#include <stdint.h>
#include <stdbool.h>
#include "types.h"

// Paired structs — stored high-byte first to match Modbus wire format.
// Blocks can be memcpy'd directly into FC04 response buffers.
typedef struct { uint8_t status;    uint8_t brightness; } ballast_bs_t;
typedef struct { uint8_t min;       uint8_t max;        } ballast_mm_t;
typedef struct { uint8_t power_on;  uint8_t sys_fail;   } ballast_pf_t;
typedef struct { uint8_t fade_rate; uint8_t fade_time;  } ballast_fade_t;
typedef struct { uint8_t status;    uint8_t brightness; } group_status_t;

// FC04 input register layout (total 350 registers = 0x015C max):
//   0x0000-0x003F  block0[64]       status | brightness
//   0x0040-0x007F  block1[64]       min    | max
//   0x0080-0x00BF  block2[64]       power_on | sys_fail
//   0x00C0-0x00FF  block3[64]       fade_rate | fade_time
//   0x0100-0x010F  group_status[16] ORed member status | last cmd level
//   0x0110-0x014F  group_mask[16][8] 64-bit per-group membership bitmask
//   0x0150         scan_status      0=idle 1=scanning 2=complete
//   0x0151-0x0154  present[4]       64-bit ballast-present bitmask
//   0x0155-0x0158  fading[4]        64-bit active-fade bitmask
//   0x0159-0x015C  dirty[4]         set on block0 change, cleared on FC04 read
typedef struct {
    ballast_bs_t   block0[64];
    ballast_mm_t   block1[64];
    ballast_pf_t   block2[64];
    ballast_fade_t block3[64];
    group_status_t group_status[16];
    uint8_t        group_mask[16][8];
    uint16_t       scan_status;
    uint16_t       present[4];
    uint16_t       fading[4];
    uint16_t       dirty[4];
    // Config (mirrored in holding regs 0x1011-0x1013)
    uint16_t       poll_interval;   // units: 100ms, default 5
    uint8_t        mb_slave_addr;   // 1-247
    uint8_t        baud_index;      // 0=9600 1=19200 2=38400
} register_map_t;

extern register_map_t g_regs;

// Written by regs_apply_result() when reg_target == REG_RAW_RESPONSE.
// Polled by the FC44 busy-wait loop in modbus.c.
extern volatile dali_response_type_t g_raw_resp_type;
extern volatile uint8_t              g_raw_resp_byte;

#define FC04_MAX_ADDR   0x015C
#define SCAN_IDLE       0
#define SCAN_SCANNING   1
#define SCAN_COMPLETE   2

#define DALI_STATUS_FADE_RUNNING 0x10

void     regs_init(void);

// FC04 bulk read — memcpy-safe for aligned block reads
void     regs_read_input_bulk(uint16_t addr, uint8_t count, uint8_t *buf);

// FC03 single-register read; false if address unknown
bool     regs_read_holding(uint16_t addr, uint16_t *val);

// FC06/FC10 write; false if address or value invalid.
// Does NOT enqueue DALI commands — caller does that.
bool     regs_write_holding(uint16_t addr, uint16_t val);

// Update one field from a DALI query response
void     regs_apply_result(uint8_t ballast_idx, reg_target_t target, uint8_t value);

// Apply a full DALI status byte: updates block0.status, fading bitmask, dirty bit
void     regs_apply_status(uint8_t ballast_idx, uint8_t dali_status);

void     regs_set_present(uint8_t ballast_idx, bool present);
void     regs_clear_ballast(uint8_t ballast_idx);

// Returns dirty word for ballasts [word_idx*16 .. word_idx*16+15] and clears it atomically
uint16_t regs_read_clear_dirty(uint8_t word_idx);

// Update group membership from QUERY_GROUP_0_7 or QUERY_GROUP_8_15 response byte
void     regs_update_group_mask(uint8_t ballast_idx, uint8_t bits, bool high_groups);
void     regs_rebuild_group_status(uint8_t group_idx);

// Called by bg_tasks when poll_interval holding reg changes
void     regs_set_poll_interval(uint16_t val);

#endif
