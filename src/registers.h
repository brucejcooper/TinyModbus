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

// FC04 input register layout (total 351 registers = 0x015D max):
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
//   0x015D         raw_cmd_result   high=0x00 idle, 0xFF pending, 0x01 nak, 0x02 ok, 0x03 err
//                                   low byte: DALI response (when ok) or error subtype (when err)
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
    uint16_t       raw_cmd_result;   // FC04 0x015D, see RAW_RESULT_* below
    // Config (mirrored in holding regs 0x1012-0x1013; beyond FC04 window)
    uint8_t        mb_slave_addr;   // 1-247
    uint8_t        baud_index;      // 0=9600 1=19200 2=38400
} register_map_t;

extern register_map_t g_regs;

// ---- FC04 input register addresses ----------------------------------------

#define FC04_DIRTY_BASE     0x0159u  // first dirty word register (4 words: 0x0159-0x015C)
#define FC04_DIRTY_END      0x015Cu  // last dirty word register
#define FC04_RAW_RESULT     0x015Du  // async raw DALI command result (see RAW_RESULT_*)
#define FC04_MAX_ADDR       FC04_RAW_RESULT

// ---- FC04 / FC03 special block addresses ----------------------------------

#define FC04_VERSION_BLOCK  0x0200u  // version string, 16 registers, read-only

// ---- Modbus holding register (FC03/FC06) addresses ------------------------

// Raw DALI forward frame — [15:8]=frame byte 0, [7:0]=frame byte 1
#define MBREG_RAW_CMD           0x1000u  // single send, waits for response
#define MBREG_RAW_CMD_TWICE     0x1001u  // send twice, no response expected

// Direct arc power control — [15:8]=DALI address byte, [7:0]=arc level
#define MBREG_DAPC              0x1002u

// Per-ballast action commands — write low byte = DALI address byte
#define MBREG_GOTO_LAST_ACTIVE  0x1003u
#define MBREG_OFF               0x1004u
#define MBREG_UP                0x1005u
#define MBREG_DOWN              0x1006u
#define MBREG_STEP_UP           0x1007u
#define MBREG_STEP_DOWN         0x1008u
#define MBREG_RECALL_MAX        0x1009u
#define MBREG_RECALL_MIN        0x100Au

// DTR set commands — write low byte = value to load into DTR
#define MBREG_SET_DTR0          0x100Bu
#define MBREG_SET_DTR1          0x100Cu
#define MBREG_SET_DTR2          0x100Du

// Bus scan — write 0x0001 to trigger a full bus scan
#define MBREG_SCAN_TRIGGER      0x1010u

// Configuration registers (persisted to EEPROM)
#define MBREG_RESCAN_INTERVAL   0x1011u  // seconds between periodic bus rescans; 0 = disabled
#define MBREG_SLAVE_ADDR        0x1012u  // Modbus slave address, 1-247
#define MBREG_BAUD_INDEX        0x1013u  // 0=9600 1=19200 2=38400

#define SCAN_IDLE       0
#define SCAN_SCANNING   1
#define SCAN_COMPLETE   2

#define DALI_STATUS_FADE_RUNNING 0x10

// raw_cmd_result high-byte status codes (FC04 0x015D)
#define RAW_RESULT_IDLE    0x0000u  // no command in progress
#define RAW_RESULT_PENDING 0xFF00u  // command sent, awaiting response
#define RAW_RESULT_NAK     0x0100u  // completed, no response (normal for send-twice)
#define RAW_RESULT_OK(b)   (0x0200u | (uint8_t)(b))  // response received
#define RAW_RESULT_ERR(e)  (0x0300u | (uint8_t)(e))  // bus/timing error

// Initialise the register map to defaults and load persisted config from EEPROM.
// Slave address preference: EEPROM > USERROW formula > 1.
void     regs_init(void);

// Copy count FC04 input registers starting at addr into buf (big-endian, 2 bytes each).
// The register_map_t layout mirrors the FC04 address space, so this is a direct memcpy.
// Caller must validate addr and count before calling.
void     regs_read_input_bulk(uint16_t addr, uint8_t count, uint8_t *buf);

// Read one FC03 holding register at addr into *val.
// Returns false for any address not in the holding register map.
bool     regs_read_holding(uint16_t addr, uint16_t *val);

// Write one FC06/FC10 holding register; validates value range for config registers.
// Returns false if addr is unknown or val is out of range.
// Does NOT enqueue DALI commands — dispatch_action_reg in modbus.c does that.
bool     regs_write_holding(uint16_t addr, uint16_t val);

// Store one field from a DALI query response into the appropriate register block.
// Called from ISR context via cmdq_result_received; must be short.
void     regs_apply_result(uint8_t ballast_idx, reg_target_t target, uint8_t value);

// Apply a DALI status byte: updates block0.status, the fading bitmask, and the dirty bit.
void     regs_apply_status(uint8_t ballast_idx, uint8_t dali_status);

// Set or clear the present bit for ballast_idx in the present bitmask.
void     regs_set_present(uint8_t ballast_idx, bool present);

// Reset all register fields for ballast_idx to absent/default and clear its present bit.
void     regs_clear_ballast(uint8_t ballast_idx);

// Atomically snapshot and clear dirty[word_idx] (covers ballasts word_idx*16 to word_idx*16+15).
// Called by modbus FC04 handler to detect changed ballasts since the last read.
uint16_t regs_read_clear_dirty(uint8_t word_idx);

// Recompute group membership for ballast_idx from a QUERY_GROUP response byte.
// high_groups=false → groups 0–7; high_groups=true → groups 8–15.
// Rebuilds all 16 group_status entries after updating the mask.
void     regs_update_group_mask(uint8_t ballast_idx, uint8_t bits, bool high_groups);

// Recompute group_status[group_idx].status by ORing the status of all member ballasts.
void     regs_rebuild_group_status(uint8_t group_idx);

// Return the current periodic rescan interval in seconds (0 = disabled).
uint16_t regs_get_rescan_interval(void);

#endif
