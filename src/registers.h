#ifndef REGISTERS_H
#define REGISTERS_H

#include <stdint.h>
#include <stdbool.h>
#include "types.h"

// Paired structs - stored high-byte first to match Modbus wire format.
// Blocks can be memcpy'd directly into FC04 response buffers.
typedef struct { uint8_t status;    uint8_t brightness; } ballast_bs_t;
typedef struct { uint8_t min;       uint8_t max;        } ballast_mm_t;
typedef struct { uint8_t power_on;  uint8_t sys_fail;   } ballast_pf_t;
typedef struct { uint8_t fade_rate; uint8_t fade_time;  } ballast_fade_t;

// register_map_t backs both FC03 (holding) and FC04 (input) addressing.  The
// in-memory byte order matches the Modbus wire (big-endian per register), so
// the bulk-read paths can memcpy a contiguous slice straight into the response.
//
// FC03/FC06/FC10 holding registers (read-write):
//   0x0000-0x003F  block0[64]       status | brightness   (write low byte = DAPC level)
//   0x0040-0x007F  block1[64]       min    | max
//   0x0080-0x00BF  block2[64]       power_on | sys_fail
//   0x00C0-0x00FF  block3[64]       fade_rate | fade_time
//   0x0110-0x014F  group_mask[16][8] 64-bit per-group membership bitmask
//
// FC04 input registers (read-only):
//   0x0100-0x0103  present[4]        64-bit ballast-present bitmask
//   0x0104-0x0107  dirty[4]          set on block0 change, cleared on FC04 read
//   0x0108         raw_cmd_result    high=0x00 ok (low byte = response byte; 0x0000 also = no cmd issued yet),
//                                    0x01 nak, 0x02 err (low byte = error subtype),
//                                    0xFF pending (low byte = DALI sends remaining: 0x01 single / 0x02 twice).
//                                    Holds the outcome of the most recent raw command; not cleared on read.
//                                    A new RAW_CMD write overwrites it (DEVICE_BUSY only when still PENDING).
//   0x0109-0x010F  (reserved hole)   keeps group_mask anchored at 0x0110 — read returns zeroes
//   0x0200         version block     16-register special case (see process_fc04)
//
// Layout note: present/dirty/raw_cmd_result sit at 0x0100-0x0108 with a 7-register
// padding hole (_reserved) at 0x0109-0x010F so that group_mask's wire address
// (0x0110) is preserved across this trim.  The 14-byte cost is intentional: it
// avoids a second master-visible breaking change to FC03 group-mask reads.
typedef struct {
    ballast_bs_t   block0[64];          // 0x0000
    ballast_mm_t   block1[64];          // 0x0040
    ballast_pf_t   block2[64];          // 0x0080
    ballast_fade_t block3[64];          // 0x00C0
    uint16_t       present[4];          // 0x0100
    uint16_t       dirty[4];            // 0x0104
    uint16_t       raw_cmd_result;      // 0x0108, see RAW_RESULT_* below
    uint16_t       _reserved[7];        // 0x0109-0x010F (anchor group_mask at 0x0110)
    uint8_t        group_mask[16][8];   // 0x0110-0x014F (FC03 wire address unchanged)
} register_map_t;

extern register_map_t g_regs;

// ---- FC04 input register addresses ----------------------------------------

#define FC04_PRESENT_BASE   0x0100u  // first FC04 input register (start of contiguous ladder)
#define FC04_DIRTY_BASE     0x0104u  // first dirty word register (4 words: 0x0104-0x0107)
#define FC04_DIRTY_END      0x0107u  // last dirty word register
#define FC04_RAW_RESULT     0x0108u  // async raw DALI command result (see RAW_RESULT_*)
#define FC04_INPUT_END      0x0108u  // inclusive end of the contiguous FC04 ladder

// ---- FC04 / FC03 special block addresses ----------------------------------

#define FC04_VERSION_BLOCK  0x0200u  // version string, 16 registers, read-only

// ---- Modbus holding register (FC03/FC06) addresses ------------------------

// Raw DALI forward frame - [15:8]=frame byte 0, [7:0]=frame byte 1
//
// All DALI commands the master might want to issue (GOTO_LAST_ACTIVE,
// OFF, UP, DOWN, STEP_*, RECALL_*, ADD_TO_GROUP, RESET, etc.) go through
// these raw registers — there is no per-command holding-register facade.
// Per-ballast level changes are more easily done via the per-ballast
// brightness bank (0x0000-0x003F) which arms SYNC_WRITE_LEVEL and ACKs
// immediately.
#define MBREG_RAW_CMD           0x1000u  // single send, waits for response
#define MBREG_RAW_CMD_TWICE     0x1001u  // send twice, no response expected

// Bus scan - write 0x0001 to trigger a full bus scan
#define MBREG_SCAN_TRIGGER      0x1010u

// Configuration registers - all persisted to USERROW (survives chip erase +
// reflash so a re-flashed unit keeps its address / baud / rescan interval).
// Each write triggers a NVMCTRL page-erase-write (~3.4 ms, CPU halted) and
// is skipped if the value already matches.  Runtime baud/slave switches are
// deferred until the FC06/FC10 response has cleared the wire.
#define MBREG_RESCAN_INTERVAL   0x1011u  // seconds between periodic bus rescans; 0 = disabled
#define MBREG_SLAVE_ADDR        0x1012u  // Modbus slave address, 1-247
#define MBREG_BAUD_INDEX        0x1013u  // 0=9600 1=19200 2=38400

// IEC 62386-102 commissioning channel.  Write triggers a run, read returns
// the live status word with read-clears semantics on terminal high bytes
// (mirrors the FC04 0x0108 raw-command-result pattern).  See dali_sync.h
// COMM_STATUS_* macros for the wire-format encoding.
//
//   Write 0x0000 — abort (no-op if idle / already terminal)
//   Write 0x0001 — start full commissioning (wipes existing addresses)
//   Write 0x0002 — start incremental commissioning (preserves addressed gear)
//   Write any other value → ILLEGAL_VALUE
//   Write 0x0001/0x0002 while already running → DEVICE_BUSY
#define MBREG_COMMISSION        0x1014u

// System reset.  Write the magic value 0xA5A5 to force an immediate software
// reset of the gateway (RSTCTRL software reset) once the Modbus ACK for this
// write has cleared the wire.  Any other value is rejected (ILLEGAL_ADDRESS).
// A recovery hatch so a Modbus master can power-cycle the gateway logic without
// physical access (used by the controller's `gw_reboot` command).
#define MBREG_SYSTEM_RESET       0x1015u
#define MBREG_SYSTEM_RESET_MAGIC 0xA5A5u

#define DALI_STATUS_FADE_RUNNING 0x10

// raw_cmd_result encoding (FC04 0x0108).  Holds the outcome of the most
// recent raw command, or PENDING while one is in flight.  Not cleared on
// FC04 read; each new RAW_CMD write overwrites whatever was there, so a
// master can re-read the last result as many times as it likes.  The
// initial value (0x0000, set by regs_init's memset) is indistinguishable
// from RAW_RESULT_OK(0); that's intentional — a master that hasn't issued
// any raw commands sees the same value as one whose last command returned
// 0, and the only rejection signal a master cares about is the Modbus
// DEVICE_BUSY exception returned when the channel is currently PENDING.
#define RAW_RESULT_OK(b)        ((uint16_t)(uint8_t)(b))     // 0x00xx — response byte (0x0000 also = no cmd issued yet)
#define RAW_RESULT_NAK          0x0100u                       // completed, no backward frame (normal for send-twice)
#define RAW_RESULT_ERR(e)       (0x0200u | (uint8_t)(e))      // bus/timing error; low byte = error subtype
#define RAW_RESULT_PENDING      0xFF00u                       // command in flight; OR with sends-remaining for the low byte

// Initialise the register map to zeroed defaults.  All persisted config
// (slave addr, baud, rescan interval) lives in USERROW and is read on demand
// via the userrow_* accessors below - no caching in the register map.
void     regs_init(void);

// USERROW-backed config.  USERROW survives chip erase + reflash, so units
// keep their identity through firmware updates.  Reads are zero-overhead;
// writes go through NVMCTRL page-erase-write (~3.4 ms, CPU halted during
// the operation) and are skipped if the value already matches.
//
// Layout: byte 0 = slave addr (encoded as addr - 2; 0xFF = address 1),
//         byte 1 = baud index (0/1/2; 0xFF = 9600),
//         bytes 2-3 = rescan interval big-endian (0xFFFF = 64 s).
// Reserved Modbus addresses (0, 248-255) are clamped to 1 on read.
uint8_t  userrow_get_slave_addr(void);
bool     userrow_set_slave_addr(uint8_t addr);    // false on out-of-range
uint8_t  userrow_get_baud_index(void);
bool     userrow_set_baud_index(uint8_t idx);     // false on out-of-range
uint16_t userrow_get_rescan_interval(void);
bool     userrow_set_rescan_interval(uint16_t v);

// Copy count FC04 input registers starting at addr into buf (big-endian, 2 bytes each).
// The register_map_t layout mirrors the FC04 address space, so this is a direct memcpy.
// Caller must validate addr and count before calling.
void     regs_read_input_bulk(uint16_t addr, uint8_t count, uint8_t *buf);

// Read one FC03 holding register at addr into *val.
// Returns false for any address not in the holding register map.
bool     regs_read_holding(uint16_t addr, uint16_t *val);

// Write one FC06/FC10 holding register; validates value range for config registers.
// Returns false if addr is unknown or val is out of range.
// Does NOT enqueue DALI commands - dispatch_action_reg in modbus.c does that.
bool     regs_write_holding(uint16_t addr, uint16_t val);

// Store one field from a DALI query response into the appropriate register block.
// Called from ISR context via dali_result_received; must be short.
void     regs_apply_result(uint8_t ballast_idx, reg_target_t target, uint8_t value);

// Apply a DALI status byte: updates block0.status and sets the dirty bit.
// The FADE_RUNNING bit is observable directly via the status byte (bit 4) — there
// is no separate fading bitmap to maintain.
void     regs_apply_status(uint8_t ballast_idx, uint8_t dali_status);

// Set or clear the present bit for ballast_idx in the present bitmask.
void     regs_set_present(uint8_t ballast_idx, bool present);

// Reset all register fields for ballast_idx to absent/default and clear its present bit.
void     regs_clear_ballast(uint8_t ballast_idx);

// Atomically snapshot and clear dirty[word_idx] (covers ballasts word_idx*16 to word_idx*16+15).
// Called by modbus FC04 handler to detect changed ballasts since the last read.
uint16_t regs_read_clear_dirty(uint8_t word_idx);

// Recompute group membership for ballast_idx from a QUERY_GROUP response byte.
// high_groups=false → groups 0-7; high_groups=true → groups 8-15.
void     regs_update_group_mask(uint8_t ballast_idx, uint8_t bits, bool high_groups);

#endif
