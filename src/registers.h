#ifndef REGISTERS_H
#define REGISTERS_H

#include <stdint.h>
#include <stdbool.h>
#include "types.h"

// Paired struct - stored high-byte first to match Modbus wire format.
// block0 can be memcpy'd directly into FC04/FC03 response buffers.
typedef struct { uint8_t status;    uint8_t brightness; } ballast_bs_t;

// register_map_t backs FC03 (holding) and FC04 (input) addressing.  The
// in-memory byte order matches the Modbus wire (big-endian per register).
//
// Config banks retired (task 04 / task-03 Phase D): the per-ballast config
// fields (min/max/power-on/sys-fail/fade) that used to live at 0x0040-0x00FF
// are GONE — the controller now owns config via raw DALI queries (reads) and
// DTR0+SET pairs (writes), so the gateway no longer caches or serves them.
// FC03 reads of 0x0040-0x00FF now return ILLEGAL DATA ADDRESS (see
// regs_read_holding).  The offset-trick (struct offset == addr*2) that the old
// contiguous banks provided is retired with them; the remaining fields are
// addressed by name (see regs_read_input_bulk / regs_read_holding).
//
// COMPATIBILITY / DEPLOYMENT ORDER: a gateway running this firmware requires a
// controller in raw-DALI-config mode (the legacy bank-read/-write path is gone;
// FC03 0x0040-0x00FF returns exception 0x02).  Roll out CONTROLLER-FIRST: flash
// controllers that source config over raw DALI before flashing gateways with
// this build.  A pre-Phase-D controller pointed at this gateway will get
// ILLEGAL DATA ADDRESS on its config full-scan.
//
// FC03/FC06/FC10 holding registers (read-write):
//   0x0000-0x003F  block0[64]        status | brightness   (write low byte = DAPC level)
//   0x0040-0x00FF  (retired)         ILLEGAL DATA ADDRESS — config moved to raw DALI
//   0x0110-0x014F  group_mask[16][8] 64-bit per-group membership bitmask
//
// FC04 input registers (read-only):
//   0x0100-0x0103  present[4]        64-bit ballast-present bitmask
//   0x0104-0x0107  dirty[4]          set on block0 change, cleared on FC04 read
//   0x0108         raw_cmd_result    legacy 0x1000/0x1001 result: high=0x00 ok (low
//                                    byte = response byte; 0x0000 also = no cmd issued yet),
//                                    0x01 nak, 0x02 err (low byte = error subtype),
//                                    0xFF pending (low byte = DALI sends remaining).
//                                    Not cleared on read.  A new RAW_CMD write overwrites
//                                    it (DEVICE_BUSY only when still PENDING).
//   0x0200         version block     16-register special case (see process_fc04)
//   0x0400-0x0409  queue status block (see MBREG_QSTAT_ALIAS / FC04_QSTAT_BASE)
typedef struct {
    ballast_bs_t   block0[64];          // 0x0000  status | brightness (live arc level)
    uint16_t       present[4];          // 0x0100  present/dirty/raw kept contiguous so
    uint16_t       dirty[4];            // 0x0104  the 0x0100-0x0108 FC04 ladder is one
    uint16_t       raw_cmd_result;      // 0x0108  memcpy (see regs_read_input_bulk)
    uint8_t        group_mask[16][8];   // group membership (0x0110-0x014F)
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

// RX diagnostics block, 16 registers, read-only (16-reg read only).  Layout:
//   reg0 rx_overflow   reg1 frame_error   reg2 crc_error   reg3 frames_ok
//   reg4 last_fail_len   reg5..12 last CRC-failed frame bytes (2/reg, big-endian)
//   reg13..15 spare
#define FC04_DIAG_BLOCK     0x0300u

// System-info block, 4 registers, read-only (4-reg read only).  No persistent
// storage is used (no EEPROM wear): uptime is a RAM counter, boot count is
// .noinit SRAM, reset reason is the latched RSTCTRL.RSTFR.  Layout (big-endian):
//   reg0  uptime_s high word   reg1  uptime_s low word   (32-bit seconds since boot)
//   reg2  boot_count           (reboots since the last cold power-on; .noinit SRAM)
//   reg3  reset_reason         (low byte = RSTCTRL.RSTFR flags latched at boot:
//                               bit0 PORF, 1 BORF, 2 EXTRF, 3 WDRF, 4 SWRF, 5 UPDIRF)
// A master detects a gateway reboot when uptime_s drops or boot_count changes.
#define FC04_SYSINFO_BLOCK  0x0310u

// Reboot diagnostics, sourced without EEPROM (defined in main.c).  See the
// FC04_SYSINFO_BLOCK layout above; g_uptime_s lives in dali_sync.{c,h}.
extern volatile uint16_t g_boot_count;
extern volatile uint8_t  g_reset_reason;

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

// ---- Raw command queue: submit window + status block (task 04) ------------
//
// Submit window (master-write-only; FC 0x10 or FC23 write range).  0-4 command
// records; the write LENGTH (2/4/6/8 registers) says how many.  Each record is
// 2 registers:
//   reg0: [addr:8 | data:8]          16-bit DALI forward frame
//   reg1: [id:5 | flags:3 | dtr0:8]  id 1..31 (0 reserved for the internal
//                                    legacy channel), QF_* flags, DTR0 preload
// All-or-nothing: a write asking for more records than free slots is rejected
// whole with SLAVE_DEVICE_BUSY (0x06) — nothing is enqueued.
//
// Window size = the longest atomic DALI sequence the controller submits in one
// window write.  Today that is a DTR0-preload pair (one record).  A future DT8
// colour set (DTR0/DTR1/DTR2 + SET, DALI type-8 device) is still ONE indivisible
// unit and must submit inside one window write; the 4-record window leaves the
// headroom.  See tasks/04-fc23-queue.md.
#define MBREG_QCMD_BASE         0x1100u  // QCMD[0]; window spans 0x1100-0x1107
#define MBREG_QCMD_END          0x1107u  // 4 records × 2 registers, inclusive

// Status block (gateway-write-only): 10 registers.  Lives in FC04 input space
// at 0x0400 AND is aliased into holding space at 0x1180 so FC23's read range
// (holding-only) can target it.  Layout (both mirrors identical):
//   0x0400 / 0x1180        QSTAT   [free_slots:8 | accepted_id:8]
//   0x0401 / 0x1181        QPROG   [executing_id:8 | completed_id:8]
//   0x0402-09 / 0x1182-89  QRES[0..7]  [result:8 | answer:8]   (slot = id & 7)
// `result` reuses the RAW_RESULT_* high-byte encoding (0x00 OK / 0x01 NAK /
// 0x02 ERR); `answer` is the backward-frame byte on OK.  There is no PENDING —
// pending is "completed_id has not yet reached this id".
#define FC04_QSTAT_BASE         0x0400u  // QSTAT..QRES[7] in FC04 input space
#define MBREG_QSTAT_ALIAS       0x1180u  // holding-space alias of 0x0400
#define MBREG_QSTAT_ALIAS_END   0x1189u
#define GWQ_STATUS_REGS         10u      // QSTAT + QPROG + QRES[8]

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
//   Write 0x03YY — re-provision the gear currently at short address YY (0..63)
//                  onto the lowest free addresses (fixes a collision at YY;
//                  leaves all other gear untouched)
//   Write any other value → ILLEGAL_VALUE
//   Write a valid command while already running → DEVICE_BUSY
#define MBREG_COMMISSION        0x1014u

#define MBREG_COMM_REPROV_BASE  0x0300u   // OR with the target short address (0..63)
// True if `v` is a syntactically valid MBREG_COMMISSION write command.  Used by
// the FC06/FC10 handlers to choose ILLEGAL_VALUE vs DEVICE_BUSY on a failed arm.
#define MBREG_COMM_VALID(v) \
    ((v) <= 2u || (((v) & 0xFF00u) == MBREG_COMM_REPROV_BASE && ((v) & 0x00FFu) < 64u))

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
