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
//   0x0200         version block     16-register special case (see process_fc04)
//   0x1010-0x1019  queue status block (see MBREG_QSTAT_BASE; also served under FC03)
typedef struct {
    ballast_bs_t   block0[64];          // 0x0000  status | brightness (live arc level)
    uint16_t       present[4];          // 0x0100  present/dirty kept contiguous so the
    uint16_t       dirty[4];            // 0x0104  0x0100-0x0107 FC04 ladder is one memcpy
    uint8_t        group_mask[16][8];   // group membership (0x0110-0x014F)
} register_map_t;

extern register_map_t g_regs;

// ---- FC04 input register addresses ----------------------------------------

#define FC04_PRESENT_BASE   0x0100u  // first FC04 input register (start of contiguous ladder)
#define FC04_DIRTY_BASE     0x0104u  // first dirty word register (4 words: 0x0104-0x0107)
#define FC04_DIRTY_END      0x0107u  // last dirty word register
#define FC04_INPUT_END      0x0107u  // inclusive end of the contiguous FC04 ladder

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

// ---- Modbus register map (task 04, relocated to a round 0x1000 base) --------
//
// One number space for the queue, addressed the same under FC03 (holding) and
// FC04 (input).  The legacy single-slot raw channel (old 0x1000/0x1001 +
// FC04 0x0108) is REMOVED — those addresses are reused by the submit window,
// and every other pre-queue address (config banks, old raw result) returns
// ILLEGAL DATA ADDRESS.
//
// ---- Command queue v2: slot command block + slot status block --------------
//
// A self-describing 8-slot lifecycle queue (task 06).  See mbq_slots.h and
// controller/docs/modbus-command-queue.md.  The v1 id-ledger (QSTAT
// [free_slots|accepted_id], QPROG [executing_id|completed_id], QRES[8] ring) is
// GONE — and with it, 0x1018-0x1019 are now ILLEGAL DATA ADDRESS.
//
// Command block (master-write; FC 0x10 or FC23 write range): 8 slots × 2 regs.
//   0x1000 + 2*slot = a 32-bit OPAQUE command (2 regs).  The queue does not
//   interpret it; the DALI adapter packs/unpacks (dali_sync.h QF_* contract).
// A write covers 1..8 contiguous slots and is ALL-OR-NOTHING: if ANY targeted
// slot is ACTIVE (pending/executing) the whole write is rejected with
// SLAVE_DEVICE_BUSY (0x06).  idle + collected-terminal slots are writable.
#define MBREG_QCMD_BASE         0x1000u  // slot 0 command; block spans 0x1000-0x100F
#define MBREG_QCMD_END          0x100Fu  // 8 slots × 2 registers, inclusive

// Status block (gateway-write; served at the SAME addresses under FC03 holding —
// FC23's read range targets it — and FC04 input): 8 slots × 1 reg.
//   0x1010 + slot = [status:8 | response8:8].  status = lifecycle 0 idle /
//   1 pending / 2 executing / 3 complete / 4 failed.  response8 = the value on
//   complete (backward-frame answer, or 0 for a no-answer command) or the code
//   on failed.  The gateway writes response8 BEFORE status under the read fence,
//   so a fenced read never sees a terminal status with a stale response8.
#define MBREG_QSTAT_BASE        0x1010u  // slot 0 status; block spans 0x1010-0x1017
#define MBREG_QSTAT_END         0x1017u  // 8 slots × 1 register, inclusive

// ---- Action / config registers (relocated to 0x1F00+) ---------------------

// Bus scan - write 0x0001 to trigger a full bus scan
#define MBREG_SCAN_TRIGGER      0x1F00u

// Configuration registers - all persisted to USERROW (survives chip erase +
// reflash so a re-flashed unit keeps its address / baud / rescan interval).
// Each write triggers a NVMCTRL page-erase-write (~3.4 ms, CPU halted) and
// is skipped if the value already matches.  Runtime baud/slave switches are
// deferred until the FC06/FC10 response has cleared the wire.
#define MBREG_RESCAN_INTERVAL   0x1F01u  // seconds between periodic bus rescans; 0 = disabled
#define MBREG_SLAVE_ADDR        0x1F02u  // Modbus slave address, 1-247
#define MBREG_BAUD_INDEX        0x1F03u  // 0=9600 1=19200 2=38400

// IEC 62386-102 commissioning channel.  Write triggers a run, read returns
// the live status word with read-clears semantics on terminal high bytes.
// See dali_sync.h COMM_STATUS_* macros for the wire-format encoding.
//
//   Write 0x0000 — abort (no-op if idle / already terminal)
//   Write 0x0001 — start full commissioning (wipes existing addresses)
//   Write 0x0002 — start incremental commissioning (preserves addressed gear)
//   Write 0x03YY — re-provision the gear currently at short address YY (0..63)
//                  onto the lowest free addresses (fixes a collision at YY;
//                  leaves all other gear untouched)
//   Write any other value → ILLEGAL_VALUE
//   Write a valid command while already running → DEVICE_BUSY
#define MBREG_COMMISSION        0x1F04u

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
#define MBREG_SYSTEM_RESET       0x1F05u
#define MBREG_SYSTEM_RESET_MAGIC 0xA5A5u

#define DALI_STATUS_FADE_RUNNING 0x10

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
