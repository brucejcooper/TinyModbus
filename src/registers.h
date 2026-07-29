#ifndef REGISTERS_H
#define REGISTERS_H

#include <stdint.h>
#include <stdbool.h>
#include "types.h"

// TASK 05 — STATELESS PIPE: there is no register map any more.
//
// block0 (arc/status), present[], dirty[] and group_mask[] are DELETED. They
// were the gateway's cache of the bus, and the whole point of the strip is that
// the gateway caches nothing: the controller interrogates the ballasts and holds
// the only copy. That removes ~672 B of SRAM and makes a gateway reboot
// consequence-free, since there is no longer any state to lose.
//
// Their addresses (0x0000-0x00FF, 0x0100-0x0107, 0x0110-0x014F) now return
// ILLEGAL DATA ADDRESS. A client still reading them is a version mismatch and
// should fail loudly rather than receive plausible zeros.

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
#define MBREG_SLOT_CMD_BASE         0x1000u  // slot 0 command; block spans 0x1000-0x100F
#define MBREG_SLOT_CMD_END          0x100Fu  // 8 slots × 2 registers, inclusive

// Status block (gateway-write; served at the SAME addresses under FC03 holding —
// FC23's read range targets it — and FC04 input): 8 slots × 1 reg.
//   0x1010 + slot = [status:8 | response8:8].  status = lifecycle 0 idle /
//   1 pending / 2 executing / 3 complete / 4 failed.  response8 = the value on
//   complete (backward-frame answer, or 0 for a no-answer command) or the code
//   on failed.  The gateway writes response8 BEFORE status under the read fence,
//   so a fenced read never sees a terminal status with a stale response8.
#define MBREG_SLOT_STATUS_BASE        0x1010u  // slot 0 status; block spans 0x1010-0x1017
#define MBREG_SLOT_STATUS_END         0x1017u  // 8 slots × 1 register, inclusive

// ---- Action / config registers (relocated to 0x1F00+) ---------------------


#define MBREG_SLAVE_ADDR        0x1F02u  // Modbus slave address, 1-247
#define MBREG_BAUD_INDEX        0x1F03u  // 0=9600 1=19200 2=38400



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


// Read one FC03 holding register at addr into *val.
// Returns false for any address not in the holding register map.
bool     regs_read_holding(uint16_t addr, uint16_t *val);

// Write one FC06/FC10 holding register; validates value range for config registers.
// Returns false if addr is unknown or val is out of range.
// Does NOT enqueue DALI commands - dispatch_action_reg in modbus.c does that.
bool     regs_write_holding(uint16_t addr, uint16_t val);







#endif
