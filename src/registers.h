#ifndef REGISTERS_H
#define REGISTERS_H

#include <stdint.h>
#include <stdbool.h>
#include "errors.h"
#include "mbq_slots.h"   // MBQ_SLOTS — the queue block sizes derive from it

// ─────────────────────────────────────────────────────────────────────────────
//                            MODBUS REGISTER MAP
//
// The complete, authoritative list. Any address not in this table returns
// ILLEGAL DATA ADDRESS (0x02); any function code not listed returns ILLEGAL
// FUNCTION (0x01). A client reading an unimplemented address is a version
// mismatch and should fail loudly rather than receive plausible zeros.
//
//  ADDRESS        FC              W  ACCESS  CONTENT
//  ─────────────  ──────────────  ─  ──────  ──────────────────────────────────
//  0x1000-0x1003  10, 23(write)   2  W       COMMAND block, MBQ_SLOTS(2) x 2 regs.
//                                            0x1000+2*slot = one 32-bit opaque
//                                            command. Writes cover 1..MBQ_SLOTS
//                                            CONTIGUOUS slots, all-or-nothing:
//                                            any targeted slot ACTIVE rejects
//                                            the whole write DEVICE_BUSY (0x06).
//                                            Write length is in REGISTERS —
//                                            n_slots = wr_count / 2.
//  0x1010-0x1011  03, 04, 23(rd)  1  R       STATUS block, one reg per slot:
//                                            [status:8 | response8:8]. Served at
//                                            the same numbers under holding and
//                                            input, because FC23's read range is
//                                            holding-only. Any 1..MBQ_SLOTS
//                                            contiguous slots may be read.
//  0x0200-0x020F  04             16  R       Version string. Read exactly 16.
//  0x0300-0x030F  04             16  R       RX diagnostics. Read exactly 16.
//  0x0310-0x0313  04              4  R       Sysinfo. Read exactly 4.
//  0x1F02         03, 06          1  R/W     Modbus slave address, 1-247.
//  0x1F03         03, 06          1  R/W     Baud index: 0=9600 1=19200 2=38400.
//  0x1F05         06              1  W       System reset; write 0xA5A5.
//
// Coils (FC01/FC05/FC0F) are not supported and return ILLEGAL FUNCTION.
// ─────────────────────────────────────────────────────────────────────────────

// ---- Read-only blocks ------------------------------------------------------

#define FC04_VERSION_BLOCK  0x0200u  // version string, 16 registers

// RX diagnostics.  reg0 rx_overflow, reg1 frame_error, reg2 crc_error,
// reg3 frames_ok, reg4 last_fail_len, reg5..12 the last CRC-failed frame
// (2 bytes/reg, big-endian), reg13 internal fault count (saturating at 0xFFFF),
// reg14 [tm_err_t | tm_fault_site_t] of the FIRST internal fault, reg15 spare.
// Zeroed on reset.
//
// reg13/reg14 are the errors that could not be returned to a caller — a bad slot
// index in the DALI completion ISR, a refused command launch (see errors.h).
// reg13 != 0 means an internal invariant broke; reg14 says which one.  A healthy
// gateway reads 0 here for its whole uptime.
#define FC04_DIAG_BLOCK     0x0300u

// System info.  No persistent storage is used (no EEPROM wear): uptime is a RAM
// counter, boot_count is .noinit SRAM guarded by a magic word, reset_reason is
// the hardware-latched RSTCTRL.RSTFR.  Big-endian:
//   reg0-1  uptime_s (32-bit, seconds since boot)
//   reg2    boot_count (reboots since the last cold power-on)
//   reg3    reset_reason (low byte = RSTFR: bit0 PORF, 1 BORF, 2 EXTRF,
//                         3 WDRF, 4 SWRF, 5 UPDIRF)
// A master spots a reboot when uptime_s drops or boot_count changes.
#define FC04_SYSINFO_BLOCK  0x0310u

// The uptime / boot-count / reset-reason values behind this block live in
// sysinfo.{c,h} — they are properties of the chip having booted, not of the
// register map.
// dali_sync.{c,h} for the third.

// ---- Command queue (see the map above; protocol in mbq_slots.h) -----------

#define MBREG_SLOT_CMD_BASE         0x1000u  // slot 0 command
// Derived, not hardcoded: the two ends must move together with MBQ_SLOTS, and a
// stale literal here would silently accept writes into unbacked slots.
#define MBREG_SLOT_CMD_END          (MBREG_SLOT_CMD_BASE + 2u * MBQ_SLOTS - 1u)

#define MBREG_SLOT_STATUS_BASE        0x1010u  // slot 0 status
#define MBREG_SLOT_STATUS_END         (MBREG_SLOT_STATUS_BASE + MBQ_SLOTS - 1u)

// ---- Config / action registers --------------------------------------------

#define MBREG_SLAVE_ADDR        0x1F02u  // Modbus slave address, 1-247
#define MBREG_BAUD_INDEX        0x1F03u  // 0=9600 1=19200 2=38400

// System reset.  Write the magic value 0xA5A5 to force an immediate software
// reset of the gateway (RSTCTRL software reset) once the Modbus ACK for this
// write has cleared the wire.  Any other value is rejected (ILLEGAL_ADDRESS).
// A recovery hatch so a Modbus master can power-cycle the gateway logic without
// physical access (used by the controller's `gw_reboot` command).
#define MBREG_SYSTEM_RESET       0x1F05u
#define MBREG_SYSTEM_RESET_MAGIC 0xA5A5u

// No-op: there is no register map to initialise. Kept as a call
// site so the boot sequence reads the same and a future addition has a home.
void     regs_init(void);

// USERROW-backed config.  USERROW survives chip erase + reflash, so units
// keep their identity through firmware updates.  Reads are zero-overhead;
// writes go through NVMCTRL page-erase-write (~3.4 ms, CPU halted during
// the operation) and are skipped if the value already matches.
//
// Layout: byte 0 = slave addr (encoded as addr - 2; 0xFF = address 1),
//         byte 1 = baud index (0/1/2; 0xFF = 9600),
//         bytes 2-3 = unused (written, but carry nothing).
// Reserved Modbus addresses (0, 248-255) are clamped to 1 on read.
// Setters return TM_ERR_BAD_VALUE for an out-of-range argument and
// TM_ERR_HW_FAULT if the NVM write itself did not take — two outcomes a bool
// could not tell apart, and the master needs to: one is "your request was wrong",
// the other is "this gateway's flash is failing".
uint8_t  userrow_get_slave_addr(void);
tm_err_t userrow_set_slave_addr(uint8_t addr);
uint8_t  userrow_get_baud_index(void);
tm_err_t userrow_set_baud_index(uint8_t idx);


// Read one FC03 holding register at addr into *val.
//   TM_OK              *val holds the register
//   TM_ERR_NO_SUCH_REG address is not in the holding map
tm_err_t regs_read_holding(uint16_t addr, uint16_t *val);

// Write one FC06/FC10 holding register; validates the value for config registers.
// Does NOT enqueue DALI commands — dispatch_action_reg in modbus.c does that.
//   TM_OK              written (or already equal — a no-change write is success)
//   TM_ERR_NO_SUCH_REG address is not in the holding map
//   TM_ERR_BAD_VALUE   address is valid but the value is out of range
//   TM_ERR_HW_FAULT    the USERROW/NVM write did not take
// These were one `false` return. The master was told ILLEGAL_ADDRESS — "no such
// register" — for an out-of-range baud index, a bad slave address, a wrong reset
// magic and a failed NVM write alike, which is both wrong and undiagnosable from
// the far end of an RS-485 bus.
tm_err_t regs_write_holding(uint16_t addr, uint16_t val);


#endif
