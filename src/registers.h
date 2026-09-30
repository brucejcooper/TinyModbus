#ifndef REGISTERS_H
#define REGISTERS_H

#include <stdint.h>
#include <stdbool.h>
#include "errors.h"

// ─────────────────────────────────────────────────────────────────────────────
//                            MODBUS REGISTER MAP
//
// The complete, authoritative list. Any address not in this table returns
// ILLEGAL DATA ADDRESS (0x02); any function code not listed returns ILLEGAL
// FUNCTION (0x01). A client reading an unimplemented address is a version
// mismatch and should fail loudly rather than receive plausible zeros.
//
// THE COMMAND QUEUE IS NOT IN THIS MAP. Under v4 it is reached only through
// FC 0x41 (see QUEUE-PROTOCOL.md and modbus.c), which carries the command and
// the ack as a payload rather than as register writes. The v2 submit window
// (0x1000-0x1003) and slot status block (0x1010-0x1011) are RETIRED and now
// return ILLEGAL DATA ADDRESS, in line with every previous retirement — see
// DESIGN-HISTORY.md.
//
//  ADDRESS        FC              W  ACCESS  CONTENT
//  ─────────────  ──────────────  ─  ──────  ──────────────────────────────────
//  0x0200-0x020F  04             16  R       Version string. Read exactly 16.
//  0x0210-0x021F  04             16  R       Build string (git describe). Read exactly 16.
//  0x0300-0x030F  04             16  R       RX diagnostics. Read exactly 16.
//  0x0310-0x0313  04              4  R       Sysinfo. Read exactly 4.
//  0x0320-0x0327  04              8  R       Queue counters. Read exactly 8.
//  0x1F02         03, 06          1  R/W     Modbus slave address, 1-247.
//  0x1F03         03, 06          1  R/W     Baud index: 0=9600 1=19200 2=38400.
//  0x1F05         06              1  W       System reset; write 0xA5A5.
//
// Every FC04 block read is EXACT-LENGTH: the counts above are the only ones
// accepted, and any other count is ILLEGAL DATA ADDRESS rather than a short
// read, so a client built against a different map fails loudly.
//
// Coils (FC01/FC05/FC0F) are not supported and return ILLEGAL FUNCTION.
// ─────────────────────────────────────────────────────────────────────────────

// ---- Read-only blocks ------------------------------------------------------

#define FC04_VERSION_BLOCK  0x0200u  // version string, 16 registers
// Build string: the `git describe --long --dirty --tags` the image was built
// from (Makefile FW_VERSION), ASCII, NUL-padded to 32 bytes (truncated if
// longer). The queue generation in FC04_VERSION_BLOCK says which PROTOCOL a
// gateway speaks; this says which BUILD it runs, so a fielded gateway can be
// identified without reading its flash back over UPDI. "-dirty" means it was
// built from an uncommitted tree.
#define FC04_BUILD_BLOCK    0x0210u  // build string, 16 registers

// RX diagnostics.  reg0 rx_overflow, reg1 frame_error, reg2 crc_error,
// reg3 frames_ok, reg4 last_fail_len, reg5..12 the last CRC-failed frame
// (2 bytes/reg, big-endian), reg13 internal fault count (saturating at 0xFFFF),
// reg14 [tm_err_t | tm_fault_site_t] of the FIRST internal fault, reg15 spare.
// Zeroed on reset.
//
// reg4 is CAPPED at the reg5..12 capture size (16 bytes): a CRC-failed frame
// longer than that (up to MAX_PKT_SZ) still only fills 16 bytes of buf, so
// reg4 never claims more frame bytes than reg5..12 actually holds.
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

// Queue counters (QUEUE-PROTOCOL.md §6).  Read EXACTLY 8 registers, big-endian,
// 16-bit WRAPPING counters.  These are the whole cold-path observability story
// for the queue: nothing rides the FC 0x41 reply beyond the protocol's own
// bytes, because hot-path bytes are the scarce resource.  Read on demand from
// the CLI or the bench, never polled hot.
//
//   reg0  0x0320  accepts        commands enqueued (A_ACCEPTED replies)
//   reg1  0x0321  a_full         enqueues refused, queue full (A_FULL replies).
//                                NON-ZERO IS NORMAL — it is the protocol's flow
//                                control, not a canary.  A_FULL means both
//                                command slots are busy, i.e. the DALI engine
//                                is fully fed (the goal state); the master
//                                re-offers immediately rather than pacing
//                                against an estimate, so under one saturated
//                                gateway ~72% of transactions are refusals, and
//                                that refusal share is the HEADROOM gauge
//                                (QUEUE-PROTOCOL.md §5) — it converts into
//                                useful enqueues as gateways are added.  The
//                                one thing that IS still a fault is the
//                                invariant-break branch in cq_accept() (cq.c),
//                                which deliberately does NOT increment this
//                                counter — see cq.c for why
//   reg2  0x0322  executes       commands handed to the DALI adapter
//   reg3  0x0323  results        results pushed onto the response queue
//   reg4  0x0324  acks           response entries popped by a matching ack
//   reg5  0x0325  stalls         arbiter refusals caused by a FULL response
//                                queue.  Unlike a_full, this one IS reachable in
//                                normal operation — it is the backpressure rule
//                                doing its job whenever the master stops acking,
//                                so a non-zero value is not alarming by itself.
//                                It is the counter the backpressure validation
//                                gate reads to prove the rule engaged
//   reg6  0x0326  q_discard      FC 0x41 frames dropped with no reply for a bad
//                                length or a length/flag inconsistency.  Held
//                                here rather than in the RX diagnostic block
//                                because it is a protocol fact, not a line fact
//   reg7  0x0327  spare          reads 0
//
// Steady-state sanity: accepts == executes == results == acks, each lagging the
// one before it by at most the queue depths.  This layout is duplicated in the
// controller's `mbq` stats parser and in CLAUDE.md — all three must move
// together.
#define FC04_QDIAG_BLOCK    0x0320u

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
// keep their identity through firmware updates.  Reads are zero-overhead
// (they see a staged-but-uncommitted write immediately, so a read-back right
// after a set() is never stale).  Writes are skipped if the value already
// matches, and otherwise only STAGE the new byte in SRAM — the setter itself
// never touches flash and never halts.  The actual NVMCTRL page-erase-write
// (~3.4 ms, CPU halted for the duration) happens later, from userrow_service(),
// which the main loop calls only when the DALI engine is confirmed idle. See
// registers.c for why deferring it this way matters (a synchronous commit
// could halt the CPU mid-DALI-frame) and why the timing is race-free.
//
// Layout: byte 0 = slave addr (encoded as addr - 2; 0xFF = address 1),
//         byte 1 = baud index (0/1/2; 0xFF = 9600),
//         bytes 2-3 = unused (written, but carry nothing).
// Reserved Modbus addresses (0, 248-255) are clamped to 1 on read.
// Setters return TM_ERR_BAD_VALUE for an out-of-range argument and
// TM_ERR_HW_FAULT if the byte could not even be staged — two outcomes a bool
// could not tell apart, and the master needs to: one is "your request was wrong",
// the other is "this gateway's flash is failing".
uint8_t  userrow_get_slave_addr(void);
tm_err_t userrow_set_slave_addr(uint8_t addr);
uint8_t  userrow_get_baud_index(void);
tm_err_t userrow_set_baud_index(uint8_t idx);

// Commit any staged USERROW writes (see the setters above) to flash. Call
// from the main loop only when start_next_dali_work() (dali_sync.h) reports
// nothing outstanding — no command executing, none queued, bus not settling.
// No-op when nothing is staged. See registers.c's userrow_service() for the
// full race analysis against the DALI completion ISR.
void     userrow_service(void);


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
