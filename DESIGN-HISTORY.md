# TinyModbus — design history

Everything the gateway **used to be**, kept here so the source and README can
describe only what it **is**.

Nothing in this file is current. It exists so that a surprising design choice
can be traced back to the thing it replaced, and so an old client or an old
capture can still be interpreted. If it contradicts the code, the code wins.

---

## The gateway was once the brain (before task 05)

`dali_sync.c` was ~1100 lines: a work selector, a per-ballast cache, presence
and dirty bitmaps, group masks, a rescan cursor and a commissioning state
machine. The controller asked it for state; it answered from its own picture of
the bus.

That picture was the problem. A gateway reboot lost it, and everything derived
from it — most visibly group membership, which is what made a group toggle
behave as though every light were already off.

**Now:** the gateway is a stateless pipe. The controller composes complete DALI
frames and submits them as opaque queue commands; the gateway shifts bytes onto
the wire without interpreting them and holds nothing a reboot could lose.

### What went with it

| Removed | Where it lives now |
|---|---|
| Per-ballast arc/status cache | Controller (`dali_state.c`), read from the gear |
| Presence / dirty bitmaps | Controller |
| Group membership masks + ADD/REMOVE fan-out | Controller issues the frames itself |
| Commissioning state machine (`OP_COMMISSION`) | Controller (`dali_commission.c`) |
| Rescan cursor | Controller's sweep |
| Boot scan | Controller's job |
| ~30 DALI command opcodes, address macros, commissioning specials | Controller composes frames |

Only `DALI_ADDR_SET_DTR0` survives, because `QF_DTR0` makes the preload a
gateway-side two-step inside one slot lifecycle.

## Removed register ranges

All return ILLEGAL DATA ADDRESS. A client still reading them is a version
mismatch and should fail loudly rather than receive plausible zeros.

| Range | Was |
|---|---|
| `0x0000-0x003F` | per-ballast status \| brightness |
| `0x0040-0x00FF` | per-ballast config banks (retired task 04) |
| `0x0100-0x0103` | present bitmask |
| `0x0104-0x0107` | dirty bitmask |
| `0x0110-0x014F` | group membership masks |
| `0x1F00` | SCAN_TRIGGER |
| `0x1F01` | RESCAN_INTERVAL |
| `0x1F04` | COMMISSION trigger/status |

USERROW bytes 2-3 were the rescan interval; still written so the layout is
unchanged.

## The v1 command queue

Replaced by the self-describing slot queue (task 06). v1 inferred a command's
fate from a running id ledger — accept/complete counters and a hashed result
ring — that had to be *reconstructed* after any disruption. Every
recovery-corruption bug was that reconstruction being subtly wrong.

v2 removes the class by construction: each slot reports its own state, so the
master reads a command's fate rather than inferring it. No ledger, no id
hashing, no resync/rebuild/rollback. `0x1018-0x1019` (v1 `QRES[6/7]`) are gone.

`cmd_entry_t` was v1's queue entry; the name outlived the queue and made the
step struct look like a duplicate of a slot.

## Representations that were duplicated

- **Two enums for one thing.** `cmd_type_t` (`CMD_SEND`, `CMD_SEND_TWICE`,
  `CMD_QUERY`, `CMD_DIRECT_ARC`) and `dali_kind_t` classified every send twice
  under two sets of names, with a one-for-one conversion between them.
  `CMD_DIRECT_ARC`/`DALI_KIND_DAPC` had no producer and no distinct behaviour.
- **A parallel copy of the command.** A per-send struct held `{kind, addr, data,
  repeat}`, all derivable from the 4-byte command plus the step. `repeat` could
  contradict `kind`.
- **`dali_current_op_t`.** Its `op` field was set and cleared in exact lockstep
  with the busy flag; `addr` was written `0xFF` and never read.
- **Four parallel slot arrays**, later one `mbq_slot_t` — the arrays existed to
  avoid a multiply on a 7-byte stride; padding to 8 buys the same indexing.
- **Three copies of the provisioning state names** (CLI, MQTT bridge, and none
  where the web UI needed one).

## Naming that described the wrong thing

- `dali_idle` / `dali_busy` read as inverses. They are different layers — the
  wire, and slot ownership — and both can be false at once. Now one
  `dali_chan`-style state, then `dali_bus_busy` plus
  `dali_sync_owns_channel()`.
- `reg_target_t` described a settings-scan path stripped in task 05.
- `dali_init()` advertised starting the "RTC PIT for periodic bus rescan"; the
  rescan became the controller's job and the PIT now only drives uptime
  (`sysinfo.c`).
- `g_uptime_s` lived in `dali_sync` because the PIT once also drove a DALI
  rescan cursor; `g_boot_count`/`g_reset_reason` were defined in `main.c` but
  declared in `registers.h`.

## Error handling before `tm_err_t`

Functions returned `bool`, or `void` with a silent early return.
`regs_write_holding` collapsed five distinct causes into one `false`, so a
master writing `baud=5` was told ILLEGAL_ADDRESS — "no such register".

`mbq_begin` returned without writing `*out`, so a caller ignoring the failure
re-fired the previous DALI frame. `mbq_submit` reported a malformed range as
DEVICE_BUSY, inviting infinite retries of something that could never succeed.

FC04 had a generic tail that set a byte count and transmitted **without filling
the payload** — dead only because its allow-list returned false unconditionally.

## Fences that the write order already covered

`mbq_slots.c` used `ATOMIC_BLOCK` in `mbq_submit` and `mbq_read_status`. Both
were unnecessary: the only concurrent writer is the completion ISR, and writing
`response8` before `status` already guarantees a reader never sees a terminal
status with a stale value. A global `cli` there blocks the DALI bit-timing
callbacks and the USART RX ISR — the shape of the 2026-06-05 RX starvation bug.

---

## Behavioural changes vs the pre-refactor firmware

The flag-driven refactor (branch `task-1-flag-driven`) introduces several wire-protocol-visible changes. Masters built against earlier firmware revisions need the following adjustments:

- **Writes ACK at the Modbus wire layer, not at DALI completion.** Per-ballast bank writes (0x0000–0x00FF), group-mask writes (0x0110–0x014F), and raw command writes (0x1000 / 0x1001) all return their normal echo-style ACK as soon as the cache is updated and the work is armed. The DALI command that pushes the change to the ballast is dispatched asynchronously by the work selector. Masters that previously took the write-ACK as "ballast confirmed" must switch to **dirty-bit polling** (FC04 0x0104–0x0107, read-clears) to detect when the change has propagated.
- **Per-ballast cache addresses moved from FC04 (input regs) to FC03 (holding regs).** Addresses 0x0000–0x00FF (status/brightness, min/max, power-on/sys-fail, fade rate/time) and 0x0110–0x014F (group masks) are now read via FC03 and written via FC06/FC10. Reading them via FC04 returns `ILLEGAL_ADDRESS`.
- **Coil function codes (FC01 / FC05 / FC0F) removed.** All three return `ILLEGAL_FUNCTION`. Masters that used FC05 / FC0F to drive ON/OFF should write the brightness register low byte (0 = off, ≥1 = on / proportional). FC01 (read coils) likewise has no replacement; read the brightness bank via FC03 instead.
- **Action holding registers 0x1002–0x100A removed.** DAPC at 0x1002, GOTO_LAST_ACTIVE at 0x1003, OFF/UP/DOWN/STEP_UP/STEP_DOWN/RECALL_MAX/RECALL_MIN at 0x1004–0x100A all return `ILLEGAL_ADDRESS`. DAPC is now done via the brightness register (0x0000–0x003F); the rest are issued via the raw command channel (0x1000 / 0x1001).
- **`RESCAN_INTERVAL` is retired (task 05).** The gateway does no polling of its own; the background sweep is the controller's, tunable there as `dali_sweep_interval_ms`. Its USERROW bytes are left unused.
- **Async raw command pending state uses `0xFFnn` with non-zero low byte.** The 0x0108 pending encoding is `0xFF02` (after writing 0x1001 RAW_CMD_TWICE) or `0xFF01` (after writing 0x1000 RAW_CMD); the low byte does NOT decrement mid-twice-send (the firmware transitions directly from `0xFF02` to the terminal value once both sends complete). Masters that strict-equal `value == 0xFF00` for "still pending" will miss both initial arming states; match the high byte instead (`(value & 0xFF00) == 0xFF00`).
- **DTR-setter holding registers removed (subtask 9).** `SET_DTR0` (0x100B), `SET_DTR1` (0x100C), and `SET_DTR2` (0x100D) all return `ILLEGAL_ADDRESS`. They were thin wrappers around raw DALI sends; issue the equivalent special command via the raw channel instead — write `0xA3VV` to 0x1000 to load DTR0 with `VV`, `0xC3VV` for DTR1, `0xC5VV` for DTR2 (single send, no backward frame expected; the result register at FC04 0x0108 reaches `NAK` once the send completes). Config writes (min/max, power-on/sys-fail, fade) are no longer done on the gateway at all — the controller issues them as raw DTR0+SET pairs through the command queue (the `send-DTR0-first` record flag makes each pair one atomic queue entry).
- **FC04 input register layout cleaned up (subtask 8).** Present, dirty, and raw-command-result moved down to a contiguous block at 0x0100–0x0108: present[4] at 0x0100–0x0103, dirty[4] at 0x0104–0x0107, raw_cmd_result at 0x0108. The fading bitmap (formerly 0x0155–0x0158), group_status array (formerly 0x0100–0x010F), and scan_status (formerly 0x0150) are removed entirely — they were derivable from the per-ballast status byte / group_mask, or vestigial from the pre-flag-driven era. Masters that read presence/dirty/raw_result must update their addresses; the master-visible group_mask range at FC03 0x0110–0x014F is intentionally preserved by a 14-byte padding hole inside `register_map_t`. Fade-running detection switches from the removed bitmap to bit 4 of the per-ballast status byte at FC03 0x0000–0x003F (high byte).
- **Raw command result high-byte encoding revamped and FC04 read-clears removed.** OK was `0x02xx`; it is now `0x00xx` (the register value is just the response byte, with `0x0000` doing double duty as "no command issued yet"). ERR was `0x03xx`; it is now `0x02xx` with the same error-subtype low byte. PENDING (`0xFFxx`) and NAK (`0x0100`) are unchanged. Reads of FC04 0x0108 no longer clear the register — the outcome of the last raw command persists until the next RAW_CMD write overwrites it. The arm gate now only checks PENDING: any other state is silently overwritten on the next RAW_CMD write (so masters that want to preserve a prior outcome must read it before re-arming).
- **Config banks (0x0040–0x00FF) retired (task 04).** The per-ballast min/max, power-on/sys-fail, and fade banks no longer exist; FC03 reads and FC06/FC10 writes there return `ILLEGAL DATA ADDRESS`. Config is owned by the controller over raw DALI. A gateway on this firmware requires a raw-config controller — see "Config-bank retirement". Deploy controller-first.
- **Command queue + register-map relocation (task 04) — supersedes the several raw-channel bullets above.** The legacy single-slot raw channel (0x1000/0x1001 RAW_CMD + FC04 0x0108 result) is **removed**; all raw DALI now goes through the id-tagged command queue. The register map moved to a round 0x1000 base: submit window 0x1000–0x1007 (FC10/FC23 write range), status block 0x1010–0x1019 (served under both FC03 and FC04), **FC23 (0x17)** support; and the action/config registers relocated to 0x1F00+ (scan 0x1F00, rescan 0x1F01, slave 0x1F02, baud 0x1F03, commission 0x1F04, reset 0x1F05). Every pre-queue raw/config-bank address returns ILLEGAL DATA ADDRESS. This gateway and the controller are now a **matched firmware set** — no mixed-version operation; deploy both together. See "Raw Command Queue".
- **Command queue v2 — self-describing 8-slot lifecycle (task 06) — supersedes the task-04 id-tagged FIFO above.** The command queue is now DALI-agnostic (opaque 32-bit command in, `[status:8 | response8:8]` out) with DALI as a payload adapter on top. The register map at the same base is **incompatible** with v1: the command block is 0x1000–0x100F (8 slots × 2 regs, `0x1000 + 2*slot`, all-or-nothing reject-active write) and the status block is 0x1010–0x1017 (one `[status | response8]` register per slot; `0x1018–0x1019` now ILLEGAL). The controller-assigned id, the result ring, `accepted_id`/`completed_id`, and the boot-count resync are all gone — the slot statuses are the truth and a gateway reboot is detected as "idle means gone". This is a **fresh matched-set break** (queue v2 controller ↔ queue v2 gateway); a task-04 (v1) master cannot drive this firmware and vice versa. See "Raw Command Queue".

## Protocol break: queue v2 vs the id-tagged FIFO

**Changelog / matched firmware set.** Queue v2 (task 06) is a **protocol break** from the task-04 id-tagged FIFO: the command block at 0x1000 and the status block at 0x1010 have incompatible semantics (self-describing slots vs. an id ledger + result ring). The controller (`switch_gateway_firmware`) and this gateway firmware are a **matched set** — a v1 master cannot drive a v2 gateway and vice versa, and there is no mixed-version operation. **Deploy both together.** (The earlier task-04 break — the removal of the pre-queue single-slot raw channel at 0x1000/0x1001 + FC04 0x0108 — still stands; those legacy addresses return ILLEGAL DATA ADDRESS.)

---

## The v2 slot queue (task 06), and the FC23 transport under it (task 04)

Replaced by **queue v4** (task 08). `QUEUE-PROTOCOL.md` is the current
specification; everything below is here so an old capture can still be read.

**What v2 was.** Eight — later three — slots, each self-describing its own
lifecycle. The master wrote a command into a slot it *named*, then read the
whole status block back: one register per slot, `[status:8 | response8:8]`,
with `status` in `0 idle / 1 pending / 2 executing / 3 complete / 4 failed`.
Terminals were sticky and were freed by being overwritten. FC23 carried it —
write range the command block at `0x1000–0x100F`, read range the slot status
block at `0x1010–0x1017` — so one transaction both submitted and collected.
A gateway reboot cleared every slot to idle, which the master read as the
**"idle means gone"** signal.

**Why it went.** FC23 forces the master to *name* the register it writes, so
it may only submit into a slot it already knows is free — and it learns that
from a read. Every DALI frame therefore cost a poll that carried no work, then
a submit. Measured on the bench: **1.73 Modbus transactions per DALI command
and 99 % RS-485 occupancy for a single saturated gateway**, which left no room
for a second DALI bus. v4 lets the *gateway* choose the slot and retains
results until acked, which collapses that to **1.03 transactions and 37 %**.

**Retired with it** (all now `ILLEGAL DATA ADDRESS`, and FC23 itself
`ILLEGAL FUNCTION`):

| Range / code | Was |
|---|---|
| `0x1000–0x100F` | command block, 2 registers per slot |
| `0x1010–0x1017` | slot status block, `[status \| response8]` per slot |
| FC23 (`0x17`) | the read/write-multiple transport for both blocks |

**Also gone: a draft FC65/FC66 pair** that briefly occupied `0x41`/`0x42`.
It let the gateway pick the slot and returned an `accepted_slot` byte with a
snapshot taken *before* the enqueue. It was never driven by a shipped
controller. v4 reuses `0x41` with entirely different semantics, so a capture
showing `0x41` must be dated before it can be decoded.

**And `controller/docs/modbus-command-queue.md`**, a standalone specification
of v2's generic slot pattern, deleted in task 08. Its subject no longer exists;
`QUEUE-PROTOCOL.md` supersedes it and is normative for both halves.

**What survived unchanged into v4:** the queue is still DALI-agnostic (an
opaque 4-byte command in, a status + answer byte out, with DALI as a payload
adapter on top), and the **publication ordering** rule — write the payload
bytes before the byte that publishes them, read in the mirror order — which is
the one invariant carried forward verbatim, because it is what lets the
completion ISR and the Modbus reader share state with no interrupt fences.
