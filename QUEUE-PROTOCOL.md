# DALI command queue — Modbus contract

Authoritative description of how the controller (RP2040 master) drives the
TinyModbus gateway's DALI command queue.

Two transports carry the same queue:

| | function code | status |
|---|---|---|
| **FC23** Read/Write Multiple Registers | `0x17` | standard, always supported |
| **FC65** Enqueue + status | `0x41` | user-defined, optional |
| **FC66** Status only | `0x42` | user-defined, optional |

FC23 is the compatibility path and is never removed. FC65/FC66 exist because
FC23 costs an extra round trip per DALI frame — see [Why FC65 exists](#why-fc65-exists).

**Both paths are ordinary Modbus RTU.** Same addressing, same CRC-16, same t3.5
framing, same exception replies. MODBUS Application Protocol V1.1b reserves
function codes 65–72 and 100–110 for user-defined use, so FC65/FC66 are
conformant rather than an extension of the standard set. Neither path skips
silence, dispatches early, or shortcuts the state machine.

---

## 1. The queue

`MBQ_SLOTS` slots (currently **3**), each holding one opaque 4-byte command and
a 2-byte status. The gateway executes **one DALI frame at a time**; the slots
exist to keep it fed, not to parallelise it.

Slot lifecycle:

```
IDLE ──enqueue──► PENDING ──arbiter picks──► EXECUTING ──DALI answers──► COMPLETE
                                                                     └──► FAILED
```

`COMPLETE`/`FAILED` are terminal and **sticky** — the slot holds its result
until it is reused. There is no explicit "collect" or "ack" step; the master
learns the result by reading status, and the slot is freed by being overwritten.

### Slot status register

One 16-bit register per slot, big-endian on the wire:

```
 bit  15                     8 7                      0
     ┌─────────────────────────┬─────────────────────────┐
     │        status           │       response8         │
     └─────────────────────────┴─────────────────────────┘
```

| `status` | value | meaning |
|---|---|---|
| `MBQ_S_IDLE` | `0` | free, writable |
| `MBQ_S_PENDING` | `1` | accepted, waiting for the DALI engine |
| `MBQ_S_EXECUTING` | `2` | DALI frame in flight |
| `MBQ_S_COMPLETE` | `3` | terminal — `response8` is the answer byte |
| `MBQ_S_FAILED` | `4` | terminal — `response8` is a failure code |

`response8` is `0` unless the status is terminal.

| failure code | value | meaning |
|---|---|---|
| `DALI_RESP_NAK` | `0x01` | no (or too short) backward frame where one was expected |
| `DALI_RESP_COLLISION` | `0x02` | garbled backward frame — two or more gear answered |
| `DALI_RESP_BUSFAULT` | `0x03` | transport fault: short pulse, stuck bus |

> **Ordering guarantee.** `mbq_complete()` writes `response8` **before** `status`;
> `mbq_read_status()` reads `status` **before** `response8`. That pairing is what
> makes a terminal status imply a `response8` that is already correct. Reversing
> either side reintroduces a stale-value race. This is load-bearing, not style.

### Command payload (4 bytes, opaque to the queue)

The queue layer has no knowledge of DALI; these bytes are interpreted by the
adapter in `dali_sync.c`.

| byte | field | meaning |
|---|---|---|
| 0 | `addr` | DALI address byte — short address, group, broadcast (`0xFE`), or a special opcode (e.g. `0xA3` SET DTR0) |
| 1 | `data` | DALI data byte |
| 2 | `flags` | see below |
| 3 | `dtr0` | value preloaded into DTR0 when `QF_DTR0` is set |

| flag | value | effect |
|---|---|---|
| `QF_TWICE` | `0x01` | send the forward frame twice (SET-class); no answer expected |
| `QF_DTR0` | `0x02` | emit `0xA3 <dtr0>` back-to-back **before** the frame |
| `QF_ANSWER` | `0x04` | a backward frame is expected; capture the answer byte |

---

## 2. Packet formats

CRC-16 (Modbus polynomial) over all preceding bytes, **low byte transmitted
first**, as standard.

### FC65 — enqueue one command, return status · request, 8 bytes

```
┌─────┬──────┬──────┬──────┬───────┬──────┬────────┬────────┐
│ 00  │  01  │  02  │  03  │  04   │  05  │   06   │   07   │
├─────┼──────┼──────┼──────┼───────┼──────┼────────┼────────┤
│slave│ 0x41 │ addr │ data │ flags │ dtr0 │ CRC lo │ CRC hi │
└─────┴──────┴──────┴──────┴───────┴──────┴────────┴────────┘
              └──────── command, 4 bytes ────────┘
```

### FC66 — status only · request, 4 bytes

```
┌─────┬──────┬────────┬────────┐
│ 00  │  01  │   02   │   03   │
├─────┼──────┼────────┼────────┤
│slave│ 0x42 │ CRC lo │ CRC hi │
└─────┴──────┴────────┴────────┘
```

### Response to FC65 **and** FC66 · 5 + 2×`MBQ_SLOTS` bytes (11 at 3 slots)

```
┌─────┬──────┬──────────┬────────┬─────────┬────────┬─────────┬────────┬─────────┬────────┬────────┐
│ 00  │  01  │    02    │   03   │   04    │   05   │   06    │   07   │   08    │   09   │   10   │
├─────┼──────┼──────────┼────────┼─────────┼────────┼─────────┼────────┼─────────┼────────┼────────┤
│slave│ echo │ accepted │ slot0  │ slot0   │ slot1  │ slot1   │ slot2  │ slot2   │ CRC lo │ CRC hi │
│     │  FC  │  _slot   │ status │ resp8   │ status │ resp8   │ status │ resp8   │        │        │
└─────┴──────┴──────────┴────────┴─────────┴────────┴─────────┴────────┴─────────┴────────┴────────┘
                                  └──────────── status block, 2 bytes/slot ─────────────┘
```

`accepted_slot`:

| value | meaning |
|---|---|
| `0` … `MBQ_SLOTS-1` | the command was placed in this slot |
| `0xFF` (`MBQ_NONE`) | **not taken** — every slot was active. Not an error; resend |

FC66 always replies `accepted_slot = 0xFF` (it enqueues nothing).

### Exception response · 5 bytes

```
┌─────┬────────────┬──────┬────────┬────────┐
│slave│ FC | 0x80  │ code │ CRC lo │ CRC hi │
└─────┴────────────┴──────┴────────┴────────┘
```

Only `0x01 ILLEGAL FUNCTION` is expected in practice, from a gateway that
predates FC65. See [Capability negotiation](#5-capability-negotiation).

**A full queue is deliberately NOT an exception.** Replying
`SLAVE_DEVICE_BUSY (0x06)` would discard the status block in the same breath —
and the status block is precisely what the master needs in order to learn that a
slot has freed. Back-pressure is signalled in-band by `accepted_slot = 0xFF`
*alongside* a complete status block.

### Framing rules

- **Fixed request lengths.** 8 bytes for FC65, 4 for FC66. A frame of any other
  length fails validation and is **discarded without a reply**, exactly as a
  corrupt frame would be. (Contrast: an *unrecognised* function code is accepted
  on CRC alone so it can be answered `ILLEGAL FUNCTION` rather than left to time
  out — the master cannot know the length of a code the gateway doesn't have.)
- **Broadcast (address 0) is rejected.** Only write function codes are legal on
  broadcast; FC65/FC66 are not among them, so a broadcast enqueue is silently
  ignored. Enqueue is inherently addressed — the reply carries the slot.

---

## 3. Slot selection, and why the order matters

`mbq_enqueue_any()` picks the slot:

1. first `IDLE` slot, if any;
2. otherwise the first non-active slot — i.e. `COMPLETE` or `FAILED`;
3. otherwise `MBQ_NONE`.

Preferring `IDLE` means an uncollected result is only overwritten when there is
no alternative.

> **The gateway snapshots the status block BEFORE performing the enqueue.**
>
> Step 2 can reuse a slot whose terminal result the master has not yet seen. If
> the reply were built after the enqueue, that result would be gone — the
> command would look silently lost. Snapshotting first means the master always
> sees the completion **in the very transaction that overwrites it**.
>
> Consequence for the master: `accepted_slot`'s entry in the status block
> describes the slot's state **before** this command was placed — the *previous*
> occupant. It does **not** describe the command just sent.

This is the single most important semantic difference from FC23, where the
gateway writes *then* reads, so a submitted slot's status reflects the new
command.

---

## 4. The full cycle

Three slots, controller streaming commands C1, C2, … at a gateway whose DALI bus
takes ~33 ms per frame. `▸` marks the DALI engine's own progress between
transactions.

| # | request | gateway does | `accepted` | status block returned | master concludes |
|---|---|---|---|---|---|
| 1 | `FC65 C1` | snapshot `[IDLE,IDLE,IDLE]`, place C1 → slot 0 | `0` | `IDLE, IDLE, IDLE` | C1 is live in slot 0 |
| | | ▸ slot 0 `PENDING`→`EXECUTING`, DALI frame starts | | | |
| 2 | `FC65 C2` | snapshot `[EXEC,IDLE,IDLE]`, place C2 → slot 1 | `1` | `EXEC, IDLE, IDLE` | C2 live in slot 1; C1 still running |
| 3 | `FC65 C3` | snapshot `[EXEC,PEND,IDLE]`, place C3 → slot 2 | `2` | `EXEC, PEND, IDLE` | C3 live in slot 2; queue now full |
| | | ▸ C1 finishes → slot 0 `COMPLETE`, resp8 = value; C2 starts | | | |
| 4 | `FC65 C4` | snapshot `[COMPLETE,EXEC,PEND]`, no IDLE → reuse slot 0, place C4 | `0` | **`COMPLETE(v)`**, `EXEC`, `PEND` | **C1's result delivered** *and* C4 enqueued — one transaction |
| 5 | `FC65 C5` | snapshot `[PEND,EXEC,PEND]` — all active | `0xFF` | `PEND, EXEC, PEND` | C5 **not taken**; resend it next time |
| | | ▸ C2 finishes → slot 1 `COMPLETE` | | | |
| 6 | `FC65 C5` | snapshot `[PEND,COMPLETE,PEND]`, reuse slot 1 | `1` | `PEND`, **`COMPLETE(v)`**, `PEND` | C2's result delivered; C5 now live |

Steady state is transaction 4/6 repeating: **each transaction simultaneously
reports one completion and enqueues one command.** That is the whole point —
one Modbus transaction per DALI frame, where FC23 needs two.

### The same cycle over FC23, for comparison

FC23 must name the register it writes, so the master may only submit into a slot
it *already knows* is free — knowledge that is one round trip old.

| # | request | result |
|---|---|---|
| 1 | `FC23` read status, write nothing | learns slot 0 went `COMPLETE` |
| 2 | `FC23` read status, write slot 0 | delivers the result, submits the next command |

Two transactions per DALI frame. Measured on the bench: **2.04** transactions
per frame at 2 and at 3 slots. Eight slots reached 1.38 only because a deeper
queue lets one poll reveal several completions at once, amortising it — the
wasted poll never went away.

---

## 5. Capability negotiation

There is no capability register. The master **tries FC65** and reacts to the
reply:

- normal reply → the gateway supports it; keep using it for this slave;
- exception `0x01 ILLEGAL FUNCTION` → an older gateway; fall back to FC23
  permanently for this slave and never retry;
- timeout → ordinary retry logic, not a capability signal.

Per-slave, not global: a bus may carry mixed firmware. Note that controller and
gateway are already a matched set for `MBQ_SLOTS`, so the fallback matters
mainly during a staged rollout.

---

## 6. Byte cost

At 19200 baud, 8N1 (10 bits/byte ≈ 521 µs/byte):

| | FC23 | FC65 | saving |
|---|---|---|---|
| request | 17 B — 8.85 ms | **8 B — 4.17 ms** | 4.68 ms |
| response (3 slots) | 11 B — 5.73 ms | 11 B — 5.73 ms | — |
| **wire per transaction** | **14.58 ms** | **9.90 ms** | **4.68 ms** |
| transactions per DALI frame | 2.04 (measured) | 1.0 (design) | |

FC23's request restates a read address, read count, write address, write count
and byte count on **every** transaction — nine bytes of framing for a four-byte
payload, and all of it constant.

Projected bus time per DALI frame: **~14 ms against 39.4 ms measured today** at
3 slots over FC23. To be confirmed by measurement, not taken on trust — this
document's projections have been wrong twice already.

---

## 7. Open questions

1. **Pacing.** FC65 makes one transaction per DALI frame *sufficient*, but the
   client currently re-polls immediately (`next_at = now`), so it will still
   issue ~2.4 transactions per 33 ms frame and simply collect `0xFF` for most of
   them. Harmless with one gateway (nothing else wants the bus) and actively
   good for latency, but with **two** gateways a spinning client starves the
   other bus. Should the client back off to a completion estimate when
   `accepted_slot == 0xFF`, or keep spinning and let the scheduler arbitrate?

2. **`resend` semantics on `0xFF`.** The command was definitively not taken, so
   resending is safe and needs no idempotency reasoning. Worth confirming this
   holds for `QF_TWICE` (SET-class) commands, which are not idempotent if they
   *were* to land twice.

3. **Reuse of a `FAILED` slot** currently ranks equal with `COMPLETE` in step 2
   of slot selection. Should a `FAILED` result be preferred for reuse (it is
   less informative), or is first-fit fine?

4. **Two reconcile policies.** The FC23 path's ambiguity handling
   (`resolve_ambiguous`, the `k == submit_slot` trust rule) exists because a
   submit can land while its response is lost. FC65 removes that ambiguity
   entirely — acceptance is explicit. Keep them as separate functions, or one
   with a mode flag?

---

## 8. Why FC65 exists

Short version: **FC23 forces the master to name a slot, so it can only submit
into one it already knows is free.** It learns that from a read. So every DALI
frame costs a read-only poll that carries no work, then a submit.

Letting the *gateway* choose the slot removes the dependency, and the reply's
`accepted_slot` closes the loop. The byte saving (17 → 8) is real but secondary;
eliminating the poll is the substance.
