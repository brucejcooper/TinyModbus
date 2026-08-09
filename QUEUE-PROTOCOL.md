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
 bit  15     13 12          8 7                      0
     ┌─────────┬──────────────┬─────────────────────────┐
     │  state  │     seq      │       response8         │
     │ (3 bits)│   (5 bits)   │        (8 bits)         │
     └─────────┴──────────────┴─────────────────────────┘
```

`state` needs 5 values, so 3 bits; the remaining 5 carry the slot's **accept
sequence**, which is what gives every command an identity on the wire at no
byte cost. See [Identity](#25-identity-and-lost-replies).

| `state` | value | meaning |
|---|---|---|
| `MBQ_S_IDLE` | `0` | free, writable |
| `MBQ_S_PENDING` | `1` | accepted, waiting for the DALI engine |
| `MBQ_S_EXECUTING` | `2` | DALI frame in flight |
| `MBQ_S_COMPLETE` | `3` | terminal — `response8` is the answer byte |
| `MBQ_S_FAILED` | `4` | terminal — `response8` is a failure code |

`response8` is `0` unless the state is terminal.

`seq` is stamped by the **gateway** from a monotonic counter when a slot goes
idle→pending, and stays with the command until the slot is reused. It is the
same field the arbiter already uses to run commands in submission order
(`mbq_next_pending()` picks the lowest seq, wrap-aware) — this exposes it rather
than inventing anything. 5 bits wrap at 32.

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

The master supplies **no** identifier. Identity is the gateway's `seq`, returned
in the status block — see [Identity](#25-identity-and-lost-replies).

### FC66 — status only · request, 4 bytes

```
┌─────┬──────┬────────┬────────┐
│ 00  │  01  │   02   │   03   │
├─────┼──────┼────────┼────────┤
│slave│ 0x42 │ CRC lo │ CRC hi │
└─────┴──────┴────────┴────────┘
```

### Response to FC65 **and** FC66 · 4 + 2×`MBQ_SLOTS` bytes (10 at 3 slots)

```
┌─────┬──────┬────────┬───────┬────────┬───────┬────────┬───────┬────────┬────────┐
│ 00  │  01  │   02   │  03   │   04   │  05   │   06   │  07   │   08   │   09   │
├─────┼──────┼────────┼───────┼────────┼───────┼────────┼───────┼────────┼────────┤
│slave│ echo │ slot0  │ slot0 │ slot1  │ slot1 │ slot2  │ slot2 │ CRC lo │ CRC hi │
│     │  FC  │ st|seq │ resp8 │ st|seq │ resp8 │ st|seq │ resp8 │        │        │
└─────┴──────┴────────┴───────┴────────┴───────┴────────┴───────┴────────┴────────┘
              └──────────── status block, 2 bytes/slot ──────────┘
```

**That is the whole reply.** No `accepted_slot`, no acknowledgement field: the
status block already says which slot holds which `seq`, which is strictly more
information. Both function codes return the identical shape, so there is one
parser.

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
slot has freed. Back-pressure is signalled by the absence of a new `seq`,
*alongside* a complete status block.

### 2.5 Identity and lost replies

**Under FC23, the slot index *is* the identity.** The master names the register
it writes, so it knows what it put where, and the status block is indexed by
slot. Correlation is free.

**FC65 gives that away** — the gateway chooses the slot. So identity has to come
back in the reply, and a reply can be lost. The master would otherwise face:

> I sent an enqueue and got nothing back. Did it land? If so, where?

Neither answer is safe to assume: resend and a `QF_TWICE` SET-class command may
execute **twice**; don't resend and it may never execute at all.

**The `seq` in each slot's status byte answers both questions, for free.** The
master keeps the `(slot, seq)` pairs it has seen. On every reply:

| observation | meaning |
|---|---|
| slot *k* holds a `seq` the master already knows | same command, possibly advanced state |
| slot *k* holds a **new** `seq` | this is the master's most recent enqueue — bind it to slot *k* |
| no slot holds a new `seq` | the enqueue was **not** accepted — resend |

After a lost reply the master simply issues FC66 (4 bytes) and applies the same
three rules. There is no separate recovery path, no acknowledgement field, and
no idempotency reasoning — which is exactly what FC23 needs `resolve_ambiguous()`
and the per-command `idempotent` flag for.

**Why a new `seq` is unambiguous:** the bus is single-master and half-duplex, so
exactly one transaction is outstanding at a time and the master will not issue
the next enqueue until it has resolved the last. At most **one** command can be
un-bound, so at most one unrecognised `seq` can appear.

**Why 5 bits suffice:** wrap is at 32, and the master observes the full status
block on every single transaction. Mistaking a wrapped `seq` for a live one would
require 32 enqueues between two observations, which cannot happen when there is
one transaction in flight and the master sees every reply. `MBQ_SLOTS` is 3, so
only three sequences are live at once.

**Ordering is unaffected.** `seq` is the same field `mbq_next_pending()` already
uses to run commands in submission order, wrap-aware — this exposes it rather
than adding anything. Narrowing it 8 → 5 bits does require the arbiter's
comparison to sign-extend from bit 4 instead of bit 7.

**Gateway reboot** resets the counter, so sequences restart at 0 and could
coincide with live ones. That is already detected independently: a reboot clears
every slot to `IDLE`, and a slot the master believes is live reading `IDLE` is
the existing "idle means gone" rule. Reboot handling runs first and discards all
bindings, so no stale `seq` survives to be matched.

### Framing rules

- **Fixed request lengths.** 8 bytes for FC65, 4 for FC66. A frame of any other
  length fails validation and is **discarded without a reply**, exactly as a
  corrupt frame would be. (Contrast: an *unrecognised* function code is accepted
  on CRC alone so it can be answered `ILLEGAL FUNCTION` rather than left to time
  out — the master cannot know the length of a code the gateway doesn't have.)
- **A discarded FC65 is indistinguishable from a lost reply**, and needs no
  special handling: the master times out and runs the §2.5 sequence check, which
  reports "never landed" and it resends. One recovery path covers corruption in
  either direction.
- **Broadcast (address 0) is rejected.** Only write function codes are legal on
  broadcast; FC65/FC66 are not among them, so a broadcast enqueue is silently
  ignored. Enqueue is inherently addressed — the reply carries the slot.

---

## 3. Slot selection and transaction order

An FC65 transaction is **write then read**, in that order:

1. place the command;
2. build the reply from the status block **as it stands afterwards**.

So the reply shows the command just sent — `PENDING`, or already `EXECUTING` if
the arbiter picked it up before the reply was built (which it will whenever the
DALI bus was idle). The master gets its `(slot, seq)` binding from the very
transaction that created it, exactly as FC23 does.

### Which slot, and the rule that makes write-then-read safe

`mbq_enqueue_any()` picks:

1. the first `IDLE` slot; failing that
2. the first slot holding an **already-reported** terminal; failing that
3. nothing — no new `seq` appears in the reply, and the master resends.

Step 2 is the load-bearing one. **A terminal may only be overwritten once it has
appeared in a reply.** The gateway keeps one `reported` bit per slot, set when a
status block containing that terminal is transmitted; it is not on the wire.

Without that bit, write-then-read destroys results in the *ordinary* case, not a
rare one. A completion landing between two enqueues would be overwritten by the
next write and the reply — built afterwards — would never show it. At 3 slots
under load that is the steady state, so most results would simply vanish. The
`reported` bit costs one transaction of latency before a slot can be recycled
and nothing else.

### The one remaining lossy path

If the reply that first carried a terminal is **lost in transit**, the gateway
has marked it reported but the master never saw it, and the next enqueue may
reuse the slot.

Bounded and detectable, not silent:

- `QF_TWICE` SET-class commands have **no answer** to lose.
- `QF_ANSWER` queries do, but the master sees it precisely — a `seq` it was
  tracking disappears having never shown a terminal. Queries are idempotent, so
  it re-issues.
- It needs a lost reply to begin with. Measured at 19200: zero frame or CRC
  errors across thousands of transactions.

Closing even this costs 2 bytes — see [open question 5](#7-open-questions).

---

## 4. The full cycle

Three slots, controller streaming commands C1, C2, … at a gateway whose DALI bus
takes ~33 ms per frame. `▸` marks the DALI engine's own progress between
transactions.

`R` marks a slot whose terminal has been reported and is therefore recyclable.

| # | request | gateway does | status block returned | master concludes |
|---|---|---|---|---|
| 1 | `FC65 C1` | place C1 → slot 0 as seq 1, then read | **`PEND 1`**, `IDLE`, `IDLE` | seq 1 is new → C1 bound to slot 0, immediately |
| 2 | `FC65 C2` | place C2 → slot 1 as seq 2, then read | `EXEC 1`, **`PEND 2`**, `IDLE` | C2 bound; C1 executing |
| 3 | `FC65 C3` | place C3 → slot 2 as seq 3, then read | `EXEC 1`, `PEND 2`, **`PEND 3`** | C3 bound; queue full |
| | | ▸ C1 finishes → slot 0 `COMPLETE 1`, resp8 = value | | |
| 4 | `FC65 C4` | no IDLE, no *reported* terminal → **nothing placed**; read reports slot 0 and marks it `R` | **`COMPLETE 1(v)`**, `EXEC 2`, `PEND 3` | **C1's result delivered.** No new seq → C4 not taken, resend |
| 5 | `FC65 C4` | slot 0 is `R` → reuse it, place C4 as seq 4, then read | **`PEND 4`**, `EXEC 2`, `PEND 3` | C4 bound to slot 0 |
| | | ▸ C2 finishes → slot 1 `COMPLETE 2` | | |
| 6 | `FC65 C5` | no IDLE, no `R` → nothing placed; report slot 1, mark `R` | `PEND 4`, **`COMPLETE 2(v)`**, `PEND 3` | C2's result delivered; C5 not taken |
| 7 | `FC65 C5` | slot 1 is `R` → reuse, place C5 as seq 5 | `PEND 4`, **`PEND 5`**, `EXEC 3` | C5 bound |
| 8 | `FC65 C6` | placed → slot 2 as seq 6, **reply lost on the wire** | *(never seen)* | timeout — did C6 land? |
| 9 | `FC66` | nothing; just reports | `PEND 4`, `EXEC 5`, **`PEND 6`** | seq 6 unrecognised → C6 **did** land, in slot 2. Bind it, do not resend |

Nothing is ever lost here: every terminal is transmitted before its slot becomes
recyclable, and the master's binding arrives in the transaction that created it.

**The cost is visible at transactions 4 and 6** — a full queue spends one
transaction reporting-and-freeing before the next can be placed. So a saturated
3-slot queue alternates *report* and *place*, giving **2 transactions per DALI
frame**, which is exactly what we were trying to escape.

That is not a flaw in write-then-read; it is the queue being too shallow. With
4 slots the recycling transaction overlaps a still-executing command and the
alternation disappears. **This needs measuring, not assuming** — see
[open question 6](#7-open-questions).

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
| response (3 slots) | 11 B — 5.73 ms | **10 B — 5.21 ms** | 0.52 ms |
| **wire per transaction** | **14.58 ms** | **9.38 ms** | **5.20 ms** |
| transactions per DALI frame | 2.04 (measured) | 1.0 (design) | |

Identity costs **zero bytes** — `seq` rides in the 5 bits the state enum was
already wasting. The reply is a byte *shorter* than the first FC65 draft, which
carried an `accepted_slot` that the per-slot sequences make redundant.

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
   issue ~2.4 transactions per 33 ms frame and collect nothing new from most of
   them. Harmless with one gateway (nothing else wants the bus) and actively
   good for latency, but with **two** gateways a spinning client starves the
   other bus. Should the client back off to a completion estimate when no slot
   is placeable, or keep spinning and let the scheduler arbitrate?

2. ~~**`resend` semantics.**~~ **Resolved by the sequence** (§2.5). A resend is
   safe precisely when no new `seq` appeared, which the gateway reports rather
   than the master inferring. The master needs no tag allocation discipline at
   all — it holds the command and retries the identical bytes.

3. **Reuse of a `FAILED` slot** currently ranks equal with `COMPLETE` in step 2
   of slot selection. Should a `FAILED` result be preferred for reuse (it is
   less informative), or is first-fit fine?

4. **Two reconcile policies.** The FC23 path's ambiguity handling
   (`resolve_ambiguous`, the `k == submit_slot` trust rule) exists because a
   submit can land while its response is lost. FC65 removes that ambiguity
   entirely — a new `seq` is proof of acceptance. Keep them as separate
   functions, or one with a mode flag?

5. **The last lossy path** (§3): a terminal whose only reply was lost can be
   recycled before the master ever saw it. Closing it costs 2 bytes — the reply
   carries `MBQ_SLOTS + 1` status entries, the extra being whatever terminal
   this write displaced. 12 bytes instead of 10, and no lossy path at all.
   Worth 1.04 ms per transaction?

6. **Slot count, again.** The `reported` bit means a full queue spends one
   transaction recycling before the next placement, so a saturated **3**-slot
   queue alternates report/place — 2 transactions per DALI frame, which is what
   FC23 already does. 4 slots should overlap the recycling with a still-executing
   command and remove the alternation, at 2 more bytes per reply. This is the
   question the whole exercise turns on and it must be **measured**: every
   projection in this document has been wrong at least once.

---

## 8. Why FC65 exists

Short version: **FC23 forces the master to name a slot, so it can only submit
into one it already knows is free.** It learns that from a read. So every DALI
frame costs a read-only poll that carries no work, then a submit.

Letting the *gateway* choose the slot removes the dependency, and the reply's
`accepted_slot` closes the loop. The byte saving (17 → 8) is real but secondary;
eliminating the poll is the substance.
