# DALI command queue — Modbus contract (v3)

Authoritative description of how the controller (RP2040 master) drives the
TinyModbus gateway's DALI command queue.

> **Implementation status.** This document is the v3 design. The gateway
> currently implements a draft-1 FC65 (`accepted_slot` in the reply, snapshot
> before enqueue, no `seq` on the wire); the controller drives FC23 only. The
> deltas are listed in [§9](#9-deltas-from-what-exists). Until both land, the
> code wins for interpreting captures.

Two transports carry the same queue:

| | function code | status |
|---|---|---|
| **FC23** Read/Write Multiple Registers | `0x17` | standard, always supported |
| **FC65** Enqueue + status | `0x41` | user-defined, primary path |
| **FC66** Status only | `0x42` | user-defined, recovery/poll |

FC23 is the compatibility path and is never removed. FC65/FC66 exist because
FC23 costs an extra round trip per DALI frame — see [Why FC65 exists](#8-why-fc65-exists).

**Both paths are ordinary Modbus RTU.** Same addressing, same CRC-16, same t3.5
framing, same exception replies. MODBUS Application Protocol V1.1b reserves
function codes 65–72 and 100–110 for user-defined use, so FC65/FC66 are
conformant rather than an extension of the standard set. Neither path skips
silence, dispatches early, or shortcuts the state machine.

## What v3 changes, in one paragraph

Draft 2 protected uncollected results with a `reported` bit — a terminal slot
could only be reused after its result had been transmitted once. That rule made
a saturated 3-slot queue alternate *report* / *place*: **2 transactions per
DALI frame, which is exactly what FC23 already costs** (its own §4 admitted
this and reached for a 4th slot). It also left one lossy path: a terminal whose
only reply was lost could be recycled unseen. v3 replaces the bit with
**displacement**: an enqueue may overwrite a terminal slot *immediately*, and
the displaced result is carried in the same reply **and** retained in a
1-entry displaced register until the next enqueue, so a lost reply can never
lose it. v3 also adds an explicit **boot epoch byte** to every reply, replacing
the "idle means gone" reboot inference whose corner cases produced the task-04
recovery-bug swamp. Net: **one transaction per DALI frame at saturation, three
slots, no lossy path, reboots detected by comparison rather than inference.**

---

## 1. The queue

`MBQ_SLOTS` slots (**3**), each holding one opaque 4-byte command and a 2-byte
status. The gateway executes **one DALI frame at a time**; the slots exist to
keep it fed, not to parallelise it. Three slots hold: the command **executing**,
the **next** pending, and one that is either a *second* next or a terminal
awaiting displacement — which is precisely the "current + next, possibly two
next" the system needs, and the depth-sweep knee measured in task 07
(throughput saturates at depth 3; deeper only adds latency).

Slot lifecycle:

```
IDLE ──enqueue──► PENDING ──arbiter picks──► EXECUTING ──DALI answers──► COMPLETE
  ▲                                                                  └──► FAILED
  └────────────── displaced by a later enqueue ◄─────────────────────────┘
```

`COMPLETE`/`FAILED` are terminal and **sticky** — the slot holds its result
until a later enqueue displaces it. There is no collect step and no ack step:
the master learns results from status blocks, and every displaced result is
guaranteed to reach the master (see [§3](#3-slot-selection-and-displacement)).

### Slot status register

One 16-bit register per slot, big-endian on the wire — served identically by
FC65/FC66 replies and by FC03/FC04/FC23 reads of the status block (one wire
format, both transports; this is a breaking change for the FC23 parser's masks):

```
 bit  15     13 12          8 7                      0
     ┌─────────┬──────────────┬─────────────────────────┐
     │  state  │     seq      │       response8         │
     │ (3 bits)│   (5 bits)   │        (8 bits)         │
     └─────────┴──────────────┴─────────────────────────┘
```

`state` needs 5 values, so 3 bits; the remaining 5 carry the slot's **accept
sequence**, which gives every command an identity on the wire at zero byte
cost. See [Identity](#25-identity-and-recovery).

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
(`mbq_next_pending()` picks the lowest seq, wrap-aware — the 5-bit narrowing
means that comparison sign-extends from bit 4). 5 bits wrap at 32; safety
argument in [§2.5](#25-identity-and-recovery).

| failure code | value | meaning |
|---|---|---|
| `DALI_RESP_NAK` | `0x01` | no (or too short) backward frame where one was expected |
| `DALI_RESP_COLLISION` | `0x02` | garbled backward frame — two or more gear answered |
| `DALI_RESP_BUSFAULT` | `0x03` | transport fault: short pulse, stuck bus |

> **Ordering guarantee.** `mbq_complete()` writes `response8` **before**
> `status`; every status reader reads `status` **before** `response8`. That
> pairing is what makes a terminal status imply a `response8` that is already
> correct. Reversing either side reintroduces a stale-value race. This is
> load-bearing, not style.

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

The master supplies **no** identifier. Identity is the gateway's `seq`,
returned in the status block — see [Identity](#25-identity-and-recovery).

### FC66 — status only · request, 4 bytes

```
┌─────┬──────┬────────┬────────┐
│ 00  │  01  │   02   │   03   │
├─────┼──────┼────────┼────────┤
│slave│ 0x42 │ CRC lo │ CRC hi │
└─────┴──────┴────────┴────────┘
```

### Response to FC65 **and** FC66 · 7 + 2×(`MBQ_SLOTS`+1) bytes = 13 at 3 slots

```
┌─────┬──────┬──────┬────────┬───────┬────────┬───────┬────────┬───────┬────────┬───────┬────────┬────────┐
│ 00  │  01  │  02  │   03   │  04   │   05   │  06   │   07   │  08   │   09   │  10   │   11   │   12   │
├─────┼──────┼──────┼────────┼───────┼────────┼───────┼────────┼───────┼────────┼───────┼────────┼────────┤
│slave│ echo │ boot │ disp   │ disp  │ slot0  │ slot0 │ slot1  │ slot1 │ slot2  │ slot2 │ CRC lo │ CRC hi │
│     │  FC  │  8   │ st|seq │ resp8 │ st|seq │ resp8 │ st|seq │ resp8 │ st|seq │ resp8 │        │        │
└─────┴──────┴──────┴────────┴───────┴────────┴───────┴────────┴───────┴────────┴───────┴────────┴────────┘
                     └─ displaced ──┘ └───────────── status block, 2 bytes/slot ───────┘
```

Three fields beyond the status block, and each earns its bytes:

- **`boot8`** — the low byte of the gateway's boot counter (`sysinfo`
  `g_boot_count`). A change relative to the master's last-seen value **is** the
  reboot signal: no inference from slot states, no anomaly heuristics, works
  even when the master has nothing in flight. One byte closes the entire
  class of "papered-over reboot" reconciliation bugs that task 04 spent three
  fix cycles on (a retransmit landing on a freshly rebooted gateway used to
  masquerade as a live queue). Aliasing needs exactly 256 reboots between two
  observations — accepted.
- **`displaced`** — the [status-register-format](#slot-status-register) entry
  of the terminal that the **most recent enqueue** overwrote, or `0x0000`
  (state `IDLE`) if that enqueue used a free slot or there has been none. This
  is a *register*, not just a reply field: it persists until the next enqueue,
  so the result it carries survives a lost reply. See
  [§3](#3-slot-selection-and-displacement).
- No `accepted_slot` (draft 1) and no `reported` bit (draft 2). The per-slot
  `seq` makes the first redundant; displacement makes the second unnecessary.

Both function codes return the identical shape, so there is one parser.

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
and the status block is precisely what the master needs in order to learn that
a slot has freed. Back-pressure is signalled by the absence of a new `seq`,
*alongside* a complete status block. (Task-07 finding, generalised: the
transport must never auto-retry on back-pressure — on the FC65 path
back-pressure isn't even an error, so the hazardous retry path has nothing to
attach to.)

### 2.5 Identity and recovery

**Under FC23, the slot index *is* the identity.** The master names the register
it writes, so correlation is free — and ambiguity after a lost reply needs the
`resolve_ambiguous()` machinery and per-command idempotency reasoning.

**FC65 gives the slot choice to the gateway**, so identity comes back in the
reply as `seq`, and every reconciliation reduces to four rules applied to any
status block, in this order:

| # | observation | meaning |
|---|---|---|
| 0 | `boot8` differs from last seen | **gateway rebooted** — run reboot handling first ([§2.6](#26-reboot-handling)), then continue below against the fresh block |
| 1 | slot *k* holds a `seq` bound to a live command | same command, possibly advanced state; deliver its result if terminal (and unbind) |
| 2 | slot *k* holds a **new** `seq` | this is the master's most recent enqueue — bind it to slot *k* |
| 3 | a live binding's `seq` is gone from its slot | it was **displaced** — its result is in the `displaced` field (match by `seq`); deliver and unbind |
| — | no new `seq` after an enqueue | the enqueue was **not** accepted — pace, then resend |

After a lost reply (timeout), the master issues FC66 (4 bytes) and applies the
same rules. **There is one recovery path**, and it needs no idempotency
reasoning: if the command landed, its `seq` is visible (rule 2 — even if it
already executed and went terminal, the slot still shows it); if it didn't, no
new `seq` exists and resending is safe by construction.

**Why a new `seq` is unambiguous:** the bus is single-master and half-duplex,
so exactly one transaction is outstanding at a time, and the master does not
issue the next enqueue until it has resolved the last (by reply or by FC66).
At most **one** command can be un-bound, so at most one unrecognised `seq` can
appear — and at most one displacement can have occurred since the master's
last observation, which is why a **1-entry** displaced register is sufficient
and why rule 3's match-by-`seq` cannot alias.

**Why 5 bits suffice:** wrap is at 32 and at most `MBQ_SLOTS` = 3 sequences are
live at once. Mistaking a wrapped `seq` for a live one would require 32
enqueues between two observations, which single-outstanding operation makes
impossible. The residual corner — a *stale* displaced entry aliasing a live
`seq` — is closed structurally: the displaced register is overwritten (or
cleared to `IDLE`) by **every** accepted enqueue, and rule 3 only consults it
when a live binding has just vanished, at which point it can only hold that
binding's own displacement.

### 2.6 Reboot handling

On a `boot8` change the master, in order:

1. **discards all bindings** — every live `seq` belongs to the dead epoch;
2. disposes of the commands those bindings carried: commands the *client*
   marked idempotent (queries, absolute SETs) are resubmitted; non-idempotent
   ones (UP/DOWN steps) are delivered to their submitters as `LOST` — whether
   they executed before the reboot is unknowable, and pretending otherwise is
   how v1 corrupted itself;
3. applies rules 1–3 to the block in the same reply: after a reboot at most
   one slot is non-idle (the enqueue this very transaction carried, if any) —
   it binds by rule 2 and is *not* part of the loss, since it landed in the
   new epoch.

This replaces draft 2's "idle means gone" inference entirely. That rule had a
real (if narrow) hole: a master whose only live command sat un-executed in slot
0 could see a post-reboot block — its own new enqueue in slot 0, `PENDING`,
seq 0 — that is bit-identical to "nothing happened yet", and would resend into
a double execution. `boot8` makes the epoch explicit and the hole gone.

`boot8` also lets the client keep an honest reboot counter per slave (visible
in `mbq` stats) even when the queue was empty across the reboot — previously
invisible, and exactly the early-warning signal a browning-out gateway gives.

### Framing rules

- **Fixed request lengths.** 8 bytes for FC65, 4 for FC66. A frame of any other
  length fails validation and is **discarded without a reply**, exactly as a
  corrupt frame would be. (Contrast: an *unrecognised* function code is
  accepted on CRC alone so it can be answered `ILLEGAL FUNCTION` rather than
  left to time out — the master cannot know the length of a code the gateway
  doesn't have.)
- **A discarded FC65 is indistinguishable from a lost reply**, and needs no
  special handling: the master times out, issues FC66, and applies §2.5. One
  recovery path covers corruption in either direction.
- **Broadcast (address 0) is rejected.** Only write function codes are legal on
  broadcast; FC65/FC66 are not among them, so a broadcast enqueue is silently
  ignored. Enqueue is inherently addressed — the reply carries the binding.

---

## 3. Slot selection and displacement

An FC65 transaction is **enqueue, then read**, in that order:

1. place the command (possibly displacing a terminal — below);
2. build the reply from the status block **as it stands afterwards**, plus the
   displaced register and `boot8`.

So the reply shows the command just sent — `PENDING`, or already `EXECUTING` if
the arbiter picked it up before the reply was built (which it will whenever the
DALI engine was idle). The master gets its `(slot, seq)` binding from the very
transaction that created it.

> Draft 1 snapshotted the block *before* the enqueue for exactly the reason the
> displaced register now handles: an overwritten terminal must not vanish
> unreported. With displacement carrying it explicitly, the *after* snapshot is
> both safe and strictly better — it shows the new command's binding
> immediately.

### `mbq_enqueue_any()` picks:

1. the first `IDLE` slot — and **clears** the displaced register to `0x0000`;
2. failing that, the terminal (`COMPLETE`/`FAILED`) with the **oldest `seq`**
   (wrap-aware) — its status entry moves into the displaced register, and the
   slot is re-armed with the new command;
3. failing that (all three slots pending/executing), nothing: no new `seq`
   appears in the reply, and the master paces and resends.

Rule 2's oldest-first keeps result delivery in submission order when two
terminals coexist. `COMPLETE` and `FAILED` rank equally — displacement is
lossless, so there is nothing to protect by preferring one (draft 2's open
question 3, dissolved rather than answered).

### Why no result can be lost

A displaced result reaches the master through one of two doors, and both are
always open:

- **the reply carrying it arrives** — delivered by rule 3 immediately; or
- **that reply is lost** — the master resolves via FC66 *before its next
  enqueue* (single-outstanding discipline), and the displaced register still
  holds the result, because only an accepted enqueue overwrites it and the
  master hasn't issued one.

Terminals that are *not* displaced sit sticky in their slots and are delivered
by rule 1 on any subsequent reply. Draft 2's residual lossy path — a terminal
recycled after its only report was lost in transit — no longer exists, at a
cost of 2 bytes per reply instead of a `reported` bit, a 4th slot, and a
transaction of recycling latency.

The one genuinely unreachable result is a command in flight across a **gateway
reboot** — the result never existed. That is §2.6's `LOST` disposition, and no
wire protocol can do better than report it honestly.

---

## 4. The full cycle

Three slots, controller streaming commands C1, C2, … at a gateway whose DALI
bus takes ~25–38 ms per frame while an FC65 transaction costs ~11 ms of wire.
`▸` marks the DALI engine's own progress between transactions. `D:` is the
displaced register.

| # | request | gateway does | reply (disp · slots) | master concludes |
|---|---|---|---|---|
| 1 | `FC65 C1` | slot 0 idle → seq 1, D cleared | D:`—` · **`PEND 1`**, `IDLE`, `IDLE` | C1 bound to slot 0 |
| 2 | `FC65 C2` | slot 1 idle → seq 2, D cleared | D:`—` · `EXEC 1`, **`PEND 2`**, `IDLE` | C2 bound; C1 executing |
| 3 | `FC65 C3` | slot 2 idle → seq 3, D cleared | D:`—` · `EXEC 1`, `PEND 2`, **`PEND 3`** | C3 bound; queue full — **pace ~one frame** |
| | | ▸ C1 finishes → slot 0 `COMPLETE 1(v₁)` | | |
| 4 | `FC65 C4` | no idle; oldest terminal = slot 0 → **displace**, re-arm as seq 4 | **D:`COMP 1(v₁)`** · **`PEND 4`**, `EXEC 2`, `PEND 3` | C1's result delivered via D; C4 bound. **One transaction did both.** |
| | | ▸ C2 finishes → slot 1 `COMPLETE 2(v₂)` | | |
| 5 | `FC65 C5` | displace slot 1, seq 5 | **D:`COMP 2(v₂)`** · `PEND 4`, **`PEND 5`**, `EXEC 3` | C2 delivered; C5 bound |
| | | ▸ C3 finishes → slot 2 `COMPLETE 3(v₃)` | | |
| 6 | `FC65 C6` | displace slot 2, seq 6 — **reply lost on the wire** | *(never seen)* | timeout — did C6 land? did C3's result survive? |
| 7 | `FC66` | nothing placed; D untouched since #6's enqueue | **D:`COMP 3(v₃)`** · `EXEC 4`, `PEND 5`, **`PEND 6`** | seq 6 is new → C6 **did** land (rule 2). C3's binding vanished → rule 3 finds it in D → **result recovered**. Nothing resent, nothing lost. |

Steady state is transactions 4–5: **one FC65 per DALI frame**, each delivering
the oldest result and enqueueing one command. The report/place alternation of
draft 2 (its transactions 4 and 6 — a wasted reporting pass per frame at
saturation) is gone, which was the entire point.

### The same cycle over FC23, for comparison

FC23 must name the register it writes, so the master may only submit into a
slot it *already knows* is free — knowledge that is one round trip old. A
saturated FC23 queue therefore alternates a read-only poll with each submit:
**2.04 transactions per DALI frame, measured** (task 07), at 28 bytes each.

---

## 5. Capability negotiation

There is no capability register. The master **tries FC65** and reacts to the
reply:

- normal reply → the gateway supports it; keep using it for this slave;
- exception `0x01 ILLEGAL FUNCTION` → an older gateway; fall back to FC23
  permanently for this slave and never retry;
- timeout → ordinary retry logic, not a capability signal.

Per-slave, not global: a bus may carry mixed firmware. Controller and gateway
remain a matched set for `MBQ_SLOTS` and the status-register format (the seq
packing changes the FC23 view too), so the fallback matters mainly during a
staged rollout; the FC23 client keeps its own `resolve_ambiguous()` reconcile
unchanged (draft 2's open question 4: two policies, kept separate — the FC65
one is smaller, not a mode of the other).

---

## 6. Client policy

The controller-side rules that make the wire protocol deliver its numbers.
These live in `mbq_client.c` (transport) and `dali_queue.c` (scheduler); the
split is unchanged from task 07.

- **Single outstanding, resolve before next enqueue.** Every FC65 is resolved —
  by its reply or by FC66 after timeout — before another FC65 is issued to the
  same slave. This discipline is what §2.5's uniqueness arguments and the
  1-entry displaced register rest on. (It is also simply what a half-duplex
  RS-485 master does.)
- **Pace, don't spin.** When a reply shows nothing placeable (3 active slots)
  or the backlog is empty, the client schedules its next transaction at the
  **estimated completion of the oldest executing command** (frame-time
  estimate by command class: ~25 ms SET, ~38 ms query), rather than re-polling
  immediately. With one gateway, spinning is merely wasted bus (~2.4 polls per
  frame measured in the soak); with two gateways on the shared RS-485 it
  starves the other's traffic. Pacing resolves draft 2's open question 1 in
  favour of the shared bus; the scheduler's per-slave round-robin remains the
  backstop.
- **In-flight cap = 3** (`DQ_INFLIGHT_MAX`), the measured knee. The FC23-era
  "cap at `MBQ_SLOTS`−1 to preserve a free write target" rule does not apply
  to FC65 — displacement means a full-of-terminals queue is still writable.
- **No transport-level retry of executed work.** A timed-out transaction is
  resolved by FC66 + §2.5, never by blind re-send. DEVICE_BUSY cannot occur on
  the FC65 path (back-pressure is in-band); on the FC23 fallback the task-07
  finding stands — deliver DEVICE_BUSY to the client, do not re-send under it.
- **Failure delivery.** `FAILED` + code goes to the submitter's callback
  verbatim. Retry policy belongs to the *scheduler's* priority classes (e.g.
  one retry for an idempotent query on `BUSFAULT`; never for UP/DOWN steps),
  not to the transport — the transport cannot know what is safe to repeat.
- **Reboot disposition** per §2.6, using the client's existing per-command
  idempotent flag; count reboots per slave from `boot8`.
- **Dead-slave backoff** (task-07 finding, orthogonal but worth landing with
  this): a slave that times out consecutively gets an exponential poll backoff
  — the dead relay board was costing ~8 % of the bus in retry timeouts.

---

## 7. Byte cost

At 19200 baud, 8N1 (10 bits/byte ≈ 521 µs/byte):

| | FC23 | FC65 v3 | saving |
|---|---|---|---|
| request | 17 B — 8.85 ms | **8 B — 4.17 ms** | 4.68 ms |
| response | 11 B — 5.73 ms | **13 B — 6.77 ms** | −1.04 ms |
| wire per transaction | 14.58 ms | **10.94 ms** | 3.64 ms |
| transactions per DALI frame | **2.04 (measured)** | **1.0 (design)** | |
| **wire per DALI frame** | **~29.7 ms** | **~10.9 ms** | **~2.7×** |

The two reply bytes v3 adds over draft 2 (`boot8` + displaced entry) buy the
explicit reboot epoch and the closed lossy path; the transaction they eliminate
per frame at saturation repays them ~14× over.

Against the soak's measured ~35.5 ms of DALI time per frame, one 10.9 ms
transaction per frame puts Modbus at **~31 % bus occupancy for a saturated
gateway** — comfortable room for the second DALI bus's gateway on the same
RS-485 at full rate, which FC23's ~84 % could never accommodate.

**These are projections and this document's projections have been wrong twice.
The soak probe exists precisely to check them** — see [§10](#10-validation).

---

## 8. Why FC65 exists

**FC23 forces the master to name a slot, so it can only submit into one it
already knows is free — knowledge that is one round trip old.** Every DALI
frame therefore costs a read-only poll that carries no work, then a submit.
Letting the gateway choose the slot removes the dependency; the per-slot `seq`
gives identity back; displacement lets the same transaction that submits also
deliver the oldest result. The byte saving (17 → 8 request) is real but
secondary; eliminating the second transaction per frame is the substance.

---

## 9. Deltas from what exists

Gateway (`TinyModbus`), all small:

1. `mbq_slots.c/h` — pack `seq` into the status wire format
   (`[state:3|seq:5|resp8:8]`); narrow the arbiter's wrap-compare to 5 bits;
   `mbq_enqueue_any()` gains the oldest-terminal displacement rule and the
   displaced register (+2 B RAM); expose `mbq_displaced()`.
2. `modbus.c` — FC65 becomes enqueue-then-snapshot (currently snapshot-first),
   drops `accepted_slot`, and the reply builder adds `boot8` + displaced entry
   (reply grows 10 → 13 B; `FCQ_RSP_PAYLOAD` = 5 + 2×(`MBQ_SLOTS`+1)).
3. FC03/FC04/FC23 status reads serve the new register format unchanged in
   address, changed in bit layout — **protocol break**, matched set as always.

Controller:

4. `mbq_client.c` — FC65 primary path: per-slave capability probe, the §2.5
   four-rule reconcile (new, small), §2.6 reboot handling keyed on `boot8`,
   pacing, and the FC23 fallback kept as-is behind the probe.
5. FC23 fallback parser masks updated for the new status format.

Explicitly resolved from draft 2's open questions: 1 pacing (§6), 3 FAILED
ranking (§3, dissolved), 4 two reconciles (§5, kept separate), 5 lossy path
(§3, closed by the displaced register), 6 slot count (**stays 3** — the
alternation that motivated a 4th slot is gone; a 4th would cost 2 B on every
reply forever and the depth sweep showed nothing past 3 but latency).

---

## 10. Validation

Reusing the task-07 instrumentation, which was built for exactly this:

1. **Soak** (`soak` CLI, DTR0 stride-97 probe): depth sweep 1–3. Gate:
   0 bad / 0 lost / 0 lag-1 over ≥ 5000 pairs, and **transactions-per-frame
   ≈ 1.0 at depth 3** (`mbq_client_txn_counts()` ÷ frames) — the number this
   whole design exists to hit. Expect pair rate ≥ the FC23 path's 14.0/s;
   the DALI ceiling is ~15.9.
2. **Reboot drill**: `gw_reboot` mid-soak at full rate — clean verdict, reboot
   counted via `boot8`, non-idempotent disposition logged. (Task-07 baseline:
   passes on FC23; must still pass, now without any anomaly inference.)
3. **Lost-reply drill**: a controller-side debug knob drops the next N FC65
   replies after parse (`mbq drop-reply`), forcing the FC66 path — verify
   rule-2 binding (no resend, no double execution) and rule-3 displaced
   recovery (result delivered despite the lost reply). This drills the §3
   "both doors" claim deterministically instead of waiting for line noise.
4. **Bus-share check**: soak both gateways concurrently once the second bus
   exists; `busload` per-slave occupancy stays under ~2/3 total and neither
   starves. Until then, single-gateway `busload` confirms the ~31 % projection.
5. **FC23 fallback**: capability probe against a gateway image with FC65
   compiled out; soak must pass (slower) on the fallback.
