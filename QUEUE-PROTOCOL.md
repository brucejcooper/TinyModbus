# DALI command queue — Modbus contract (v4)

Authoritative specification of how the controller (RP2040 master) drives the
TinyModbus gateway's DALI command queue, and the implementation plan for both
sides.

> **Lineage.** v4 is the synthesis of Bruce's queue language
> (`ALTERNATE-QUEUE-APPROACH.md`: single FC, ack-by-seq, response retention)
> with the failure-handling rules of the v3 draft (see git history of this
> file). Decisions taken 2026-08-09: timeout-based reboot detection (no boot
> byte on the hot path), a two-sequence response, queue depth 2 as a tweakable
> constant, round-robin fairness at the controller, diagnostics in cold-path
> registers rather than the reply.
>
> **Implementation status: NOT YET BUILT.** The gateway currently implements
> the draft-1 FC65/FC66 and the FC23 slot queue; the controller drives FC23
> only. Both are replaced by this design — see [§9](#9-what-v4-removes) and
> the [implementation plan](#10-implementation-plan). Until it lands, the code
> wins for interpreting captures.

---

## 0. The model in one paragraph

One user-defined function code carries all queue traffic. A request optionally
**enqueues** one DALI command and optionally **acks** (deletes) the head of the
gateway's **response queue**; the reply reports the fate of the enqueue (an
**accept byte** with a gateway-assigned sequence) and the current **head** of
the response queue. Results are retained until explicitly acked, so no reply
loss can destroy one. In steady state each transaction enqueues one command,
acks the previous result, and collects the next — **one ~14-byte transaction
per DALI frame**. Reboots are detected by result deadlines plus two structural
rules (seq collision, impossible-empty-head), with the existing sysinfo
`boot_count` register as the cold-path arbiter for the one genuinely ambiguous
corner. The gateway holds nothing the master cannot reconstruct.

Both queues are small and bounded:

| | depth | RAM | notes |
|---|---|---|---|
| command queue | `CMDQ_DEPTH` = **2** | 2 × 6 B | 1 executing + 1 pending; build-time constant, may become 3 — nothing on the wire changes |
| response queue | `RESPQ_DEPTH` = **4** | 4 × 2 B | results held until acked; steady-state occupancy is 2 |

**Backpressure rule (load-bearing):** the arbiter may not start a command
unless the response queue has a free entry. A full response queue stalls DALI
execution rather than ever dropping a result. Because the ack in a request is
processed *before* the enqueue, a single transaction can free an entry and
refill the pipeline.

---

## 1. Wire formats

Function code **0x41** (FC65), user-defined per MODBUS V1.1b (codes 65–72 and
100–110 are reserved for user definition — the draft's 0x40 is *not* in that
range, so v4 uses 0x41; its semantics are a clean break from the never-deployed
draft-1 FC65). Ordinary Modbus RTU in every other respect: same addressing,
CRC-16 (low byte first), t3.5 framing.

### Bitfield conventions

Every packed byte in this protocol has the same shape — **control in the top 3
bits, sequence in the low 5**:

```
 bit   7    5   4          0
     ┌───────┬───────────────┐
     │ ctl:3 │     seq:5     │
     └───────┴───────────────┘
```

Sequences run **1–31**; `0` is reserved and means "none" (no ack requested, no
seq assigned, queue empty).

### Request · 5, 7 or 8 bytes

```
┌─────┬──────┬──────────────────┬────────┬────────┬────────┬────────┬────────┐
│ 00  │  01  │        02        │  03*   │  04*   │  05*   │        │        │
├─────┼──────┼──────────────────┼────────┼────────┼────────┼────────┼────────┤
│slave│ 0x41 │ [flags:3|ack:5]  │  addr  │  data  │  dtr0  │ CRC lo │ CRC hi │
└─────┴──────┴──────────────────┴────────┴────────┴────────┴────────┴────────┘
                                 └──── enqueue payload, present iff ────┘
                                       the frame is 7–8 bytes long
```

| flag | bit | effect |
|---|---|---|
| `QF_TWICE` | 7 | send the forward frame twice (SET-class); no answer expected |
| `QF_DTR0` | 6 | emit `0xA3 <dtr0>` back-to-back **before** the frame; the `dtr0` byte is present |
| `QF_ANSWER` | 5 | a backward frame is expected; capture the answer byte |

`ack` (bits 4–0): if non-zero, delete the head of the response queue **iff**
the head's seq equals this value. A mismatch is a silent no-op — the reply's
head field shows the master what the real head is, which is all it needs to
re-converge.

Three request lengths, distinguished — and validated — by length + flags:

| length | meaning | validity rule |
|---|---|---|
| **5 B** | ack and/or poll, no enqueue | flags must be `000` |
| **7 B** | enqueue without DTR0 preload (+ optional ack) | `QF_DTR0` must be clear |
| **8 B** | enqueue with DTR0 preload (+ optional ack) | `QF_DTR0` must be set |

Any other length, or a length/flag inconsistency, is **discarded without a
reply**, exactly as a corrupt frame would be — to the master it is
indistinguishable from a lost frame and shares the one recovery path (§4).
A 5-byte frame with `ack = 0` is a pure status poll and is always valid.

**Broadcast (address 0) is silently ignored.** Enqueue is inherently addressed
— the reply carries the binding.

### Response · 6 or 7 bytes

To a request that attempted an enqueue (7–8 B request):

```
┌─────┬──────┬────────────────────┬────────────────────┬───────┬────────┬────────┐
│ 00  │  01  │         02         │         03         │  04   │   05   │   06   │
├─────┼──────┼────────────────────┼────────────────────┼───────┼────────┼────────┤
│slave│ 0x41 │ [astatus:3|aseq:5] │ [hstatus:3|hseq:5] │ resp8 │ CRC lo │ CRC hi │
└─────┴──────┴────────────────────┴────────────────────┴───────┴────────┴────────┘
                └── accept byte ──┘ └───── head of response queue ────┘
```

To a 5-byte (ack/poll) request the accept byte is **absent** — 6 bytes total,
head + CRC only. The master always knows which shape to expect because it sent
the request.

**Accept byte** — the fate of *this request's* enqueue:

| `astatus` | value | meaning |
|---|---|---|
| `A_ACCEPTED` | 0 | enqueued; `aseq` (1–31) is the command's identity |
| `A_FULL` | 1 | command queue full — not enqueued, `aseq` = 0; pace and resend |
| — | 2–7 | reserved |

**Head entry** — the oldest unacked result, unchanged until acked:

| field | meaning |
|---|---|
| `hseq` = 0 | response queue empty; `hstatus` and `resp8` are 0 |
| `hseq` ≠ 0, `hstatus` = `H_OK` (0) | command `hseq` succeeded; `resp8` = answer byte (`QF_ANSWER`) or 0 |
| `hseq` ≠ 0, `hstatus` = `H_FAILED` (1) | command `hseq` failed; `resp8` = failure code below |
| `hstatus` 2–7 | reserved |

| failure code | value | meaning |
|---|---|---|
| `DALI_RESP_NAK` | `0x01` | no (or too short) backward frame where one was expected |
| `DALI_RESP_COLLISION` | `0x02` | garbled backward frame — two or more gear answered |
| `DALI_RESP_BUSFAULT` | `0x03` | transport fault: short pulse, stuck bus |

**No Modbus exceptions on this path.** Queue-full is in-band (`A_FULL`), which
is what keeps the status delivery alive under back-pressure, and there is
nothing for a transport auto-retry to latch onto — the task-07 DEVICE_BUSY
double-execution hazard is unconstructible here. `ILLEGAL FUNCTION` can only
come from a pre-v4 gateway (see [§8 versioning](#8-versioning-and-rollout)).

---

## 2. Gateway processing

Per accepted request, **strictly in this order**:

1. **Ack** — if `ack` ≠ 0 and it equals the head's seq, pop the head. (First,
   so the entry it frees is available to the backpressure rule this same
   transaction.)
2. **Enqueue** — if the frame carries a command and a command-queue slot is
   free, stamp it with the next seq (1–31, skipping 0) and set
   `astatus = A_ACCEPTED`; otherwise `A_FULL`. The command bytes are copied
   out before the reply is built over the buffer.
3. **Build the reply** from the state *as it now stands*: accept byte (iff an
   enqueue was attempted), then the current head.

Execution is strict FIFO in accept order. On completion the DALI adapter
pushes `{seq, hstatus, resp8}` onto the response queue tail.

**Publication ordering (load-bearing, the v2 lesson kept):** the completion
path writes the entry's bytes fully **before** publishing it (single-byte
count/tail increment, atomic on AVR); the Modbus builder reads the count
**before** the entry. A reader can never see a published entry with stale
bytes. Same discipline for the accept path: command bytes and seq are written
before the slot is marked pending for the arbiter.

**Seq assignment is exclusively the gateway's.** The master never proposes a
seq; it learns each command's identity from the accept byte. This is what
makes a gateway reboot self-correcting: the counter restarts, the master
simply learns the new numbers — there is no ledger to reconcile, which was
v1's fatal flaw.

**Why a live seq cannot recur without a reboot:** a seq stays pinned in the
gateway (command slot or response entry) until its result is acked, and acks
are head-only, in order. Reaching the same value again requires 31 further
accepts, which requires ≥ 27 acks, which would have drained the pinned entry
first. Therefore *an accept byte carrying a seq the master still tracks is
proof of a gateway reboot* — rule R2 below.

---

## 3. The full cycle

Depth 2, controller streaming C1, C2, … at a gateway whose DALI frames take
~25–38 ms while a transaction costs ~7.5 ms of wire. `▸` marks DALI progress.
Response queue shown as `RQ`.

| # | request | gateway does | reply | master concludes |
|---|---|---|---|---|
| 1 | `ENQ C1, ack 0` | accept seq 1 → EXEC | `A_ACCEPTED 1` · head — | C1 = seq 1 |
| 2 | `ENQ C2, ack 0` | accept seq 2 (pending) | `A_ACCEPTED 2` · head — | C2 = seq 2; queue full → **pace ~1 frame** |
| | | ▸ C1 done → RQ `[1]`, C2 → EXEC | | |
| 3 | `ENQ C3, ack 0` | slot free → accept seq 3 | `A_ACCEPTED 3` · **head 1 (v₁)** | C1's result delivered |
| | | ▸ C2 done → RQ `[1,2]` | | |
| 4 | `ENQ C4, ack 1` | pop 1; accept seq 4 | `A_ACCEPTED 4` · **head 2 (v₂)** | C2 delivered; steady state: **one txn per frame, each acks one + collects one + enqueues one** |
| | | ▸ C3 done → RQ `[2,3]` | | |
| 5 | `ENQ C5, ack 2` | pop 2; accept seq 5 — **reply lost** | *(never seen)* | timeout: did the frame land? |
| 6 | `POLL (5 B, ack 0)` | nothing; reports | **head 3 (v₃)** | head moved past 2 ⇒ the ack in #5 was applied ⇒ **the whole frame landed** (ack and enqueue are one frame) ⇒ C5 = predicted seq 5, bound. C3's result in hand. Nothing resent, nothing lost. |

The recovery at #6 is the two-sequence design paying off: because ack and
enqueue travel together, the head's movement is an **atomic landed/not-landed
indicator** for the entire lost transaction. Had the head still shown seq 2,
the frame never reached the gateway and resending it — ack and all — is safe.

---

## 4. Recovery

One timer and four rules cover every loss mode. The master runs, per slave:

- **Transaction timeout** (existing 50 ms transport timeout): reply not seen.
- **Result deadline**: an accepted command must produce a result within
  `RESULT_DEADLINE_MS` (**150 ms** default: worst queueing at depth 2 is one
  ~50 ms DTR0-pair ahead, plus its own ~50 ms, ×1.5 margin; tunable).

and resolves every anomaly with these rules, applied to any reply:

| # | observation | conclusion / action |
|---|---|---|
| **R1** | reply to an enqueue+ack lost → poll: head advanced past the acked seq | the lost frame **landed**; bind the command to the predicted seq (last accepted + 1, skipping 0) |
| **R1′** | same, but head unchanged (still the seq the master tried to ack) | the lost frame **never arrived**; resend it verbatim — ack included — with no idempotency concern |
| **R2** | an accept byte returns a seq the master still tracks | **gateway rebooted** (§2 proof); dispose all tracked commands (below), keep only the just-accepted one |
| **R3** | head is empty when the master holds an unacked result, or holds a seq the master isn't tracking | **gateway rebooted** — an unacked head cannot disappear, and an unknown seq has no other source; dispose as below |
| **R4** | result deadline expires; a poll still shows no result for that seq | command lost to a reboot or a gateway wedge; read sysinfo `boot_count` (FC04, cold path) to classify and count it; dispose |

**Disposition on reboot/loss** (unchanged policy from v2/v3, drilled in task
07): commands the *client* marked idempotent (queries, absolute SETs) are
resubmitted automatically; non-idempotent ones (UP/DOWN steps) are reported to
their submitter as `LOST` — whether they executed is unknowable and pretending
otherwise is how v1 corrupted itself. Results that existed only in the dead
gateway's response queue are reported as `RESULT_LOST` (the command ran; its
answer is gone).

**The one ambiguous corner** — an *ack-less* enqueue whose reply is lost (only
happens on a cold start or after idle, since steady state always piggybacks
acks): the master cannot use R1. It polls until either the predicted seq
appears as a result (**landed**) or the deadline passes with the response
queue observed empty (**never landed** — retention means a result cannot have
been and gone). Before resending a **non-idempotent** command down this path
it reads sysinfo `boot_count`: unchanged ⇒ the command genuinely never landed,
resend is safe; changed ⇒ dispose as `LOST`, because "executed, then the
result died with the reboot" cannot be excluded. Idempotent commands just
resend. This is the entire residual cost of dropping the hot-path boot byte:
one cold-path register read, on a double-fault, for one command class.

**Controller reboot:** drain the slave — poll, ack whatever head appears,
repeat until empty — then start fresh. Stale acks from the dead controller
epoch are harmless by the mismatch-is-a-no-op rule.

---

## 5. Client policy

Transport (`mbq_client`) and scheduler (`dali_queue`) split as today.

- **Single outstanding per slave.** Every transaction is resolved — by reply,
  or by the §4 procedure — before the next is issued to that slave. The
  prediction rule and R1 depend on it.
- **Round-robin across slaves with work pending** (fairness is the
  scheduler's, not the protocol's). With per-slave single-outstanding this
  bounds any slave's wait to one transaction time per competing slave.
- **Pace, don't spin.** On `A_FULL`, or when awaiting results with an empty
  backlog, the next transaction for that slave is scheduled at the estimated
  completion of its oldest in-flight command (~25 ms SET / ~40 ms query /
  ~50 ms DTR0-pair), not immediately. Spinning wastes shared bus (measured
  ~2.4 polls/frame in the FC23 soak) even when it starves nobody.
- **In-flight cap = `CMDQ_DEPTH`.** The client tracks accepts minus results
  and stops submitting at the cap — `A_FULL` is the backstop, not the flow
  control (task-07 lesson).
- **Ack discipline: always piggyback.** Every request carries the ack for the
  most recent result the master has consumed. This keeps response-queue
  occupancy at ~2, keeps R1 available (the recovery shortcut needs a
  piggybacked ack), and makes the ack-less corner rare by construction.
- **No transport auto-retry of enqueue frames.** Timeout → §4. Poll/ack-only
  frames may retry freely (idempotent by construction — re-acking a popped
  seq is a no-op).
- **Failure delivery verbatim.** `H_FAILED` + code goes to the submitter's
  callback; retry policy belongs to the scheduler's priority classes (e.g.
  one retry for an idempotent query on `BUSFAULT`, never for UP/DOWN), not to
  the transport.
- **Dead-slave backoff** (task-07 finding): consecutive transaction timeouts
  put the slave on exponential poll backoff — the dead relay board was
  costing ~8 % of the bus in retry timeouts.

---

## 6. Diagnostics (cold path)

Nothing rides the hot-path reply beyond §1. Instead:

- **Gateway**, in the existing FC04 diag/sysinfo space (`0x0300`/`0x0310`
  blocks): the established `boot_count`, `reset_reason`, `uptime`, plus a new
  queue counter block — accepts, executes, results-pushed, acks, `A_FULL`
  replies, respq-full stalls (the backpressure rule engaging), discarded
  malformed frames. All 16-bit wrapping counters, read on demand (CLI /
  bench), never polled hot.
- **Controller**, in `mbq` CLI stats per slave: reboots inferred (by rule
  R2/R3/R4 separately — the split tells you *how* reboots are being noticed),
  recovery polls, R1 vs R1′ outcomes, deadline expiries, `LOST`/`RESULT_LOST`
  dispositions, `A_FULL` counts, round-trip times (the existing
  `mbq_client_txn_stats` machinery carries over).

The soak's verification does not depend on hot-path observability: every probe
command carries its own expectation (task-07 design), and the counters above
are enough to localise any discrepancy after the fact.

---

## 7. Byte cost and bus duty at 19200

8N1, one byte = 521 µs of wire.

| transaction | request | response | total | wire time |
|---|---|---|---|---|
| enqueue, no DTR0 (+piggybacked ack) | 7 B | 7 B | **14 B** | **7.3 ms** |
| enqueue with DTR0 (+ack) | 8 B | 7 B | 15 B | 7.8 ms |
| poll / ack-only | 5 B | 6 B | 11 B | 5.7 ms |

Steady state is one enqueue transaction per DALI frame. Against measured DALI
frame times (task-07 soak: 25 ms SET, ~38 ms query, 71 ms per DTR0+query
pair), and ~2 ms per transaction of t3.5 + turnaround overhead (measured
24.6 ms bus occupancy on a 22.9 ms wire in the soak):

| traffic profile | FC23 (measured) | v3 draft | **v4** |
|---|---|---|---|
| wire per transaction | 14.6 ms | 10.9 ms | **7.3–7.8 ms** |
| transactions per frame | 2.04 | 1.0 | **1.0** |
| all-SET stream (25 ms frames) | >100 % (Modbus-bound) | ~52 % | **~37 %** |
| soak mix (71 ms pairs) | ~84 % | ~36 % | **~27 %** |
| **two saturated gateways, all-SET worst case** | impossible | marginal | **~74 % — workable; ~50–55 % on realistic mixes** |

Idle duty is ~0: with no backlog and no results outstanding there is nothing
to poll (the dirty-poll/sweep layers above have their own cadence).

**Projections, not measurements — this document's projections have been wrong
twice.** The soak gate in §11 is the truth test, and the two-gateway row is
the one to re-check on hardware once the second bus exists.

---

## 8. Versioning and rollout

No capability probe: v4 removes the fallback path (see §9), so controller and
gateway are a **matched set**, as every queue generation has been. The
gateway's version register (FC04 `0x0200`) identifies firmware; the controller
checks it at discovery and refuses — loudly, per slave — to drive a
non-matching queue generation rather than limping (a v4 request to a draft-1
gateway would parse as a different frame shape; the reply-length mismatch is
detected and the slave is marked incompatible, surfaced in `mbq` stats and the
log).

Rollout: bench flash both sides together (`bench_flash_ota.sh` + UPDI),
validate per §11, then one production maintenance window for controller + all
gateways (wall switches dead during the window — schedule accordingly).

---

## 9. What v4 removes

All of it deliberate:

| removed | why |
|---|---|
| FC23 queue path (submit window `0x1000`, slot status block `0x1010`, `submit_window()`, FC23 handler) | matched-set deploys make a live fallback worthless, and a second queue stack is a second set of semantics to test and keep correct — the v1/v2 bug history is entirely in reconcile paths that were rarely exercised |
| draft-1 FC65/FC66 handlers (`accepted_slot`, snapshot-before) | superseded by this design on the same function code |
| v2 slot model (`mbq_slots` slot lifecycle, `MBQ_S_*` on the wire, per-slot status registers) | replaced by command FIFO + response queue |
| v3 draft's displaced register, reported bit, boot byte, FC66 | subsumed: retention-until-ack is strictly stronger than the displaced register; reboot detection moved to rules R2–R4 |
| capability negotiation | no fallback to negotiate; version register instead |

FC03/FC04 remain for sysinfo, diag, version, and config registers — untouched.

**Budget (measured baseline, `88f455a`): flash 11 028 / 16 384 B, RAM 137 /
1024 B.** Task 05's strip left far more headroom than the task-04-era "~800 B"
figure suggests, so nothing here is flash-forced. Expect flash roughly flat to
slightly down (FC23 handler + slot machinery out, FIFO + counters in) and RAM
about level: 12 B command queue + 8 B response queue + ~14 B counters against
the 24 B of `s_slot[3]` retired. Both are gated in the plan below — a
*regression* is a finding, but there is room to absorb one if it is justified.

---

## 10. Implementation plan

Matched-set change; land as one task (gateway + controller + soak), bench
first. Branch per repo, checkpoint commits per stage, push per stage
(teammates audit via forgejo origin).

### Stage G — gateway (TinyModbus)

**G1. Queue core** — rework `mbq_slots.{c,h}` into the v4 core (suggest
renaming to `cq.{c,h}` in the same commit so the old slot API cannot be
half-used):

- `cmdq[CMDQ_DEPTH]`: `{cmd[4], seq, state}` — states FREE / PENDING /
  EXECUTING only (terminal states no longer exist; completion moves data to
  the response queue and frees the slot).
- `respq[RESPQ_DEPTH]`: packed `{[hstatus:3|seq:5], resp8}` ring, head/count.
- Seq counter 1–31 skipping 0; `cq_accept(cmd) → seq | FULL`;
  `cq_ack(seq) → popped?`; `cq_head(out2)`; arbiter hooks `cq_next()` /
  `cq_begin()` / `cq_complete(seq, status, resp8)`.
- **Backpressure**: `cq_next()` returns none unless `respq` has a free entry.
- **Publication ordering** per §2 (bytes before single-byte publish, both
  directions); keep the comment that marks it load-bearing.

**G2. Modbus handler** — in `modbus.c`: FC 0x41 with the three-length
validation table (5/7/8 + flag consistency; violations discarded silently,
counted); processing order ack → enqueue → build; reply shapes 6/7 B. Delete
FC23 (`process_fc23`, `submit_window`), draft FC65/66 handlers, and the
`0x1000`/`0x1010` register ranges (now ILLEGAL DATA ADDRESS, consistent with
every previous retirement). Add the §6 counter block in the diag space.

**G3. DALI adapter** — `dali_sync.c`: consume `cq_begin()`'s 4-byte command
(same `QF_*` semantics, unchanged), completion calls
`cq_complete(seq, H_OK/H_FAILED, resp8)`. Mechanical.

**G4. Budget gate** — `avr-size` before/after for flash and BSS; recompute
stack margin. Expected: flash down, RAM down ~40 B. A regression here is a
finding, not a footnote.

### Stage C — controller

**C1. Transport** — rewrite `mbq_client.{c,h}` around a per-slave FSM:
`IDLE → AWAIT_REPLY → (resolve | TIMEOUT → RECOVERY_POLL → resolve |
DEADLINE_WAIT)`. Keeps the existing public API shape (submit with flags +
idempotent marker + callback/ctx; `mbq_client_inflight()`;
`mbq_client_txn_stats()`) so `dali_queue` consumers are untouched. New
internals: predicted-seq tracking, piggyback-ack bookkeeping, rules R1–R4,
result-deadline timers, the sysinfo `boot_count` gate, per-slave
version/compat latch (§8). Delete the FC23 submit/reconcile
(`resolve_ambiguous` and friends) — v4's reconcile is R1–R4 and nothing else.

**C2. Scheduler** — `dali_queue.c`: `DQ_INFLIGHT_MAX` := `CMDQ_DEPTH` (2);
round-robin across slaves with pending work; pacing timer per §5. Priority
classes, atomic DTR0 pairs, and the submit-side API are unchanged.

**C3. Debug knob for drills** — `mbq drop-reply <slave> <n>`: discard the
next *n* parsed replies from that slave inside `mbq_client` (after transport
accounting, before resolution), forcing the §4 paths deterministically.

**C4. CLI/stats** — extend `mbq` per §6; `dq`/`busload`/`soak` carry over.

### Stage V — validation (gates for sign-off)

Reusing task-07 instrumentation:

1. **Soak**, DTR0 stride-97 probe, depth sweep {1, 2, 3}: 0 bad / 0 lost /
   0 lag-1 over ≥ 5000 pairs, **transactions-per-frame ≈ 1.0 at depth 2**
   (the number this design exists to hit), pair rate ≥ 14/s (FC23 baseline;
   DALI ceiling ~15.9). Depth 3 run decides whether `CMDQ_DEPTH` moves.
2. **Reboot drill**: `gw_reboot` mid-soak at full rate — clean verdict; the
   reboot noticed by R2/R3/R4 (stats show which); non-idempotent disposition
   logged; verify against sysinfo `boot_count` that no reboot went uncounted.
3. **Lost-reply drill** (`mbq drop-reply`): (a) drop on an enqueue+ack →
   verify R1/R1′ resolve with zero resends of landed commands and zero double
   executions (DALI-level check: the probe's expectations catch any); (b)
   drop on an ack-less first enqueue → verify the deadline-drain path and the
   boot_count gate.
4. **Backpressure test**: soak variant that withholds acks for N transactions
   → respq fills, arbiter stalls (counter shows it), nothing lost, drain
   recovers to full rate.
5. **Malformed-frame test**: bench sender emits the 6-byte and
   flag-inconsistent lengths → discarded + counted, no reply, no state
   change.
6. **Duty measurement**: `busload` occupancy at saturation vs the §7
   projection (~37 % all-SET, ~27 % soak mix); re-run the two-gateway row
   when the second bus exists.

### Post-validation

Move the task doc to `tasks/done/`, update both READMEs (register map, this
protocol summary, removed ranges) and `DESIGN-HISTORY.md` (v2 slot model and
the v3 draft join the history), PR via forgejo.

---

## 11. Decisions log (2026-08-09)

| decision | call | rationale |
|---|---|---|
| reboot detection | timeout + rules R2–R4, no hot-path boot byte | command durations are well known; the two structural rules close the inference holes; sysinfo `boot_count` arbitrates the one double-fault corner off the hot path |
| response carries two sequences | accept byte (3+5) + head entry (3+5+8) | one 5-bit field cannot carry both the enqueue's fate and the head's identity; the split also makes lost-transaction recovery atomic (R1) |
| queue depth | 2, build-time constant | "current + next" is the requirement; depth 3 is a one-line change if the depth-2 soak shows DALI starvation; the wire is depth-agnostic |
| starvation | controller round-robins slaves with work | fairness is scheduler policy, not protocol; pacing retained for bus economy only |
| observability | cold-path diag registers + CLI stats | hot-path bytes are the scarce resource; the soak's self-verifying probe doesn't need wire-level visibility |
| FC23 queue fallback | removed | two queue stacks don't fit 16 KB flash; matched-set deploys make a live fallback worthless |
| function code | 0x41, redefined | 0x40 is outside the MODBUS user-defined ranges (65–72, 100–110); draft-1 FC65 never shipped in a controller |
