# DALI command queue — Modbus contract (v4)

Authoritative specification of how the controller (RP2040 master) drives the
TinyModbus gateway's DALI command queue.

> **Lineage.** v4 is the synthesis of Bruce's queue language — single FC,
> ack-by-seq, response retention — with the failure-handling rules of the v3
> draft (see git history of this file). Those proposals lived in
> `ALTERNATE-QUEUE-APPROACH.md`, deleted once this document became their
> answer. Decisions taken 2026-08-09: timeout-based reboot detection (no boot
> byte on the hot path), a two-sequence response, queue depth 2 as a tweakable
> constant, round-robin fairness (since moved into the transport), diagnostics in cold-path
> registers rather than the reply.
>
> **Status: IMPLEMENTED AND VALIDATED** on the bench, 2026-08-09/10, on
> `task/08-queue-v4` in both repos (gateway `TinyModbus`, controller
> `switch_gateway_firmware`). FC23 and the draft FC65/FC66 are gone. Measured
> results and the regression checklist are in [§10](#10-validation); the
> blow-by-blow of the build lives in `tasks/08-queue-v4.md`.
>
> **Read [§5](#5-client-policy) before touching the client.** Its pacing rule
> was got wrong once, then removed entirely: `A_FULL` is the flow control now,
> not an anomaly, and the estimator that policed admission is gone. The section
> records both the rule and why it no longer applies.
>
> Where this document and the code disagree, this document is the intent and
> the code is the bug — except where §10's measurements say otherwise.

---

## 0. The model in one paragraph

One user-defined function code carries all queue traffic. A request optionally
**enqueues** one DALI command and optionally **acks** (deletes) the head of the
gateway's **response queue**; the reply reports the fate of the enqueue (an
**accept byte** with a gateway-assigned sequence) and the current **head** of
the response queue. Results are retained until explicitly acked, so no reply
loss can destroy one. Each transaction enqueues one command, acks the previous result, and collects
the next, in **one ~14-byte transaction**. A DALI frame therefore costs a
*minimum* of one transaction — but the client does not try to hit that minimum:
it re-offers on `A_FULL` rather than pacing against an estimate, so a saturated
gateway costs ~3.6 transactions per command, most of them refusals that mean
"the DALI engine is full". See §5 for why that trade was taken and §10 for the
capacity it leaves. Reboots are detected by result deadlines plus two structural
rules (seq collision, impossible-empty-head), with the existing sysinfo
`boot_count` register as the cold-path arbiter for the one genuinely ambiguous
corner. The gateway holds nothing the master cannot reconstruct.

Both queues are small and bounded:

| | depth | RAM | notes |
|---|---|---|---|
| command queue | `CMDQ_DEPTH` = **2** | 2 × 6 B | 1 executing + 1 pending; build-time constant. Raising it is **not** a one-line change — see the depth note below |
| response queue | `RESPQ_DEPTH` = **4** | 4 × 2 B | results held until acked; steady-state occupancy is 2 |

**Backpressure rule (load-bearing):** the arbiter may not start a command
unless the response queue has a free entry. A full response queue stalls DALI
execution rather than ever dropping a result. Because the ack in a request is
processed *before* the enqueue, a single transaction can free an entry and
refill the pipeline.

> **Measured note on the two depths.** A well-behaved client cannot fill the
> response queue: its in-flight cap bounds unacked results at `CMDQ_DEPTH + 1`
> = 3, against `RESPQ_DEPTH` 4. That is why the hardware backpressure drill
> comes back with the gateway's `stalls` counter at zero — the rule is real and
> correct, but it is a safety net for a misbehaving master, not a working part
> of the steady state. It is covered by `cq_test`'s `test_backpressure`.
>
> **Consequently `CMDQ_DEPTH` = 3 is not a one-line change.** It raises the
> bound to 4 and consumes the entire margin, so `RESPQ_DEPTH` must go to 6 in
> the same commit. Note also that **nothing on the wire carries the depth** —
> §1 is deliberately depth-agnostic — so a depth bump applied to one side only
> ships silently and presents as an unexplained rise in `a_full` against a
> gateway that is not actually saturated. Recommended before
> anyone touches it: publish `CMDQ_DEPTH` in the queue-counter block's spare
> register (`0x0327`, currently documented as "reads 0") and have the
> controller latch incompatible on a mismatch at discovery, exactly as §8 does
> for the version. Two bytes of gateway flash, no wire change, and zero
> degrades cleanly as "older gateway, depth unknown".

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
| `QW_TWICE` | 7 | send the forward frame twice (SET-class); no answer expected |
| `QW_DTR0` | 6 | emit `0xA3 <dtr0>` back-to-back **before** the frame; the `dtr0` byte is present |
| `QW_ANSWER` | 5 | a backward frame is expected; capture the answer byte |

> **`QW_*` are the WIRE positions and are deliberately not the internal `QF_*`
> bits the DALI adapter tests.** The gateway's parser normalises between the
> two (`queue_flags_to_qf` in `modbus.c`), which is what keeps the wire layout
> from leaking into the issuance code. Naming them `QF_*` here — as an earlier
> revision of this table did — invites exactly the confusion the split exists
> to prevent. Definitions: `TinyModbus/src/fc41.h` (wire) and
> `controller/src/mbq_client.h` (`MBQ_QF_*`, the master's copy of the same wire
> positions); those two must agree, and nothing on the wire checks it.

`ack` (bits 4–0): if non-zero, delete the head of the response queue **iff**
the head's seq equals this value. A mismatch is a silent no-op — the reply's
head field shows the master what the real head is, which is all it needs to
re-converge.

Three request lengths, distinguished — and validated — by length + flags:

| length | meaning | validity rule |
|---|---|---|
| **5 B** | ack and/or poll, no enqueue | flags must be `000` |
| **7 B** | enqueue without DTR0 preload (+ optional ack) | `QW_DTR0` must be clear |
| **8 B** | enqueue with DTR0 preload (+ optional ack) | `QW_DTR0` must be set |

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
| `hseq` ≠ 0, `hstatus` = `H_OK` (0) | command `hseq` did what it was asked; `resp8` = answer byte (`QW_ANSWER`), or 0 when no answer was expected |
| `hseq` ≠ 0, `hstatus` = `H_FAILED` (1) | command `hseq` broke; `resp8` = failure code below |
| `hseq` ≠ 0, `hstatus` = `H_NAK` (2) | command `hseq` **expected** a backward frame and got silence; `resp8` = 0 |
| `hstatus` 3–7 | reserved |

| failure code | value | meaning |
|---|---|---|
| *(`0x01` reserved)* | `0x01` | was `DALI_RESP_NAK`; retired 2026-08-13, see below |
| `DALI_RESP_COLLISION` | `0x02` | garbled backward frame — two or more gear answered |
| `DALI_RESP_BUSFAULT` | `0x03` | transport fault: short pulse, stuck bus |

**`H_NAK` is a fact, not a verdict, and that is the whole point.** Silence on the
DALI bus means different things depending on what was asked, and the gateway
cannot know which:

- a **yes/no query** (COMPARE, QUERY LAMP FAILURE) answers *yes* with a backward
  frame and *no* by staying silent — so silence **is** the answer;
- a **value query** (QUERY ACTUAL LEVEL) that hears silence got no answer at
  all, which is evidence the gear is absent;
- a command with **no `QW_ANSWER`** never expected a reply, so silence means
  nothing happened that should not have. That is `H_OK`, not a NAK — this is
  the case the old encoding got wrong.

Only the master knows which question it asked, so the gateway reports the event
and the master renders the verdict (`dq_yes_no()` on the controller side). Note
that for a yes/no query a `DALI_RESP_COLLISION` is also *yes*: two or more gear
answering at once is the normal case in the commissioning bisect, and a caller
that treats it as an error breaks the binary search. A `DALI_RESP_BUSFAULT` is
neither yes nor no — nothing was learned.

Before 2026-08-13 there was no `H_NAK`: both cases above arrived as `H_FAILED`
carrying `DALI_RESP_NAK`, so every fire-and-forget SET was reported as a
failure, the failure counters counted normal outcomes, and "absent" and "said
no" were distinguishable only by the submitter remembering which flags it had
set. `0x01` stays reserved rather than reused so a mismatched old master decodes
an unknown code instead of a wrong one.

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
| **R1** | reply to an enqueue+ack lost → recovery poll shows the head **no longer holding the acked seq** (and the reboot tests below do not fire) | the ack applied, therefore the whole frame **landed**; bind the command to the predicted seq, do **not** resend |
| **R1′** | same, but the head **still holds** the seq the master tried to ack | the ack did not apply, therefore the frame **never arrived**; resend it verbatim — ack included — with no idempotency concern |
| **R2** | an accept byte returns a seq the master still tracks | **gateway rebooted** (§2 proof); dispose all tracked commands (below), keep only the just-accepted one |
| **R3** | the head holds a seq the master is not tracking, or the head is empty while the master holds a consumed-but-unacked result **and has no ack unresolved** | **gateway rebooted** — an unacked result cannot disappear, and an unknown seq has no other source; dispose as below |
| **R4** | result deadline expires; a poll still shows no result for that seq | command lost to a reboot or a gateway wedge; read sysinfo `boot_count` (FC04, cold path) to classify and count it; dispose |

> **A deadline expiry has a THIRD cause, and R4 must not mistake it for a
> wedge: the backpressure rule doing its job.** If the master has stopped
> acking, the gateway deliberately stalls its arbiter (§0) — nothing is
> executing, every outstanding deadline expires, and R4 would resend a command
> the gateway is still holding. That is a **double execution of a real DALI
> frame, reported to nobody**, and it was reproduced. The master cannot see the
> stall in the reply (the gateway's `stalls` counter is cold-path), but it does
> not need to: it knows whether the head it is looking at is one it has
> consumed and failed to ack. **While the head is known-blocked, push the
> deadlines out.** The 150 ms figure above assumes a gateway that is executing;
> a stalled one is not late, it is waiting for you.

### The lost-reply procedure, in evaluation order

R1/R1′ and R3 can look at the *same* observation and disagree, so the order is
part of the contract. On a timed-out request that carried an ack, poll once and
take the **first** matching branch:

1. head **still shows the acked seq** → **R1′**: never arrived, resend verbatim.
2. head shows a seq the master is **not tracking** → **R3**: rebooted.
3. head is **empty** → *ambiguous*: "the ack applied and the queue happened to
   drain" and "a reboot wiped the queue" are indistinguishable from the head
   alone. **Read sysinfo `boot_count`** (cold path, one extra transaction on a
   path that is already exceptional): changed ⇒ **R3** reboot; unchanged ⇒
   **R1** landed.
4. head shows a **different seq the master is tracking** → **R1**: landed
   (a reboot would have emptied the queue, so a tracked seq proves the epoch
   survived). Deliver that result as usual.

**The predicted seq is a prediction, and it is verified.** R1 binds the command
to `last_accepted + 1` (skipping 0). When that result surfaces, its seq must
equal the prediction; a mismatch is a **detected fault** — count it, log it, and
dispose of the command per policy. It must never be silently accepted, because
a silent mis-binding is precisely the result-misattribution class that took
task 04 three fix cycles.

> **Why R1 may assume the enqueue was *accepted* rather than `A_FULL`.** The
> two are indistinguishable once the reply is lost. `A_FULL` is rare — but an
> earlier draft of this box claimed it was **unreachable**, and R1's safety was
> rested on that. **The claim was false, and this correction is the important
> part**: a safety argument that is wrong while the thing it justifies happens
> to survive for unrelated reasons is exactly how a later refactor quietly
> removes the real protection.
>
> What is actually true:
>
> - On the **queue-full path**, `A_FULL` is unreachable *if* the client's
>   in-flight cap is a strict accepts-minus-results count. It isn't: §5's cap is
>   measured against *estimated* gateway occupancy, because a strict count makes
>   the combined collect-and-enqueue transaction impossible (the oldest result
>   is uncollected at the instant the request is built, so the cap is always
>   reached and you are back to two transactions per frame). **An early estimate
>   therefore reaches `A_FULL` legitimately.**
> - There is a **second path entirely**: the gateway's `cq_accept()` reports
>   `A_FULL` if its ring cursor and slot state disagree — an invariant break the
>   client's cap cannot see or prevent. The gateway now raises `tm_fault()` for
>   that case and keeps it out of the `a_full` counter, so the counter means one
>   thing again.
>
> **R1's real protection is not this argument.** It is R2 (a phantom seq is
> still tracked, so the accept byte trips the reboot rule) plus the prediction
> check (§4's "the predicted seq is verified"). Both were confirmed on the
> bench to catch a command bound to a seq the gateway never assigned.
>
> `A_FULL` remains an **anomaly worth shouting about** — count it, log it — but
> read it as "the estimate ran early", not "the bookkeeping is broken". Zero
> were observed across the throughput and backpressure runs; two appeared
> across a reboot drill.

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

**Controller reboot: drain, wait, drain again.** Poll and ack whatever head
appears until the queue reads empty; then **wait one `RESULT_DEADLINE_MS` and
drain again** before submitting anything. The second pass is not belt-and-
braces. A command accepted by the gateway before the controller died is still
executing, and its result will land in the response queue *after* the first
drain — where the fresh controller, tracking nothing, would read it as "a seq I
am not tracking" and fire R3, declaring a gateway reboot that did not happen
(and disposing of whatever it had just submitted). The wait bounds this: the
gateway holds at most `CMDQ_DEPTH` commands and each completes within a frame
time, so nothing from the dead epoch can survive `RESULT_DEADLINE_MS` of
draining. A response queue that was *full* when the controller died stalls the
arbiter, but the first drain unstalls it, which is why the wait comes after.

Stale acks from the dead epoch are harmless by the mismatch-is-a-no-op rule.

**Before the first accept byte is observed there is no baseline, and the master
must not pretend otherwise.** The gateway's counter is wherever its uptime left
it; a freshly started controller cannot know it. So until a slave's first
accept byte arrives, that slave is in a **no-baseline** state where:

- prediction is unavailable, so a lost reply to that first enqueue resolves
  conservatively — idempotent commands resend, non-idempotent ones are disposed
  `LOST`. (Once per controller start, and only if that one reply is lost.)
- a head bearing an untracked seq is **consumed and discarded**, not read as
  R3. It is a leftover from the previous controller epoch, which is the norm
  here, not evidence of a gateway reboot.

The state ends at the first accept byte. In practice the drain-wait-drain above
usually ends it sooner, and it costs nothing when it doesn't.

**The predicted seq is derived from the last *observed* accept, never from a
locally maintained counter.** The two differ after a reboot the master did not
notice: the gateway's counter restarts at 1 while a local mirror would run on.
Taking the prediction from the last accept byte actually seen makes an
unnoticed reboot self-correcting — the first normal reply re-baselines it — and
is why an unnoticed reboot with an empty queue is harmless rather than a latent
mis-binding.

**Not every gateway reboot is detected by these rules, and that is correct.** A
reboot with nothing in flight and nothing unacked destroys nothing, so no rule
fires and none needs to. Reboot *counting* for diagnostics is a separate,
already-existing mechanism: `dirty_poll` reads sysinfo `boot_count` every
`gw_diag_interval_ms` (30 s default) and publishes it to HA. The §4 rules are
the safety *detector* for in-flight work; the diag poll is the *counter*. Do
not conflate them, and do not add a hot-path reboot field to serve the counter
— that was v3's `boot8`, and this is why v4 does without it.

---

## 5. Client policy

Transport (`mbq_client`) and scheduler (`dali_queue`) split as today.

- **Single outstanding per slave.** Every transaction is resolved — by reply,
  or by the §4 procedure — before the next is issued to that slave. The
  prediction rule and R1 depend on it.
- **Round-robin across participants, at the TRANSPORT.** Fairness is no longer
  the scheduler's job. The bus asks each participant — relay boards and DALI
  gateways alike — in rotation whenever it falls idle with nothing committed,
  and the first with work gets the wire (`modbus_register_offer`). With
  per-slave single-outstanding this bounds any participant's wait to one
  transaction time per competing participant.

- **~~Pace, don't spin.~~ SUPERSEDED 2026-08-12.** This section used to require
  the client to schedule its next transaction at the *estimated* completion of
  its oldest in-flight command, and to treat `A_FULL` as unreachable. Both are
  now wrong, by deliberate decision:

  - **`A_FULL` IS the flow control.** It is not an anomaly and not a canary. A
    refusal means the gateway's queue is full, i.e. the DALI engine is fully
    fed — the goal state. The client re-offers immediately. Under one saturated
    gateway ~72 % of transactions are refusals, and that share is the HEADROOM
    gauge: those probes convert into useful enqueues as gateways are added.
  - **The estimator is gone.** It was admission control based on a guess, and a
    command whose estimate had expired but whose result had not arrived still
    held a local slot — so the client ran out of slots and polled instead,
    which is how it reached 1.79 txns/command. Offer-based admission has no
    guess to get wrong.

  What is bought back: no guesswork, lower command latency (a retry lands
  within a transaction of a slot opening, where a deliberately-late estimate
  does not), and a much smaller client. What is paid: more Modbus transactions.
  That trade was taken knowingly — the bus has the capacity (see §10).

  > **This is not a micro-optimisation; it is the design.** At depth 2 the
  > client is at its cap essentially always, and a DALI frame (~35 ms) is about
  > three transactions long (~12 ms). A client that polls the moment it cannot
  > enqueue therefore burns two wasted transactions per frame — **measured
  > 2.04 transactions per command on the first v4 bench run, worse than FC23's
  > 1.73**, with the throughput and the verdict both perfectly fine. The whole
  > point of v4 is that ONE transaction collects the oldest result and submits
  > the next command; that only happens if the client *waits* for the result to
  > exist instead of asking whether it exists yet.
  >
  > In steady state a correct client issues **no polls at all**. A poll is for
  > recovery (§4) and for a result that is genuinely overdue — not for finding
  > out whether the gateway is done.

  Getting the estimate slightly **late** is much cheaper than early: late costs
  a little latency, early costs a whole wasted transaction and lands you back
  at 2 txns/frame. On a poll that finds nothing, back off further rather than
  re-polling at the same cadence.
- **No in-flight cap on the client.** The estimator-based admission cap this
  bullet used to describe is GONE with the pacing it enforced (see the
  retraction above) — the client submits until `A_FULL` refuses it. `A_FULL`
  IS the flow control now; there is no cap left for it to backstop.
- **Ack discipline: always piggyback.** Every request carries the ack for the
  most recent result the master has consumed. This keeps response-queue
  occupancy at ~2, keeps R1 available (the recovery shortcut needs a
  piggybacked ack), and makes the ack-less corner rare by construction.
- **No transport auto-retry of enqueue frames.** Timeout → §4. Poll/ack-only
  frames may retry freely (idempotent by construction — re-acking a popped
  seq is a no-op).
- **Outcome delivery verbatim.** `H_OK`, `H_NAK` and `H_FAILED` + code each
  reach the submitter's callback distinctly (`MBQ_R_COMPLETE`, `MBQ_R_NAK`,
  `MBQ_R_FAILED`). The transport interprets none of them — collapsing `H_NAK`
  into a failure would destroy information the producer cannot reconstruct,
  since only it knows whether silence meant *no* or *absent*. Retry policy
  belongs to the scheduler's priority classes (e.g. one retry for an idempotent
  query on `BUSFAULT`, never for UP/DOWN), not to the transport.
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

### Wire time is not the whole transaction

A pure byte count overstates the win, and this document has been burned by
exactly that kind of optimism before. **Measured on this rig, 2026-08-09,
FC23 baseline** (`soak 3 0 n=300 depth=3`, busload read at the finish):

```
RS-485 bus 99% busy   1039 txns   19.46 ms avg   turnaround avg 10.27 ms
1.73 transactions per command    14.71 pairs/s   0 failures
```

FC23's 28 bytes are only 14.58 ms of that 19.46 ms. The remaining **~4.9 ms is
fixed per-transaction cost** — gateway turnaround (~1.4 ms of thinking once the
request is on the wire), t3.5 silence, and the master's SM overhead. It does
not shrink when the frame does.

So a v4 enqueue transaction costs **~12.2 ms** charged, not 7.3:
`7.3 ms wire + ~4.9 ms fixed`. Per DALI frame, against 1.73 → 1.0 transactions:

| | FC23 (**measured**) | **v4 (projected)** | ratio |
|---|---|---|---|
| charged per transaction | 19.46 ms | ~12.2 ms | 1.6× |
| transactions per command | 1.73 | 1.0 | 1.7× |
| **bus time per DALI frame** | **33.7 ms** | **~12.2 ms** | **2.8×** |
| soak mix (~35.5 ms/frame) | **99 %** | **~34 %** | |
| all-SET stream (25 ms frames) | >100 %, Modbus-bound | ~49 % | |
| two saturated gateways, soak mix | impossible | ~68 % — workable | |
| two saturated gateways, all-SET | impossible | ~98 % — **marginal** | |

**Set expectations by the 34 %, not the 27 % a byte count suggests.** The honest
conclusion for the second DALI bus: v4 makes two gateways comfortable on
realistic traffic and still marginal if both stream pure SET-class commands at
maximum rate. That is a large improvement on "impossible" and it is not
unlimited headroom.

Idle duty is ~0: with no backlog and no results outstanding there is nothing
to poll (the dirty-poll/sweep layers above have their own cadence).

**A note on where the remaining time goes.** At ~10.3 ms turnaround measured
from request start, the gateway spends ~1.4 ms between the last request byte
and the first reply byte — plausibly one main-loop pass rather than parse cost.
If a future task wants more headroom, that fixed overhead is now the largest
single term in a v4 transaction (~40 % of it), and it is a gateway-side
scheduling question, not a protocol one. **Out of scope here; do not chase it
in this task.**

**Everything in the v4 column is a projection. The gate in §11 is the truth
test** — and the FC23 column beside it is real, measured on the rig this
change will be judged against.

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

## 10. Validation

Built and validated on the bench, 2026-08-09/10 — gateway `TinyModbus`
`task/08-queue-v4`, controller `switch_gateway_firmware` same branch, one DALI
gateway at slave 3 driving a ballast at short address 0, 19200 baud.

The probe is the task-07 soak (`soak` CLI): alternating `SET DTR0 = v` /
`QUERY CONTENT DTR0`, where `v` walks a stride-97 cycle so consecutive pairs
never share an expected answer and a one-command-stale result is
distinguishable from random corruption. Every command carries its own
expectation, so verification does not depend on delivery order.

### Measured, against the FC23 design v4 replaces

| | FC23 | v4 (2026-08-10) | **v4 today** |
|---|---|---|---|
| RS-485 occupancy, saturated | **99 %** | 37 % | **~95 %** (see note) |
| Modbus transactions per DALI command | 1.73 | 1.03 | **~3.6** |
| charged per transaction | 19.46 ms | 13.1 ms | **8.96 ms** (38400) |
| throughput (write/read pairs) | 14.71/s | 13.9–14.1/s | **14.5–14.8/s** |
| endurance | — | 5000 pairs / 10 000 frames, **0 bad**, 0 failures of 20 748 transactions |
| gateway flash / RAM | 11 028 B / 137 B | 10 462 B / 153 B |

**Read the last column with §5 in hand.** Occupancy went back up because the
client now re-offers on `A_FULL` instead of pacing against an estimate — and
that is not capacity lost. Only the *accepted* enqueues are obligatory work,
and each produces exactly one DALI frame: **29.3 accepts/s = 26 % of the bus per
saturated gateway.** The other ~72 % of transactions are refusals spending idle
bus time, and they convert into useful enqueues as gateways are added:

| gateways | accepts needed | bus used by useful work | revisit interval |
|---|---|---|---|
| 1 | 29.3/s | 26 % | 9 ms |
| **2** | 58.6/s | **53 %** | 18 ms |
| 3 | 87.9/s | 79 % | 27 ms |
| 4 | 117/s | 105 % — over | 36 ms |

A gateway slot frees every 34.1 ms, so the rotation must revisit each gateway
inside that window; at four the interval (36 ms) exceeds it, exactly where the
bus-time budget also runs out. **Three saturated gateways is the ceiling**, and
a second is comfortable — which is what this protocol existed to make possible.

### Regression checklist

Re-run these against any change to the queue on either side. They are gates,
not suggestions — three defects in this protocol's own bring-up had a *clean
verdict and full throughput* while being badly wrong, so **correctness alone
does not pass**.

1. **Accept rate, not transaction rate.** The gate is the gateway's `accepts`
   counter ≈ the DALI frame rate (~29/s saturated), and `executes` equal to it.
   `txns per command` is EXPECTED to be ~3.6 and `bus busy` ~95 % — high
   `a_full` is the DALI engine being fed, not waste (§5). What would be a real
   regression is `accepts` falling while `a_full` stays high, which means the
   client is being refused and not converting refusals into enqueues.
2. **Throughput.** ≥ ~14.5 pairs/s at 38400. Materially below that with the
   gateway queue NOT full means the client is failing to keep it fed; check the
   `pacing` breakdown in `soak` for whether it reports "queue full" (DALI-bound,
   correct) or "tx busy" (Modbus-bound).
3. **Endurance.** ≥ 5000 pairs, zero bad, zero transport failures.
4. **Gateway reboot** (`gw_reboot <slave>` mid-soak, several runs, varying
   where the reboot lands): zero mis-attributed values, zero stale reads, zero
   out-of-order. **One `RESULT_LOST` per reboot is correct** — a query that was
   executing when the gateway restarted has no answer any more, and §4 requires
   saying so rather than inventing one. The soak scores that as a failure; read
   the breakdown, not the verdict.
5. **Lost replies** (`mbq drop-reply`): clean, with `R1 landed` climbing and no
   resend of a command that actually landed. Note this can only ever exercise
   **R1** — the knob drops a reply the gateway already sent, so the frame always
   landed. **R1′ has no hardware coverage** and lives in the host test.
6. **Backpressure** (`mbq no-ack`): clean, nothing lost, full rate on resume.
   Expect the gateway's `stalls` counter to stay **0** — see §0: a client that
   honours its in-flight cap cannot fill the response queue, so this drill
   demonstrates the margin rather than the stall. The rule itself is covered by
   `test/cq_test.c`.
7. **Host tests**, both sides: `make test` in `TinyModbus`, and the `cc` line in
   the header of `controller/test/mbq_client_test.c`.

**Read `busload` immediately at the soak's finish.** The window keeps
accumulating, so a late read dilutes it badly — a 75 s gap once reported 25 %
for a run that was really 99 %.

## 11. Decisions log (2026-08-09)

| decision | call | rationale |
|---|---|---|
| reboot detection | timeout + rules R2–R4, no hot-path boot byte | command durations are well known; the two structural rules close the inference holes; sysinfo `boot_count` arbitrates the one double-fault corner off the hot path |
| response carries two sequences | accept byte (3+5) + head entry (3+5+8) | one 5-bit field cannot carry both the enqueue's fate and the head's identity; the split also makes lost-transaction recovery atomic (R1) |
| queue depth | 2, build-time constant | "current + next" is the requirement; depth 3 is a one-line change if the depth-2 soak shows DALI starvation; the wire is depth-agnostic |
| starvation | controller round-robins slaves with work | fairness is scheduler policy, not protocol |
| pacing | client waits for the oldest command to be due whenever it may not enqueue | **not "bus economy only" as first written — it is what delivers the one-transaction-per-frame property.** Measured: without it, 2.04 txns/command, worse than the FC23 it replaces |
| FC23 queue fallback | removed | matched-set deploys make a live fallback worthless, and a second queue stack is a second set of reconcile semantics to keep correct — where every v1/v2 bug lived. (Not flash-forced: measured headroom is 5.3 KB.) |
| observability | cold-path diag registers + CLI stats | hot-path bytes are the scarce resource; the soak's self-verifying probe doesn't need wire-level visibility |
| function code | 0x41, redefined | 0x40 is outside the MODBUS user-defined ranges (65–72, 100–110); draft-1 FC65 never shipped in a controller |
