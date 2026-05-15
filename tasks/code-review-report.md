# TinyModbus pre-merge code review

Branch: `task-1-flag-driven`
HEAD: `9b5bc38` (tag `task-1-complete`)
Reviewer: senior, single-pass, multi-hour read
Scope: every `src/*.{c,h}`, README spec sections, both task specs, comparison report

---

## 1. Executive summary

One reachable correctness bug at P0 (commissioning bisect terminates wrongly when
the lowest random address is odd: PROGRAM is issued without re-sending the corrected
SEARCHADDR, causing VERIFY-fail). One additional P0 candidate is a benign-looking
data-race on `pending_apply_mask` that I traced and concluded is *not* reachable —
flagged as "verified safe" rather than as a bug. Six P1 architectural / robustness
concerns, mostly around documented-but-still-worth-surfacing edges of the threading
model. Eight P2 quality findings (one stale comment, one technically-wrong
comment, several "looks subtle but is actually correct — confirmed" notes worth
leaving in the record). Three P3 simplification opportunities, each tiny.

Overall posture: the firmware is in good shape. The flag-driven refactor (task 1)
landed cleanly, and the threading model is documented at the top of `dali_sync.c`
and matches what the code does. The one P0 is the kind of bisect off-by-one a
single hand-trace catches; it almost certainly went unnoticed because the spec
documents only ran the trace on the YES-at-bb=0 branch. **Block merge on the P0;
the P1 / P2 / P3 are non-blocking observations.**

| Severity | Count |
|----------|-------|
| P0       | 1 (+ 1 verified-safe near-miss) |
| P1       | 6     |
| P2       | 8     |
| P3       | 3     |

---

## 2. P0 — correctness bugs

### P0-1: Commissioning bisect — PROGRAM issued without re-sending SEARCHADDR when bb=0 returns NO

**File:** `src/dali_sync.c`, lines 891–935 (the `COM_COMPARE` branch in `dali_sync_completion`).

**What's wrong.** When the bisect reaches `bisect_bit == 0` and COMPARE returns
NO, the handler sets bit 0 of `commission_state.search` (via `com_set_bit(bb)`)
and then transitions directly to `COM_PROGRAM_ADDR` — **without** rewinding
through `COM_SEARCH_H/M/L` to push the corrected search address out to the gears.
The gears' internal SEARCHADDR registers therefore still hold the value last sent
(bit 0 cleared). PROGRAM_SHORT_ADDRESS only programs gears whose `randomAddress
== searchAddress`, so the gear we just isolated (whose random has bit 0 = 1) is
**not** programmed. The subsequent VERIFY then NAKs, `err_streak` increments, and
after >3 ballasts in a row hit this same case the run aborts with status
`0x0303 ERR_VERIFY_FAIL`.

**Reachability.** Approximately 50 % of randomly-generated 24-bit DALI addresses
have bit 0 = 1 (`random` is uniformly distributed). For each ballast in the bus,
the bisect path through `bb == 0 NO` triggers iff the lowest remaining random has
bit 0 set — i.e. about half of all bisect rounds. So in practice every other
ballast in a typical commissioning run hits this. With `err_streak > 3` aborting
the run, a real-world full-bus commissioning of more than ~6 ballasts will almost
certainly trip the verify-fail abort.

**Evidence — the offending code:**

```c
case COM_COMPARE: {
    bool yes = (type != DALI_RESPONSE_NAK);
    uint8_t bb = commission_state.bisect_bit;
    if (!yes) {
        if (bb == 24u) { ... break; }
        com_set_bit(bb);                    // <-- restore the bit
    } else if (bb == 24u && ...exhaust...) { ... break; }

    // Common bisect advance: bb=24 → 23, bb>0 → bb-1,
    // bb==0 → bisect done (search holds the random_addr).
    if (bb == 0u) {
        commission_state.phase = COM_PROGRAM_ADDR;     // <-- BUG: skips SEARCH_H
    } else {
        commission_state.bisect_bit = bb - 1u;
        com_clear_bit(bb - 1u);
        commission_state.phase = COM_SEARCH_H;
    }
    more = true;
    break;
}
```

**Worked trace (single ballast with `random = 0x000001`).**

| bb | search before | SEARCH+COMPARE result | after restore/clear | next phase |
|----|--------------:|-----------------------|---------------------|------------|
| 24 | 0xFFFFFF      | YES                   | clear bit 23 → 0x7FFFFF | COM_SEARCH_H (bb=23) |
| 23..2 | (cleared each round) | YES (always; random=1 ≤ search) | progressively narrows search to 0x000003 | … |
| 1  | 0x000001      | YES (1 ≤ 1)           | clear bit 0 → 0x000000 | COM_SEARCH_H (bb=0) |
| 0  | 0x000000      | NO (random 1 > 0)    | set bit 0 → 0x000001  | COM_PROGRAM_ADDR — but **gear-side SEARCHADDR is still 0x000000** |

PROGRAM_SHORT_ADDRESS goes out, gear's random (1) ≠ gear's SEARCHADDR (0), gear
ignores. VERIFY fails. err_streak++. Loop retries the bisect — same trace —
err_streak hits 4 — `COMM_STATUS_ERR_VERIFY_FAIL`.

**What should happen.** When `bb == 0` and `!yes`, the corrected search needs to
be pushed to the gears before PROGRAM. The simplest fix is to transition to
`COM_SEARCH_H` (instead of `COM_PROGRAM_ADDR`) on the bb=0 NO path, but with a
"don't decrement bisect_bit, don't clear another bit" guard — i.e. just re-send
SEARCH_H/M/L with the restored value, then move to PROGRAM. Equivalent:
unconditionally re-send SEARCHADDR before each PROGRAM (slightly wasteful in the
YES case, where the gears already have the correct value, but harmless).

The python-dali `gear/sequences.py::Commissioning` reference handles this by
re-sending SEARCHADDR after every modification of the local search variable, not
just on the "narrow" branch.

**Severity rationale.** A reachable, deterministic correctness bug in the
headline new feature of this branch. Affects ~50 % of bisect rounds. Surfaces as
the run latching `ERR_VERIFY_FAIL` rather than completing, so the master gets a
clear error rather than silently-wrong addressing — but commissioning is the
whole point of the task. Block-merge on this one.

**Cross-check evidence the existing hand-traces missed this.** Task 2 outcome
(`tasks/done/2 - dali commissioning.md`, lines 240–245) lists five hand-traces.
Trace #1 ("3-ballast bus full commissioning") doesn't specify the random
addresses chosen and the comment-level walkthrough of the bisect chain is just
"24 SEARCH triplets + COMPARE rounds → PROGRAM …" — i.e. it did not actually
trace a NO-at-bb=0 case.

---

### P0-2 (verified safe, no change): apparent race on `pending_apply_mask`

**File:** `src/modbus.c`, lines 97 / 345–347 / 507–519 / 617 / 646.

**What I checked.** `pending_apply_mask` is set with a non-atomic OR
(`|= APPLY_X`) inside `note_runtime_config_write` (called from
`process_fc06`/`process_fc10`, main context, line 345–347), and read+cleared in
`apply_pending_config` (called from the `USART0_TXC_vect` ISR, line 646, and
from `modbus_process` itself for broadcasts, line 617).

The naïve concern is that the LDS / OR / STS in `|=` could be preempted by TXC
firing between LDS and STS — TXC clears the mask (sees old value, applies it),
the LDS-OR-STS resumes and writes the new bit back, but TXC has already fired
and the new bit will never be applied.

**Why it's safe.** TXC only fires after a complete USART0 transmission. The
firmware has not yet started transmitting at line 345 — `note_runtime_config_write`
runs *before* `start_tx` is called (process_fc06 calls regs_write_holding →
note_runtime_config_write → dispatch_action_reg → start_tx in that order; FC10
follows the same ordering). So no TXC ISR is pending while line 345 executes.

I verified this is the only path that writes `pending_apply_mask` after init.
The `|= APPLY_X` is therefore single-context. No fix needed — but worth a
one-line `// Single-writer (main); ISR only reads + clears.` comment so the next
reviewer doesn't have to re-derive it.

---

## 3. P1 — architectural / robustness concerns

### P1-1: Bus-shorted-during-commissioning produces an indefinite no-progress loop

**File:** `src/dali_state_wait_for_idle.c`, in concert with
`src/dali_state_idle.c::dali_idle_state_enter` and the OP_COMMISSION dispatch.

`bus_stuck_timeout` (the 200-Te safety escape) calls `dali_idle_state_enter()`
without clearing `dali_busy`. If the bus is still shorted at that point
(physical fault, external master holding the bus), `dali_idle_state_enter`
re-checks `dali_is_bus_active()` and re-enters `dali_wait_for_idle_state_enter`,
which re-enters `on_bus_shorted`, which arms a fresh 200-Te timer. We loop
indefinitely at ~83 ms per cycle, never sleeping (`main` is gated on `dali_idle
&& modbus_idle && !dali_busy` and `dali_busy` stays true).

`wdt_reset()` is called every loop iteration, so the WDT does not fire. Modbus
remains responsive; the master can issue an abort to 0x1014 (the abort flag is
checked at every `dispatch_current` boundary, which keeps firing). So the system
is recoverable, but if the master is unaware of the short, it will just see the
commissioning never completing and the slave never sleeping (small power impact
on a battery scenario; cosmetic on a wall-powered one).

Possible improvements (not changes I'm asking for, just naming the option):
- Have `bus_stuck_timeout` clear `dali_busy` and stamp `commission_state.status =
  COMM_STATUS_ERR_ABORTED` (or a new "bus stuck" subtype), so the master sees the
  fault.
- Increase the 200-Te bound so the loop is genuinely rare in practice (it
  probably already is).

This is documented obliquely by the "bus integrity flags" gap in the comparison
report (Section D, item 7) but the indefinite-loop interaction with commissioning
is new on this branch.

### P1-2: Multi-step writes don't re-arm SYNC_READ_FULL on bus-error mid-sequence

**File:** `src/dali_sync.c`, lines 760–785 (the `OP_WRITE_MINMAX` /
`OP_WRITE_POWER` / `OP_WRITE_FADE` completion branch).

On a bus-error mid-sequence (`type` is one of MALFORMED / PULSE_*), the handler
clears the write flag and calls `apply_retry_pending(idx, SYNC_READ_FULL)`. The
helper either marks the ballast absent (on second consecutive NAK) or sets
`SYNC_RETRY_PENDING | SYNC_READ_FULL`. Net effect: cache will be reconfirmed by
a follow-up full scan. ✓

But — `apply_retry_pending` checks `sync_flags[idx] & SYNC_RETRY_PENDING`. The
RETRY_PENDING bit is set/cleared by both the level-read path (lines 798, 810,
808) and this write path. If a level-read NAK previously set RETRY_PENDING, then
a write-sequence bus-error second-bumps it to "absent" — but the level-read NAK
had nothing to do with the write attempt. Consequence: a flaky bus that NAKs
level-reads will tip a write-sequence bus-error into a presence drop, even
though the write-sequence got one shot at it.

This is consistent with the spec's "two-NAK eviction" semantics, just slightly
indirect. Worth noting but not a defect.

### P1-3: `dali_sync_completion` writes `commission_state.status` (16-bit) from LVL1 ISR; main reads it from non-ATOMIC contexts

**File:** `src/dali_sync.c`, line 956 (and lines 919, 964, 990–991, 911–912);
read sites at `pick_work` line 456 (just `phase`, atomic), and `regs_read_holding`
line 186 (under TCB0+ATOMIC, OK).

`pick_work` reads `commission_state.phase` (1 byte, atomic). ✓
`regs_read_holding` reads `commission_state.status` (2 bytes) under TCB0=0 +
ATOMIC_BLOCK. ✓ Status is also read non-atomically inside `dali_sync_arm_commission`
at line 611 — but inside that function's TCB0+ATOMIC. ✓

What I want to flag for completeness is the COMPARE handler's read of
`commission_state.next_short_addr` at line 919 (combined with the bb==24
comparison): this is 1 byte, atomic. ✓ And lines 951–954 combine writes to
`next_short_addr`, `count`, `err_streak`, `status` — all in ISR context. Main
reading any of these would race the ISR. The only main-context read of
`next_short_addr` is inside `dali_sync_arm_commission`'s TCB0+ATOMIC. ✓

So all good — but the threading model deserves to call this out *explicitly* at
the top of `dali_sync.c`. The current "Threading model" comment lists `sync_flags`,
`dali_busy`, `current_op`, `scan_full_*`, `raw_cmd_*`, `group_write`,
`g_regs.raw_cmd_result`, and `g_regs.dirty[]` — but not `commission_state`. Add
it. (P2 in another taxonomy; promoting because it is the largest new mutable
state on this branch.)

### P1-4: `s_active.repeat--` from idle ISR is non-atomic but only-ISR-touched — verified safe

**File:** `src/dali_state_idle.c` line 49, `src/dali_sync.c` line 221.

`s_active.repeat` is written only from inside `issue_dali_command` (which runs
either from main inside `start_next_dali_work` BEFORE the bit-level state machine
is armed, or from ISR inside `dali_sync_dispatch_current`) and decremented by
`dali_result_received` from ISR. Concurrent main + ISR write would race. But:
- Main writes `s_active.repeat` only inside `start_next_dali_work`, after the
  ATOMIC + TCB0 fence has set `dali_busy = true; dali_idle = false`. Until
  `dali_transmitting_state_enter` is called, no ISR will fire that could write
  `s_active`.
- After the bit-level state machine is armed, `s_active` is read/decremented only
  from the ISR chain (LVL0 TCA0 and LVL1 TCB0). No main-context writes happen
  until completion releases `dali_busy`.

Net: single-writer at any instant, no actual race. Verified, no change.

### P1-5: `regs_write_holding` group-write rollback path leaves a tiny visible window

**File:** `src/registers.c`, lines 264–272.

The group-write path predictively updates the cache (lines 264–265), then calls
`dali_sync_arm_group_write`. If arming fails (race with another arm — another
master? actually impossible since `regs_write_holding` is single-threaded from
main, but documented as "paranoid"), the rollback writes the OLD bytes back
(lines 270–271). Between the predictive update and the rollback, an FC03 read
from another master path — which there isn't in this single-threaded firmware,
but **also a `dali_sync_completion` running concurrently** could see the new
values briefly.

Specifically, an in-flight OP_GROUP_BIT completion (line 866 onwards) reads
`group_write.pending_adds` / `pending_removes` and clears bits — but it never
reads `g_regs.group_mask[]` directly. So the predictive-update-then-rollback
window is invisible to the ISR. ✓

The only consumer of `g_regs.group_mask` from ISR context is
`apply_invalidation` (called from `issue_dali_command`, called from
`dispatch_current`, called from `dali_idle_state_enter` after the wait-for-idle
gap — i.e. ISR context). If the master writes a group register while a different
DALI work item is in flight (writing some unrelated group command), the
predictive-update-then-rollback sequence could briefly publish the new bytes
that an in-flight `apply_invalidation` would walk over to fan out the
invalidation. The rollback would happen after `apply_invalidation` had already
returned, so the in-flight invalidation might invalidate ballasts based on the
new group membership rather than the old. But the rollback only fires on a
race-with-itself which is impossible from main, so this branch is unreachable.
Verified safe.

### P1-6: `pick_work` reads `g_regs.raw_cmd_result` (2 bytes) without atomic protection

**File:** `src/dali_sync.c`, line 464: `if ((uint8_t)(g_regs.raw_cmd_result >> 8) == 0xFFu)`.

Although it's framed as a single-byte-equality test, `g_regs.raw_cmd_result` is a
16-bit volatile and the compiler may emit a 16-bit load. `pick_work` is itself
called inside `start_next_dali_work`'s TCB0+ATOMIC block (line 555), so LVL1 is
fenced and LVL0 ISRs cannot run during the load. ✓

I'm flagging this only because the dispatch-current path (line 335, OP_RAW
branch) uses an explicit `ATOMIC_BLOCK { rc_lo = (uint8_t)g_regs.raw_cmd_result; }`
and pick_work doesn't — the inconsistency makes it look like one of them is
wrong. Both are in fact correct (pick_work is already inside an ATOMIC; the
explicit ATOMIC_BLOCK at line 335 is needed because dispatch_current can be
called outside one). Add a one-line comment at line 464 noting that the read is
already ATOMIC by virtue of the caller's block.

---

## 4. P2 — code quality / consistency

### P2-1: `dali.h` comment on opcode 0xA6 is wrong

**File:** `src/dali.h`, lines 42–43.

> // (DALI opcode 0xA6 is reserved by IEC 62386-102 - never issue it.)

Per IEC 62386-102 ed.2 and python-dali `gear/general.py` lines 857–865, opcode
0xA6 is `QUERY MANUFACTURER SPECIFIC MODE` — not reserved. The decision to *not*
issue it from internal scans is correct (it's manufacturer-defined), but the
rationale documented in the comment is wrong. Already noted in
`tasks/dali-comparison-report.md` ("Honourable mentions"); calling it out here
to keep it on the to-fix list.

Suggested wording: "DALI opcode 0xA6 is `QUERY MANUFACTURER SPECIFIC MODE` per
IEC 62386-102; we don't issue it from internal scans because the response is
manufacturer-defined."

### P2-2: `OP_FULL_SCAN` clears `SYNC_RETRY_PENDING` on completion but never sets it

**File:** `src/dali_sync.c`, lines 837 and 846.

Both the success and abort branches do `sync_flags_clear(idx, SYNC_READ_FULL |
SYNC_RETRY_PENDING)`. The full-scan path never *sets* `SYNC_RETRY_PENDING` (it's
only set in the level-read and write-sequence paths via `apply_retry_pending`).
Clearing a never-set bit is harmless; just dead inclusion. Could be tightened to
`SYNC_READ_FULL` only, saving a token.

### P2-3: `pick_work` priority comment is slightly stale

**File:** `src/dali_sync.c`, lines 441–451 (the priority list comment) vs. lines
452–548 (the actual implementation).

The comment lists priority 4 as "SYNC_WRITE_*" and priority 5 as "SYNC_READ_LEVEL"
but the function comment block at line 9 still says "raw → group bit → SYNC_WRITE_*
→ SYNC_READ_LEVEL → full-scan continue → full-scan new" — i.e. it omits
commissioning, which is now priority 1. Add commissioning to the file-level
comment for consistency. (The body of `pick_work`'s docstring at lines 441–451
DOES list commissioning correctly.)

### P2-4: Threading model comment in `dali_sync.c` doesn't list `commission_state`

**File:** `src/dali_sync.c`, lines 19–32. (P1-3 above also flagged this.)

Add: `commission_state: written from main only inside dali_sync_arm_commission
and regs_read_holding (terminal-state clear) under TCB0+ATOMIC. All other
updates run from ISR context (dali_sync_completion / dispatch_current); 16-bit
status writes from ISR are LVL1, fenced against main reads by main's TCB0=0
around its ATOMIC blocks.`

This already exists as a comment block ON the `commission_state_t` struct
(`dali_sync.h` lines 137–139) — just propagate the same statement up to the
file-level threading model.

### P2-5: `frame_is_valid` still admits FC01 / FC05 / FC0F by size, even though they're never honoured

**File:** `src/modbus.c`, lines 478–484.

Documented in README "Known Limitations" (line 512); not a bug. Calling it out
here so it stays in the cleanup queue. The behavioural comment explains why the
current shape is preserved (graceful ILLEGAL_FUNCTION reply rather than a
size-check drop), which is a valid stance.

### P2-6: `apply_pending_config` could use a one-line comment about the single-writer property

See P0-2 above.

### P2-7: `regs_write_holding` group-mask byte indexing — verified, no change

**File:** `src/registers.c`, lines 254–265.

The byte indexing is `g_regs.group_mask[group_idx][word_idx * 2u]` for the high
byte and `[word_idx * 2u + 1u]` for the low byte. `word_idx` is 0..3 (one of
four registers per group). Each register exposes 16 ballasts, so the per-register
byte pair is `group_mask[g][word_idx*2]` (ballasts `word_idx*16..word_idx*16+7`,
high byte of the wire register) and `[word_idx*2+1]` (ballasts
`word_idx*16+8..word_idx*16+15`, low byte). This matches the layout used by
`regs_update_group_mask` (which writes `group_mask[g][ballast_idx >> 3]`).

The byte order — high byte first → low ballast indices first — is correct for
big-endian wire format. Verified, no change.

### P2-8: `OP_WRITE_LEVEL` always sets dirty bit even when level didn't actually change

**File:** `src/dali_sync.c`, lines 747–757.

The handler unconditionally calls `mark_dirty(idx)` on OK or NAK. This is
intentional per the task 1 outcome notes (a DAPC has no backward frame so we
can't tell whether the ballast actually moved; the master knows the cache is
stale and re-reads — the dirty bit is the signal). It is consistent with the
"every brightness write triggers a follow-up SYNC_READ_LEVEL" call-out in the
task 1 outcome (line 561). Just documenting that I checked.

---

## 5. P3 — simplification opportunities

### P3-1: `lowest_bit_set` returns 16 sentinel — could collapse to `__builtin_ctz`

**File:** `src/dali_sync.c`, lines 410–414.

`__builtin_ctz(mask)` is undefined for mask=0, so the existing pre-check
(`if (adds)` at line 480) covers it. Switching saves a few bytes of flash and is
a one-cycle bit instruction on AVR (well, plus the loop body — avr-gcc 7.3 may
or may not inline well, would need to measure). Low priority.

### P3-2: `write_op_from_flags` and `write_flag_for_op` are inverse maps and could be a single table

**File:** `src/dali_sync.c`, lines 417–434.

Two parallel switch statements maintaining the same `OP_WRITE_X ↔ SYNC_WRITE_X`
correspondence. A single PROGMEM table indexed by op or flag bit position would
collapse both. Saves a few bytes of flash. Tiny, only worth doing if you're
already in there.

### P3-3: `dali_sync_completion` `OP_FULL_SCAN` NAK and bus-error branches are identical — collapse

**File:** `src/dali_sync.c`, lines 832–850.

The OK branch advances `scan_full_step`; the else-branch (NAK or bus-error)
aborts the ballast's scan. The else-branch is one path, not two. Already as
collapsed as it can be. (Listing for completeness — already simple.)

---

## 6. Cross-checks performed

What I traced explicitly, cited line by line in this report:

- **Bisect bit walk**, sentinel `bisect_bit == 24` → 23 → … → 0, both YES and NO
  paths at every position. **Found bug at bb=0 NO** (P0-1).
- **VERIFY OK + 0xFF strict check**, including MALFORMED → fail-safe interpretation
  comment.
- **PROGRAM_SHORT_ADDRESS / VERIFY_SHORT_ADDRESS encoding** — both use
  `DALI_PROG_SHORT(next_short_addr)` = `(short<<1)|1`. ✓
- **Abort flag plumbing**: `regs_write_holding` 0x1014 = 0 → sets
  `commission_state.abort_requested` only if phase ≠ IDLE → checked at the top of
  the OP_COMMISSION branch in `dispatch_current`. ✓ Including the read-after-write
  ordering: phase is 1-byte atomic; `abort_requested` write is unconditionally
  safe.
- **0x1014 read-clears semantics** in `regs_read_holding` — TCB0=0 + ATOMIC_BLOCK,
  matches the 0x0108 raw-result pattern at `process_fc04`. ✓
- **SYNC_READ_FULL set after VERIFY-yes** at line 953 — confirmed.
- **`dali_sync_arm_commission`** TCB0+ATOMIC fence over the whole header check
  + state-init block. ✓ The wipe loop following has its own TCB0+ATOMIC. ✓
- **All `sync_flags` RMW sites**: every one outside `sync_flags_set` /
  `sync_flags_clear` is either an init (`sync_flags[i] = 0` inside the
  arm_commission wipe ATOMIC, line 663) or a read-only check (`sync_flags[idx]`
  in pick_work / `apply_retry_pending` / `note_present_on_ok`). The PIT ISR
  comment at line 1048 explicitly switched from `|=` to `sync_flags_set` — done.
- **`g_regs.raw_cmd_result` access pattern**: every write is in
  `dali_sync_completion` (LVL1) under explicit ATOMIC, every main-context read or
  RMW is under TCB0=0 + ATOMIC. ✓
- **`g_regs.dirty[]` access pattern**: `mark_dirty` runs in ISR context (single
  writer); `regs_read_clear_dirty` is the only main-context RMW and is under
  ATOMIC. ✓ Caveat: `mark_dirty` is `g_regs.dirty[word] |= bit` — not atomic
  against itself. But it's only ever called from the LVL0/LVL1 ISR chain, and
  LVL0 is fenced from re-entrant LVL0 by SREG-I being clear during the ISR. So
  mark_dirty from one LVL0 callback can't race mark_dirty from another LVL0
  callback. LVL1 (TCB0_INT_vect) doesn't directly call mark_dirty, but it can
  preempt LVL0 and call `dali_result_received` → `dali_sync_completion` →
  `mark_dirty`. Since the LVL0 caller of mark_dirty is also via
  `dali_sync_completion`, and `dali_sync_completion` is itself reentrant-from-ISR
  (it's called from both LVL0 wait-for-response timeout and LVL1 RX-error paths),
  there *is* a window where a LVL0 mark_dirty's load-or-store could be preempted
  by a LVL1 mark_dirty. **Found a near-miss; verified safe.** Why safe: the LVL1
  TCB0 RX completion only fires once per RX frame, and the ISR chain it triggers
  runs to completion (including its mark_dirty) before the LVL0 caller resumes.
  Whether the LVL0 caller's load was before or after the LVL1 store determines
  which dirty bits land — but the OR semantics are commutative; both bits end up
  set in the final word regardless of interleaving. ✓
- **`current_op` access pattern**: written by main (start_next_dali_work) under
  ATOMIC + TCB0=0; written by ISR (dali_sync_completion) without explicit
  ATOMIC. Read by ISR (`dispatch_current`) — same context as the writer. Read by
  ISR (`dali_sync_completion`) — same context. No main-context reads. ✓
- **Raw command ARM → DISPATCH → COMPLETION** sequencing: `arm_raw` sets
  `g_regs.raw_cmd_result = 0xFFnn` under ATOMIC; pick_work picks it up; dispatch
  reads the low byte under ATOMIC (line 335); the bit-level state machine sends;
  completion writes terminal `g_regs.raw_cmd_result` under ATOMIC (line 1018).
  All read-clear sites at FC04 0x0108 are TCB0=0 + ATOMIC. ✓
- **`dali_busy` write paths**: set inside start_next_dali_work's
  TCB0=0+ATOMIC; cleared inside `dali_sync_completion` (ISR). Main-loop reads
  `dali_busy` non-atomically (1 byte, atomic on AVR) at main.c:70 / 74 — single
  load, OK. ✓
- **`dali_idle` write paths**: set inside `dali_idle_state_enter` (ISR); cleared
  inside `cleanup` / `start_next_dali_work` (under ATOMIC). Read non-atomically
  at main.c:70 / 74 — single load, OK. ✓
- **`group_write` access pattern**: written under TCB0=0 + ATOMIC in the arm
  path; written by ISR (completion) without explicit ATOMIC. Main-context read
  (`dali_sync_group_busy`) is TCB0=0 + ATOMIC. ✓
- **`scan_cursor`, `work_cursor`, `tick`** (PIT ISR locals): `tick` is `static
  uint8_t tick = 0;` at function scope — only PIT-context, no race. `scan_cursor`
  is `static volatile uint8_t` at file scope — written only by PIT, read only by
  itself. `work_cursor` is plain (non-volatile) — only `pick_work` reads/writes
  it, and pick_work runs only from main inside an ATOMIC. ✓
- **`classify_invalidation` cases**:
  - DAPC (addr bit 0 = 0) → SYNC_READ_LEVEL ✓
  - data < 0x20 → SYNC_READ_LEVEL ✓ (matches step / scene / RECALL_MIN/MAX /
    GO_TO_LAST_ACTIVE / etc.)
  - data == 0x20 (RESET) → SYNC_READ_LEVEL | SYNC_READ_FULL ✓
  - data < 0x90 → SYNC_READ_FULL — over-invalidates `0x21
    STORE_CURRENT_LEVEL_DTR0` (documented), `0x22..0x29` reserved, `0x30..0x3F`
    reserved, `0x80 SET_SHORT_ADDRESS`, `0x81 ENABLE_WRITE_MEMORY`, `0x82..0x8F`
    reserved. Documented as "well within the existing rescan budget." ✓
  - data ≥ 0x90 → 0 (no invalidation; queries/specials). ✓

  No false negatives identified — every command that should invalidate hits at
  least one of the four ranges.

What I did NOT do, in priority order if a deeper review is wanted:

- **Hand-build the firmware.** I read the Makefile and the threading model but
  did not run `make build` to confirm it still compiles, nor did I inspect the
  `.su` files to confirm stack high-water mark is unchanged. The task 2 outcome
  reports stack at 49 B unchanged; I trusted that.
- **Static-verify all integer-arithmetic sites for overflow.** I spot-checked
  the FC04 / FC03 entry points (which already cast to uint32_t) and the
  commissioning addr-byte math. I did *not* exhaustively walk every
  `(uint16_t)addr + count` arithmetic site.
- **Check `regs_apply_result` REG_FADE_RATE nibble split for fade_time vs
  fade_rate.** The code at registers.c:336 does `fade_rate = value & 0x0Fu;
  fade_time = (value >> 4) & 0x0Fu;`. The DALI spec is:
  `QUERY_FADE_TIME_FADE_RATE` returns "high nibble = fade time, low nibble =
  fade rate". Verified against dali.h:42 comment ("high nibble = fade time
  (0..15), low nibble = fade rate (0..15)"). ✓ But I did not cross-check against
  python-dali's gear/general.py to be 100% sure.
- **DTR1 / DTR2 unreachable from internal sequences** — the `_DTR1` / `_DTR2`
  macros are gone (per the task 1 outcome). I verified `DALI_ADDR_SET_DTR1` and
  `DALI_ADDR_SET_DTR2` are not defined in `dali.h`; only `DALI_ADDR_SET_DTR0`
  remains. ✓
- **End-to-end trace of `incremental` mode commissioning where the bus has
  already-addressed gear** — the task 2 outcome says trace #2 covered this.
  I verified the lowest-unused logic at `com_lowest_unused` by code-reading,
  but did not re-walk the full trace.

---

## 7. Out-of-scope notes

- The comparison report (`tasks/dali-comparison-report.md`) lists eight major
  follow-on items: 24-bit forward frames (DALI-2 input devices), memory bank
  reads, DT8 colour, DT1 emergency, D4i diagnostics, scene config, group/broadcast
  Modbus convenience, on-bus collision detection during own-TX. These are
  intentionally future work — out of scope for this review and out of scope for
  the merge. Mentioning them only so the maintainer doesn't spend time looking.
- `frame_is_valid` admitting retired coil function codes by size (P2-5 above) is
  a known limitation in the README. Not a defect.
- The "WAIT_FOR_IDLE skipped after a backward-frame NAK" entry in the README
  Known Limitations is, as far as I can tell from tracing, *not* actually
  reachable in the current code: every NAK path goes through `dali_result_received`
  → `dali_sync_completion` → `dali_wait_for_idle_state_enter`. I'd suggest
  re-reading and possibly removing that limitation, but I did not want to make
  the call without a maintainer's confirmation.
- The receiver dead-band / pulse classification (`PULSE_IN_MIDDLE`) and the ±20 %
  windows are documented and intentional. Not a defect.
- Multi-master coexistence (yield-only) is a documented design choice. Not a
  defect.
- The 0x015D / 0x1014 read-clears pattern is a deliberate design pattern that
  could be factored into a helper, but the duplication is small (one ATOMIC
  block per call site) and the explicit code is easier to read than a
  parameterised helper. P3-class but I'm not asking for it.

---

End of report.
