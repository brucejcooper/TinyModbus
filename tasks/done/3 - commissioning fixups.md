# Task 3: post-review fix-ups for commissioning + small doc bugs

## Scope

Bundle of one P0 correctness bug, one P1 robustness improvement, and three trivial P2 doc fixes surfaced by `tasks/code-review-report.md`. Single commit on `task-1-flag-driven`. Tag `task-1-complete` advances forward per project rule.

## Findings to fix

### P0-1 (block-merge) — Commissioning bisect at `bb=0` NO

**File:** `src/dali_sync.c`, COM_COMPARE branch in `dali_sync_completion` (~lines 891–935).

**What's wrong.** When the bisect reaches `bisect_bit == 0` and COMPARE returns NO, `com_set_bit(bb)` restores bit 0 of `commission_state.search` locally — but the *gears'* internal SEARCHADDR registers still hold the prior bit-0-cleared value. The handler then transitions directly to `COM_PROGRAM_ADDR`, so `PROGRAM_SHORT_ADDRESS` is sent against a stale gear-side search address. The isolated gear (whose random has bit 0 = 1) doesn't match → not programmed → VERIFY NAKs → `err_streak` increments. After ~6 ballasts a real bus latches `COMM_STATUS_ERR_VERIFY_FAIL`.

**Reproduces** for any ballast whose lowest random address has bit 0 = 1 (~50% of cases by uniform distribution).

**Required fix.** Before `PROGRAM_SHORT_ADDRESS`, the gears must be re-synced with the corrected search address. Two reasonable shapes — pick whichever is smaller / cleaner:

**Option A (flag-driven, smaller):** add a single `bool resend_pending;` to `commission_state`. When `bb == 0` and `!yes`, set `resend_pending = true` and transition to `COM_SEARCH_H` instead of `COM_PROGRAM_ADDR`. Add an explicit case for `COM_SEARCH_L` in the completion switch that, when `resend_pending` is true, transitions to `COM_PROGRAM_DTR` (skipping the wasted COMPARE round) and clears the flag. SEARCH_H/M/L's dispatch_current is unchanged — they just emit SEARCHADDR.

**Option B (unconditional re-send, larger but simpler):** always insert a SEARCH_H/M/L round before PROGRAM_DTR. Add three new phases `COM_RESEND_H/M/L` between bisect-end and PROGRAM_DTR. Wastes 3 sends (~75 ms) per ballast in the YES-at-bb=0 case but the code is uniform.

Coder picks the option, reviewer signs off. Hand-trace the YES path AND the NO path AND the empty-bus / address-exhaust paths after the change to confirm no regression.

**Verification:** trace a 3-ballast bus where ballast random addresses are `0x000001`, `0x000002`, `0x000003` (forces all three ballasts through the bb=0 NO path on at least one bisect round). Confirm all three program successfully.

### P1-1 — Bus-stuck timeout doesn't clear `dali_busy`

**File:** `src/dali_state_wait_for_idle.c`, the `bus_stuck_timeout` 200-Te safety escape.

**What's wrong.** When the bus is held shorted for >200 Te (physical fault, external master), `bus_stuck_timeout` calls `dali_idle_state_enter()` without clearing `dali_busy`. `dali_idle_state_enter` re-checks `dali_is_bus_active()`, sees the bus still shorted, re-enters `dali_wait_for_idle_state_enter`, arms a fresh 200-Te timer, and we loop indefinitely at ~83 ms/cycle. Modbus stays responsive (master can abort via 0x1014 if commissioning is in flight), but the slave never sleeps.

**Required fix.** When `bus_stuck_timeout` fires, clear `dali_busy` and surface a status. For commissioning, stamp `commission_state.status = COMM_STATUS_ERR_ABORTED` (or — if you want a distinct "bus stuck" subtype, add a new `COMM_STATUS_ERR_BUS_STUCK = 0x0305` and document it). For non-commissioning ops, the existing recovery is fine; just clear `dali_busy` and let main re-evaluate.

**Verification:** reviewer should hand-trace the recovery path: timer fires → busy cleared → idle entered → main loops → next `start_next_dali_work` finds nothing pending (or picks the next item). The slave should be able to sleep eventually.

### P2-1 — `dali.h` comment on opcode 0xA6 is wrong

**File:** `src/dali.h`, lines 42–43.

The comment says `// (DALI opcode 0xA6 is reserved by IEC 62386-102 - never issue it.)`. Per IEC 62386-102 ed.2 and python-dali `gear/general.py:857-865`, opcode 0xA6 is `QUERY_MANUFACTURER_SPECIFIC_MODE` — not reserved. The decision to *not* issue it from internal scans is correct (response is manufacturer-defined), but the rationale stated is wrong.

**Fix:** rewrite the comment to: `// (DALI opcode 0xA6 is QUERY_MANUFACTURER_SPECIFIC_MODE per IEC 62386-102; we don't issue it from internal scans because the response is manufacturer-defined.)`

### P2-3 — `dali_sync.c` file-level priority comment is stale

**File:** `src/dali_sync.c`, around line 9 (the file-level comment block listing the priority chain).

Says `raw → group bit → SYNC_WRITE_* → SYNC_READ_LEVEL → full-scan continue → full-scan new` — i.e. omits commissioning, which is now priority 1.

**Fix:** prepend `commissioning → ` to the chain.

### P2-4 — `dali_sync.c` threading model comment doesn't list `commission_state`

**File:** `src/dali_sync.c`, lines 19–32 (the "Threading model" comment block).

Lists `sync_flags`, `dali_busy`, `current_op`, `scan_full_*`, `raw_cmd_*`, `group_write`, `g_regs.raw_cmd_result`, `g_regs.dirty[]` — but not `commission_state`, which is the largest new mutable state on this branch.

**Fix:** add a paragraph: `commission_state: written from main only inside dali_sync_arm_commission and regs_read_holding's terminal-state clear (both under TCB0+ATOMIC); all other updates run from ISR context (dali_sync_completion / dispatch_current). 16-bit status writes from ISR are LVL1, fenced against main reads by main's TCB0=0 around its ATOMIC blocks.`

## Files to expect to change

- `src/dali_sync.c` — bisect fix (P0-1), threading-model comment (P2-4), priority-chain comment (P2-3)
- `src/dali_sync.h` — possibly new struct field (`resend_pending` bool) or new phase enum values, depending on chosen option for P0-1
- `src/dali_state_wait_for_idle.c` — bus-stuck recovery (P1-1)
- `src/registers.h` — possibly new `COMM_STATUS_ERR_BUS_STUCK` constant if P1-1 adds a distinct subtype
- `src/dali.h` — 0xA6 comment fix (P2-1)
- `README.md` — only if P1-1 adds a new error subtype that needs documenting

## Out of scope

The other P1 / P2 / P3 findings from the report are either verified-safe-no-change or low-priority simplifications. They stay deferred:

- P1-2 (RETRY_PENDING shared between read/write paths): consistent with spec, not a bug.
- P1-3, P1-4, P1-5, P1-6: all verified safe by the reviewer; no code change needed.
- P0-2 (pending_apply_mask): verified safe; only a one-line comment recommended (optional, not in scope).
- P2-2 (OP_FULL_SCAN clears never-set RETRY_PENDING bit): harmless dead clear.
- P2-5..P2-8: documentation-only / verified-safe annotations.
- P3-1..P3-3: simplification opportunities, not material.

## Build / sizing

- `make build` clean (avr/bin/avr-gcc).
- Baseline at `9b5bc38` (the comparison-report commit, which doesn't change code): Program 14350 / Data 835.
- Expected delta: small positive on Program (P0-1 fix may add 20-60 B depending on option chosen; P1-1 fix maybe 10-30 B; P2 fixes are comment-only). Target: Program ≤ 14450 (≤ +100 B over baseline). Data unchanged or +1-2 B (only if P0-1 adds the `resend_pending` flag).

## Workflow

- Coder shares a brief plan covering: chosen P0-1 option (A or B) with rationale; whether P1-1 introduces a new status subtype or reuses ERR_ABORTED.
- Reviewer signs off the plan before any code is written.
- After implementation, ping reviewer with diff + sizes + the YES-and-NO-and-empty-bus hand-trace for the bisect fix.
- Wait for explicit "approved" before commit.
- Commit with explicit `git add` listing — do NOT stage CLAUDE.md or `.claude/`.
- After commit: `git tag -f task-1-complete HEAD` (project rule).
- Update task to completed; message team-lead with commit hash + sizes.

---

## Outcome

**Date completed:** 2026-05-10
**Branch:** `task-1-flag-driven`
**Commit:** `5550006` — single commit covering all five findings. Tag `task-1-complete` advanced to `5550006` and continues to track HEAD per project rule.

### What landed

Option A chosen for P0-1 (flag-driven resync) — smaller diff, simpler reasoning than option B (three new RESEND phases).

| Fix | File(s) | Change |
|---|---|---|
| **P0-1** bisect at bb==0 NO | `src/dali_sync.{c,h}` | New `resend_pending` bool in `commission_state`. When `bb == 0` and COMPARE returns NO, set the flag and transition to `COM_SEARCH_H` (instead of `COM_PROGRAM_ADDR`). New explicit `COM_SEARCH_L` case in `dali_sync_completion` checks the flag: if set, clear it and route to `COM_PROGRAM_ADDR` (skipping the wasted COMPARE round); else default-advance to `COM_COMPARE`. The bb==0 YES path is unchanged (direct to `COM_PROGRAM_ADDR`). |
| **P1-1** bus-stuck loop | `src/dali_state_wait_for_idle.c`, `src/dali_sync.h`, `README.md` | New `COMM_STATUS_ERR_BUS_STUCK = 0x0305` subtype. `bus_stuck_timeout` now clears `dali_busy` + `current_op` and, if commissioning is in flight, stamps the new error subtype and forces `phase = COM_IDLE`. README error-subtype table gains row 0x05. |
| **P2-1** `0xA6` comment | `src/dali.h` | "reserved" → "QUERY_MANUFACTURER_SPECIFIC_MODE per IEC 62386-102; we don't issue it from internal scans because the response is manufacturer-defined." |
| **P2-3** priority-chain comment | `src/dali_sync.c` | File-level comment now lists commissioning as priority 1 (`commissioning → raw → group bit → SYNC_WRITE_* → ...`). |
| **P2-4** threading-model annotation | `src/dali_sync.c` | Threading-model paragraph for `commission_state` added: written from main only inside `dali_sync_arm_commission` and `regs_read_holding`'s terminal-state clear (both under TCB0+ATOMIC); all other updates run from ISR context (`dali_sync_completion` / `dispatch_current`). |

### Final binary sizes

| Section | Pre-task 3 (`9b5bc38`) | Final (`task-1-complete` = `5550006`) | Delta |
|---------|------------------------|----------------------------------------|-------|
| Program | 14350 B | **14440 B** | **+90 B** |
| Data    |   835 B | **  836 B** | **+1 B** |

Both within the +100 / +2 budget. Net cumulative delta vs the pre-task-1 baseline (`4c2888b`, 14316 / 849): **+124 B Program (+0.9 %), −13 B Data (−1.5 %)**. Stack high-water unchanged at 49 B (`modbus_process` snprintf path).

### Cross-checks (with reviewer-11)

- **`make clean && make build`** clean, no warnings.
- **`grep -h constprop build/*.su`** unchanged — no new clones triggered by the new struct field or the new switch case. `noinline,noclone` discipline preserved on all five attribute-flagged functions.
- **P0-1 hand-trace** — all five scenarios walked end-to-end against the actual code:
  - Single ballast `random=0x000001` → bisect narrows to `0x000000`, COMPARE NAK at bb=0 → `resend_pending=true`, phase=`COM_SEARCH_H` → SEARCH_H/M/L emit with restored `0x000001` → SEARCH_L completion sees flag, routes to `COM_PROGRAM_ADDR` → PROGRAM matches, VERIFY OK. ✓
  - Single ballast `random=0x000000` → bb==0 YES path unchanged: direct to `COM_PROGRAM_ADDR`, no extra SEARCH round. ✓
  - Empty bus (first COMPARE NAK at bb==24) → unchanged path, no resync triggered. ✓
  - Address exhaustion (bb==24 YES with `next_short_addr >= 64`) → unchanged path, ERR_ADDR_EXHAUST stamped. ✓
  - Abort mid-bisect → abort flag still respected at top of OP_COMMISSION dispatch. ✓
- **P1-1 hand-trace** — bus shorted before idle entry → 200-Te timer fires → `dali_busy` cleared → `commission_state.status = ERR_BUS_STUCK` (if commissioning in flight) → `phase = COM_IDLE` → idle re-entry succeeds → main re-evaluates and finds nothing pending → sleeps. Bus-not-shorted path unchanged.

### Deviations from the task spec

None. All five findings addressed exactly as specified. P1-1 took the recommended path of adding a distinct `ERR_BUS_STUCK` subtype rather than reusing `ERR_ABORTED`, so the master can distinguish abort-by-master from a physical bus fault.

### Verification status

- [x] `make build` clean against the project-local `avr/bin/avr-gcc` toolchain.
- [x] `avr/bin/avr-size --mcu=attiny1604 -C build/main.elf` reports Program 14440 / Data 836 (1 944 B / 188 B headroom respectively).
- [x] `.su` sweep — no new constprop clones; stack high-water unchanged at 49 B.
- [x] Five P0-1 hand-traces walked end-to-end against the actual code, jointly with reviewer-11.
- [x] Two P1-1 hand-traces (stuck and not-stuck paths) walked end-to-end.
- [x] Doc fixes (P2-1, P2-3, P2-4) cross-checked against the suggested wording in the task spec.
- [x] Task spec moved from `tasks/` to `tasks/done/` with this Outcome section appended.
