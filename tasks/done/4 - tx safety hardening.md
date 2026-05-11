# Task 4: TX-asserted watchdog + polarity documentation

## Scope

Defensive hardening of the DALI TX (PB0) safety posture, surfaced by the TX-stuck audit at `tasks/tx-safety-audit.md`. Five fixes in a single commit on `task-1-flag-driven`:

1. **P2-1 TX-asserted watchdog** (new defensive layer)
2. **P1-A datasheet-dependency doc note** (one-paragraph comment)
3. **P2-2 `dali_init.c:20` stale comment fix**
4. **P2-3 `dali.h` polarity convention comment**
5. **D1 README PB0 pin-table disambiguation**

No P0 bugs are being fixed — the audit found none. This is a pure defence-in-depth pass.

## Background

The audit established (and corrected the original brief's polarity assumption):
- `OUTCLR (PB0 = LOW)` = bus released / safe state.
- `OUTSET (PB0 = HIGH)` = bus asserted.
- An external transistor inverts polarity between MCU pin and bus line.

Today the only firmware-level guarantee that PB0 returns LOW after being driven HIGH is "the next TCA0 tick fires a callback that includes OUTCLR/OUTTGL". The cadence is one TCA0 tick = 417 µs, so the implicit bound per `OUTSET` is ~417 µs. If TCA0 ever stops firing, the WDT (~8.2 s) is the only backstop — and 8.2 s of bus-asserted is a long time.

The proposed watchdog closes that gap.

## Fix 1 — TX-asserted watchdog (P2-1)

**File:** `src/tick.c` (the `TCA0_OVF_vect` ISR — verify the actual location; might also live in `src/dali_init.c` or wherever the OVF handler is). Coder should confirm by `grep -rn 'TCA0_OVF_vect\|ISR(TCA0' src/`.

**Logic:** after `dali_tick()` runs in the existing OVF ISR, check whether PB0 is currently asserted *and* there is no pending bit-level state-machine work. If both are true, the state machine has stalled with the bus driven low and the firmware must release it.

```c
// Defensive: if the bit-level state machine stalled while PB0 is HIGH
// (asserted), force a release.  Today this never fires under correct
// operation — every OUTSET in send_one is paired with a `dali_timer = 1`
// assignment that schedules the next callback (which always OUTCLRs or
// OUTTGLs within ~417 µs).  The guard catches a hypothetical state where
// dali_timer reached zero without a callback that releases the line.
if ((DALI_PORT.IN & DALI_TX_bm) && dali_timer == 0u) {
    DALI_PORT.OUTCLR = DALI_TX_bm;
}
```

**Placement:** at the end of the OVF ISR body, *after* `dali_tick()` has had a chance to fire its callback (otherwise the check would fight the legitimate transitions in `half_bit_toggle` etc.). Verify the existing OVF structure to make sure the check doesn't accidentally trip on a normal mid-bit half-period.

**Important nuance:** during a normal TX, between `send_one`'s OUTSET and the next `half_bit_toggle`, PB0 is HIGH but `dali_timer` is 1 (not 0). So the guard `(IN & TX_bm) && dali_timer == 0` should NOT fire during normal TX. Confirm this is true by hand-tracing through `send_one` → `dali_timer = 1` → next OVF → `dali_tick` decrements to 0 → fires `half_bit_toggle` (which OUTTGLs and assigns `dali_timer = 1` again) → the guard runs AFTER dali_tick so it observes `dali_timer = 1` again. ✓

Reviewer must independently verify this hand-trace.

**Cost estimate:** ~10 bytes flash, 0 bytes RAM. Watchdog never fires under correct operation; only triggers on genuine fault.

**`DALI_PORT.IN` access:** `IN` is the read-back of the actual pin state, not the OUT register. Available on all AVR0/1 PORT peripherals. Coder should verify by reading the existing `dali_is_bus_active` (in `dali_init.c`) which already uses `DALI_PORT.IN` — same pattern.

## Fix 2 — Datasheet-dependency doc note (P1-A)

**File:** `src/dali_init.c`, near the PB0 setup block (likely around lines 19-25, after `OUTCLR / DIRSET`).

Add a short comment:

```c
// PB0 returns to high-Z (DIR=0, OUT=0) on every reset source per
// ATtiny1604 datasheet § 16.5.4.  This is the WDT-during-TX recovery
// mechanism: if the WDT fires while PB0 is OUTSET (bus asserted), the
// reset clears OUT and DIR atomically, the external transistor base
// pull-down keeps the bus released, then this dali_init runs and
// re-establishes PB0 = driven LOW (released).  The OUTCLR-then-DIRSET
// sequence below transitions PB0 high-Z → driven LOW without ever
// passing through driven-HIGH.
```

Wording can be adapted by the coder; the load-bearing facts are: the recovery depends on a documented datasheet property, and the init sequence avoids transient HIGH states.

## Fix 3 — `dali_init.c:20` stale comment (P2-2)

**File:** `src/dali_init.c`, the existing comment at line 20.

Current text says something to the effect of "OUTCLR stays low - only DIR is toggled" which is **incorrect** — the firmware keeps `DIR` permanently set and toggles `OUT`. The audit suggests:

```c
// TX: configured as output, driven LOW (released bus).
// OUTSET asserts the bus; OUTCLR releases.
```

Confirm by reading the actual current comment first; the wording in the audit is paraphrased.

## Fix 4 — `dali.h` polarity convention comment (P2-3)

**File:** `src/dali.h`, around the existing `DALI_TX_bm` / `DALI_PORT` definitions (audit says "near line 91" — verify).

Add:

```c
// DALI_TX (PB0) polarity convention: OUTSET (HIGH) asserts the bus;
// OUTCLR (LOW) and high-Z release.  The external transistor inverts
// between MCU pin and bus line.  Every TX callback in the bit-level
// state machine maintains the invariant that PB0 is released within
// one TCA0 tick (~417 µs) of every OUTSET.
```

This codifies the invariant for future maintainers; otherwise someone might add a new OUTSET site without realising it must be paired with a scheduled callback that releases.

## Fix 5 — README PB0 pin-table disambiguation (D1)

**File:** `README.md`, the "Pin Assignment (ATtiny1604)" table near line 415.

Current row reads roughly:

> | PB0 | DALI TX — open-drain output, driven low to pull bus down |

The "driven low to pull bus down" describes the bus level, not the MCU pin. Disambiguate:

> | PB0 | DALI TX — drives the external bus transistor. **OUTSET asserts the bus** (drives bus low via the transistor); **OUTCLR / high-Z releases**. Reset state (high-Z) is released. |

Coder may adjust wording to fit the table's voice.

## Build / sizing

- `make build` clean (avr/bin/avr-gcc).
- Baseline at `cee394a`: Program 14440 / Data 836.
- Expected delta: +10 to +20 bytes Program (the watchdog check), Data unchanged.
- Target: Program ≤ 14470 (≤ +30 B). Reject anything larger as scope creep.

## Workflow

- Coder shares a brief plan covering: where exactly `TCA0_OVF_vect` lives, where in the ISR body the guard goes (post-tick), the hand-trace confirming the guard doesn't fire during normal TX.
- Reviewer signs off the plan before any code is written.
- Implementation, then ping reviewer with diff + sizes + the hand-trace.
- Reviewer must independently verify: (a) the guard placement is post-`dali_tick`; (b) the hand-trace through `send_one → OVF → half_bit_toggle` is correct (`dali_timer == 1` at the guard, NOT 0); (c) `DALI_PORT.IN` is the right way to read the actual pin state; (d) doc edits match the audit's wording intent.
- Wait for explicit "approved" before commit.
- Commit with explicit `git add` listing — do NOT stage CLAUDE.md, `.claude/`, or anything outside scope.
- After commit: `git tag -f task-1-complete HEAD` (project rule).
- Update task to completed; message team-lead with commit hash + sizes.

## Out of scope

- Hardware bench-test of the WDT-during-TX recovery (mentioned as P1-A in the audit's recommendations). The doc note is the firmware-side action; physical verification is a separate user-side activity.
- Adding a counter-based TX-asserted watchdog (the audit's alternative form, ~30 bytes + 1 byte RAM, threshold 50 ms). The simple IN-register check is sufficient; coder should NOT propose the counter form unless they identify a specific case the simple form misses.
- Any other audit findings — the P1-1 (already fixed in task 3), P1-2 ISR-context subtleties, and verified-safe paths are not in scope.

---

## Outcome

**Date completed:** 2026-05-11
**Branch:** `task-1-flag-driven`
**Commit:** `81b14d9` — single implementation commit. Tag `task-1-complete` advanced to `81b14d9` and continues to track HEAD per project rule.

### What landed

| Fix | File | Change |
|---|---|---|
| **P2-1** TX-asserted watchdog | `src/tick.c` | After `dali_tick()` in `TCA0_OVF_vect`: `if ((DALI_PORT.IN & DALI_TX_bm) && dali_timer == 0u) DALI_PORT.OUTCLR = DALI_TX_bm;`. Cannot fire during normal TX (every `OUTSET` in `send_one` is paired with `dali_timer = 1`, so the post-tick guard observes `dali_timer != 0`). Action is **minimal** — only releases the line. Does NOT touch `dali_busy` / `current_op` / callback pointer, so the stalled state is preserved for the outer 200-TE wait-for-idle escape or the WDT to observe and recover. |
| **P1-A** datasheet dependency | `src/dali_init.c` | Comment block above the PB0 setup documenting the AVR0/1 PORTB peripheral-reset semantics (DIR=0, OUT=0 on all reset sources per ATtiny1604 datasheet § 16.5.4) that enable WDT-during-TX recovery. |
| **P2-2** stale comment | `src/dali_init.c` line ~20 | Replaced the incorrect "OUTCLR stays low - only DIR is toggled" with accurate wording — DIR is permanent, OUT is what's toggled. |
| **P2-3** polarity invariant | `src/dali.h` | New comment block near `DALI_TX_bm` codifying: OUTSET = bus asserted; OUTCLR / high-Z = released; external transistor inverts; every `OUTSET` paired with `dali_timer = 1` ensures release within one TCA0 tick. |
| **D1** README pin-table | `README.md` Pin Assignment row | Disambiguated PB0 — current row described the bus level, new row distinguishes MCU pin level from bus level explicitly. |

### Final binary sizes

| Section | Pre-task 4 (`cee394a`) | Final (`task-1-complete` = `81b14d9`) | Delta |
|---------|------------------------|----------------------------------------|-------|
| Program | 14440 B | **14462 B** | **+22 B** |
| Data    |   836 B | **  836 B** | **0 B** |

Within the +30 B / +0 B budget. Net cumulative delta vs the pre-task-1 baseline (`4c2888b`, 14316 / 849): **+146 B Program (+1.0 %), −13 B Data (−1.5 %)**. Stack high-water unchanged at 49 B.

### Cross-checks (with reviewer-12)

- **`make clean && make build`** clean, no warnings.
- **Hand-trace, normal TX cycle**: `send_one` → OUTSET PB0 → `dali_timer = 1` → next OVF → `dali_tick` → `dali_timer` decrements to 0 → callback fires (`half_bit_toggle`) which OUTTGLs and sets `dali_timer = 1` again → guard runs (post-tick) and reads `dali_timer = 1` ≠ 0 → **guard does NOT fire**. ✓ Re-walked independently by reviewer.
- **Hand-trace, stall scenario**: hypothetical case where `dali_tick` decrements `dali_timer` to 0, no callback (or a callback that doesn't re-arm) → next OVF → guard reads `(PB0 IN HIGH) ∧ (dali_timer == 0)` → fires `OUTCLR`. Bus released within one TCA0 tick (~417 µs) of the stall onset. ✓
- **Minimal-action property**: guard does not touch `dali_busy`, `current_op`, or any callback pointer. Outer recovery (200-TE bus-stuck timer in `wait_for_idle`, eventually WDT) still observes the stall and runs its own cleanup. ✓
- **Comment correctness**: each of P2-2, P1-A, P2-3, D1 matches the load-bearing facts established in `tasks/tx-safety-audit.md` section 0. ✓

### Deviations from the task spec

None. Coder chose the simple IN-register form of the watchdog (not the counter form) as specified. P1-A doc note was implemented as a comment block in `dali_init.c` rather than a separate documentation file.

### Verification status

- [x] `make build` clean against the project-local `avr/bin/avr-gcc` toolchain.
- [x] `avr/bin/avr-size --mcu=attiny1604 -C build/main.elf` reports Program 14462 / Data 836 (1 922 B / 188 B headroom respectively).
- [x] `.su` sweep — no new constprop clones; stack high-water unchanged at 49 B.
- [x] Watchdog placement verified post-`dali_tick` in `TCA0_OVF_vect`.
- [x] Normal-TX hand-trace walked end-to-end against the actual code, confirming the guard does not fire.
- [x] Stall hand-trace walked, confirming the guard catches the case and releases the bus.
- [x] Doc fixes (P2-2, P1-A, P2-3, D1) cross-checked against the audit's recommended wording.
- [x] Task spec moved from `tasks/` to `tasks/done/` with this Outcome section appended.
