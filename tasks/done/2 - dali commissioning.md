# Task 2: DALI commissioning

## Goal

Add IEC 62386-102 control-gear commissioning so the gateway can discover and address fresh ballasts on a new bus without needing a separate commissioning tool. **Out of scope** for this task: 24-bit forward frames, DALI-2 input devices, memory bank reads, DT1/DT8 extensions. Control-gear commissioning only.

## Branching

Continues on `task-1-flag-driven`. Task 1 hasn't merged yet and the commissioning work depends directly on the flag-driven architecture (`dali_op_t`, `sync_flags`, `issue_dali_command`, `dali_sync_arm_*`). The `task-1-complete` tag advances forward as commissioning commits land, per project rule.

## DALI commands required

All operate on the special-address space (no per-ballast addressing). Address-byte = the special-command opcode; data-byte = parameter (or short address for PROGRAM_SHORT_ADDRESS). All come from `dali.h` — add the missing constants:

| Opcode (addr byte) | Name | Twice? | Backward frame? |
|---|---|---|---|
| 0xA1 | TERMINATE | no | no |
| 0xA3 | DTR0 = data | no | no (already supported) |
| 0xA5 | INITIALISE (data: 0x00 = all, 0xFF = without short addr, otherwise short addr) | **yes** | no |
| 0xA7 | RANDOMISE | **yes** | no |
| 0xA9 | COMPARE | no | **yes — anyone-pulls-low "0xFF"** |
| 0xAB | WITHDRAW | no | no |
| 0xB1 | SEARCHADDRH = data | no | no |
| 0xB3 | SEARCHADDRM = data | no | no |
| 0xB5 | SEARCHADDRL = data | no | no |
| 0xB7 | PROGRAM_SHORT_ADDRESS = data (`(short_addr << 1) | 1`, or 0xFF to clear) | no | no |
| 0xB9 | VERIFY_SHORT_ADDRESS (data = `(short_addr << 1) | 1`) | no | yes (0xFF if match) |
| 0xBB | QUERY_SHORT_ADDRESS | no | yes (data = `(short_addr << 1) | 1` or 0xFF if no addr) |

INITIALISE has a 15-minute spec timeout — ballasts auto-exit commissioning if no commands arrive. Commissioning sends commands frequently enough that this never trips in practice; no firmware-side timeout enforcement needed.

## Bisect algorithm

```
INITIALISE(target) × 2
RANDOMISE × 2
search ← 0xFFFFFF
short_addr ← chosen by mode (see Modbus surface)
loop:
    SEARCHADDRH/M/L ← search
    COMPARE
    if yes:
        if search bit position == 0:    # exact match — single ballast isolated
            DTR0 ← short_addr
            PROGRAM_SHORT_ADDRESS
            VERIFY_SHORT_ADDRESS
            if verify yes:
                WITHDRAW
                short_addr++; search ← 0xFFFFFF
            else:
                # programming failed — increment error count, withdraw, retry next slot
        else:
            search ← search with current bit cleared    # narrow down
    else:
        search ← search with current bit set            # back off, then move to next bit
        # move to next-lower bit position
    if no match at 0xFFFFFF after RANDOMISE:
        TERMINATE
        complete
```

The classical reference is Microchip AN1487 §"Implementation of Random Addressing" and python-dali's `gear/sequences.py::Commissioning`. Reviewer should sanity-check against one of those.

## Modbus surface

Single new holding register:

| Address | Name | Read returns | Write triggers |
|---|---|---|---|
| 0x1014 | COMMISSION | status word (see encoding below) | command word (see encoding below) |

**Write encoding** (FC06 to 0x1014):

| Value | Action |
|---|---|
| 0x0000 | Abort: send TERMINATE, return to idle. No-op if already idle. |
| 0x0001 | Start full commissioning: INITIALISE(0x00 = all), erase existing short addresses, assign 0..N−1 to discovered ballasts. **Destructive** — overwrites any existing addressing. |
| 0x0002 | Start incremental commissioning: INITIALISE(0xFF = without short addr), only assign to currently-unaddressed ballasts; ballasts that already have a short address are not disturbed. New ballasts get the lowest unused short addresses. |
| anything else | ILLEGAL_VALUE |

If the channel is already busy (running), writes other than 0x0000 return DEVICE_BUSY.

**Read encoding** (FC03 of 0x1014):

| High byte | Low byte | Meaning |
|---|---|---|
| 0x00 | 0x00 | Idle (no commissioning ever run, or last run completed and master has not consumed status) |
| 0x01 | count so far | Running — `count` is the number of ballasts addressed so far this run |
| 0x02 | total addressed | Complete — `count` is the total number of ballasts addressed in the just-finished run |
| 0x03 | error subtype | Error / aborted — see subtype table |

Error subtypes: 0x01 = aborted by master, 0x02 = no ballasts found (RANDOMISE produced zero responders), 0x03 = programming verify failed for >3 consecutive ballasts (bus issue or bad ballast), 0x04 = ran out of short addresses (all 64 in use, more ballasts present).

**Read-clears semantics** for terminal states (0x02, 0x03) — same shape as 0x015D / FC04 0x0108: reading 0x1014 when high byte is 0x02 or 0x03 returns the value AND resets the underlying state to 0x0000 atomically. While running (0x01) or idle (0x00), reads are non-clearing.

## Implementation outline

### Data structures (in `dali_sync.c`)

```c
// New op type alongside the existing ones in dali_op_t.
OP_COMMISSION,

// Phase tracker for the commissioning state machine.  ~1 byte.
typedef enum __attribute__((packed)) {
    COM_IDLE,
    COM_INITIALISE_1, COM_INITIALISE_2,    // INITIALISE × 2
    COM_RANDOMISE_1, COM_RANDOMISE_2,      // RANDOMISE × 2
    COM_SEARCH_H, COM_SEARCH_M, COM_SEARCH_L,
    COM_COMPARE,
    COM_PROGRAM_DTR, COM_PROGRAM_ADDR,     // DTR0 then PROGRAM_SHORT_ADDRESS
    COM_VERIFY,
    COM_WITHDRAW,
    COM_TERMINATE,
} commission_phase_t;

volatile struct {
    uint8_t            search_h, search_m, search_l;  // current 24-bit search address
    uint8_t            next_short_addr;               // next short address to assign (0..63)
    uint8_t            bisect_bit;                    // current bit being bisected (0..23)
    uint8_t            count;                         // ballasts addressed so far
    uint8_t            err_streak;                    // consecutive verify-failures
    commission_phase_t phase;
    uint8_t            mode;                          // 1 = full, 2 = incremental
    bool               abort_requested;               // set by master writing 0x0000
    uint16_t           status;                        // exposed via FC03 0x1014
} commission_state;
```

~10–14 bytes of static state. Fits in the ~200 bytes RAM headroom.

### Function additions

- `bool dali_sync_arm_commission(uint8_t mode);` — called from `regs_write_holding` for 0x1014 writes. Sets up `commission_state`, returns false if already running.
- New branches in `start_next_dali_work` (pick_work) — commissioning slot between raw command (priority 1) and group bit (priority 2). Highest-priority interactive work, since master is actively polling for status.
- New branches in `dali_sync_dispatch_current` for OP_COMMISSION — issues the next command based on `commission_state.phase`.
- New branches in `dali_sync_completion` for OP_COMMISSION — advances phase based on the response, manages bisect logic, handles error paths.

### Modbus integration

- `MBREG_COMMISSION` define in `registers.h` at 0x1014.
- `regs_read_holding` for 0x1014: returns `commission_state.status` with read-clears semantics on terminal high bytes (0x02, 0x03) — same pattern as `process_fc04`'s raw_cmd_result handling. **Note**: this is the first holding register read with this clearing semantics; the existing pattern is on FC04. Implement carefully (atomic snapshot + clear inside ATOMIC_BLOCK + TCB0 fence).
- `regs_write_holding` for 0x1014: validate value (0x0000/0x0001/0x0002 only), call `dali_sync_arm_commission` for non-zero values or set the abort flag for 0x0000. Returns false on illegal value.
- `dispatch_action_reg` adds branch for MBREG_COMMISSION.

### Cache effect

Successful commissioning mutates the bus's addressing, which doesn't directly affect our cache (our cache is keyed by short address; new ballasts at addresses 0..N−1 will appear in the cache next time a SCAN_TRIGGER or PIT cursor visits them). The implementation should automatically `sync_flags_set(i, SYNC_READ_FULL)` for each newly-programmed short address so the post-commissioning cache fill happens without a manual SCAN_TRIGGER.

For incremental commissioning (mode 0x0002), unaddressed ballasts are assigned the lowest unused short addresses. "Unused" = `presence` bit is 0 in our cache. This avoids stomping on already-addressed ballasts.

## Edge cases (must handle)

1. **No ballasts on bus**: after RANDOMISE, the first COMPARE at search=0xFFFFFF returns no response. Status → 0x0302 (no ballasts found), TERMINATE, idle.
2. **Master aborts mid-run**: write 0x0000 to 0x1014 → set `abort_requested`; on next dispatch, send TERMINATE and transition to status 0x0301 (aborted).
3. **Verify fails**: PROGRAM_SHORT_ADDRESS / VERIFY_SHORT_ADDRESS round-trip mismatch — withdraw the candidate, increment err_streak, move on. >3 consecutive failures → status 0x0303 and abort.
4. **Bus runs out of short addresses**: in full mode, 64 ballasts is the cap; if a 65th is found, status → 0x0304 and TERMINATE. In incremental mode, the cap is `64 − already_assigned`.
5. **Concurrent Modbus writes during commissioning**: per-ballast bank writes still arm SYNC_WRITE_*, but they queue behind the OP_COMMISSION priority and execute after commissioning ends. This is fine — the work selector's priority chain handles it.
6. **Concurrent raw command writes during commissioning**: same — the raw command channel arms and waits behind OP_COMMISSION.

## Verification

Per the CLAUDE.md task workflow:

1. **Build clean**: `make build` with avr/bin/avr-gcc.
2. **Sizes**: report Program / Data / stack high-water vs the pre-task baseline (currently `task-1-complete = 395eab5`, sizes 13040 / 823). Target: Program ≤ 14040 (≤ +1000 B), Data ≤ 850 (≤ +27 B). Stack should not grow appreciably (commissioning adds one new function-call layer in pick_work + dispatch_current + completion).
3. **Hand-trace**: walk a master-initiated full commissioning of a 3-ballast bus end-to-end. Master writes 0x1014 = 0x0001 → INITIALISE × 2 → RANDOMISE × 2 → bisect on first ballast → PROGRAM_SHORT_ADDRESS 0 → VERIFY → WITHDRAW → bisect on second ballast → PROGRAM 1 → ... → fourth COMPARE returns no → TERMINATE → status 0x0203 → master reads 0x1014, sees 0x0203, status clears to 0x0000.
4. **Hand-trace incremental mode**: same setup but with ballasts 0 and 1 already addressed; mode 0x0002 should leave them alone and only address the third (which gets short address 2).
5. **Hand-trace abort**: master writes 0x0001, then writes 0x0000 mid-run; expected: TERMINATE issued promptly, status → 0x0301, master clears it.

## Files to expect to change

- `src/dali.h` — new opcodes (INITIALISE, RANDOMISE, COMPARE, etc.).
- `src/dali_sync.h` — OP_COMMISSION, commission_state, dali_sync_arm_commission proto.
- `src/dali_sync.c` — bisect logic, phase machine, work-selector slot, completion branch.
- `src/registers.h` — MBREG_COMMISSION define at 0x1014.
- `src/registers.c` — holding-register read/write handlers.
- `src/modbus.c` — dispatch_action_reg branch.
- `README.md` — new "Commissioning" subsection under Modbus Interface; behavioural change note.

## Out of scope (deferred for separate tasks)

- 24-bit forward frames / DALI-2 input device support.
- Memory bank reads (GTIN, firmware version, etc.).
- DT8 colour control.
- DT1 emergency lighting / self-test.
- Energy reporting (D4i bank 202).

These were inventoried in `tasks/dali-comparison-report.md` and may become tasks 3+ later.

## Out of scope (intentionally never)

- Active collision detection during own TX (single-master assumption documented in README).
- DALI Alliance certification (cost-prohibitive for an open project of this scale).

---

## Outcome

**Date completed:** 2026-05-10 (final acceptance pending; review iteration ongoing)
**Branch:** `task-1-flag-driven` (continuing from task 1; task 2 layers on top)
**Commit:** `5c9ee70` — single implementation commit. Tag `task-1-complete` advanced to `5c9ee70` and continues to track HEAD per project rule.

### What landed

| Surface | Detail |
|---|---|
| New DALI opcodes (`src/dali.h`) | TERMINATE 0xA1, INITIALISE 0xA5, RANDOMISE 0xA7, COMPARE 0xA9, WITHDRAW 0xAB, SEARCHADDRH/M/L 0xB1/B3/B5, PROGRAM_SHORT_ADDRESS 0xB7, VERIFY_SHORT_ADDRESS 0xB9, QUERY_SHORT_ADDRESS 0xBB. |
| New work-selector op (`src/dali_sync.h`) | `OP_COMMISSION` at priority 1 (above raw command — interactive master polling drains promptly). |
| New phase machine (`src/dali_sync.c`) | 10-phase enum (`COM_*`) covering INITIALISE × 2, RANDOMISE × 2, SEARCH H/M/L, COMPARE, PROGRAM_DTR + PROGRAM_ADDR, VERIFY, WITHDRAW, TERMINATE; bisect logic walking `bisect_bit` 23 → 0; first-pass sentinel at 24. |
| Modbus surface (`src/registers.h`, `src/registers.c`, `src/modbus.c`) | `MBREG_COMMISSION` at 0x1014. Write 0x0001 = full, 0x0002 = incremental, 0x0000 = abort. Read returns status word with read-clears semantics on terminal high bytes (mirrors FC04 0x0108 raw_cmd_result pattern). |
| Cache integration | Each successful PROGRAM+VERIFY round arms `SYNC_READ_FULL` on the newly-programmed short address so the cache populates without manual SCAN_TRIGGER. `regs_set_present` called immediately on VERIFY-yes. |
| README | New "Commissioning" subsection under Modbus Interface; updated Behavioural Changes for the new register; existing "Addressing and Commissioning" DALI-protocol prose now points at the firmware implementation. |

### Final binary sizes

| Section | Pre-task 2 (`395eab5`) | Final (`task-1-complete` = `5c9ee70`) | Delta |
|---------|------------------------|----------------------------------------|-------|
| Program | 13040 B | **14350 B** | **+1310 B** |
| Data    |   823 B | **  835 B** | **+12 B** |

The +1310 B Program is +310 B over the task-spec target of +1000 B. Reviewer-10 escalated the overrun rather than unilaterally accepting it; team-lead approved the waiver after coder-10 demonstrated three concrete cuts that returned only ~12 B in aggregate (gcc 7.3 -O3 had pre-collapsed the obvious savings). The chip retains 2 028 B free flash (87.6 % used). Data is well under the +27 B target. Stack high-water unchanged at 49 B.

### Cuts attempted (and what they returned)

| Cut | Saving |
|---|---|
| Hoist TCB0+ATOMIC fence in arm path | ~4 B |
| Drop `repeat` column from PROGMEM dispatch table | ~6 B |
| Collapse SEARCH_H/M/L data_byte switch into arithmetic | ~2 B |
| **Total** | **~12 B** |

The next-cheapest cut was dropping incremental commissioning mode (~50–80 B) or read-clears on 0x1014 (~70 B). Both were rejected as scope drops in service of an arbitrary number; full spec coverage retained.

### Constprop hygiene

`grep -h constprop build/*.su` confirms zero new constprop clones. The pre-existing `start_tx.part.1.constprop` baseline is unchanged. `noinline, noclone` retained on `issue_dali_command`, `issue_write_step`, `com_reset_bisect`, `com_lowest_unused`, `dali_sync_arm_commission`. Any future feature work that adds heavy raw-command callers should re-measure to confirm the attribute pressure still holds.

### Hand-traces verified (with reviewer-10)

1. **Full commissioning of a 3-ballast bus**: master writes 0x1014 = 0x0001 → INITIALISE × 2 → RANDOMISE × 2 → bisect ballast 1 (24 SEARCH triplets + COMPARE rounds) → PROGRAM_SHORT_ADDRESS 0 → VERIFY → WITHDRAW → bisect ballast 2 → PROGRAM 1 → … → fourth full-range COMPARE returns NAK → TERMINATE → status `0x0203` (complete, 3 addressed) → master reads 0x1014 → returned + atomic-cleared to 0x0000.
2. **Incremental commissioning**: ballasts 0, 1 already addressed; master writes 0x0002 → INITIALISE 0xFF only addresses unaddressed gear → ballast 2 gets short address 2 → status `0x0201`.
3. **Abort mid-run**: master writes 0x0000 → `abort_requested` set → next dispatch emits TERMINATE → phase → `COM_IDLE` → status `0x0301`.
4. **Empty bus**: first full-range COMPARE returns NAK → status `0x0302`, no ballasts found.
5. **Address exhaustion**: full mode at 64 addressable ballasts; 65th COMPARE-yes → status `0x0304`, TERMINATE.

### Deviations from spec

- **+310 B Program over the +1000 B target** — see "Cuts attempted". Trade was full spec coverage vs a number; full coverage retained.
- **No DTR1/DTR2 support yet** — commissioning only uses DTR0 (PROGRAM_SHORT_ADDRESS uses DTR0 as the source register per spec). DTR1/DTR2 only matter for memory bank reads, DT8 colour, and a few extended queries — all out of scope.

### Out of scope (still deferred)

The DALI comparison report (`tasks/dali-comparison-report.md`) inventories 24-bit forward frames, memory bank reads, DT8 colour, DT1 emergency self-test, and D4i energy reporting as additional gaps versus reference implementations. Each may become a separate task; commissioning was prioritised first because it was the largest single feature gap and the user's explicit ask.

### Verification status

- [x] `make build` clean against the project-local `avr/bin/avr-gcc` toolchain.
- [x] `avr/bin/avr-size --mcu=attiny1604 -C build/main.elf` reports Program 14350 / Data 835 (2 028 B / 189 B headroom respectively).
- [x] `.su` sweep — no new constprop clones; stack high-water unchanged at 49 B (`modbus_process` snprintf path).
- [x] Hand-traces (full / incremental / abort / empty / exhaustion) walked end-to-end against the actual code, jointly with reviewer-10.
- [x] README integrated: new Commissioning subsection, behavioural change note, addressing-prose pointer to the firmware implementation.
- [x] Task spec moved from `tasks/` to `tasks/done/` with this Outcome section appended.
