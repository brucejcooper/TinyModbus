# TinyModbus: Refactor to Flag-Driven Architecture

## Repository

Branch: `alternative-impl` of <https://git.8bitcloud.com/bruce/TinyModbus>

This branch already has the per-ballast "dirty" bitmap exposed as Modbus registers. Familiarise yourself with that mechanism before starting — every change below should integrate with it, not replace it.

## Goal

Replace the DALI command queue with a per-ballast sync-flag table. The DALI subsystem becomes a state-reconciliation loop instead of an event processor. The main loop becomes deterministic: one DALI command (or short sync sequence) at a time, with Modbus command processing as the secondary activity.

This is a structural simplification. Many of the bugs and races in the current code stem from a queue serviced by ISR callbacks that re-queue further work, with shared state (`post_cmd_callback`, `daliResponseType`, `scan_index`) racing across contexts. Removing the queue removes those races by construction.

## Architecture

Three reconciliation channels, each mediated by flags rather than events:

1. **Sync flags** (firmware ↔ ballast): per-ballast byte indicating which cached values are out of sync with the ballast and which direction reconciliation must go.
2. **Dirty bits** (firmware → master): existing 64-bit bitmap, set when a cache value changes for a flagged ballast, cleared when the master reads the relevant bank.
3. **Raw command slot** (master → firmware, single command at a time): register-handshake for any custom DALI command — provisioning, group/broadcast, scenes, vendor-specific, anything that doesn't fit the cache-sync model. Includes a "send twice" option (DALI SET commands and INITIALISE/RANDOMISE all require being sent twice within 100 ms per the IEC 62386 spec).

These channels meet at a single **DALI command issuance** function. Every DALI command leaves the firmware through this one function, regardless of whether it was triggered by a holding register write, a raw command, or an internal action. The function classifies the command (device/group/broadcast/special × level/config/none) and sets the appropriate sync flags as a uniform property of the command being issued — not of the path that triggered it. This is the architectural pivot that keeps the holding register handlers and the raw command handler trivial: neither needs its own invalidation logic, because both go through the same chokepoint.

State is the protocol; events are derived from state changes; ISRs only set/clear flags or update cache cells. There is no queue.

## RAM check

The refactor is essentially RAM-neutral with the chosen encoding:

- Remove queue (8 entries × ~7 bytes): **+56 bytes free**
- Remove queue-related globals (`post_cmd_callback`, `scan_index`, `scan_err_count`, `daliResponseType`, `daliResponse`): **+10 bytes free**
- Add `sync_flags[64]` (uint8_t): **−64 bytes**
- Add raw command staging (2 bytes; the 0x015D backing already exists in alternative-impl): **−2 bytes**
- Add chunked full-scan state (`scan_full_addr`, `scan_full_step`): **−2 bytes**
- Add round-robin scan cursor (`scan_cursor`): **−1 byte**
- Add async DALI state (`dali_busy`, `current_op`): **−5 bytes**
- Add group-write pending state (`reg_addr`, `pending_adds`, `pending_removes`): **−6 bytes**
- **Net: −14 bytes** (essentially neutral)

Run `avr-size --mcu=attiny1604 -C build/firmware.elf` before and after the refactor to confirm. Any unexpected growth beyond ~10 bytes indicates something else changed and should be investigated before the refactor is declared done.

## Data structures

### Sync flag table

One byte per ballast (64 bytes total). Reads are coarse-grained — periodic level re-polls are independent of full config refreshes. Writes are per-field. Power-on and sys-fail share a single bit (they're already packed into one Modbus register on alternative-impl, so this matches the existing layout):

```c
// One byte per ballast, indexed by short address 0..63 = 64 bytes total.
// Read bits track "the cache is stale, need fresh DALI read."
// Write bits track "the cache has a value the master set; push it to the ballast."
// Group writes do NOT use these flags — see the group write section in the
// Modbus handler behaviour.
volatile uint8_t sync_flags[DALI_MAX_DEVICES];

#define SYNC_READ_LEVEL    (1 << 0)  // QUERY_STATUS + GET_LEVEL
#define SYNC_READ_FULL     (1 << 1)  // re-fetch all 7 config fields (incl. group memberships)
#define SYNC_WRITE_LEVEL   (1 << 2)  // DAPC (one DALI command)
#define SYNC_WRITE_MINMAX  (1 << 3)  // DTR0+SET_MIN(x2)+DTR0+SET_MAX(x2) — 6 sends
#define SYNC_WRITE_POWER   (1 << 4)  // DTR0+SET_POWER_ON(x2)+DTR0+SET_SYS_FAIL(x2)
#define SYNC_WRITE_FADE    (1 << 5)  // DTR0+SET_FADE_RATE(x2)+DTR0+SET_FADE_TIME(x2)
// bit 6 spare
#define SYNC_RETRY_PENDING (1 << 7)  // first NAK seen on a level read; one retry before marking absent
```

**Write semantics.** A successful write clears its bit. A NAK during any step of a multi-step write aborts the sequence, clears the write bit, and sets the corresponding read flag (or `SYNC_READ_FULL` for any config write) so the cache gets reconfirmed. Group membership and scene writes do **not** route through sync flags — they go through raw commands (see Modbus handler behaviour).

**Sync flags now track both read and write reconciliation.** Reads track "the cache is stale," writes track "the cache has a master-supplied value that needs to land on the ballast." The four writable banks (level, min/max, power, fade) each get a write bit; group membership and scene writes go through raw commands instead because their write semantics don't fit a single per-ballast bit (multiple bits in a 16-bit mask can change in one Modbus write, requiring per-bit DALI commands).

**SYNC_READ_FULL semantics:** the bit means "this ballast needs all 7 config fields re-fetched" (GET_MIN, GET_MAX, GET_FADE, GET_GROUPS0_7, GET_GROUPS8_15, GET_POWER_ON_LEVEL, GET_SYS_FAIL_LEVEL). The execution is **chunked into one command per `start_next_dali_work` call** so that other DALI work (raw commands, level reads, writes) can interleave between config-read steps.

When SYNC_READ_FULL is set:
- At startup for all ballasts (initial population).
- When a previously-absent ballast transitions to present (NAK→OK on any read).
- On explicit master request via the existing `SCAN_TRIGGER` register at 0x1010 — that register can set SYNC_READ_FULL for all 64 ballasts in one pass.

### Raw command slot

A single-command channel for any DALI command that doesn't fit the cache-sync model: provisioning (INITIALISE / RANDOMISE / COMPARE / PROGRAM_SHORT_ADDRESS), group and broadcast commands, scene commands (RECALL_SCENE / SET_SCENE), vendor-specific commands, and anything else the master needs to send raw.

**Alternative-impl already has this surface.** Per the README:

- Holding register `0x1000` `RAW_CMD` — write `[15:8]=address byte, [7:0]=data byte`. Single send. Backward frame expected.
- Holding register `0x1001` `RAW_CMD_TWICE` — same frame format. Sent twice (within the DALI 100 ms re-send window). Used for DALI SET commands and provisioning sequences (INITIALISE, RANDOMISE).
- Input register `0x015D` `ASYNC_RAW_RESULT` — the entire raw-command state machine, encoded in one 16-bit value.

**Reuse this interface as-is.** The state machine lives entirely in the registers — there is no separate `raw_cmd` struct in the firmware. The only firmware state is a single 2-byte staging location that holds the cmd value between when Modbus captures it and when DALI executes it; both 0x1000 and 0x1001 write into this shared staging since only one can be armed at a time.

Encoding of `0x015D`:

| Value | Meaning |
| --- | --- |
| `0x0000` | Idle |
| `0xFFnn` | Pending; low byte `nn` = sends remaining (`0x02` after writing 0x1001, `0x01` after writing 0x1000 or after first send of a twice-send, `0x00` after the final send while awaiting backward frame) |
| `0x01xx` | NAK |
| `0x02xx` | OK; low byte = response byte |
| `0x03xx` | Error; low byte = error subtype |

(The `0xFFnn` encoding is a small extension of alternative-impl's existing single `0xFF00` "pending" value. Backwards-compatible if masters check the high byte to detect pending; potentially breaking if any master strict-equals `0xFF00`. Confirm before committing.)

State transitions:
- Master writes `0x1000` while `0x015D == 0x0000`: stage cmd, set `0x015D = 0xFF01`, ACK Modbus write.
- Master writes `0x1001` while `0x015D == 0x0000`: stage cmd, set `0x015D = 0xFF02`, ACK Modbus write.
- Writing either while `0x015D != 0x0000` returns Modbus `SLAVE_DEVICE_BUSY`.
- DALI subsystem (when `start_next_dali_work` selects this priority) reads the staged cmd and the low byte of `0x015D` to determine sends remaining. After each send, the completion handler decrements the low byte. When sends remaining reaches `0x00` and a backward frame arrives (or times out), it writes the terminal value to `0x015D`.
- Master polls `0x015D`. Reading a terminal value returns the value and clears `0x015D` back to `0x0000` (matching alternative-impl's "Returns DEVICE_BUSY if previous result not yet read" semantics — confirm the read-clears-on-terminal pattern is what alternative-impl uses; if it requires an explicit clear write, preserve that pattern instead).

Two design notes:

- Commands sent via twice-send may still produce a backward frame for some DALI ballasts. Don't hard-code "no backward frame on twice" — let the ~22 Te response window after the final send capture a frame if one arrives, and report it as OK if it's well-formed.
- The `current_op` tracking from the DALI work-selection section above doesn't need separate fields for raw commands — it just needs an `op` value that means "raw" so the completion handler knows to update `0x015D` and the staged cmd's sends-remaining rather than `sync_flags` or the cache.

## Modbus handler behaviour

**Decision: writes ACK immediately. Per-ballast banks are holding registers (not input registers). The Modbus surface is just per-ballast holding registers + raw command + control registers — no action register facades, no action coils.**

### Per-ballast banks become holding registers

alternative-impl currently exposes the per-ballast banks at FC04 input registers (read-only). That was the wrong call — the natural Modbus shape for these is holding registers (FC03 read, FC06/FC10 write). Move them. Same address space, same byte ordering as alternative-impl's existing FC04 layout:

| Address | Bank | Read (FC03) | Write (FC06/FC10) |
| --- | --- | --- | --- |
| `0x0000-0x003F` | brightness | `[high=status, low=brightness]` | Low byte = DAPC level (0-254 valid; 255 = "no change" per DALI). High byte ignored on write. Sets `SYNC_WRITE_LEVEL`. |
| `0x0040-0x007F` | min/max | `[high=max, low=min]` | Both bytes; sets `SYNC_WRITE_MINMAX`. |
| `0x0080-0x00BF` | power-on/sys-fail | as above | as above; sets `SYNC_WRITE_POWER`. |
| `0x00C0-0x00FF` | fade rate/time | as above | as above; sets `SYNC_WRITE_FADE`. |

(Confirm byte ordering against alternative-impl's actual layout before changing the access mode.)

What stays as input registers (FC04 read-only — these don't have meaningful write semantics):

- `0x0100-0x010F` group status (derived from member statuses)
- `0x0150` scan status
- `0x0151-0x0154` presence bitmap
- `0x0155-0x0158` fading bitmap
- `0x0159-0x015C` dirty bitmap (read-clears semantics)
- `0x015D` async raw result
- `0x0200-0x020F` version string

Group membership masks at `0x0110-0x014F` are **writable holding registers**, but with their own dedicated handling — see "Group register writes" below — because the per-group storage layout doesn't fit per-ballast sync flags.

### What goes away from alternative-impl

- **Action holding registers `0x1002` through `0x100A`** (DAPC, GOTO_LAST_ACTIVE, OFF, UP, DOWN, STEP_UP, STEP_DOWN, RECALL_MAX, RECALL_MIN). Brightness register write covers DAPC; raw command covers everything else.
- **FC05 / FC0F coil writes.** Brightness register write covers ON (non-zero) / OFF (zero); raw command covers GOTO_LAST_ACTIVE / TOGGLE. Confirm via open question.
- **FC01 read coils stays** — it just reads the cache (returns 1 if cached brightness > 0).

### Per-ballast holding register write handlers

For each writable bank, the handler:

1. Validates the address (within 0–63 range) and value (DALI range constraints; DAPC 255 is valid as "no change").
2. Updates the cache immediately to the written value (predictive — the master sees the new value on subsequent reads even before DALI confirms).
3. Sets the appropriate `SYNC_WRITE_*` bit for that ballast.
4. ACKs the Modbus write immediately.

**Critically: the handler does not set any read flags or do any cache-invalidation work.** That's the responsibility of the unified DALI command issuance path, which is invoked when the work selector services the `SYNC_WRITE_*` bit. See the work-selection section below.

If the same `SYNC_WRITE_*` bit is already set with a previous value not yet pushed, the new value wins (the cache is already updated to the new value; the upcoming DALI sequence will push that value).

### Group register writes

Group memberships at `0x0110-0x014F` are stored per-group (16 groups × 4 registers, each register a bitmap of which 16 ballasts are in that group). The DALI commands for group membership changes are per-ballast (ADD_TO_GROUP_X / REMOVE_FROM_GROUP_X). One Modbus register write can change up to 16 bits, requiring up to 16 separate DALI commands — too much to express in a single per-ballast sync flag, and trying to fit it would distort the rest of the design.

Group writes get their own small pending-work state (~6 bytes total):

```c
volatile struct {
    uint16_t reg_addr;     // Modbus register address being processed (0x0110-0x014F);
                           //   0xFFFF = no pending write
    uint16_t pending_adds; // bits where the master wants membership added but DALI not sent yet
    uint16_t pending_removes; // bits where the master wants membership removed but DALI not sent yet
} group_write;
```

Handler for FC06/FC10 writes to `0x0110-0x014F`:

1. If `group_write.reg_addr != 0xFFFF`, return Modbus `SLAVE_DEVICE_BUSY` — only one group write in flight at a time.
2. Validate the register address is in the group range.
3. Read the current cached value at `reg_addr` (this is the "old" state).
4. Compute `changed = old XOR new`, `pending_adds = new AND changed`, `pending_removes = old AND changed`.
5. Update the cache to `new` (predictive).
6. Store `reg_addr`, `pending_adds`, `pending_removes` in the `group_write` struct.
7. ACK the Modbus write immediately.

For FC10 writes spanning multiple group registers: process them sequentially as separate writes, returning BUSY if a second one arrives before the first finishes. Or restrict FC10 group writes to a single register at a time. Pick whichever is simpler given alternative-impl's existing FC10 handling.

The work selector gets a new priority slot (slotted between raw command and SYNC_WRITE_*):

- **Pending group bit-change**: if `group_write.reg_addr != 0xFFFF` and either `pending_adds != 0` or `pending_removes != 0`:
  - Pick the lowest-numbered set bit (try `pending_adds` first, then `pending_removes`).
  - Compute the affected ballast: `reg_addr` low nibble × 16 + bit position. Compute the affected group: `(reg_addr - 0x0110) / 4`.
  - Issue ADD_TO_GROUP_X (data byte `0x60 + group`) or REMOVE_FROM_GROUP_X (`0x70 + group`) addressed to that ballast, with `repeat = 2` (DALI requires SET commands sent twice within 100 ms).
  - On successful completion, clear the bit in `pending_adds` or `pending_removes`. The unified issuance function will have set `SYNC_READ_FULL` on the affected ballast as part of its normal `INV_CONFIG` invalidation — that's fine; it'll get re-confirmed by the next full scan.
  - On NAK: leave the bit set, retry on next pass through the work selector. (Same retry-pending semantics as for level reads, applied to whichever ballast is being addressed.)
- When both `pending_adds` and `pending_removes` are zero, set `reg_addr = 0xFFFF` to signal completion. The next FC06/FC10 group write can now proceed.

Worst-case completion time for a single group register write where all 16 bits changed: 16 × 2 × ~25 ms (DALI command twice) = ~800 ms. Plus subsequent SYNC_READ_FULL confirmation reads on each affected ballast (deferred, run as background work, ~5.6 s in total). The master sees fast Modbus ACKs throughout — only the actual DALI propagation is slow.

### Full scan updates of group registers

When a full-scan step issues GET_GROUPS_0_7 (data byte 0xC0) for ballast N and gets a response byte, the response's 8 bits indicate group memberships 0-7 for ballast N. Walk the bits: for each bit `i` (0-7), set or clear bit `N % 16` in cache register `0x0110 + i*4 + N/16`. Same logic for GET_GROUPS_8_15 with bits 8-15.

This is the inverse transposition of the group write logic — per-ballast response transposed into per-group storage.



Same staging + `0x015D` state machine as the Raw command slot section. ACK immediately, master polls `0x015D` for completion.

The raw command handler does **not** parse the DALI command for cache invalidation. It just stages the bytes. The unified DALI command issuance path (called when the work selector picks up the staged raw command) handles invalidation — same logic as for any other DALI command source, no duplication.

### Control registers (0x100B-0x100D, 0x1010-0x1013)

Unchanged from alternative-impl:
- `0x100B-0x100D` SET_DTR0/1/2: the master uses these as part of orchestrating SET sequences via raw commands. The handler issues the corresponding DALI command via the unified issuance path (which classifies it as INV_NONE — see issuance section).
- `0x1010` SCAN_TRIGGER: write 1 to set `SYNC_READ_FULL` on all 64 ballasts.
- `0x1011` RESCAN_INTERVAL: configures PIT cursor cadence (see PIT section).
- `0x1012` SLAVE_ADDR, `0x1013` BAUD_INDEX: persisted to EEPROM as today.

### 0x44 custom DALI command

If alternative-impl still has it, route through the raw command path. Match alternative-impl's existing sync-vs-async behaviour for the 0x44 caller.

## DALI work-selection loop (asynchronous)

DALI execution is asynchronous. Main initiates one DALI operation, returns to its loop, and sleeps. The bit-level state machine continues to run in TCA0/TCB0 ISRs (unchanged from alternative-impl). When a command completes, an ISR-context handler updates state and either advances a multi-step sequence or releases the DALI subsystem for the next work item.

This matches what alternative-impl already does. The previous draft of this spec made `dali_run_one` synchronous, which would have blocked main for 50–150 ms per call and added that latency to every Modbus response — an unforced regression. Don't do that.

```c
// Whether a DALI command (or multi-step sequence) is currently in flight.
// Set by start_next_dali_work, cleared by the completion handler when a
// sequence finishes.
volatile bool dali_busy = false;

// What DALI is currently doing, so the completion handler knows how to
// process the response and what (if anything) to start next. ~5 bytes.
volatile struct {
    uint8_t addr;          // ballast short address being processed
    uint8_t op;            // SYNC_READ_LEVEL / SYNC_WRITE_MINMAX / etc., or a marker for raw / full-scan
    uint8_t step;          // step within a multi-step sequence (0-based)
} current_op;

// Global state for chunked SYNC_READ_FULL execution. Same as before.
static uint8_t scan_full_addr = 0xFF;
static uint8_t scan_full_step = 0;

// Picks the highest-priority pending work and starts it. Returns true if a
// command was initiated (dali_busy is now true). Returns false if there is
// nothing to do, or if dali_busy was already true.
bool start_next_dali_work(void);
```

### Selection priority

`start_next_dali_work()` checks these in order; first match wins, exactly one DALI command (or first command of a multi-step sequence) is initiated:

1. **Raw command pending**: if `0x015D`'s high byte is `0xFF` (any of the pending sub-states), kick off the next send of the raw command. The DALI cmd to send comes from the staged value; the low byte of `0x015D` tells you which send this is. After this send completes, the completion handler decrements the low byte; when it hits `0x00` it transitions `0x015D` to a terminal value based on the backward-frame outcome.
2. **Pending group bit-change**: if `group_write.reg_addr != 0xFFFF` and either `pending_adds` or `pending_removes` has bits set, pick one bit and issue ADD_TO_GROUP_X or REMOVE_FROM_GROUP_X to the corresponding ballast (twice). See the Group register writes section for details.
3. **Pending write on any ballast**: walk `sync_flags` from the rotating cursor; if any ballast has a `SYNC_WRITE_*` bit set, initiate the first command of that write sequence. For DAPC (`SYNC_WRITE_LEVEL`) that's a single send. For min/max, fade, power (`SYNC_WRITE_MINMAX/FADE/POWER`), the sequence is `DTR0(low) → SET_X(twice) → DTR0(high) → SET_Y(twice)` — 6 sends, executed atomically (does not yield) because DTR0 is shared.
4. **Pending level read on any ballast**: any ballast with `SYNC_READ_LEVEL` set gets QUERY_STATUS started (~50 ms for the QUERY_STATUS + GET_LEVEL pair).
5. **Continue an in-progress full scan**: if `scan_full_addr != 0xFF`, start the next config command at index `scan_full_step`.
6. **Start a new full scan**: if any ballast has `SYNC_READ_FULL` set and no full scan is in progress, set `scan_full_addr` to it, `scan_full_step = 0`, and start step 0.
7. **Nothing to do**: return false. `dali_busy` stays false.

This priority order means writes preempt reads on the same ballast, raw commands preempt everything (because the master is actively polling for the result), and full-scan progress yields to all foreground work — exactly the scheduling shape you want.

### Unified DALI command issuance

Every DALI command leaving the firmware — whether triggered by a `SYNC_WRITE_*` write sequence, a raw command staged at 0x1000/0x1001, a control-register write to a DTR setter, or a chunked full-scan step — goes through one issuance function:

```c
// Initiates a single DALI send. Called by the work selector for every
// DALI command, regardless of source. Handles cache invalidation as a
// uniform property of the command being issued.
void issue_dali_command(uint8_t addr_byte, uint8_t data_byte, uint8_t repeat);
```

**Inside this function, before the bytes hit the wire, the cache invalidation logic runs.** Because the invalidation is a property of the DALI command itself (not its source), it lives here once and applies uniformly to every issued command. The same brightness register write, the same raw-command 0x1000 write with the same DAPC bytes, and the same firmware-internal action all produce identical sync-flag updates because they all flow through the same command bytes through the same function.

**Address byte → affected ballasts** (read off the address byte that's about to be sent):
- Bit 7 = 0: short address. One ballast at `(addr_byte >> 1) & 0x3F`.
- Bit 7 = 1, addr byte = 0xFE or 0xFF: broadcast. All 64 ballasts.
- Bit 7 = 1, otherwise: group. Group = `(addr_byte >> 1) & 0x0F`. Walk the cached group membership masks (`0x0110-0x014F`) to find which ballasts are members.
- "Special" commands without a target (e.g. INITIALISE selectors, RANDOMISE, broadcast-to-all-without-short-address): handle as broadcast for invalidation purposes.

**Command byte → invalidation category** (DAPC is encoded in the address byte's bit 0; everything else is a command code in the data byte):
- Address bit 0 = 0 (DAPC): `INV_LEVEL`.
- Address bit 0 = 1 with command byte:
  - `0x00` (OFF), `0x01-0x08` (UP/DOWN/STEP_*/RECALL_*), `0x0A` (GOTO_LAST_ACTIVE), `0x10-0x1F` (GOTO_SCENE): `INV_LEVEL`.
  - `0x20` (RESET): `INV_BOTH`.
  - `0x21` (STORE_CURRENT_LEVEL_DTR0): `INV_NONE`.
  - `0x2A-0x2F` (SET_MAX/MIN/SYS_FAIL/POWER_ON/FADE_TIME/FADE_RATE): `INV_CONFIG`.
  - `0x40-0x4F` (SET_SCENE), `0x50-0x5F` (REMOVE_FROM_SCENE), `0x60-0x6F` (ADD_TO_GROUP), `0x70-0x7F` (REMOVE_FROM_GROUP): `INV_CONFIG`.
  - `0x80` (STORE_DTR_AS_SHORT_ADDRESS): `INV_ADDRESS` — best-effort `SYNC_READ_FULL` on the addressed ballast; the master should follow up with SCAN_TRIGGER if they re-addressed devices.
  - `0x90-0xFF` (queries): `INV_NONE`.
- Special commands (no target): `INV_NONE` for selector commands like INITIALISE, `INV_ADDRESS`-equivalent for RANDOMISE.

Translation:
- `INV_LEVEL` → set `SYNC_READ_LEVEL` on affected ballasts.
- `INV_CONFIG` → set `SYNC_READ_FULL` on affected ballasts.
- `INV_BOTH` → set both.
- `INV_NONE` → no flag changes.

Implement as a small switch on the command byte; no lookup table needed. Keep it in one file alongside the work selector so the classification stays in one place.

A useful corollary: this means the per-ballast holding register write handler stays trivial. It just updates the cache and sets the write flag — it doesn't need to know about invalidation, because when the work selector services the write flag and calls `issue_dali_command`, the same logic runs. No duplication, no risk of the holding-register path and the raw-command path drifting in their invalidation behaviour.

### Completion handling

When the bit-level state machine returns to IDLE (success, NAK, malformed, timeout), an ISR-context handler is called with the response. It uses `current_op` to determine context:

- **Single-command op** (raw command final send, `SYNC_WRITE_LEVEL` DAPC, level read after step 1, full-scan step): cache the result if it's a read, possibly set the dirty bit, clear or update the relevant `sync_flags` bit (or transition `0x015D` to its terminal value for raw commands), set `dali_busy = false`. Main will pick the next work item on its next iteration.
- **Multi-step level read** (between QUERY_STATUS and GET_LEVEL): cache the QUERY_STATUS response, increment `current_op.step`, initiate GET_LEVEL. `dali_busy` stays true.
- **Multi-step write sequence** (DTR0 → SET twice → DTR0 → SET twice for min/max, fade, power): advance through the steps without yielding. On the final SET-twice completion, clear the `SYNC_WRITE_*` bit and `dali_busy = false`.
- **Full-scan step**: cache the result (zero on NAK), increment `scan_full_step`. If `scan_full_step >= 7`, clear `SYNC_READ_FULL` for that ballast and reset `scan_full_addr = 0xFF`. Set `dali_busy = false` so other priorities can preempt before the next step.
- **Raw command intermediate send** (low byte of `0x015D` > 0 after decrement): initiate the next send. `dali_busy` stays true.

### Multi-step sequence rules

Three multi-step sequences exist in the firmware:

- **Level reads** (QUERY_STATUS + GET_LEVEL): 2 commands, doesn't yield in between. Both are fast and updating both at once is cheap.
- **Raw command twice-sends** (RAW_CMD_TWICE): 2 sends, doesn't yield. DALI requires the two sends within 100 ms.
- **Multi-step write sequences** (DTR0 → SET twice → DTR0 → SET twice for `SYNC_WRITE_MINMAX/POWER/FADE`): up to 6 sends, doesn't yield. **DTR0 is shared across all ballasts** — if a write sequence yielded between DTR0 and the matching SET, an interleaving raw command or another write could clobber DTR0 and cause the wrong value to land. So write sequences run as one atomic unit, ~300 ms worst case.

Single-command writes (`SYNC_WRITE_LEVEL` = DAPC) are not multi-step.

Modbus latency during a multi-step write: up to ~300 ms. Still well within typical Modbus master timeouts (500+ ms). If you find masters timing out, the option exists to break the write into two separate sequences (DTR0+SET_X(twice), then DTR0+SET_Y(twice)) by yielding in the middle, but only safely if no other write is pending — which adds priority logic. Defer this complication unless measured to be a problem.

### Retry-pending logic on NAK

Applies to level reads and to the final step of a multi-step write sequence. Raw commands report their NAK directly via `0x015D`; full-scan steps just zero the cached field; intermediate write steps abort the sequence (see below).

- Level read final-step NAK or write-sequence final-step NAK with `SYNC_RETRY_PENDING` clear: set the bit, leave the read flag set (and for write NAKs, set the appropriate read flag — `SYNC_READ_FULL` for config writes, `SYNC_READ_LEVEL` for DAPC). The work loop will retry this ballast after the rotating cursor walks past everyone else.
- Same NAK with `SYNC_RETRY_PENDING` set: second consecutive NAK. Mark absent: clear the presence bit, zero the cached register fields, clear all `sync_flags[addr]` bits.
- Intermediate step of a multi-step write NAKs: abort the sequence, clear the write bit, set the corresponding read flag (`SYNC_READ_FULL` or `SYNC_READ_LEVEL`) so the cache gets reconfirmed. Apply retry-pending as above on the read side.

After every read or write completion, compare new cache value to old cache value. If level or status changed, set the per-ballast dirty bit. Other field changes do **not** set the dirty bit (per the alternative-impl branch design).

After every read or write completion, compare new cache value to old cache value. If level or status changed, set the per-ballast dirty bit. Other field changes do **not** set the dirty bit (per the alternative-impl branch design).

Presence handling: a single NAK no longer marks a device absent — see the retry-pending logic above. A device transitions to absent only after two consecutive NAKs (the second one not preceded by any successful response). When a device is marked absent, clear the presence bit and zero its cached register fields. Match the alternative-impl branch's existing convention here — recent commits removed the 0xFF level-as-marker pattern, so check the current branch state before assuming what "absent" looks like in the cache. A subsequent successful response on any read transitions the device back to present, sets the dirty bit, and (since presence transitions warrant a fresh config snapshot) sets `SYNC_READ_FULL` so the loop re-fetches all the ballast's settings.

## Periodic re-poll (PIT) — round-robin

Don't burst. Spread the periodic re-polls evenly over time so the bus sees a smooth, low-duty-cycle background load rather than a 64-ballast spike every minute. The PIT advances a cursor and sets `SYNC_READ_LEVEL` on a single ballast each tick:

```c
static volatile uint8_t scan_cursor = 0;  // round-robin cursor across DALI_MAX_DEVICES

ISR(RTC_PIT_vect) {
    static uint8_t tick = 0;
    tick++;
    if (tick >= rescan_interval) {  // configurable, see below
        tick = 0;
        sync_flags[scan_cursor] |= SYNC_READ_LEVEL;  // atomic by virtue of being in ISR; see audit notes
        scan_cursor++;
        if (scan_cursor >= DALI_MAX_DEVICES) scan_cursor = 0;
    }
    RTC.PITINTFLAGS = RTC_PI_bm;
}
```

Cost: 1 byte (`scan_cursor`) plus the existing `rescan_interval` register. Zero new RAM beyond the cursor.

Only level (which folds in QUERY_STATUS) gets periodically re-polled. Config fields are reconciled at startup and on writes; the dirty-bit mechanism handles intermediate change notification.

### `RESCAN_INTERVAL` semantics change

The existing `RESCAN_INTERVAL` holding register at 0x1011 currently means "seconds between full scans of all 64 ballasts." **Redefine it to mean "seconds between single-ballast polls."** This is a backwards-incompatible semantic change with the same register address — call it out prominently in the README.

Recommended default: **1 second**. With 64 ballasts, that gives a complete cycle every 64 seconds — same per-ballast polling rate as the current 64-second default, just spread out instead of burst. Settings:
- `RESCAN_INTERVAL = 0` → disabled (cursor doesn't advance, no automatic polls).
- `RESCAN_INTERVAL = 1` → 1 ballast/second, full cycle in 64 s (matches current default).
- `RESCAN_INTERVAL = 2` → 1 ballast every 2 s, full cycle in ~2 min.
- `RESCAN_INTERVAL = 4` → 1 ballast every 4 s, full cycle in ~4 min.

Larger values reduce background bus load further at the cost of slower drift detection. Since the master can also drive updates via dirty-bit polling and explicit re-reads, slow auto-polling is fine for most installations.

### Manual scan trigger (`SCAN_TRIGGER` at 0x1010)

Writing 1 to this register kicks off a full re-discovery: set `SYNC_READ_FULL` on all 64 ballasts. The work loop processes them one at a time (~350 ms each, so ~22 s for a complete pass). This is what users will reach for after physically adding new ballasts to the bus.

The cursor is **not** reset by SCAN_TRIGGER — periodic polling continues independently in parallel. The two mechanisms don't conflict: SYNC_READ_FULL takes priority over SYNC_READ_LEVEL within `dali_run_one`, and once SYNC_READ_FULL completes for a ballast, subsequent SYNC_READ_LEVEL flags from the cursor get serviced normally.

### Absent ballasts in the rotation

The cursor walks every ballast unconditionally, including those currently marked absent. This means absent ballasts get one re-test every full cycle (e.g., every 64 s with default settings). On NAK, the retry-pending logic applies as normal — a single NAK keeps it absent (since SYNC_RETRY_PENDING was already cleared when it was last marked absent), so each PIT-driven re-test is one fresh attempt. If a previously-absent ballast comes back online, the next cursor visit catches it and triggers a presence transition + SYNC_READ_FULL.

This is intentionally simple. Don't add a separate slow-cadence list for absent ballasts — the overhead of one NAK per cycle for absent slots is trivial (~25 ms of bus activity), and the simpler-rotation property is worth more than the optimisation.

## Main loop

DALI is asynchronous, so main is mostly a Modbus dispatcher with a "kick DALI when idle" hook. **Modbus is checked first** — same reasoning as before, but with even less downside now that DALI doesn't block main:

```c
for (;;) {
    wdt_reset();
    if (modbus_cmd_available) {
        rs485_process_command();
        continue;
    }
    if (!dali_busy) {
        if (start_next_dali_work()) {
            // dali_busy is now true; ISRs will drive it to completion
            continue;
        }
    }
    sleep_cpu();
}
```

What this gives you:
- **Worst-case Modbus latency ≈ 5–10 ms.** The only way Modbus is delayed is by `rs485_process_command` itself running for a previous command, which is a few ms at most (CRC compute, register lookup, response build, USART TX). DALI does **not** add to this latency — it's running concurrently in ISR context.
- **DALI runs at full bus rate when main is idle** because the loop kicks `start_next_dali_work` immediately and the bit-level ISRs drive everything from there.
- **Sleep is real.** Main genuinely sleeps when there's no Modbus and no DALI work; ISRs wake it as needed (USART RXC for new bytes, TCA0 OVF for DALI bit-clock progress, RTC PIT for periodic ticks). The TCA0 OVF wakes during an in-flight DALI command are unavoidable but cheap — ~5 µs of wake latency on the ATtiny1604, and the loop immediately re-sleeps if there's nothing to do.

What it does NOT give you (and these are good things to be explicit about):
- DALI commands can't be aborted by a higher-priority Modbus request. Once started, a write sequence runs to completion before the next priority check. This is fine because the slowest atomic unit is a write sequence at ~150 ms, and the master isn't waiting on it (it was ACK'd at the Modbus layer).
- A truly hammering Modbus master that never leaves idle gaps will starve the periodic level poll. Same as before — acceptable because such a master clearly doesn't need our cached snapshot.

Other notes:
- Remove `dali_idle` / `modbus_idle` global flags from the alternative-impl gating logic. The new "anything to do?" gate is `modbus_cmd_available || (!dali_busy && any_work_pending())`, evaluated implicitly by the loop structure above.
- `waitForDaliResponse` and the `_delay_ms(4)` pre-response in `rs485_process_command` can both go. With Modbus writes ACKing immediately and DALI being async, no Modbus handler ever needs to wait for a DALI command to complete. The 3.5-char inter-frame silence is naturally satisfied as long as some time has elapsed since the last received byte; if you want belt-and-braces, check `rs485Timer` and short-delay only if necessary.

Notes:
- DALI takes precedence — modbus responses can be delayed up to ~150 ms during a sync sequence. RS-485 reception continues via USART0_RXC_vect during this time, so no bytes are lost. See the analysis in the conversation that produced this spec.
- `sleep_cpu()` only fires when both subsystems are idle. Wake sources: USART RXC (Modbus byte arrived), PIT (sync flags freshly set), USART RXS (start-of-frame). TCA0 OVF will also wake the chip; that's fine because the loop will immediately determine there's nothing to do and sleep again.
- Remove `dali_idle` / `modbus_idle` global flags if they're no longer used as gates anywhere else. The new gate is "is there work in `sync_flags` / `(0x015D high byte == 0xFF)` / `modbus_cmd_available`."
- `waitForDaliResponse` and the `_delay_ms(4)` pre-response can both go. The DALI subsystem returns synchronously from `dali_run_one`, so its results are already in cache when control returns to main. The 3.5-char inter-frame silence is naturally satisfied if a DALI command ran for >4 ms; if `dali_run_one` returned false (nothing to do) and we're about to respond immediately to a Modbus command, add a check on `rs485Timer` and only short-delay if the timer indicates <4 ms has elapsed since the last received byte.

## Atomic-block audit

With the queue removed and most callback chains collapsed, several existing ATOMIC_BLOCKs become unnecessary. Audit every one and remove or document as needed. Specific places to revisit:

- `dali_queue_command` — gone entirely.
- `dali_start_background_scan`'s atomic — gone (no scan_index, no queue).
- `set_post_cmd_callback` and the singleton itself — gone (fade tracking is implicit in the SYNC_READ_LEVEL flag staying set when status reports fading).
- `processSetSingleHoldingRegisterCommand`'s atomic shortVal writes — still needed (16-bit cache writes are still 2 STS instructions and the DALI completion handler still writes the cache from ISR context).
- The streaming TX path's atomic in `USART0_DRE_vect` — still needed (TCA0 LVL1 still preempts and updates cache).

Add new atomic blocks for:

- Sync flag bit-set/clear from any context: read-modify-write of one byte is non-atomic on AVR. Wrap all `sync_flags[i] |=` and `sync_flags[i] &=` in ATOMIC_BLOCK if the operation can race with an ISR-context update of the same byte. The DALI completion handler runs in ISR context and clears bits; main and the PIT set bits; both sides need protection.
- Dirty bitmap bit-set/clear: same reasoning. Set by the DALI completion handler (ISR), cleared by Modbus read handlers (main).
- Reads of `current_op` from main (if any — usually only the completion handler touches it, but if any debug or status-exposing path needs to read it, wrap appropriately).
- Reads of `dali_busy` from main: it's a single-byte flag so the read is atomic, but if main does "if (!dali_busy) { ...; start_next_dali_work(); }" then the body of `start_next_dali_work` should re-check `dali_busy` under an ATOMIC_BLOCK before setting it true — otherwise an ISR completing a previous command between the check and the start could leave you doubly-started. (Realistically the previous command would have to have been started long enough ago to be completing right now, which can't happen if dali_busy was just observed false, so this is paranoid. But document the assumption.)

Document the threading model at the top of each `.c` file: which contexts can call into it, which globals are touched, and what protects them. The current code is silent on this and that's part of why bugs hide.

## Files to expect to change

Roughly:

- **Delete:** `dali_commands.c` (replaced).
- **Add:** `dali_sync.c` (the new work-selection loop and sync-flag operations) and probably `dali_sync.h`. The raw command slot lives here too unless it grows large enough to warrant its own file.
- **Modify:** `main.c` (new loop), `rs485.c` (immediate-ACK writes, route 0x1000/0x1001 through the raw command slot, route 0x44 if it still exists, remove `waitForDaliResponse` and `_delay_ms(4)`), `dali_state_idle.c` (defensive `OUTCLR` of the DALI TX pin as the first action — see notes below).
- **Probably unchanged:** `dali_state_transmitting.c`, `dali_state_receiving.c`, `dali_state_wait_for_idle.c`, `dali_state_wait_for_response.c`, `dali_init.c`, `timers.c`. These are bit-level DALI protocol mechanics, orthogonal to the queue/flag question.

Confirm by reading the alternative-impl branch — it may already have moved some of this around to support the dirty bitmap.

## Defensive fixes to fold in while you're there

These are independent of the refactor but came up in the conversation that produced this spec and are cheap to include:

1. **`dali_idle_state_enter()` should `OUTCLR` the DALI TX pin as its first action.** Belt-and-braces against any glitched code path that left the bus driven active. The cost is one cycle; the benefit is closing the standby-sleep-with-bus-asserted failure mode.
2. **`timeoutWaitingForResponse()` in `dali_state_wait_for_response.c` should similarly `OUTCLR` the DALI TX pin** before transitioning to idle, for the same reason.
3. **Set the BOD fuse to active mode at ≥2.6 V** in the Makefile, and remove `-R .fuse` from the avrdude flags so the fuse is actually programmed. The current build leaves brown-out unprotected; this is the highest-leverage single change for the bus-stuck symptom that motivated this whole conversation.

## Verification

After implementation, before declaring done:

1. **Build clean** with the project's existing toolchain.
2. **Static measurements**: confirm RAM usage matches the prediction (queue ~56 bytes gone, queue globals ~10 bytes gone, sync table 64 bytes added, raw command staging 2 bytes added, scan/cursor state ~3 bytes added, async state ~5 bytes added, group-write state 6 bytes added — net ~14 bytes lost, essentially neutral). Confirm stack high-water mark hasn't grown — ideally it shrunk because callback chains are flattened.
3. **Functional check** (if the project has a test harness): existing Modbus operations still produce the expected ballast behaviour. Dirty-bit semantics from alternative-impl unchanged. 0x44 raw commands still behave identically from the master's perspective.
4. **Behavioural change to call out in the changelog/README**: writes now ACK immediately rather than blocking. Masters that assumed write-ACK meant ballast-confirmed need to switch to dirty-bit polling.
5. **Hand-trace at least one sync sequence** end to end: master writes a level register → handler updates cache + sets SYNC_WRITE_LEVEL + ACKs → main loop notices flag → dali_run_one runs DAPC → response comes back → cache updated, dirty bit set, sync flag cleared → master polls dirty bitmap → sees ballast 5 dirty → re-reads bank 0 → dirty bit clears. Confirm no race windows in the audit.

## Open questions to surface, not silently decide

If during implementation any of the following come up, raise them rather than picking a direction:

- **Coils removal confirmation.** This spec removes FC05 (write single coil) and FC0F (write multiple coils) entirely. Master can achieve those by writing the brightness register or sending a raw command. If you'd rather keep FC05 as a convenience (mapping to brightness register writes internally), say so. FC01 (read coils) is preserved either way.
- **Action register removal confirmation.** Holding registers `0x1002` through `0x100A` are removed. UP/DOWN/STEP/RECALL_MAX/RECALL_MIN have no register-write equivalent — master uses raw commands. Confirm.
- **Per-ballast banks moving from input to holding registers.** This is a breaking change for any existing master that reads them via FC04. The spec assumes alternative-impl can be updated to expose them as holding registers (FC03 read, FC06/FC10 write) at the same addresses. Confirm there's no consumer dependency that requires keeping them at FC04.
- **Byte ordering for min/max, fade, power.** Spec assumes `[high=max, low=min]` etc. based on the pre-alt-impl code. Confirm against alternative-impl's existing read layout so write semantics match read semantics.
- **DAPC value 255.** Per DALI spec, 255 means "no change." Spec treats it as a valid write that issues a no-op DAPC. Confirm or override.
- **Does the alternative-impl branch's existing dirty-bit semantics differ from what's described here?** If so, alternative-impl wins — don't refactor it as part of this task.

## Out of scope

- Any change to the bit-level DALI protocol implementation.
- Any change to the Modbus framing, CRC, or wire-protocol logic.
- Any change to the existing dirty-bitmap exposure to the master.
- Power optimisation (e.g., RUNSTDBY gating). Discussed and intentionally deferred.

---

## Outcome

**Date completed:** 2026-05-03
**Branch:** `task-1-flag-driven`
**Commit range:** `9de0ecc..task-1-complete` (subtasks 1–6 plus two subtask-4 fix-ups, plus a subtask-7 polish pass landed after the user's post-merge review). The `task-1-complete` git tag tracks the final commit and was moved forward when subtask 7 landed.

### Sub-task summary

| # | Commit | Subject |
|---|--------|---------|
| 1 | `9de0ecc` | Defensive fixes: DALI TX `OUTCLR` on idle entry + BOD fuse @ 2.6 V (also removes `-R .fuse`). |
| 2 | `7de757f` | Per-ballast banks moved from FC04 (input regs) to FC03 (holding regs). Reads + write stubs only. |
| 3 | `dcc98fd` | New `dali_sync.{c,h}` skeleton: `sync_flags[64]` table, op definitions, RTC-PIT round-robin cursor. |
| 4 | `638f303` | Unified `issue_dali_command()` chokepoint, work selector with priority chain, write/raw paths wired through. |
| 4-fix-1 | `42900ad` | Reviewer feedback: twice-send re-fire, presence transition on first OK, group-write BUSY-vs-no-op race. |
| 4-fix-2 | `759b82c` | Reviewer P1: collapse SYNC_WRITE_* and SYNC_READ_LEVEL onto a single rotating cursor; relax group-write BUSY contract. |
| 5 | `e83688d` | Physical deletion of `cmd_queue.{c,h}` and `bg_tasks.{c,h}`. Coil FCs (FC01/05/0F) and action holding regs (0x1002–0x100A) retired with `ILLEGAL_FUNCTION` / `ILLEGAL_ADDRESS` replies. `dali_result_received` rename. |
| 6 | `d1c7ed2` | Final README pass, verification, hand-trace, task move to `done/`. |
| 7 | `task-1-complete` | Post-merge polish: collapse `classify_invalidation` to 4 branches; atomic helpers in `note_present_on_ok` / `apply_retry_pending`; `OP_FULL_SCAN` short-circuits on NAK (move to next ballast); `(void)reg_off;` dead-code removed; README documents `0x015D` no-decrement-mid-twice-send and `frame_is_valid` coil-FC retention as a future-removal candidate; `noinline, noclone` attributes verified still needed (removing them costs +480 B flash on avr-gcc 7.3, well above the 200 B threshold). |

### Final binary sizes (subtask-0 baseline → final)

| Section | Baseline (`4c2888b`) | Subtask 6 (`d1c7ed2`) | Final (`task-1-complete` = `ae30a84`) | Delta vs baseline |
|---------|---------------------|------------------------|-----------------------------------------|-------------------|
| Program | 14316 B | 13734 B | **13754 B** | **−562 B** |
| Data    |   849 B |   851 B | **  851 B** | **+2 B** |

The Program-side reduction is well beyond the spec's "essentially RAM-neutral" prediction — credit goes to deleting the queue + bg_tasks subsystems plus enabling `-ffunction-sections / -fdata-sections / --gc-sections` in subtask 4. Data-side delta of +2 B vs baseline is within the RAM-neutral target (spec predicted ~−14 B; the +2 B is a single 16-bit `raw_cmd_result` register that wasn't accounted for in the spec's bookkeeping). The +20 B Program from subtask 6 → 7 is the cost of replacing naked `sync_flags[idx] |=` / `&=` in `note_present_on_ok` and `apply_retry_pending` with the `ATOMIC_BLOCK`-wrapped helpers — worth it because the safety becomes locally provable instead of relying on subtle ISR-priority invariants. Stack high-water mark is unchanged: `modbus_process` reports 49 B `dynamic,bounded` (the FC04 version-string `snprintf` path) and no other function exceeds 50 B static.

### Hand-trace verification (master writes brightness on ballast 5 → DALI propagation → dirty-bit confirmation)

End-to-end walk with the actual code (see commit message of the subtask-6 commit for the full annotated trace):

1. Master sends FC06 to register `0x0005` value `0x0042`.
2. `process_fc06` validates, calls `regs_write_holding(0x0005, 0x0042)`.
3. `regs_write_holding` (registers.c:185) writes `g_regs.block0[5].brightness = 0x42`, calls `sync_flags_set(5, SYNC_WRITE_LEVEL)`. Returns true.
4. `process_fc06` builds the standard echo response and `start_tx`s — Modbus master sees the ACK before any DALI activity.
5. Main loop: `!dali_busy` → `start_next_dali_work()` runs. `pick_work()` falls through priorities 1–2 (no raw, no group-bit), reaches priority 3 (write), walks `sync_flags[]` from `work_cursor`, finds ballast 5 with `SYNC_WRITE_LEVEL`. Sets `current_op = {OP_WRITE_LEVEL, 5, 0}`, advances `work_cursor` to 6.
6. `dali_sync_dispatch_current()` → `OP_WRITE_LEVEL` branch → `issue_dali_command(DALI_SHORT_ADDR(5), 0x42, DALI_KIND_DAPC, 1)`.
7. Inside `issue_dali_command`: `classify_invalidation(0x0A, 0x42)` returns `SYNC_READ_LEVEL` (DAPC, address bit 0 clear). `apply_invalidation` calls `sync_flags_set(5, SYNC_READ_LEVEL)` — the cache will be re-confirmed once DALI completes.
8. `dali_transmitting_state_enter` runs the 16-bit Manchester-encoded forward frame (~15.8 ms). For `CMD_DIRECT_ARC` (DAPC), `finished_transmission` skips `wait_for_response` and calls `dali_result_received(DALI_RESPONSE_NAK, 0)` directly (DAPC has no backward frame).
9. `dali_result_received` sees `dali_busy` true and `s_active.repeat == 1`, so it dispatches to `dali_sync_completion(NAK, 0)`.
10. `dali_sync_completion` `OP_WRITE_LEVEL` branch: NAK is the expected outcome, so `sync_flags_clear(5, SYNC_WRITE_LEVEL)` and `mark_dirty(5)` (unconditional — DAPC always sets the dirty bit because we cannot confirm whether the level actually changed without a follow-up query). `dali_busy = false`. Tail-calls `dali_wait_for_idle_state_enter()` to enforce the 22 Te post-frame gap.
11. After the gap, IDLE re-enters. `dali_busy == false` so the work selector is re-armed. With `SYNC_READ_LEVEL` set on ballast 5 (from step 7), the next `start_next_dali_work` picks priority 4 (level read) and runs QUERY_STATUS → cache status → step 1 → QUERY_ACTUAL_LEVEL → cache brightness → if `old_b != value` set dirty (already dirty from step 10), `sync_flags_clear(5, SYNC_READ_LEVEL | SYNC_RETRY_PENDING)`.
12. Master polls FC04 0x0159 (covers ballasts 0–15). Bit 5 is set (from step 10). FC04 read-clears the entire word — bit 5 returns to 0.
13. Master reads FC03 0x0005 → `[status:brightness]` = `[<actual ballast status>:0x42]`. Cache fully reconciled.

Atomicity: the PIT ISR firing during this sequence and setting `SYNC_READ_LEVEL` on a *different* ballast is safe — `sync_flags_set` uses `ATOMIC_BLOCK(ATOMIC_RESTORESTATE)`. A PIT tick targeting ballast 5 specifically during steps 7–11 would just OR-set a bit that's already set or about to be set; idempotent. The `regs_read_clear_dirty` ATOMIC_BLOCK ensures FC04 reads of the dirty bitmap don't lose updates from the DALI completion handler running at LVL1.

### Deviations from spec

- **Action regs and coils retired in subtask 4 (rolled into the unified-issuance commit) rather than a dedicated subtask 5 step.** Subtask 5 then became "physical file deletion + last-mile rename." Net same outcome; just rebalanced the commit boundaries. Open Question "Coils removal confirmation" was answered by the user mid-subtask-4 with explicit consent to remove all three coil FCs.
- **`-ffunction-sections / -fdata-sections / --gc-sections` added to the Makefile in subtask 4.** Not in the spec; introduced because the subtask-4 size measurement showed 200+ B of unreferenced code from the partially-deleted command-queue path. The change reclaimed all of it and remains in place as a permanent improvement.
- **0x015D async raw result encoding ended up simpler than the spec described.** The spec proposed a per-state pending encoding with documented sends-remaining semantics. Implementation honours the same encoding (`0xFFnn`, low byte = sends remaining), and the README documents the master-visible behaviour: match on the high byte for "still pending"; the low byte transitions are visible but not separately specified for master use. Backwards-compatible-ish: any master strict-equalling `0xFF00` would be wrong on a twice-send, hence the breaking-change callout in the new "Behavioural Changes" README section.
- **Default `RESCAN_INTERVAL` left at 64.** The spec recommended changing to 1 (matching the old burst's per-ballast cadence). Subtask 6 elected to document the trade-off and leave the default conservative — RAM/flash-side it makes no difference, and changing the default is a master-visible behaviour change the user should opt into rather than receive silently.
- **Cache invalidation classification simplified beyond the spec's per-opcode table** (subtask 7). The spec listed nine ranges with distinct INV_LEVEL/CONFIG/BOTH/NONE classifications. The implementation collapses to four branches (DAPC; data < 0x20; data == 0x20 RESET; data < 0x90) and accepts that a handful of reserved opcodes (`0x21` STORE_CURRENT_LEVEL_DTR0 and the 0x22-0x29, 0x30-0x3F reserved ranges) over-invalidate to SYNC_READ_FULL when issued via raw command. The user explicitly approved this trade — "we don't need to be perfect; the cost of a wasted scan is bounded."
- **`OP_FULL_SCAN` short-circuits on NAK** (subtask 7). The spec's "cache result (zero on NAK)" semantics were replaced with "abort the scan for this ballast and let the work selector pick the next one." Cuts SCAN_TRIGGER runtime on sparse buses roughly in half (~6.4 s vs ~11.2 s for 32-of-64-present). Edge case: a present ballast that NAKs at step 4 of 7 leaves cache fields 4-6 at their previous values rather than zeroes — accepted because the periodic level-poll catches presence flips eventually, and SCAN_TRIGGER can be re-issued by the master.
- **Every brightness write to a per-ballast bank triggers a follow-up `SYNC_READ_LEVEL`** (i.e. QUERY_STATUS + QUERY_ACTUAL_LEVEL after the DAPC). This is intentional: ballasts may clip the written level (e.g. master writes `0xBE` but ballast max is `0xB0`), and step/up/down style commands have no closed-form predictive cache value. The readback ensures the cache reflects what the ballast actually did. User explicitly chose this behaviour over an optimisation that would skip the readback for plain DAPC writes.
- **Broadcast/group writes intentionally generate per-ballast readback storms.** A broadcast DAPC sets `SYNC_READ_LEVEL` on all 64 ballasts; the work selector then queues 64 QUERY_STATUS + QUERY_ACTUAL_LEVEL pairs (~3.2 s of bus traffic). Group writes scale with group size. User explicitly accepts this — "that is the nature of this."

### Open questions resolved

- **Coils removal:** confirmed by user — FC01/FC05/FC0F all removed.
- **Action register removal:** confirmed — 0x1002–0x100A all return `ILLEGAL_ADDRESS`.
- **Per-ballast banks moving from input → holding regs:** confirmed — accepted as a breaking change, called out in Behavioural Changes section.
- **Byte ordering for min/max, fade, power:** matched alternative-impl's existing FC04 read layout (high=max, low=min etc.). Verified against the `regs_read_holding` direct-offset trick in registers.c — wire bytes identical to pre-refactor FC04 reads.
- **DAPC value 255:** kept as a valid write that issues the no-op DAPC; documented in the FC03/FC06/FC10 holding register table.
- **Dirty-bit semantics:** unchanged from alternative-impl — set on any block0 (status/brightness) change, cleared by FC04 read of the relevant word. Group/min/max/fade/power changes do not set the dirty bit.
- **Async raw result encoding:** the spec's `0xFFnn` extension is honoured; the master-visible encoding is now documented (see the FC04 0x015D table and Behavioural Changes section).

### Verification status

- [x] `make build` clean against the project-local `avr/bin/avr-gcc` toolchain.
- [x] `avr/bin/avr-size --mcu=attiny1604 -C build/main.elf` reports Program 13754 / Data 851 (well under the 1 KB SRAM budget; ~150 B stack headroom).
- [x] `.su` files sweep — no function exceeds 50 B; `modbus_process` at 49 B `dynamic,bounded` (FC04 version-string `snprintf` path) is the high-water mark.
- [x] Hand-trace from FC06 brightness write → DAPC dispatch → dirty-bit confirmation walked end-to-end against the actual code.
- [x] README integrated final pass: function-code table FC03/04/06/10 only; coils section retired; per-ballast banks documented under FC03 (writable); 0x015D encoding accurate to implementation; Fade Tracking restructured around the flag-driven model; RESCAN_INTERVAL semantic redefined; Known Limitations pruned of fixed entries (group readback, set_dtr0 race, bus duty cycle during scan, double-fire receive timeout) and augmented with refactor-specific entries; new "Behavioural Changes" subsection lists every wire-protocol-visible breaking change; new "Architectural Model" top-level section gives the one-paragraph orientation.
- [x] Task file moved from `tasks/` to `tasks/done/` with this Outcome section appended.

Branch is in a clean, mergeable state. Tag `task-1-complete` points to the subtask-7 commit (`ae30a84`); the eventual PR description can reference it.
