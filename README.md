# TinyModbus DALI Gateway

An ATtiny1604-based Modbus RTU to DALI gateway. It acts as a Modbus slave device and translates Modbus register reads/writes into DALI commands, allowing any Modbus-capable system (PLCs, building automation controllers, SCADA) to control DALI lighting.

The PCB design is in the `pcb` folder. It is set up to hold an RS-485 transceiver and the DALI bus driver circuitry on a compact board.

![PCB](pcb/TinyModbus.png)

---

## DALI Protocol

### Overview

DALI (Digital Addressable Lighting Interface) is a two-wire serial lighting control protocol standardised as IEC 62386. It allows individual control of up to 64 ballasts/drivers on a single bus, with support for groups, scenes, and bidirectional communication (status queries).

### Physical Layer

DALI runs over a simple two-wire bus. Key characteristics:

| Parameter | Value |
|-----------|-------|
| Nominal bus voltage | 16 V DC |
| Operating range | 9.5 – 22.5 V |
| Max bus current | 250 mA |
| Max cable distance | 300 m |
| Max cable voltage drop | 2 V |
| Topology | Free-form (star, daisy-chain, mixed) — ring not allowed |
| Polarity | Insensitive (either wire can be "+" or "−") |

The bus is powered by a dedicated DALI power supply. Bus-powered devices (sensors, push-button interfaces) draw power directly from it.

Signal levels:
- Logic **1** (idle/high): 16 V ± 6.5 V
- Logic **0** (active/low): 0 V ± 4.5 V

### Manchester (Biphase) Encoding

DALI uses **Manchester encoding** (also called biphase encoding) at **1200 bps**.

Each bit period is 833 µs. The bit is split into two equal half-periods of **416.67 µs** each (called Te in the standard):

```
Logic 1:  LOW  for first half-bit, HIGH for second half-bit
          ___
     ___|   |

Logic 0:  HIGH for first half-bit, LOW for second half-bit
      ___
     |   |___
```

Manchester encoding guarantees a transition in the middle of every bit, which provides self-clocking and ensures the bus never stays continuously high or low for longer than one half-bit period. This also means the DALI power supply is delivering current for at least half the time during any transmission.

### Frame Format

**Forward frame** (controller → ballast): 1 start bit + 8 address bits + 8 data bits + 2 stop bits = **19 bits**, transmitted MSB first. Total duration ≈ 15.8 ms.

```
 S  A7 A6 A5 A4 A3 A2 A1 A0  D7 D6 D5 D4 D3 D2 D1 D0  P P
 |←─── address byte ────────→|←─── data byte ─────────→|
```

S = start bit (logic 1), P = stop bits (logic 1, return bus to idle)

**Backward frame** (ballast → controller): 1 start bit + 8 data bits + 1 stop bit = **10 bits**. Duration ≈ 9.95 ms. The ballast must respond within 7–22 Te (≈ 2.9 – 9.2 ms) after the forward frame ends.

### Address Byte Structure

```
 A7  A6  A5  A4  A3  A2  A1  A0
  |   |   |   |   |   |   |   |
  |   └───┴───┴───┴───┴───┘   └── Command type: 0=DAPC, 1=Command
  └─────────────────────────────── Address type: 0=Short, 1=Group/Broadcast
```

| A7 | A6–A1 | A0 | Meaning |
|----|-------|----|---------|
| 0  | addr (0–63) | 0 | Direct Arc Power Control (DAPC) to short address |
| 0  | addr (0–63) | 1 | Command to short address |
| 1  | group (0–15) | 0 | DAPC to group |
| 1  | group (0–15) | 1 | Command to group |
| 1  | 111111 | 0/1 | Broadcast (A0=0 → DAPC, A0=1 → command) |

The per-ballast holding-register banks in this firmware use short addresses (0–63). The raw command registers (0x1000 / 0x1001) accept any valid DALI address byte, so group and broadcast commands are also supported through them.

### Command Types

**Direct Arc Power Control (DAPC)** — sets light output immediately:
- Data byte: 0 = off, 1–254 = proportional level (logarithmic), 255 = no change

**Standard commands** (address byte LSB = 1) — the data byte selects the command:

| Command | Data byte | Description |
|---------|-----------|-------------|
| Off | 0x00 | Turn off (no fade) |
| Up | 0x01 | Fade up by one step at the configured fade rate |
| Down | 0x02 | Fade down by one step at the configured fade rate |
| Step Up | 0x03 | Step up one level (no fade) |
| Step Down | 0x04 | Step down one level (no fade) |
| Recall Max | 0x05 | Go to maximum level |
| Recall Min | 0x06 | Go to minimum level |
| Go To Last Active | 0x0A | Restore last non-zero level |
| Go To Scene N | 0x10–0x1F | Recall scene 0–15 |
| Reset | 0x20 | Reset to factory defaults |
| Set DTR0 | (special) | Loads a value into the data transfer register before a set command |
| Query Status | 0x90 | Returns 8-bit status byte (backward frame) |
| Query Actual Level | 0xA0 | Returns current arc power level (backward frame) |
| Query Max Level | 0xA1 | Returns max arc power level |
| Query Min Level | 0xA2 | Returns min arc power level |
| Query Power On Level | 0xA3 | Returns power-on default level |
| Query System Failure Level | 0xA4 | Returns system failure default level |
| Query Fade Time/Rate | 0xA5 | Returns fade settings |
| Query Groups 0–7 | 0xC0 | Returns bitmask of group membership |
| Query Groups 8–15 | 0xC1 | Returns bitmask of group membership |

**Set commands** (require value pre-loaded into DTR0, and must be sent twice within 100 ms to take effect):

| Command | Data byte | Description |
|---------|-----------|-------------|
| Set Max Level | 0x2A | Set maximum arc power level from DTR0 |
| Set Min Level | 0x2B | Set minimum arc power level from DTR0 |
| Set System Failure Level | 0x2C | Set level on bus/power failure |
| Set Power On Level | 0x2D | Set level on power restore |
| Set Fade Time | 0x2E | Set fade time from DTR0 |
| Set Fade Rate | 0x2F | Set fade rate from DTR0 |
| Add To Group N | 0x60–0x6F | Add device to group 0–15 |
| Remove From Group N | 0x70–0x7F | Remove device from group 0–15 |

### Status Byte (Query Status Response)

| Bit | Meaning |
|-----|---------|
| 0 | Ballast status (0=ok, 1=failure) |
| 1 | Lamp failure |
| 2 | Arc power on |
| 3 | Limit error (commanded level was below min or above max) |
| 4 | Fade running |
| 5 | Reset state |
| 6 | Missing short address |
| 7 | Power cycle event |

### Fade

DALI supports smooth transitions between light levels. Two parameters control fading:

- **Fade time**: fixed-duration transition (0–15, where 0 = instant, 1 = 0.7 s, doubling each step up to ~90 s)
- **Fade rate**: steps-per-second mode (0–15, where 0 = no fade, 1 = 358 steps/s down to 1 step/s)

### Groups and Scenes

**Groups (0–15):** A device can belong to multiple groups simultaneously (stored as a 16-bit bitmask). A single group command updates all members simultaneously in one DALI transmission — much faster than addressing devices individually.

**Scenes (0–15):** Each device stores 16 preset arc power levels. A "Go To Scene N" broadcast takes the entire installation to a preset state in one frame.

### Bus Timing Constraints

| Constraint | Value |
|------------|-------|
| Min inter-frame gap | ≥ 22 Te ≈ 9.2 ms |
| Backward frame response window | 7–22 Te after forward frame |
| Set command re-send window | ≤ 100 ms between the two required sends |
| DALI frame duration (forward) | ≈ 15.8 ms |
| DALI frame duration (backward) | ≈ 9.95 ms |

### Addressing and Commissioning

New or reset devices generate a 24-bit random address. A controller discovers and assigns short addresses (0–63) using a binary search algorithm:

1. Broadcast `RANDOMISE` to generate new random addresses
2. Repeat: set a search high address, issue `COMPARE` — devices with random address ≤ search address pull the bus low
3. Binary-search down to isolate one device
4. Issue `PROGRAM SHORT ADDRESS` to assign the next free address
5. Issue `YES` to confirm, then loop until no devices respond

This firmware **implements control-gear commissioning** — see `Commissioning (FC03/FC06 0x1014)` under the Modbus Interface section for the master-facing surface. Master writes 0x0001 to `MBREG_COMMISSION` for full re-addressing or 0x0002 for incremental (assigns short addresses only to gears that don't already have one); reads the same register to poll status. Out of scope for this firmware: 24-bit forward frames (DALI-2 control-device addressing), memory-bank reads, and DT1/DT8 device-type extensions.

---

## Architectural Model

The firmware is a **flag-driven state-reconciliation loop**. Every per-ballast cached field (brightness, status, min/max, power-on/sys-fail, fade rate/time, group memberships) lives in a single `g_regs` register map exposed directly over Modbus. A 64-byte `sync_flags[]` table tracks per-ballast reconciliation work — read-flags say "the cache is stale, refresh from the ballast", write-flags say "the master supplied a value, push it to the ballast". A Modbus write updates the cache predictively and sets the appropriate write-flag, then ACKs at the wire layer immediately.

The main loop kicks `start_next_dali_work()` whenever DALI is idle. The work selector walks a fixed priority chain — commissioning state machine → raw command → pending group bit → SYNC_WRITE_* (rotating cursor) → SYNC_READ_LEVEL (same cursor) → continue in-progress full scan → start new full scan — and issues exactly one DALI command via `issue_dali_command()`. Cache invalidation is handled inside `issue_dali_command` as a uniform property of the command bytes — every DALI command from every source (per-ballast bank write, raw command, commissioning step, internal full-scan step) goes through the same chokepoint, so the invalidation logic lives once and the per-source handlers stay trivial. Commissioning sits at the top of the chain so a master raw write of e.g. `0xA3VV` (DTR0 load) can't clobber the staged DTR0 mid-bisect.

Completion runs in ISR context (`dali_sync_completion`) and either advances a multi-step sequence (level read = QUERY_STATUS + GET_LEVEL; multi-step write = DTR0 + SET twice + DTR0 + SET twice) or releases `dali_busy` for the next pick. Periodic level re-polls are spread evenly via the RTC-PIT cursor (one ballast per `RESCAN_INTERVAL` seconds, round-robin across all 64 slots) instead of bursting. Masters detect when reads have caught up by polling the dirty bitmap (FC04 0x0104–0x0107, read-clears).

There is no command queue, no event-callback chain, and no synchronous DALI work in the main loop. State *is* the protocol; events are derived from state changes; ISRs only set/clear flags or update cache cells.

---

## DALI State Machine

The firmware implements a five-state bit-level machine that drives all DALI bus activity. Every state transition is interrupt-driven — the main loop simply sleeps until a Modbus command arrives or the PIT-driven sync-flag cursor sets a flag the work selector picks up.

### Timer Architecture

Two hardware timers underpin the state machine:

- **TCA0** — the half-bit clock. Runs continuously at 10 MHz with a period of exactly one DALI half-bit (416.67 µs). On each overflow it decrements a countdown (`dali_timer`) and fires a registered callback (`dali_timer_cb`) when the countdown reaches zero. This drives both transmission timing and state-machine timeouts. Runs at CPU interrupt level 0.
- **TCB0** — the pulse-width capture timer. Active only during the RECEIVING state. Configured in Frequency Measurement mode and fed by the DALI RX pin via the AVR event system. It captures the time between consecutive edges, giving the width of each Manchester pulse without CPU polling. Configured as CPU interrupt level 1 so it can preempt TCA0 mid-bit for accurate edge timestamping.

### States

```
                    ┌─────────────────────┐
       sync work    │        IDLE         │◄────────────────────────────┐
       pending      │  (dali_idle = true) │                             │
          ┌─────────┤  watch for external │                             │
          │         │  TX on RX pin       │                             │
          │         └──────────┬──────────┘                             │
          │                    │ rising edge on RX                      │
          ▼                    │ (another master transmitting)          │
  ┌───────────────┐            ▼                                        │
  │ TRANSMITTING  │    ┌───────────────┐      22 Te bus idle            │
  │               │    │ WAIT_FOR_IDLE ├──────────────────────────────►─┤
  │ TCA0 fires    │    │               │◄─────────────────────────────┐ │
  │ each half-bit │    │ watches for   │  after RECEIVING             │ │
  └───────┬───────┘    │ bus to settle │  (ok or malformed)           │ │
          │            └───────────────┘                              │ │
          │ all bits + stop bits sent                                 │ │
          ▼                                                           │ │
  ┌───────────────┐                                                   │ │
  │  WAIT_FOR_    │  no response within 22 Te ─────────────────────►──┘ │
  │  RESPONSE     │                              NAK → back to IDLE     │
  │               │  rising edge on RX                                  │
  └───────┬───────┘  (backward frame starting)                          │
          │                                                             │
          ▼                                                             │
  ┌───────────────┐  8 bits decoded + timeout                           │
  │  RECEIVING    ├───────────────────────────────────────────────────►─┘
  │               │  (RESPONSE_OK or MALFORMED → WAIT_FOR_IDLE)
  │  TCB0 pulse   │
  │  capture ISR  │
  └───────────────┘
```

### State Descriptions

**IDLE** (`dali_state_idle.c`)

The bus is quiet and we own it. The entry path branches on `dali_busy`:
- If `dali_busy` is true, a sync op is in flight and we just finished the inter-frame gap. Clear `dali_idle`, re-check the bus, and dispatch the next step of the in-flight op via `dali_sync_dispatch_current()` (which picks the right command for the current `OP_*` and calls `issue_dali_command`).
- Otherwise, `OUTCLR` the DALI TX line (belt-and-braces release), set `dali_idle = true` (allowing the main loop to sleep or to call `start_next_dali_work()` to launch a new sync op), prepare the receive timer, and arm a one-shot rising-edge interrupt on RX. If a rising edge arrives it means another master has started transmitting, so we yield via `dali_wait_for_idle_state_enter()` until the bus is quiet again.

**TRANSMITTING** (`dali_state_transmitting.c`)

Sends the 16-bit forward frame using Manchester encoding driven entirely by TCA0 callbacks:

1. Load the 16-bit command word into `shift_reg`, set `bits_left = 17`.
2. Schedule `send_one()` via `dali_timer = 1` (one half-bit tick).
3. `send_one()` drives TX high and schedules `half_bit_toggle()`.
4. `half_bit_toggle()` toggles TX (completing the mid-bit transition), schedules the next bit's first-half function (`send_one` or `send_zero` from the MSB of `shift_reg`), shifts `shift_reg` left, decrements `bits_left`.
5. When `bits_left == 0`, schedules `start_stop_bits()`, which drives TX low for 4 Te (two full stop bits).
6. After stop bits: CMD_QUERY and CMD_SEND_TWICE call `dali_wait_for_response_state_enter()`. CMD_SEND and CMD_DIRECT_ARC call `dali_result_received(DALI_RESPONSE_NAK, 0)` directly (no response expected; skips the 22 Te response window to halve inter-command latency).

Each bit takes exactly 2 TCA0 ticks (2 × 416.67 µs = 833.33 µs = 1200 bps).

**WAIT_FOR_RESPONSE** (`dali_state_wait_for_response.c`)

After the forward frame the master must listen for a backward frame within 7–22 Te. This state:
- Pre-configures TCB0 for pulse capture (`dali_state_receiving_prepare()`).
- Arms a one-shot rising-edge interrupt on RX. A rising edge means a backward frame start bit is arriving; we call `dali_state_receiving_enter()`.
- Sets a TCA0 timeout of 22 Te. If it fires before any edge, the device did not respond: call `dali_result_received(DALI_RESPONSE_NAK, 0)` and return to IDLE.

Commands that do not expect a response (e.g. DAPC, Off) still pass through this state; the NAK timeout simply confirms silence and the state machine moves on.

**RECEIVING** (`dali_state_receiving.c`)

Decodes the Manchester-encoded backward frame (8 data bits) using TCB0 pulse-width capture. Every edge on the RX pin triggers `TCB0_INT_vect`, which:
1. Reads the captured pulse width in timer ticks.
2. Classifies the pulse as *short* (≈ half-bit, ±20%) or *long* (≈ full-bit, ±20%). Pulses outside either window are errors.
3. Looks up the next state and the bit to shift into `shiftreg` from a compact transition table:

```
State       Short pulse → (next state, bit)   Long pulse → (next state, bit)
STARTBIT    MIDDLE_1,  no shift               ERROR
START_0     MIDDLE_0,  shift 0                ERROR
START_1     MIDDLE_1,  shift 1                ERROR
MIDDLE_0    START_0,   no shift               MIDDLE_1, shift 1
MIDDLE_1    START_1,   no shift               MIDDLE_0, shift 0
```

The *short* pulse is a half-bit transition (mid-bit); the *long* pulse spans a full bit and carries the same bit value as the previous bit (a "stretched" transition at the bit boundary).

TCA0 is re-armed to 5 Te on every edge. When TCA0 fires with no new edge — i.e. the bus has been idle for more than two full bits — `timer_overflow()` is called:
- If the final state is `START_0` or `MIDDLE_1` **and** exactly 8 bits were received: `dali_result_received(DALI_RESPONSE_OK, shift_reg)`.
- Otherwise: `dali_result_received(DALI_RESPONSE_MALFORMED, 0)`.

Either way, transitions to WAIT_FOR_IDLE to enforce the minimum inter-frame gap.

**WAIT_FOR_IDLE** (`dali_state_wait_for_idle.c`)

Enforces the DALI requirement that the bus be idle for at least 22 Te before the next forward frame. Works by watching the RX line:
- If the bus is high (shorted/active), arm a level-low interrupt and wait for release.
- Once the bus goes low (idle), start a TCA0 countdown of 22 Te.
- If the bus goes high again before the countdown expires (another master, collision debris, capacitive ringing), restart.
- When 22 Te completes uninterrupted: call `dali_idle_state_enter()`.

### Sync-flag work selector

Each ballast has one byte in `sync_flags[64]` (`dali_sync.c`) tracking pending reconciliation:

- `SYNC_READ_LEVEL` — periodic level re-poll (QUERY_STATUS + QUERY_ACTUAL_LEVEL, two queries one logical op).
- `SYNC_READ_FULL` — re-fetch all 7 config fields (min, max, fade rate/time, power-on, sys-fail, groups 0–7, groups 8–15). Chunked one query per work-selector pass so other DALI work can interleave.
- `SYNC_WRITE_LEVEL / MINMAX / POWER / FADE` — master-supplied value cached predictively in `g_regs`, awaits push to the ballast. The MINMAX/POWER/FADE sequences are 4-step (DTR0 + SET twice + DTR0 + SET twice) and run atomically because DTR0 is shared across all ballasts.
- `SYNC_RETRY_PENDING` — first NAK seen on a level read or a write-final step; one retry before marking the ballast absent.

`start_next_dali_work()` walks the priority chain (commissioning state machine → raw command → pending group bit → SYNC_WRITE_* → SYNC_READ_LEVEL → continue full scan → start full scan) and, when work is found, dispatches one DALI command via `dali_sync_dispatch_current()` → `issue_dali_command()`. The unified issuance function classifies cache invalidation as a property of the outgoing address + data bytes — DAPC and step/recall commands set `SYNC_READ_LEVEL` on the affected ballast(s); SET, scene, and group-membership commands set `SYNC_READ_FULL`; queries and DTR loads set nothing. Completion is dispatched in ISR context by `dali_sync_completion`, which advances the multi-step sequence or releases `dali_busy` for the next pick.

### Periodic Rescan

The RTC Periodic Interrupt Timer fires every ~1 second (1024 cycles of the 1 kHz internal oscillator).  Each PIT tick increments a counter; once it reaches `RESCAN_INTERVAL` (0x1011), the firmware sets `SYNC_READ_LEVEL` on **one** ballast at the cursor position and advances the cursor (mod 64).  `RESCAN_INTERVAL = 0` disables the cursor entirely.  Writing 0x0001 to `SCAN_TRIGGER` (0x1010) sets `SYNC_READ_FULL` on all 64 ballasts in one pass, kicking off a full re-discovery.

**Semantic change vs pre-refactor firmware**: `RESCAN_INTERVAL` used to mean "seconds between burst rescans of the entire bus."  It now means "seconds between single-ballast level polls."  The full-cycle latency at default `RESCAN_INTERVAL = 64` is therefore 64 ballasts × 64 s ≈ 68 minutes — much slower than the previous 64 s burst.  This is a deliberate trade-off: the round-robin cursor never bursts, so background bus duty is effectively zero between polls and never spikes.

Settings:

- `RESCAN_INTERVAL = 0` — disabled (cursor never advances, no automatic polls).
- `RESCAN_INTERVAL = 1` — 1 ballast/s, full cycle in 64 s. Equivalent per-ballast polling rate to the old default 64 s burst, but spread evenly. Recommended for installations that need fast drift detection.
- `RESCAN_INTERVAL = 64` (default) — 1 ballast/min, full cycle ≈ 68 min. Recommended when masters rely on the dirty bitmap (FC04 0x0104–0x0107) for change notification rather than firmware-driven polls.
- Larger values trade slower drift detection for even lower bus load.

The default is intentionally conservative; the firmware does not change it. Tune via `make configure RESCAN_INTERVAL=...` as needed.

### Multi-Master Behaviour

The firmware detects transmissions from other DALI masters on the bus: if a rising edge arrives on RX while in IDLE, the state machine yields by entering WAIT_FOR_IDLE. It waits for the other master's transaction (forward frame + optional backward frame) to complete and the bus to settle for 22 Te before re-asserting control. The firmware does not implement active collision detection during its own transmissions.

---

## Modbus Interface

### Connection

Modbus RTU, RS-485, **9600 baud, 8N1**.

Slave address, baud rate, and rescan interval all live in USERROW so they survive a chip erase + reflash. The factory-fresh (post-erase) USERROW value of `0xFF` maps to address 1 / 9600 baud / 64 s rescan. Writes to holding registers 0x1011 / 0x1012 / 0x1013 trigger a USERROW page-erase-write; the runtime baud/slave switch is deferred until the FC06/FC10 response we are about to send has cleared the wire, so the master sees the response at the old config and the next request at the new one.

### Function Codes

| Code | Name | Description |
|------|------|-------------|
| 0x03 | Read Holding Registers | Read per-ballast cache, group masks, and gateway config (max 16 regs per request) |
| 0x04 | Read Input Registers | Read presence + dirty bitmaps, raw-command result, version block |
| 0x06 | Write Single Holding Register | Update per-ballast cache, group mask, raw command, or gateway config |
| 0x10 | Write Multiple Holding Registers | Write a contiguous block of holding registers |

Coils (FC01 / FC05 / FC0F) were retired in subtask 5: per-ballast on/off / level changes go through the per-ballast brightness bank (0x0000-0x003F) which arms the asynchronous DALI work selector and ACKs immediately; broadcast / group / specialty DALI sends go through the raw command channel (0x1000 / 0x1001).  Frames using the retired function codes get an ILLEGAL_FUNCTION exception reply.

### FC04 — Input Register Map

Read-only state derived from DALI traffic. Registers are 16-bit, addressed from 0x0000.

| Address range | Width | Content |
|---------------|-------|---------|
| 0x0100–0x0103 | 4 regs | Present bitmask — one bit per ballast, 1 = responding |
| 0x0104–0x0107 | 4 regs | Dirty bitmask — set on any block0 change, cleared when read |
| 0x0108 | 1 reg | Async raw command result (see FC06 0x1000 / 0x1001) |
| 0x0200–0x020F | 16 regs | Version string: `DALI v<maj>.<min> <10-byte-serial-as-20-hex-chars>` — read exactly 16 registers |

A master can fetch presence + dirty in a single 8-register FC04 read at 0x0100. FC04 reads outside these ranges return `ILLEGAL_ADDRESS`. The per-ballast banks formerly at 0x0000–0x00FF and the group membership masks formerly at 0x0110–0x014F have moved to the holding-register map below — this is a breaking change from earlier firmware revisions.

### FC03/FC06/FC10 — Holding Register Map

Per-ballast cached DALI state (read-write) and gateway control registers.

| Address range | Width | Content |
|---------------|-------|---------|
| 0x0000–0x003F | 64 regs | Per-ballast status \| brightness. FC03 returns the cached `[status:brightness]` pair. FC06/FC10 write: low byte = DAPC level (0–254 valid; 255 = "no change" per DALI), high byte ignored. |
| 0x0040–0x007F | 64 regs | Per-ballast min \| max level. Both bytes consumed on write. |
| 0x0080–0x00BF | 64 regs | Per-ballast power-on \| sys-fail level. Both bytes consumed on write. |
| 0x00C0–0x00FF | 64 regs | Per-ballast fade rate \| fade time. Both bytes consumed on write. |
| 0x0110–0x014F | 64 regs | Group membership masks — 16 groups × 4 registers (64-bit bitmask per group). Writes are predictively cached and propagate per-bit via ADD_TO_GROUP / REMOVE_FROM_GROUP DALI commands by the work selector. |

**Per-ballast bank writes ACK immediately at the Modbus layer.**  The actual DALI command that pushes the change to the ballast is dispatched asynchronously by the sync-flag work selector — the master no longer blocks waiting for DALI propagation.  To detect when reconciliation has completed, poll the dirty bitmap (FC04 0x0104-0x0107, read-clears semantics).  This is a behavioural change vs pre-refactor firmware where writes blocked until DALI completion.

| Address | Name | Read | Write |
|---------|------|------|-------|
| 0x1000 | RAW_CMD | 0 | `[15:8]` = first DALI frame byte, `[7:0]` = second byte. Single send; waits for backward frame. Result in FC04 0x0108. Returns DEVICE_BUSY if previous result not yet read. |
| 0x1001 | RAW_CMD_TWICE | 0 | Same frame format as 0x1000 but sent twice within the DALI 100 ms re-send window (used for SET / INITIALISE / RANDOMISE). |
| 0x1010 | SCAN_TRIGGER | 0 | Write 0x0001 to set SYNC_READ_FULL on all 64 ballasts (full re-discovery). |
| 0x1011 | RESCAN_INTERVAL | R/W | Seconds between single-ballast level re-polls (round-robin cursor). Default 64; full cycle = `RESCAN_INTERVAL × 64` seconds. Write 0 to disable. See "Periodic Rescan" above for the semantic change vs pre-refactor firmware. |
| 0x1012 | SLAVE_ADDR | R/W | Modbus slave address 1–247. Persisted to USERROW on write. |
| 0x1013 | BAUD_INDEX | R/W | 0 = 9600, 1 = 19200, 2 = 38400. Persisted to USERROW on write. |
| 0x1014 | COMMISSION | status word | Trigger / abort / poll the IEC 62386-102 control-gear commissioning state machine. See "Commissioning (FC03/FC06 0x1014)" below. |

The DALI address byte for raw command registers (0x1000, 0x1001) can be any valid DALI address byte — short address, group address, broadcast (0xFE), or special command (e.g. 0xA3 SET_DTR0).  Cache invalidation runs uniformly on every issued command via `issue_dali_command`, regardless of the master's choice between the per-ballast bank shortcut and a raw command — neither path needs explicit invalidation logic at the call site.

### Async Raw Command Result (FC04 0x0108)

After writing to 0x1000 (RAW_CMD) or 0x1001 (RAW_CMD_TWICE), poll FC04 register 0x0108 for the result.  Reading the register at a terminal state (high byte 0x01 / 0x02 / 0x03) returns the value AND clears it back to 0x0000 atomically — so the master can detect completion in a single FC04 read without a follow-up clear write.

| High byte | Low byte | Meaning |
|-----------|----------|---------|
| 0x00 | 0x00 | Idle — no command in progress (or master has just consumed the previous result) |
| 0xFF | sends remaining | Pending — high byte 0xFF, low byte = number of DALI sends remaining at arming (0x02 right after writing 0x1001; 0x01 right after writing 0x1000). The low byte does NOT decrement mid-twice-send: a RAW_CMD_TWICE stays at `0xFF02` from arming until both sends have completed and a terminal value is written. The two sends are coalesced into a single completion (the bit-level state machine handles the inter-send 22 Te re-fire) for simpler completion semantics — masters only need to distinguish "pending" from "terminal", not count sends. Match the high byte for the pending check (`(value & 0xFF00) == 0xFF00`); the only visible mid-pending value is `0xFF02`, but treating any `0xFFxx` as pending is the safe rule. |
| 0x01 | 0x00 | NAK — no backward frame received within the 22 Te response window |
| 0x02 | response byte | OK — backward frame received |
| 0x03 | error subtype | Error — bus/timing fault (1 = malformed, 2 = pulse too short, 3 = pulse in middle, 4 = pulse too long) |

### Commissioning (FC03/FC06 0x1014)

`MBREG_COMMISSION` (holding register 0x1014) drives the IEC 62386-102 control-gear commissioning state machine — the bisect-based discovery + addressing protocol that lets the gateway assign short addresses (0–63) to fresh ballasts without an external commissioning tool.

**Write encoding** (FC06 to 0x1014):

| Value | Action |
|-------|--------|
| 0x0000 | Abort: send TERMINATE, return to idle. No-op when already idle or in a terminal state (master must read 0x1014 to clear those). |
| 0x0001 | Start **full** commissioning: INITIALISE(all), wipe existing per-ballast cache, assign 0..N−1 to discovered ballasts. **Destructive** — overwrites any existing addressing. |
| 0x0002 | Start **incremental** commissioning: INITIALISE(without short addr), only address ballasts that don't currently have one. Already-addressed ballasts are not disturbed; new gear takes the lowest unused short addresses. |
| anything else | ILLEGAL_VALUE |

Writing 0x0001 or 0x0002 while a run is already in progress returns DEVICE_BUSY. 0x0000 always succeeds (no-op when idle / terminal).

**Read encoding** (FC03 of 0x1014) — high byte encodes phase, low byte encodes count or error subtype:

| High byte | Low byte | Meaning |
|-----------|----------|---------|
| 0x00 | 0x00 | Idle — no run ever, or last run completed and master has consumed status |
| 0x01 | count | Running — `count` ballasts addressed so far this run |
| 0x02 | total | Complete — `total` short addresses assigned this run (read-clears) |
| 0x03 | subtype | Error / aborted (read-clears). Subtype: 0x01 = aborted by master, 0x02 = no ballasts found, 0x03 = >3 consecutive verify failures, 0x04 = ran out of short addresses (more ballasts than free slots), 0x05 = bus shorted longer than 200 TE during commissioning (run aborted) |

**Read-clears semantics on terminal states**: reading 0x1014 when the high byte is 0x02 or 0x03 returns the value AND resets the underlying state to 0x0000 atomically — same shape as FC04 0x0108. Idle (0x00) and running (0x01) reads are non-clearing so masters can poll progress.

**Concurrency**: OP_COMMISSION is the highest-priority work-selector slot — concurrent raw commands and per-ballast bank writes that arrive mid-run are queued and execute after the run finishes. Successful PROGRAM_SHORT_ADDRESS additionally arms `SYNC_READ_FULL` on the just-addressed ballast, so the cache populates without a manual SCAN_TRIGGER.

**Timing**: a full-bus commissioning of 64 ballasts takes ~3 minutes at 1200 baud (~3 s/ballast for the bisect chain — 24 search rounds × 4 sends + program/verify/withdraw). The master can poll 0x1014 at any rate; the count low byte advances monotonically as PROGRAM/VERIFY rounds succeed.

**Aborting**: writing 0x0000 mid-run latches an abort request — the next dispatch boundary forces TERMINATE, then idle. Status latches to 0x0301 before TERMINATE issues so a master poll between abort-request and TERMINATE-completion still observes the aborted state.

### Write propagation and fade tracking

Brightness register writes are **predictive**: the master writes a new level to the per-ballast brightness register (0x0000–0x003F low byte), the firmware updates the cached value immediately, sets `SYNC_WRITE_LEVEL` on that ballast, and ACKs the Modbus write at the wire layer. The DALI DAPC command that pushes the level to the ballast is dispatched asynchronously by the work selector when DALI is next idle — typically within a few milliseconds, but the master is no longer blocked. Once the DAPC completes, the per-ballast dirty bit is set; on the next master read of the brightness bank, the dirty bit clears.

There is no separate fade-polling chain. The fade-running indicator is bit 4 of the per-ballast status byte (FC03 0x0000–0x003F high byte), updated whenever a `SYNC_READ_LEVEL` pass issues `QUERY_STATUS` against that ballast. The work selector visits each ballast on the round-robin cursor (one per `RESCAN_INTERVAL` seconds), so a fade in progress is detected the next time the cursor reaches it; the dirty bit (0x0104–0x0107) flips on every status-byte change and notifies the master that a fresh read of block0 is worth doing. Masters that need fast fade visibility should lower `RESCAN_INTERVAL` toward 1 (full cycle in 64 s) or trigger an explicit `SCAN_TRIGGER` re-discovery, then watch the dirty bitmap and re-read block0 to observe FADE_RUNNING clearing.

### Multi-Master Operation

The firmware is designed to operate as the sole DALI master on its bus. It can detect when another master is transmitting (rising edge on the RX pin) and will yield rather than collide — but it does not decode traffic it did not initiate. If another DALI master changes ballast levels, the FC04 register data will remain stale until the next round-robin level re-poll visits the affected ballast (full cycle = `RESCAN_INTERVAL × 64` seconds) or a `SCAN_TRIGGER` re-discovery. For reliable register state, use this device as the exclusive DALI master, lower `RESCAN_INTERVAL` toward 1, or have the master poll the dirty bitmap (FC04 0x0104-0x0107) for change notification.

---

## Hardware

### Pin Assignment (ATtiny1604)

| Pin | Signal |
|-----|--------|
| PA1 | RS-485 TX (USART0 alternate TXD) |
| PA2 | RS-485 RX (USART0 alternate RXD) |
| PA4 | RS-485 DE — driver enable, driven by hardware RS-485 mode (USART_RS485_EXT_gc), no firmware management needed |
| PB0 | DALI TX — drives the external bus transistor. **OUTSET (PB0 HIGH) asserts the bus** (drives bus low via the transistor); **OUTCLR (PB0 LOW) / high-Z releases**. Reset state (high-Z) is released. |
| PB1 | DALI RX — input via optocoupler (signal is inverted: HIGH = bus active/low) |

### Power

The device runs at 3.3 V, 10 MHz. It spends the majority of its time in Standby sleep mode, consuming under 1 µA. The watchdog timer resets the device if it hangs for more than 8.2 seconds.

---

## Building

The project uses a bundled AVR-GCC toolchain in `avr/bin/` — do not use the system or Homebrew avr-gcc, as they lack ATtiny1604 support and produce incorrect results. Download the bundled toolchain once with:

```sh
make download_gcc
```

Then build and flash:

```sh
make build   # compile only
make         # build, erase device, and flash
```

`pymcuprog` is required for flashing. Programming is via UPDI on the test point using a USB-UART adapter.

A standard `make` (build + erase + flash) does NOT touch USERROW — the slave address, baud rate, and rescan interval persist across reflashes. To write the persisted-config bytes:

```sh
make configure                                              # defaults: address 1, 9600 baud, 64 s rescan
make configure SLAVE_ADDR=5 BAUD_INDEX=1                    # address 5, 19200 baud, default rescan
make configure RESCAN_INTERVAL=120                          # change just the rescan
make configure SLAVE_ADDR=42 BAUD_INDEX=2 RESCAN_INTERVAL=30
```

`BAUD_INDEX` is `0=9600 / 1=19200 / 2=38400`. `make configure` rewrites all 32 USERROW bytes — the four config bytes plus 28 bytes of `0xFF`.

## Behavioural Changes (vs pre-refactor firmware)

The flag-driven refactor (branch `task-1-flag-driven`) introduces several wire-protocol-visible changes. Masters built against earlier firmware revisions need the following adjustments:

- **Writes ACK at the Modbus wire layer, not at DALI completion.** Per-ballast bank writes (0x0000–0x00FF), group-mask writes (0x0110–0x014F), and raw command writes (0x1000 / 0x1001) all return their normal echo-style ACK as soon as the cache is updated and the work is armed. The DALI command that pushes the change to the ballast is dispatched asynchronously by the work selector. Masters that previously took the write-ACK as "ballast confirmed" must switch to **dirty-bit polling** (FC04 0x0104–0x0107, read-clears) to detect when the change has propagated.
- **Per-ballast cache addresses moved from FC04 (input regs) to FC03 (holding regs).** Addresses 0x0000–0x00FF (status/brightness, min/max, power-on/sys-fail, fade rate/time) and 0x0110–0x014F (group masks) are now read via FC03 and written via FC06/FC10. Reading them via FC04 returns `ILLEGAL_ADDRESS`.
- **Coil function codes (FC01 / FC05 / FC0F) removed.** All three return `ILLEGAL_FUNCTION`. Masters that used FC05 / FC0F to drive ON/OFF should write the brightness register low byte (0 = off, ≥1 = on / proportional). FC01 (read coils) likewise has no replacement; read the brightness bank via FC03 instead.
- **Action holding registers 0x1002–0x100A removed.** DAPC at 0x1002, GOTO_LAST_ACTIVE at 0x1003, OFF/UP/DOWN/STEP_UP/STEP_DOWN/RECALL_MAX/RECALL_MIN at 0x1004–0x100A all return `ILLEGAL_ADDRESS`. DAPC is now done via the brightness register (0x0000–0x003F); the rest are issued via the raw command channel (0x1000 / 0x1001).
- **`RESCAN_INTERVAL` (0x1011) semantic change.** Previously "seconds between full-bus burst rescans"; now "seconds between single-ballast level polls" via the round-robin cursor. Same address, same units, very different cycle time at the same numeric value. See the *Periodic Rescan* section above.
- **Async raw command pending state uses `0xFFnn` with non-zero low byte.** The 0x0108 pending encoding is `0xFF02` (after writing 0x1001 RAW_CMD_TWICE) or `0xFF01` (after writing 0x1000 RAW_CMD); the low byte does NOT decrement mid-twice-send (the firmware transitions directly from `0xFF02` to the terminal value once both sends complete). Masters that strict-equal `value == 0xFF00` for "still pending" will miss both initial arming states; match the high byte instead (`(value & 0xFF00) == 0xFF00`).
- **DTR-setter holding registers removed (subtask 9).** `SET_DTR0` (0x100B), `SET_DTR1` (0x100C), and `SET_DTR2` (0x100D) all return `ILLEGAL_ADDRESS`. They were thin wrappers around raw DALI sends; issue the equivalent special command via the raw channel instead — write `0xA3VV` to 0x1000 to load DTR0 with `VV`, `0xC3VV` for DTR1, `0xC5VV` for DTR2 (single send, no backward frame expected; the result register at FC04 0x0108 reaches `NAK` once the send completes). The internal multi-step write sequences (min/max, power-on/sys-fail, fade rate/time) still drive DTR0 directly via the address-byte 0xA3 special command — no master interaction required.
- **FC04 input register layout cleaned up (subtask 8).** Present, dirty, and raw-command-result moved down to a contiguous block at 0x0100–0x0108: present[4] at 0x0100–0x0103, dirty[4] at 0x0104–0x0107, raw_cmd_result at 0x0108. The fading bitmap (formerly 0x0155–0x0158), group_status array (formerly 0x0100–0x010F), and scan_status (formerly 0x0150) are removed entirely — they were derivable from the per-ballast status byte / group_mask, or vestigial from the pre-flag-driven era. Masters that read presence/dirty/raw_result must update their addresses; the master-visible group_mask range at FC03 0x0110–0x014F is intentionally preserved by a 14-byte padding hole inside `register_map_t`. Fade-running detection switches from the removed bitmap to bit 4 of the per-ballast status byte at FC03 0x0000–0x003F (high byte).

## Known Limitations

These are residual issues that the current firmware does not address. None are runtime-fatal; each has a documented workaround or expected impact.

### Modbus

- **Inter-byte timeout conflated with inter-frame.**  Modbus §2.5.1.1 wants a 1.5-char inter-byte gap to declare a frame malformed and a separate 3.5-char inter-frame silence to mark frame end.  This firmware uses a single threshold sized for 3.5-char silence ({10, 6, 4} TCA0 ticks at 9600/19200/38400), so bytes arriving 2–3 char apart are silently accepted as part of one frame and rely on the CRC check to reject.  Any master that streams its bytes contiguously is unaffected.
- **FC10 partial execution on register-map holes.**  The holding-register map has gaps at 0x0100–0x010F (FC04-only range), 0x1002–0x100A (action regs retired in subtask 4/5), 0x100B–0x100D (DTR setters retired in subtask 9), and 0x100E–0x100F.  A multi-register write spanning a gap aborts on the first invalid address with `ILLEGAL_ADDRESS`, leaving earlier registers in the same FC10 already written and dispatched.  Workaround: do not write across the holes.
- **FC04 0x0200 returns an ASCII version string, not a register address space.**  Reading exactly 16 input registers from address 0x0200 yields the 32-byte string `"DALI v<maj>.<min> <serial>"`.  Other lengths at this address fall through to the standard register-range check and return `ILLEGAL_ADDRESS`.
- **`frame_is_valid` still recognises the retired coil function codes (FC01 / FC05 / FC0F) for size-check purposes, and the broadcast filter still admits FC05 / FC0F.**  This is intentional: well-formed coil frames pass frame validation, reach the FC dispatcher, and receive a graceful `ILLEGAL_FUNCTION` exception reply (or — for a broadcast-addressed FC05 / FC0F — are silently absorbed) rather than being dropped at the size check.  Candidate for future tightening: removing the FC01 / FC05 / FC0F cases from `frame_is_valid` and from the broadcast filter would drop those frames at validation time instead.  Nothing breaks if it stays as-is; the current shape gives masters that probe for coil support a clearer "function not supported" answer.

### DALI

- **No multi-master collision detection during our own TX.**  If another master begins driving the bus while we are already transmitting, the resulting collision is not detected; the receiving ballast simply sees a malformed frame and ignores it.
- **Receiver dead-band.**  Half-bit and full-bit pulse classification has a ±83 µs tolerance window (`DALI_HALF_BIT_USECS / 5`); pulses falling between the half-bit and full-bit ranges (~500–750 µs) are reported as `PULSE_IN_MIDDLE` and discarded.  At spec-compliant DALI bit timing this never fires.
- **Multi-step write sequences hold the bus for ~300 ms.**  `SYNC_WRITE_MINMAX`, `SYNC_WRITE_POWER`, and `SYNC_WRITE_FADE` each issue four logical steps (DTR0 + SET twice + DTR0 + SET twice = six hardware sends) atomically, because DTR0 is shared across all ballasts and an interleaving raw command or another write could clobber it mid-sequence.  Other DALI work (raw commands, level reads, full-scan steps) waits up to ~300 ms behind one of these.  `SYNC_WRITE_LEVEL` (single DAPC) and group-bit changes are unaffected — they take ≤ 50 ms each.

### Hardware / runtime

- **Stack budget tight.**  Worst-case nested-ISR depth (TCA0 LVL0 chain + TCB0 LVL1 chain + main thread mid-`modbus_process`) leaves ~30–50 bytes of headroom over the 175 B available.  The largest contributor at main level is the `snprintf` call in the FC04 version-string path (`modbus_process` reports 49 B `dynamic,bounded` in its `.su` file); if that path grows or any new ISR vector is added, monitor stack usage with the `.su` files emitted by `-fstack-usage`.
- **RS-485 DE pin floats briefly at reset.**  `main` drives PA4 low immediately, but there is a ~50–200 µs window after reset (before that store executes) where the pin is high-Z.  Subtask 1 enabled the BOD fuse at 2.6 V, narrowing the window by holding the chip in reset until Vcc is stable; an external pull-down on PA4 on the next PCB revision is the residual recommended fix.
- **`USERROW` write halts the CPU.**  A `MBREG_SLAVE_ADDR` / `BAUD_INDEX` / `RESCAN_INTERVAL` write performs a NVMCTRL page-erase-write that suspends CPU execution for ~3.4 ms.  Triggered by Modbus writes the firmware schedules so the halt lands in the master's silence window before our response, but a DALI frame in flight at that exact moment will be corrupted.  Avoid changing config during heavy DALI traffic.
