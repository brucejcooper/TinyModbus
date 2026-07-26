# TinyModbus DALI Gateway

An ATtiny1604-based Modbus RTU to DALI gateway. It acts as a Modbus slave device and translates Modbus register reads/writes into DALI commands, allowing any Modbus-capable system (PLCs, building automation controllers, SCADA) to control DALI lighting.

The PCB design is in the `pcb` folder. It is set up to hold an RS-485 transceiver and the DALI bus driver circuitry on a compact board.

![PCB](pcb/MikroeAdapter/MikroeAdapter.png)

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

The per-ballast level bank in this firmware uses short addresses (0–63). Command-queue records accept any valid DALI address byte, so group and broadcast commands are also supported through the queue.

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

This firmware **implements control-gear commissioning** — see `Commissioning (FC03/FC06 0x1F04)` under the Modbus Interface section for the master-facing surface. Master writes 0x0001 to `MBREG_COMMISSION` for full re-addressing or 0x0002 for incremental (assigns short addresses only to gears that don't already have one); reads the same register to poll status. Out of scope for this firmware: 24-bit forward frames (DALI-2 control-device addressing), memory-bank reads, and DT1/DT8 device-type extensions.

---

## Architectural Model

The firmware is a **flag-driven state-reconciliation loop**. The per-ballast cache it keeps is the *live arc level* (brightness, status) and *group memberships*, in a single `g_regs` register map exposed directly over Modbus. (Per-ballast config — min/max, power-on/sys-fail, fade — is no longer cached here; the controller owns it via raw DALI queries. See "Config-bank retirement" below.) A 64-byte `sync_flags[]` table tracks per-ballast reconciliation work — read-flags say "the cache is stale, refresh from the ballast", write-flags say "the master supplied a value, push it to the ballast". A Modbus write to the level bank updates the cache predictively and sets the write-flag, then ACKs at the wire layer immediately.

The main loop kicks `start_next_dali_work()` whenever DALI is idle. The work selector walks a fixed priority chain — commissioning state machine → **raw command queue** → pending group bit → SYNC_WRITE_LEVEL (rotating cursor) → SYNC_READ_LEVEL (same cursor) → continue in-progress settings scan → start new settings scan — and issues exactly one DALI command via `issue_dali_command()`. Cache invalidation is handled inside `issue_dali_command` as a uniform property of the command bytes — every DALI command from every source (per-ballast bank write, queued raw command, commissioning step, internal settings-scan step) goes through the same chokepoint, so the invalidation logic lives once and the per-source handlers stay trivial. The queue sits just below commissioning, so a submitted command preempts the gateway's own idle-time polling but can't clobber a staged DTR0 mid-bisect.

Completion runs in ISR context (`dali_sync_completion`) and either advances a multi-step sequence (level read = QUERY_STATUS + GET_LEVEL; a queued DTR0-first command = `0xA3<dtr0>` then the frame) or releases `dali_busy` for the next pick. Periodic level re-polls are spread evenly via the RTC-PIT cursor (one ballast per `RESCAN_INTERVAL` seconds, round-robin across all 64 slots) instead of bursting. Masters detect when reads have caught up by polling the dirty bitmap (FC04 0x0104–0x0107, read-clears).

Beyond the raw command queue (a bounded FIFO of master-submitted DALI frames), there is no event-callback chain and no synchronous DALI work in the main loop. State *is* the protocol; the reconciliation cache is driven by flags; events are derived from state changes; ISRs only set/clear flags, advance the queue, or update cache cells.

### Config-bank retirement

Earlier firmware cached every per-ballast config field (min/max, power-on/sys-fail, fade rate/time) in `g_regs` at holding registers 0x0040–0x00FF, kept in sync by a 7-step settings scan and written by DTR0+SET multi-step sequences the master triggered via bank writes. That responsibility now lives entirely on the **controller**, which reads config with raw DALI queries and writes it with raw DTR0+SET pairs (submitted through the command queue). The gateway therefore no longer stores config: the banks are gone, the settings scan is trimmed to the two group-membership queries, and the multi-step write ops are removed. FC03 reads and FC06/FC10 writes of 0x0040–0x00FF return **ILLEGAL DATA ADDRESS (0x02)**. This freed the RAM and flash the command queue needed, and cut the settings scan from 7 to 2 DALI queries per ballast. Presence detection is unaffected — it is established by the first OK response to *any* directed query (the QUERY_STATUS level read or a group query), which the trim leaves in place.

> **Compatibility:** a gateway on this firmware requires a controller that sources config over raw DALI (a pre-retirement controller reading 0x0040–0x00FF gets exception 0x02). Roll out **controller-first**.

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

- `SYNC_READ_LEVEL` — re-poll the live arc level (QUERY_STATUS + QUERY_ACTUAL_LEVEL, two queries one logical op).
- `SYNC_READ_SETTINGS` — re-fetch **group membership** (QUERY_GROUPS_0_7 + QUERY_GROUPS_8_15, one query per work-selector pass). Since the config-bank retirement this reads groups only — the min/max/power/sys-fail/fade fields are no longer cached on the gateway. **Does NOT read the live arc level**, so anything that needs fresh brightness must also set `SYNC_READ_LEVEL` (the boot scan and `SCAN_TRIGGER` set both; see below).
- `SYNC_WRITE_LEVEL` — master-supplied DAPC level cached predictively in `g_regs`, awaits a single-command push to the ballast. (The old `SYNC_WRITE_MINMAX/POWER/FADE` multi-step config writes were retired with the config banks; config writes are now raw DTR0+SET pairs submitted through the command queue.)
- `SYNC_RETRY_PENDING` — first NAK seen on a level read; one retry before marking the ballast absent.

`start_next_dali_work()` walks the priority chain (commissioning state machine → raw command queue → pending group bit → SYNC_WRITE_LEVEL → SYNC_READ_LEVEL → continue settings scan → start settings scan) and, when work is found, dispatches one DALI command via `dali_sync_dispatch_current()` → `issue_dali_command()`. The unified issuance function classifies cache invalidation as a property of the outgoing address + data bytes — DAPC and step/recall commands set `SYNC_READ_LEVEL` on the affected ballast(s); SET, scene, and group-membership commands set `SYNC_READ_SETTINGS`; queries and DTR loads set nothing. Completion is dispatched in ISR context by `dali_sync_completion`, which advances a multi-step sequence (level read, queued DTR0-first pair) or releases `dali_busy` for the next pick.

### Periodic Rescan

The RTC Periodic Interrupt Timer fires every ~1 second (1024 cycles of the 1 kHz internal oscillator).  Each PIT tick increments both the `uptime_s` counter (exposed via FC04 0x0310) and the rescan counter; once the rescan counter reaches `RESCAN_INTERVAL` (0x1F01), the firmware sets `SYNC_READ_LEVEL` on **one** ballast at the cursor position and advances the cursor (mod 64).  `RESCAN_INTERVAL = 0` disables the cursor entirely.  Writing 0x0001 to `SCAN_TRIGGER` (0x1F00) sets `SYNC_READ_SETTINGS | SYNC_READ_LEVEL` on all 64 ballasts in one pass, kicking off a full re-discovery of group membership and live level.

The boot scan (`main.c`) arms the same two flags on all ballasts. Both are required because they read disjoint state: after a reset `regs_init()` zeroes the brightness cache, and the settings scan never reads the live arc level — so without `SYNC_READ_LEVEL` a physically-ON ballast would report brightness 0 (OFF) until a command or the slow periodic cursor happened to touch it. Arming both makes a freshly-booted or rescanned gateway converge to true state on its own.

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

Slave address, baud rate, and rescan interval all live in USERROW so they survive a chip erase + reflash. The factory-fresh (post-erase) USERROW value of `0xFF` maps to address 1 / 9600 baud / 64 s rescan. Writes to holding registers 0x1F01 / 0x1F02 / 0x1F03 trigger a USERROW page-erase-write; the runtime baud/slave switch is deferred until the FC06/FC10 response we are about to send has cleared the wire, so the master sees the response at the old config and the next request at the new one.

### Function Codes

| Code | Name | Description |
|------|------|-------------|
| 0x03 | Read Holding Registers | Read per-ballast level cache, group masks, the queue status block, and USERROW config registers (max 16 regs per request) |
| 0x04 | Read Input Registers | Read presence + dirty bitmaps, the queue status block, version/diag/sysinfo blocks |
| 0x06 | Write Single Holding Register | Update per-ballast level cache, group mask, or an action/config register (0x1F00+) |
| 0x10 | Write Multiple Holding Registers | Write a contiguous holding block — including the command-queue submit window (0x1000–0x1007) |
| 0x17 | Read/Write Multiple Registers | Submit queue commands (write range) and collect status/results (read range) in one transaction — see "Raw Command Queue" |

Coils (FC01 / FC05 / FC0F) were retired in subtask 5: per-ballast on/off / level changes go through the per-ballast brightness bank (0x0000-0x003F) which arms the asynchronous DALI work selector and ACKs immediately; broadcast / group / specialty DALI sends go through the **raw command queue** (submit window 0x1000–0x1007 via FC10/FC23).  Frames using the retired function codes get an ILLEGAL_FUNCTION exception reply.

### FC04 — Input Register Map

Read-only state derived from DALI traffic. Registers are 16-bit, addressed from 0x0000.

| Address range | Width | Content |
|---------------|-------|---------|
| 0x0100–0x0103 | 4 regs | Present bitmask — one bit per ballast, 1 = responding |
| 0x0104–0x0107 | 4 regs | Dirty bitmask — set on any block0 change, cleared when read |
| 0x1010–0x1019 | 10 regs | **Command-queue status block** — served identically here (FC04) and in holding space (FC03; FC23's read range targets it). `0x1010` QSTAT `[free_slots:8 \| accepted_id:8]`, `0x1011` QPROG `[executing_id:8 \| completed_id:8]`, `0x1012–0x1019` QRES[0..7] `[result:8 \| answer:8]`. See "Raw Command Queue". A partial read (e.g. just QSTAT+QPROG) is allowed. |
| 0x0200–0x020F | 16 regs | Version string: `DALI v<maj>.<min> <10-byte-serial-as-20-hex-chars>` — read exactly 16 registers |
| 0x0300–0x030F | 16 regs | RX diagnostics — read exactly 16 registers. `reg0` rx_overflow, `reg1` frame_error, `reg2` crc_error, `reg3` frames_ok, `reg4` last_fail_len, `reg5–12` bytes of the last CRC-failed frame. Counters zeroed on reset. |
| 0x0310–0x0313 | 4 regs | System info / reboot diagnostics — read exactly 4 registers. `reg0–1` uptime_s (32-bit, big-endian, seconds since boot), `reg2` boot_count (reboots since the last cold power-on), `reg3` reset_reason (low byte = `RSTCTRL.RSTFR`: bit0 PORF, 1 BORF, 2 EXTRF, 3 WDRF, 4 SWRF, 5 UPDIRF). No EEPROM is used — uptime is a RAM counter, boot_count is `.noinit` SRAM (guarded by a magic word, survives warm resets, restarts on cold power-on), reset_reason is the hardware-latched RSTFR. A master detects a gateway reboot when uptime_s drops or boot_count changes. |

A master can fetch presence + dirty in a single 8-register FC04 read at 0x0100. The 0x0200 and 0x0300 blocks are special-cased and must be read as exactly 16 registers; the 0x0310 block as exactly 4; the 0x1010 status block accepts 1–10 registers. Other lengths at those addresses fall through to the range check. FC04 reads outside these ranges return `ILLEGAL_ADDRESS`. The per-ballast level bank formerly split across 0x0000–0x00FF and the group membership masks formerly at 0x0110–0x014F live in the holding-register map below; the per-ballast **config** banks (min/max/power-on/sys-fail/fade) that used to occupy 0x0040–0x00FF have been **retired** (see the holding map) — both are breaking changes from earlier firmware revisions.

### FC03/FC06/FC10 — Holding Register Map

Per-ballast cached DALI state (read-write) and gateway control registers.

| Address range | Width | Content |
|---------------|-------|---------|
| 0x0000–0x003F | 64 regs | Per-ballast status \| brightness (the live arc level). FC03 returns the cached `[status:brightness]` pair. FC06/FC10 write: low byte = DAPC level (0–254 valid; 255 = "no change" per DALI), high byte ignored. |
| 0x0040–0x00FF | (retired) | Per-ballast config banks (min/max, power-on/sys-fail, fade rate/time) — **retired**. The gateway no longer caches config; the controller owns it via raw DALI queries (reads) and DTR0+SET pairs (writes). FC03 reads and FC06/FC10 writes here return **ILLEGAL DATA ADDRESS (0x02)**. |
| 0x0110–0x014F | 64 regs | Group membership masks — 16 groups × 4 registers (64-bit bitmask per group). Writes are predictively cached and propagate per-bit via ADD_TO_GROUP / REMOVE_FROM_GROUP DALI commands by the work selector. |
| 0x1000–0x1007 | ≤8 regs | **Command-queue submit window** (write-only; FC10 or FC23 write range). 0–4 command records; the write length (2/4/6/8 regs) says how many. See "Raw Command Queue". |
| 0x1010–0x1019 | 10 regs | **Command-queue status block** — served identically under FC03 and FC04 (FC23's holding-only read range targets it). Layout as in the FC04 map above. |

**Per-ballast bank writes ACK immediately at the Modbus layer.**  The actual DALI command that pushes the change to the ballast is dispatched asynchronously by the sync-flag work selector — the master no longer blocks waiting for DALI propagation.  To detect when reconciliation has completed, poll the dirty bitmap (FC04 0x0104-0x0107, read-clears semantics).  This is a behavioural change vs pre-refactor firmware where writes blocked until DALI completion.

Action / config registers, relocated to 0x1F00+ in task 04:

| Address | Name | Read | Write |
|---------|------|------|-------|
| 0x1F00 | SCAN_TRIGGER | 0 | Write 0x0001 to set `SYNC_READ_SETTINGS | SYNC_READ_LEVEL` on all 64 ballasts (re-discovery: group membership + live arc level/status). Since the config-bank retirement `SYNC_READ_SETTINGS` reads only group membership. |
| 0x1F01 | RESCAN_INTERVAL | R/W | Seconds between single-ballast level re-polls (round-robin cursor). Default 64; full cycle = `RESCAN_INTERVAL × 64` seconds. Write 0 to disable. See "Periodic Rescan" above. |
| 0x1F02 | SLAVE_ADDR | R/W | Modbus slave address 1–247. Persisted to USERROW on write. |
| 0x1F03 | BAUD_INDEX | R/W | 0 = 9600, 1 = 19200, 2 = 38400. Persisted to USERROW on write. |
| 0x1F04 | COMMISSION | status word | Trigger / abort / poll the IEC 62386-102 control-gear commissioning state machine. See "Commissioning" below. |
| 0x1F05 | SYSTEM_RESET | 0 | Write magic 0xA5A5 to software-reset the gateway once the ACK has cleared the wire. |

Every pre-queue address not listed above (the removed legacy raw registers, the retired config banks) returns ILLEGAL DATA ADDRESS.  The DALI address byte inside a command record can be any valid DALI address byte — short address, group address, broadcast (0xFE), or special command (e.g. 0xA3 SET_DTR0).  Cache invalidation runs uniformly on every issued command via `issue_dali_command`, regardless of the master's choice between the per-ballast bank shortcut and the command queue.

### Raw Command Queue

The gateway holds an **8-deep, id-tagged FIFO** of raw DALI commands, fed by a 4-command submit window and read back through a result ring + status block. It replaces the old single-slot channel's "write 0x1000, poll 0x0108, back off on DEVICE_BUSY" dance: a master submits a batch and collects earlier results in the *same* transaction, so one FC23 both pushes new work and reaps finished work.

**Command record** — each command is 2 registers:

```
reg0:  [addr:8 | data:8]           the 16-bit DALI forward frame
reg1:  [id:5   | flags:3 | dtr0:8]
```

- `addr` / `data` are the two forward-frame bytes (any DALI address byte, as above).
- `id` (bits 15–11 of reg1) is the **controller-assigned** command id, cycling **1–31** (0 is the reserved none/idle sentinel — no command carries id 0).
- `flags` (bits 10–8): `bit0 send-twice` (send the forward frame twice, no answer — SET/INITIALISE/RANDOMISE); `bit1 send-DTR0-first` (emit `0xA3<dtr0>` immediately before the frame, back-to-back as one atomic queue entry — used for DTR0+SET config writes); `bit2 expects-answer` (open the backward-frame window and capture the 8-bit answer into the result ring). Only these three flags exist.
- `dtr0` (low byte of reg1) is the value preloaded into DTR0 when `send-DTR0-first` is set; ignored otherwise.

**Submit window** (holding 0x1000–0x1007, write-only): write 0–4 records with FC10 or the FC23 write range. The register count (2/4/6/8) is the record count. The submit is **all-or-nothing**: if it asks for more slots than are free, the gateway rejects the *entire* write with exception **0x06 (SLAVE_DEVICE_BUSY)** and enqueues nothing. A record carrying the reserved id 0 is rejected with ILLEGAL DATA VALUE.

**Execution** is strict FIFO in submit order. A `send-DTR0-first` record's two frames land back-to-back with nothing interleaved (the queue holds the bus for the pair). After a level-changing frame the gateway marks the target ballast dirty for a priority re-query (group frames fan out to members via the group masks). A queued command **preempts the gateway's idle-time polling** — a submitted command runs within ~1–2 DALI frame times of arrival.

**Status block** (0x1010–0x1019, served identically under FC03 and FC04):

| Reg | Name | Layout |
|-----|------|--------|
| 0x1010 | QSTAT | `[free_slots:8 \| accepted_id:8]` — free_slots = queue space now; accepted_id = id of the last record accepted |
| 0x1011 | QPROG | `[executing_id:8 \| completed_id:8]` — id on the bus now (0 = idle); id of the last result written to the ring |
| 0x1012–0x1019 | QRES[0..7] | `[result:8 \| answer:8]` — result 0x00 OK / 0x01 NAK / 0x02 ERR (answer = backward-frame byte on OK, error subtype on ERR) |

**Result-ring convention:** the result for id *N* lives at `0x1012 + (N mod 8)`, and is valid once `completed_id` has reached *N*. There is no PENDING state in the ring — "pending" is simply *completed_id has not yet reached this id*. The gateway writes the ring slot (result + answer) **before** advancing `completed_id`, and the completion runs at a level a Modbus status read fences out, so a read never sees `completed_id` reach an id whose result is not yet written.

**Id rules / invariant:** ids are controller-assigned, 1–31, 0 reserved. Because a ring slot is `id & 7`, the controller must keep **at most 7 results uncollected** at any time so a fresh result never overwrites an uncollected one — trivially satisfied when collection rides every FC23 response (queue depth is only 8).

**FC23 (0x17) pipeline:** the write executes **before** the read, so one FC23 confirms *acceptance of this submit* (via QSTAT `accepted_id` / `free_slots`) and returns *the results of earlier commands* (QPROG `completed_id` + the QRES ring) — the submit-N / collect-N−1 pattern. The write range is the submit window (0x1000); the read range is holding space, normally the 0x1010 status block (FC23's read range is holding-only). A read-only FC23 (write count 0) is accepted as a pure status/collect poll.

> **Changelog:** the pre-queue single-slot raw channel (old 0x1000/0x1001 RAW_CMD + FC04 0x0108 result register) was **removed** in task 04 — those addresses are reused by the submit window, and old raw reads/writes return ILLEGAL DATA ADDRESS. The controller and this gateway are a **matched firmware set** (introduced together in task 04); a pre-queue master cannot drive this firmware and vice versa. Deploy both together.

### Commissioning (FC03/FC06 0x1F04)

`MBREG_COMMISSION` (holding register 0x1F04) drives the IEC 62386-102 control-gear commissioning state machine — the bisect-based discovery + addressing protocol that lets the gateway assign short addresses (0–63) to fresh ballasts without an external commissioning tool.

**Write encoding** (FC06 to 0x1F04):

| Value | Action |
|-------|--------|
| 0x0000 | Abort: send TERMINATE, return to idle. No-op when already idle or in a terminal state (master must read 0x1F04 to clear those). |
| 0x0001 | Start **full** commissioning: INITIALISE(all), wipe existing per-ballast cache, assign 0..N−1 to discovered ballasts. **Destructive** — overwrites any existing addressing. |
| 0x0002 | Start **incremental** commissioning: INITIALISE(without short addr), only address ballasts that don't currently have one. Already-addressed ballasts are not disturbed; new gear takes the lowest unused short addresses. |
| anything else | ILLEGAL_VALUE |

Writing 0x0001 or 0x0002 while a run is already in progress returns DEVICE_BUSY. 0x0000 always succeeds (no-op when idle / terminal).

**Read encoding** (FC03 of 0x1F04) — high byte encodes phase, low byte encodes count or error subtype:

| High byte | Low byte | Meaning |
|-----------|----------|---------|
| 0x00 | 0x00 | Idle — no run ever, or last run completed and master has consumed status |
| 0x01 | count | Running — `count` ballasts addressed so far this run |
| 0x02 | total | Complete — `total` short addresses assigned this run (read-clears) |
| 0x03 | subtype | Error / aborted (read-clears). Subtype: 0x01 = aborted by master, 0x02 = no ballasts found, 0x03 = >3 consecutive verify failures, 0x04 = ran out of short addresses (more ballasts than free slots), 0x05 = bus shorted longer than 200 TE during commissioning (run aborted) |

**Read-clears semantics on terminal states**: reading 0x1F04 when the high byte is 0x02 or 0x03 returns the value AND resets the underlying state to 0x0000 atomically — same shape as FC04 0x0108. Idle (0x00) and running (0x01) reads are non-clearing so masters can poll progress.

**Concurrency**: OP_COMMISSION is the highest-priority work-selector slot — concurrent raw commands and per-ballast bank writes that arrive mid-run are queued and execute after the run finishes. Successful PROGRAM_SHORT_ADDRESS additionally arms `SYNC_READ_SETTINGS` on the just-addressed ballast, so the config cache populates without a manual SCAN_TRIGGER.

**Timing**: a full-bus commissioning of 64 ballasts takes ~3 minutes at 1200 baud (~3 s/ballast for the bisect chain — 24 search rounds × 4 sends + program/verify/withdraw). The master can poll 0x1F04 at any rate; the count low byte advances monotonically as PROGRAM/VERIFY rounds succeed.

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

The current board runs at 3.3 V, 10 MHz. It spends the majority of its time in Standby sleep mode, consuming under 1 µA. The watchdog timer resets the device if it hangs for more than 8.2 seconds.

**Board revision in progress (12 V → 5 V buck).** The present supply — an **HT7533 LDO** dropping 12 V to 3.3 V — **thermally shuts down under sustained DALI activity** (e.g. an inventory scan): 8.7 V × ~0.1 A in a SOT-89 overheats, the rail collapses, and the ATtiny browns out (this is what the new FC04 `0x0310` reset-reason counter exposed — repeated `BORF` reboots during a scan). The fix is a **synchronous buck (MP2331H, LCSC C6992878)** to a **5 V** rail, which also enables 20 MHz operation and stronger RS-485 drive. Full root-cause analysis, the power-supply BOM, the supply-current budget, and the firmware changes the 5 V/20 MHz board needs (F_CPU, clock prescaler, **BOD → 4.2 V**, DALI bit-timing re-verify) are documented in `CLAUDE.md` → "Power supply — board revision".

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
- **DTR-setter holding registers removed (subtask 9).** `SET_DTR0` (0x100B), `SET_DTR1` (0x100C), and `SET_DTR2` (0x100D) all return `ILLEGAL_ADDRESS`. They were thin wrappers around raw DALI sends; issue the equivalent special command via the raw channel instead — write `0xA3VV` to 0x1000 to load DTR0 with `VV`, `0xC3VV` for DTR1, `0xC5VV` for DTR2 (single send, no backward frame expected; the result register at FC04 0x0108 reaches `NAK` once the send completes). Config writes (min/max, power-on/sys-fail, fade) are no longer done on the gateway at all — the controller issues them as raw DTR0+SET pairs through the command queue (the `send-DTR0-first` record flag makes each pair one atomic queue entry).
- **FC04 input register layout cleaned up (subtask 8).** Present, dirty, and raw-command-result moved down to a contiguous block at 0x0100–0x0108: present[4] at 0x0100–0x0103, dirty[4] at 0x0104–0x0107, raw_cmd_result at 0x0108. The fading bitmap (formerly 0x0155–0x0158), group_status array (formerly 0x0100–0x010F), and scan_status (formerly 0x0150) are removed entirely — they were derivable from the per-ballast status byte / group_mask, or vestigial from the pre-flag-driven era. Masters that read presence/dirty/raw_result must update their addresses; the master-visible group_mask range at FC03 0x0110–0x014F is intentionally preserved by a 14-byte padding hole inside `register_map_t`. Fade-running detection switches from the removed bitmap to bit 4 of the per-ballast status byte at FC03 0x0000–0x003F (high byte).
- **Raw command result high-byte encoding revamped and FC04 read-clears removed.** OK was `0x02xx`; it is now `0x00xx` (the register value is just the response byte, with `0x0000` doing double duty as "no command issued yet"). ERR was `0x03xx`; it is now `0x02xx` with the same error-subtype low byte. PENDING (`0xFFxx`) and NAK (`0x0100`) are unchanged. Reads of FC04 0x0108 no longer clear the register — the outcome of the last raw command persists until the next RAW_CMD write overwrites it. The arm gate now only checks PENDING: any other state is silently overwritten on the next RAW_CMD write (so masters that want to preserve a prior outcome must read it before re-arming).
- **Config banks (0x0040–0x00FF) retired (task 04).** The per-ballast min/max, power-on/sys-fail, and fade banks no longer exist; FC03 reads and FC06/FC10 writes there return `ILLEGAL DATA ADDRESS`. Config is owned by the controller over raw DALI. A gateway on this firmware requires a raw-config controller — see "Config-bank retirement". Deploy controller-first.
- **Command queue + register-map relocation (task 04) — supersedes the several raw-channel bullets above.** The legacy single-slot raw channel (0x1000/0x1001 RAW_CMD + FC04 0x0108 result) is **removed**; all raw DALI now goes through the id-tagged command queue. The register map moved to a round 0x1000 base: submit window 0x1000–0x1007 (FC10/FC23 write range), status block 0x1010–0x1019 (served under both FC03 and FC04), **FC23 (0x17)** support; and the action/config registers relocated to 0x1F00+ (scan 0x1F00, rescan 0x1F01, slave 0x1F02, baud 0x1F03, commission 0x1F04, reset 0x1F05). Every pre-queue raw/config-bank address returns ILLEGAL DATA ADDRESS. This gateway and the controller are now a **matched firmware set** — no mixed-version operation; deploy both together. See "Raw Command Queue".

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
- **A queued DTR0-first command holds the bus for its pair.**  A command-queue record with the `send-DTR0-first` flag issues two frames (`0xA3<dtr0>` then the frame, e.g. a config DTR0+SET pair) as one atomic unit, so DTR0 can't be clobbered between them.  Other DALI work waits behind the pair (~2 frame times).  Single-frame commands (DAPC level, group-bit changes, most raw commands) are unaffected — ≤ ~50 ms each.  (The old 6-send `SYNC_WRITE_MINMAX/POWER/FADE` sequences were retired with the config banks.)

### Hardware / runtime

- **Stack budget — comfortable again after the config-bank retirement.**  Static allocation is **525 B** of the 1024 B SRAM (`.data + .bss + .noinit`, per `avr-size`), leaving **~499 B** for the stack.  The command queue + result ring + status add ~51 B, but retiring the config banks (block1/2/3 + the 14 B `_reserved` anchor hole) freed ~398 B, a net ~347 B reduction from the pre-task-04 872 B.  Worst-case nested-ISR depth (TCA0 LVL0 chain + TCB0 LVL1 chain + main thread mid-`modbus_process`) is ~104 B, so headroom is now ~395 B (was ~10–30 B).  The largest contributor at main level is the `snprintf` in the FC04 version-string path (`modbus_process` reports 49 B `dynamic,bounded` in its `.su` file).  These `.su` figures are per-frame, not cumulative — for a hard peak-stack number run a call-graph stack analyzer.  If a new ISR vector is added, check the `.su` files (`-fstack-usage`) first.
- **RS-485 DE pin floats briefly at reset.**  `main` drives PA4 low immediately, but there is a ~50–200 µs window after reset (before that store executes) where the pin is high-Z.  Subtask 1 enabled the BOD fuse at 2.6 V, narrowing the window by holding the chip in reset until Vcc is stable; an external pull-down on PA4 on the next PCB revision is the residual recommended fix.
- **`USERROW` write halts the CPU.**  A `MBREG_SLAVE_ADDR` / `BAUD_INDEX` / `RESCAN_INTERVAL` write performs a NVMCTRL page-erase-write that suspends CPU execution for ~3.4 ms.  Triggered by Modbus writes the firmware schedules so the halt lands in the master's silence window before our response, but a DALI frame in flight at that exact moment will be corrupted.  Avoid changing config during heavy DALI traffic.
