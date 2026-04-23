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
| 1  | 1111111 | 1 | Broadcast command |

The per-ballast query registers in this firmware use short addresses (0–63). The DAPC, action, and raw command registers accept any valid DALI address byte, so group and broadcast commands are also supported.

### Command Types

**Direct Arc Power Control (DAPC)** — sets light output immediately:
- Data byte: 0 = off, 1–254 = proportional level (logarithmic), 255 = no change

**Standard commands** (address byte LSB = 1) — the data byte selects the command:

| Command | Data byte | Description |
|---------|-----------|-------------|
| Off | 0x00 | Turn off (no fade) |
| Up | 0x01 | Step up one level |
| Down | 0x02 | Step down one level |
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

This firmware does **not** implement commissioning — it assumes devices are already addressed before connecting them to the bus.

---

## DALI State Machine

The firmware implements a five-state machine that drives all DALI bus activity. Every state transition is interrupt-driven — the main loop simply sleeps until a Modbus command arrives or the background scan triggers a wakeup.

### Timer Architecture

Two hardware timers underpin the state machine:

- **TCA0** — the half-bit clock. Runs continuously at 10 MHz with a period of exactly one DALI half-bit (416.67 µs). On each overflow it decrements a countdown (`dali_timer`) and fires a registered callback (`dali_timer_cb`) when the countdown reaches zero. This drives both transmission timing and state-machine timeouts. Runs at CPU interrupt level 0.
- **TCB0** — the pulse-width capture timer. Active only during the RECEIVING state. Configured in Frequency Measurement mode and fed by the DALI RX pin via the AVR event system. It captures the time between consecutive edges, giving the width of each Manchester pulse without CPU polling. Configured as CPU interrupt level 1 so it can preempt TCA0 mid-bit for accurate edge timestamping.

### States

```
                    ┌─────────────────────┐
       has          │        IDLE         │◄────────────────────────────┐
       commands     │  (dali_idle = true) │                             │
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

The bus is quiet and we own it. Two things can happen:
- If the command queue has entries, immediately call `dali_transmitting_state_enter()`.
- Otherwise, set `dali_idle = true` (allowing the main loop to sleep), arm a one-shot rising-edge interrupt on the RX pin, and wait. If a rising edge arrives it means another master has started transmitting, so we call `dali_wait_for_idle_state_enter()` to stay out of the way until the bus is quiet again.

**TRANSMITTING** (`dali_state_transmitting.c`)

Sends the 16-bit forward frame using Manchester encoding driven entirely by TCA0 callbacks:

1. Load the 16-bit command word into `shift_reg`, set `bits_left = 17`.
2. Schedule `send_one()` via `dali_timer = 1` (one half-bit tick).
3. `send_one()` drives TX high and schedules `half_bit_toggle()`.
4. `half_bit_toggle()` toggles TX (completing the mid-bit transition), schedules the next bit's first-half function (`send_one` or `send_zero` from the MSB of `shift_reg`), shifts `shift_reg` left, decrements `bits_left`.
5. When `bits_left == 0`, schedules `start_stop_bits()`, which drives TX low for 4 Te (two full stop bits).
6. After stop bits: CMD_QUERY and CMD_SEND_TWICE call `dali_wait_for_response_state_enter()`. CMD_SEND and CMD_DIRECT_ARC call `cmdq_result_received(DALI_RESPONSE_NAK, 0)` directly (no response expected; skips the 22 Te response window to halve inter-command latency).

Each bit takes exactly 2 TCA0 ticks (2 × 416.67 µs = 833.33 µs = 1200 bps).

**WAIT_FOR_RESPONSE** (`dali_state_wait_for_response.c`)

After the forward frame the master must listen for a backward frame within 7–22 Te. This state:
- Pre-configures TCB0 for pulse capture (`dali_state_receiving_prepare()`).
- Arms a one-shot rising-edge interrupt on RX. A rising edge means a backward frame start bit is arriving; we call `dali_state_receiving_enter()`.
- Sets a TCA0 timeout of 22 Te. If it fires before any edge, the device did not respond: call `cmdq_result_received(DALI_RESPONSE_NAK, 0)` and return to IDLE.

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
- If the final state is `START_0` or `MIDDLE_1` **and** exactly 8 bits were received: `cmdq_result_received(DALI_RESPONSE_OK, shift_reg)`.
- Otherwise: `cmdq_result_received(DALI_RESPONSE_MALFORMED, 0)`.

Either way, transitions to WAIT_FOR_IDLE to enforce the minimum inter-frame gap.

**WAIT_FOR_IDLE** (`dali_state_wait_for_idle.c`)

Enforces the DALI requirement that the bus be idle for at least 22 Te before the next forward frame. Works by watching the RX line:
- If the bus is high (shorted/active), arm a level-low interrupt and wait for release.
- Once the bus goes low (idle), start a TCA0 countdown of 22 Te.
- If the bus goes high again before the countdown expires (another master, collision debris, capacitive ringing), restart.
- When 22 Te completes uninterrupted: call `dali_idle_state_enter()`.

### Command Queue

`cmdq_push*()` (in `cmd_queue.c`) appends to an 8-entry ring buffer (`CMD_QUEUE_SIZE = 8`). Each entry (`cmd_entry_t`) holds:
- `type` — `CMD_DIRECT_ARC`, `CMD_SEND`, `CMD_SEND_TWICE`, or `CMD_QUERY`
- `dali_addr` / `dali_cmd` — the two bytes of the DALI forward frame
- `ballast_idx` — short address (0–63) used to index into the register map; 0xFF for group/broadcast
- `reg_target` — which register field to update with the response (`REG_BRIGHTNESS`, `REG_STATUS`, etc.)
- `repeat` — remaining send count (2 for `CMD_SEND_TWICE`, 1 otherwise)
- `on_result` — optional callback fired from ISR context when the command completes

When a command completes (or NAKs), `cmdq_result_received()` (in `dali_state_idle.c`) is called. It:
1. Stores the response byte into the register field named by `reg_target` (via `regs_apply_result()`).
2. For `CMD_SEND_TWICE`: decrements `repeat` and re-dispatches without popping or calling the callback.
3. Fires `on_result` if set, then pops the queue and enters the next state.

### Periodic Rescan

The RTC Periodic Interrupt Timer fires every ~1 second (1024 cycles of the 1 kHz internal oscillator). A software counter counts down from the value of holding register `RESCAN_INTERVAL` (0x1011, default 64), triggering `bgtasks_trigger_scan()` on expiry. Writing 0 to the register disables periodic rescans; a manual scan can still be triggered at any time via `SCAN_TRIGGER` (0x1010). The scan walks all 64 DALI addresses in order using a chained callback pattern — one query command at a time — issuing 9 queries per responding device (status, brightness, min, max, power-on, sys-fail, fade rate, fade time, groups 0–7, groups 8–15). The chain approach keeps the 8-entry queue from overflowing and allows the fade tracker to interleave between scan commands.

### Multi-Master Behaviour

The firmware detects transmissions from other DALI masters on the bus: if a rising edge arrives on RX while in IDLE, the state machine yields by entering WAIT_FOR_IDLE. It waits for the other master's transaction (forward frame + optional backward frame) to complete and the bus to settle for 22 Te before re-asserting control. The firmware does not implement active collision detection during its own transmissions.

---

## Modbus Interface

### Connection

Modbus RTU, RS-485, **9600 baud, 8N1**.

The slave address (1–247) is stored in EEPROM (holding register 0x1012, persisted on write). On first boot (EEPROM blank), it falls back to `USERROW[0] + 2` — so the factory default (`0xFF`) gives address 1. The baud rate (holding register 0x1013, default 0 = 9600) is also persisted to EEPROM.

### Function Codes

| Code | Name | Description |
|------|------|-------------|
| 0x03 | Read Holding Registers | Read configuration registers (max 16 registers per request) |
| 0x04 | Read Input Registers | Read ballast data and device info (max 16 registers per request) |
| 0x05 | Write Single Coil | Write a single coil output |
| 0x06 | Write Single Holding Register | Send a DALI command or update config |
| 0x0F | Write Multiple Coils | Write multiple coil outputs |
| 0x10 | Write Multiple Holding Registers | Write a block of holding registers |

### FC01/FC05/FC0F — Coil Map

Coil N maps to ballast N (0–63).

| Operation | Behaviour |
|-----------|-----------|
| FC01 read coil N | Returns 1 if ballast N is present and brightness > 0, else 0 |
| FC05 write coil N ON (0xFF00) | Sends GOTO_LAST_ACTIVE to ballast N; starts fade tracking |
| FC05 write coil N OFF (0x0000) | Sends OFF to ballast N; starts fade tracking |
| FC05 write coil N TOGGLE (0x5500) | Non-standard extension: reads current brightness and inverts state |
| FC0F write multiple coils | Applies ON/OFF to each coil in the bitmask; returns DEVICE_BUSY if the command queue has fewer free slots than the coil count |

**Broadcast / group operations**: FC01/FC05/FC0F only address individual ballasts (coil N = short address N). To send a DALI broadcast or group command (e.g. turn all lights off), write to the appropriate holding register action command (e.g. `MBREG_OFF` at 0x1004 with the DALI broadcast address byte 0xFF).

### FC04 — Input Register Map

All DALI ballast data is read via FC04 (Read Input Registers). Registers are 16-bit, addressed from 0x0000.

| Address range | Width | Content |
|---------------|-------|---------|
| 0x0000–0x003F | 64 regs | Per-ballast status \| brightness (high byte = DALI status bits, low byte = arc level; 0xFF = absent) |
| 0x0040–0x007F | 64 regs | Per-ballast min \| max level |
| 0x0080–0x00BF | 64 regs | Per-ballast power-on \| sys-fail level |
| 0x00C0–0x00FF | 64 regs | Per-ballast fade rate \| fade time |
| 0x0100–0x010F | 16 regs | Group status — OR of all member ballast status bytes |
| 0x0110–0x014F | 64 regs | Group membership masks — 16 groups × 4 registers (64-bit bitmask per group) |
| 0x0150 | 1 reg | Scan status: 0 = idle, 1 = scanning, 2 = complete |
| 0x0151–0x0154 | 4 regs | Present bitmask — one bit per ballast, 1 = responding |
| 0x0155–0x0158 | 4 regs | Fading bitmask — set when DALI status fade-running bit is active |
| 0x0159–0x015C | 4 regs | Dirty bitmask — set on any block0 change, cleared when read |
| 0x015D | 1 reg | Async raw command result (see FC06 0x1000 / 0x1001) |
| 0x0200–0x020F | 16 regs | Version string: `DALI v<maj>.<min> <10-byte-serial-as-20-hex-chars>` — read exactly 16 registers |

### FC03/FC06/FC10 — Holding Register Map

| Address | Name | Read | Write |
|---------|------|------|-------|
| 0x1000 | RAW_CMD | 0 | `[15:8]` = first DALI frame byte, `[7:0]` = second byte. Single send; waits for backward frame. Result in FC04 0x015D. Returns DEVICE_BUSY if previous result not yet read. |
| 0x1001 | RAW_CMD_TWICE | 0 | Same frame format as 0x1000 but sent twice (for DALI set commands). No backward frame expected. |
| 0x1002 | DAPC | 0 | `[15:8]` = DALI address byte, `[7:0]` = arc level. Sends Direct Arc Power Control. |
| 0x1003 | GOTO_LAST_ACTIVE | 0 | Low byte = DALI address byte. Sends Go To Last Active. |
| 0x1004 | OFF | 0 | Low byte = DALI address byte. Sends Off. |
| 0x1005 | UP | 0 | Low byte = DALI address byte. Sends Up. |
| 0x1006 | DOWN | 0 | Low byte = DALI address byte. Sends Down. |
| 0x1007 | STEP_UP | 0 | Low byte = DALI address byte. Sends Step Up. |
| 0x1008 | STEP_DOWN | 0 | Low byte = DALI address byte. Sends Step Down. |
| 0x1009 | RECALL_MAX | 0 | Low byte = DALI address byte. Sends Recall Max Level. |
| 0x100A | RECALL_MIN | 0 | Low byte = DALI address byte. Sends Recall Min Level. |
| 0x100B | SET_DTR0 | 0 | Low byte = value to load into DALI DTR0. |
| 0x100C | SET_DTR1 | 0 | Low byte = value to load into DALI DTR1. |
| 0x100D | SET_DTR2 | 0 | Low byte = value to load into DALI DTR2. |
| 0x1010 | SCAN_TRIGGER | 0 | Write 0x0001 to start a full bus scan immediately. |
| 0x1011 | RESCAN_INTERVAL | R/W | Seconds between automatic periodic bus rescans. Default 64. Write 0 to disable. |
| 0x1012 | SLAVE_ADDR | R/W | Modbus slave address 1–247. Persisted to EEPROM on write. |
| 0x1013 | BAUD_INDEX | R/W | 0 = 9600, 1 = 19200, 2 = 38400. Persisted to EEPROM on write. |

The DALI address byte for action registers (0x1003–0x100A) can be any valid DALI address byte — short address, group address, or broadcast (0xFE) — encoded in the standard DALI format. The same applies to DAPC (0x1002) and the raw command registers (0x1000, 0x1001).

### Async Raw Command Result (FC04 0x015D)

After writing to 0x1000 (RAW_CMD), poll FC04 register 0x015D for the result:

| Value | Meaning |
|-------|---------|
| 0x0000 | Idle — no command in progress |
| 0xFF00 | Pending — command sent, awaiting response |
| 0x0100 | NAK — no backward frame received |
| 0x02xx | OK — backward frame received; low byte = response |
| 0x03xx | Error — bus/timing fault; low byte = error subtype |

### Fade Tracking

After any DAPC or arc command sent to a short address, the firmware starts a reactive STATUS → BRIGHTNESS → STATUS polling chain for that ballast. The chain continues until the DALI status fade-running bit (bit 4) clears, then stops. At most two DALI commands are queued per chain, so many concurrent fades do not overflow the command queue. The FC04 fading bitmask (0x0155–0x0158) reflects the current state from real DALI responses.

### Multi-Master Operation

The firmware is designed to operate as the sole DALI master on its bus. It can detect when another master is transmitting (rising edge on the RX pin) and will yield rather than collide — but it does not decode traffic it did not initiate. If another DALI master changes ballast levels, the FC04 register data will remain stale until the next scheduled rescan (default: 64 seconds) or a manual scan trigger. For reliable register state, use this device as the exclusive DALI master or reduce the rescan interval (`MBREG_RESCAN_INTERVAL`, 0x1011) to suit the expected update rate.

---

## Hardware

### Pin Assignment (ATtiny1604)

| Pin | Signal |
|-----|--------|
| PA1 | RS-485 TX (USART0 alternate TXD) |
| PA2 | RS-485 RX (USART0 alternate RXD) |
| PA4 | RS-485 DE — driver enable, driven by hardware RS-485 mode (USART_RS485_EXT_gc), no firmware management needed |
| PB0 | DALI TX — open-drain output, driven low to pull bus down |
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
