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

This implementation uses **short addresses only** (individual device addresses 0–63).

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

- **TCA0** — the half-bit clock. Runs continuously at 10 MHz with a period of exactly one DALI half-bit (416.67 µs). On each overflow it decrements a countdown (`daliTimer`) and fires a registered callback (`daliTimerCallback`) when the countdown reaches zero. This drives both transmission timing and state-machine timeouts. It is configured as the highest-priority interrupt so bit-level timing is not displaced by other ISRs.
- **TCB0** — the pulse-width capture timer. Active only during the RECEIVING state. Configured in Frequency Measurement mode and fed by the DALI RX pin via the AVR event system. It captures the time between consecutive edges, giving the width of each Manchester pulse without CPU polling.

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

1. Load the 16-bit command word into `shiftReg`, set `bitsLeft = 17`.
2. Schedule `sendOne()` via `daliTimer = 1` (one half-bit tick).
3. `sendOne()` drives TX high and schedules `halfBitToggle()`.
4. `halfBitToggle()` toggles TX (completing the mid-bit transition), schedules the next bit's first-half function (`sendOne` or `sendZero` from the MSB of `shiftReg`), shifts `shiftReg` left, decrements `bitsLeft`.
5. When `bitsLeft == 0`, schedules `startStopBits()`, which drives TX low for 4 Te (two full stop bits).
6. After stop bits: calls `dali_wait_for_response_state_enter()`.

Each bit takes exactly 2 TCA0 ticks (2 × 416.67 µs = 833.33 µs = 1200 bps).

**WAIT_FOR_RESPONSE** (`dali_state_wait_for_response.c`)

After the forward frame the master must listen for a backward frame within 7–22 Te. This state:
- Pre-configures TCB0 for pulse capture (`dali_state_receiving_prepare()`).
- Arms a one-shot rising-edge interrupt on RX. A rising edge means a backward frame start bit is arriving; we call `dali_state_receiving_enter()`.
- Sets a TCA0 timeout of 22 Te. If it fires before any edge, the device did not respond: call `process_command_response(DALI_RESPONSE_NAK, 0)` and return to IDLE.

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

TCA0 is re-armed to 5 Te on every edge. When TCA0 fires with no new edge — i.e. the bus has been idle for more than two full bits — `timerOverflow()` is called:
- If the final state is `START_0` or `MIDDLE_1` **and** exactly 8 bits were received: `process_command_response(DALI_RESPONSE_OK, shiftreg)`.
- Otherwise: `process_command_response(DALI_RESPONSE_MALFORMED, 0)`.

Either way, transitions to WAIT_FOR_IDLE to enforce the minimum inter-frame gap.

**WAIT_FOR_IDLE** (`dali_state_wait_for_idle.c`)

Enforces the DALI requirement that the bus be idle for at least 22 Te before the next forward frame. Works by watching the RX line:
- If the bus is high (shorted/active), arm a level-low interrupt and wait for release.
- Once the bus goes low (idle), start a TCA0 countdown of 22 Te.
- If the bus goes high again before the countdown expires (another master, collision debris, capacitive ringing), restart.
- When 22 Te completes uninterrupted: call `dali_idle_state_enter()`.

### Command Queue

`dali_queue_command()` (in `dali_state_idle.c`) appends to a 20-entry ring buffer. Each entry holds:
- `cmd` — the 16-bit DALI command word
- `callback` — function to call when the command completes (receives response type, original command, response byte)
- `repeat` — how many times to send before dequeuing (1 for normal commands, 2 for set commands that require the double-send)

When a command completes (or NAKs), `process_command_response()` is called. It:
1. Automatically mirrors the response byte into the appropriate holding register bank based on the command type (e.g. `QUERY_STATUS` → status byte of that device's register, `GET_LEVEL` → low byte of the status bank register).
2. Decrements the repeat counter; if > 0, re-queues the same command for a second send without calling the callback.
3. Once repeat reaches 0, calls the callback and removes the entry from the queue.

### Periodic Rescan

The RTC Periodic Interrupt Timer fires every ~1 second (1024 cycles of the 1 kHz internal oscillator). A software counter (`rescan_counter`) counts down from 64, triggering `dali_start_background_scan(0)` approximately every 65 seconds. The scan walks all 64 DALI addresses in order, issuing 10 query commands per device and caching the results in the holding register banks.

### Multi-Master Behaviour

The firmware detects transmissions from other DALI masters on the bus: if a rising edge arrives on RX while in IDLE, the state machine yields by entering WAIT_FOR_IDLE. It waits for the other master's transaction (forward frame + optional backward frame) to complete and the bus to settle for 22 Te before re-asserting control. The firmware does not implement active collision detection during its own transmissions.

---

## Modbus Interface

### Connection

Modbus RTU, RS-485, **9600 baud, 8N1**.

Device address is stored in byte 0 of the `USERROW` EEPROM. The value stored has 2 added to it, so the factory default (`0xFF`) yields device address 1. Write `0x00` for address 2, etc.

```sh
pymcuprog -t uart -u ${PORT} -d ${DEVICE} -m user_row write -l 0x01
```

### Function Codes

| Code | Name | Description |
|------|------|-------------|
| 0x01 | Read Coils | Read DALI device presence (1=present, 0=absent) |
| 0x03 | Read Holding Registers | Read cached DALI device parameters |
| 0x04 | Read Input Registers | Read device info string |
| 0x05 | Write Single Coil | Toggle/on/off a single DALI device |
| 0x06 | Write Single Holding Register | Write a DALI device parameter |
| 0x0F | Write Multiple Coils | (stub — not yet implemented) |
| 0x44 | Custom: Execute DALI Command | Send a raw 16-bit DALI command and get the response |

### Holding Register Map

Holding registers are arranged in five **banks** of 64 registers, one register per DALI device address.

```
Register address = (bank × 64) + DALI_short_address
```

| Bank | Address range | Register content | Write behaviour |
|------|--------------|------------------|-----------------|
| 0 — Status | 0–63 | High byte: status bits; Low byte: current arc level | Write low byte only (0–254) → sends DAPC |
| 1 — Min/Max | 64–127 | High byte: max level; Low byte: min level | Writes both min and max via DTR0 |
| 2 — Fade | 128–191 | High byte: extended fade time (8-bit, from `QUERY_EXTENDED_FADE_TIME`); Low byte: combined fade time / fade rate byte (high nibble = basic fade time, low nibble = fade rate, from `QUERY_FADE_TIME_FADE_RATE`) | Write low byte → fade rate via DTR0; write high byte → extended fade time via DTR0 |
| 3 — Power | 192–255 | High byte: power-fail level; Low byte: power-on level | Writes both via DTR0 |
| 4 — Groups | 256–319 | 16-bit bitmask of group membership (bit N = group N) | Diffs against cached value and issues Add/Remove per changed bit |

All values are cached in RAM and refreshed by the background scan. A write immediately sends the corresponding DALI command(s) and updates the cache.

### Write Single Coil (0x05) — Simple On/Off/Toggle

Address = DALI short address (0–63).

| Value | DALI action |
|-------|-------------|
| 0x55xx | Toggle (query level, then Off or GoToLastActive) |
| 0xFFxx | Go To Last Active level |
| 0x00xx | Off |

### Custom DALI Command (0x44)

Sends a raw 16-bit DALI forward frame and returns the backward frame response (if any).

Request: `[device_id] [0x44] [cmd_hi] [cmd_lo] [repeat] [crc_lo] [crc_hi]`

- `cmd`: the 16-bit DALI command word (address byte in high byte, data byte in low byte)
- `repeat`: 1 or 2 (set commands must be sent twice)

Response: `[device_id] [0x44] [response_byte] [crc_lo] [crc_hi]`

On error (no response, collision, malformed), a Modbus exception is returned.

### Input Registers (0x04)

Reading input registers returns an ASCII device info string packed two bytes per register. The string is generated on demand into the response buffer (no fixed RAM is reserved for it):

```
Mechination DALI v<major>.<minor> ID:<10-byte-hex-serial>
```

The 10-byte serial is read from the SIGROW unique-ID registers, so each chip reports a globally unique identifier.

### Background Scan

On startup the firmware scans all 64 DALI addresses in order, issuing the full query sequence (status, level, max, min, fade, power-on, power-fail, groups 0–7, groups 8–15) per device and caching the results in the holding register banks. Devices that do not respond get their registers zeroed (level bank set to `0xFF` as an absent marker). The scan repeats continuously to keep the cache fresh and to track fading.

---

## Hardware

### Pin Assignment (ATtiny1604)

| Pin | Signal |
|-----|--------|
| PA1 | RS-485 TX |
| PA2 | RS-485 RX |
| PA4 | RS-485 DE (driver enable) |
| PB0 | DALI TX |
| PB1 | DALI RX |
| PB3 | Status LED |

### Power

The device runs at 3.3 V, 10 MHz. It spends the majority of its time in Standby sleep mode, consuming under 1 µA. The watchdog timer resets the device if it hangs for more than 8.2 seconds.

---

## Building

Install `avr-gcc` with ATtiny1604 support and `pymcuprog`. The Homebrew version of avr-gcc does not include this CPU; use the Makefile target to download the Arduino-bundled toolchain instead.

```sh
make download_gcc

# Build and flash
make
```

Programming is via UPDI on the test point. Use a serial adapter wired as described in the [jtag2updi guide](https://github.com/SpenceKonde/AVR-Guidance/blob/master/UPDI/jtag2updi.md) with a USB-FTDI adapter.

---

## Reliability Features

The firmware includes several layers of protection against bus faults and lockup:

### Watchdog Timer

The WDT runs at 8.2 s (`WDT_PERIOD_8KCLK_gc`) and is reset on every iteration of the main loop. If any blocking path runs longer than 8.2 s — or if the main loop stops for any reason — the MCU resets cleanly.

### `WAIT_FOR_IDLE` bus-stuck timeout (`dali_state_wait_for_idle.c`)

When the DALI bus is shorted (active low) longer than 200 Te ≈ 83 ms, `busStuckTimeout()` fires. It defensively releases our TX pin, cancels the edge detector, and re-enters the idle state machine. This prevents an external bus fault (faulty ballast, shorted wiring) from hanging the state machine indefinitely.

### `waitForDaliResponse()` Modbus timeout (`rs485.c`)

The Modbus handler bounds its wait for a DALI response at 600 ms. **Any Modbus command whose underlying DALI batch has not completed within ~500 ms will be reported to the master as a NAK** — well before the 8.2 s WDT fires. The DALI commands themselves remain queued and continue to execute on the bus; only the Modbus-visible status is affected. This caps how long an RS485 master can be blocked by a single command, regardless of DALI bus state.

### RS485 DE-pin startup safety (`main.c`)

The very first thing `main()` does — before clock setup or any subsystem init — is drive PA4 (RS485 DE) low and configure it as an output. This eliminates the post-reset window during which the transceiver could otherwise assert the bus while DE floats.

### Modbus packet-size validation

The RX ISR drops bytes once `sz` reaches `MAX_PKT_SZ`, and `inputIsValidModbusCommand()` requires both a CRC of zero and a function-specific exact length match before delivering a packet to the handler. The read-holding-registers path streams data directly from `dali_holding_regs` (see below) so the response is not constrained by `buf`'s capacity.

### Streaming TX for read-holding-registers

A 64-register response is 133 bytes — too large for `buf` (70 bytes). Rather than grow `buf` to fit the worst case, `sendHoldingRegs()` streams data straight from `dali_holding_regs` in the DRE ISR:

1. The 3-byte header (device addr, func code, byte count) is placed at `buf[0..2]` and the running CRC is seeded over those bytes.
2. The DRE ISR sends the 3 header bytes from `buf`, then enters streaming mode.
3. While streaming, the ISR reads each register fresh (one atomic 16-bit load per word, used for both the high and low byte) and folds each transmitted byte into the running CRC. This means the CRC always matches the bytes actually sent — even if a TCA0 ISR (e.g. the periodic scan) updates a register mid-response.
4. After the final low byte is sent, the running CRC is parked at `buf[3..4]` and the ISR sends 2 trailing CRC bytes from there.

A single read can return up to the Modbus RTU limit of 125 registers without ever buffering the data. Streaming state costs 7 bytes (`streamSrc`, `streamWords`, `streamHighByte`, `streamLowByte`, `streamCrc`).

### RAM Budget

| Item | Bytes |
|------|-------|
| ATtiny1604 SRAM | 1024 |
| `dali_holding_regs` | 640 |
| `commandQ` (20 × 5 B) | 100 |
| `buf` | 70 |
| Streaming TX state | 7 |
| Misc state | ~32 |
| **Static total** | **849** |
| **Stack headroom** | **175** |

The build sets `-fstack-usage`, emitting per-function `.su` files in `build/` after each compile. The deepest interrupt-nested call chain measured from those files (main thread executing `dali_set_min_level` → `dali_queue_command` → `dali_transmitting_state_enter`, USART0_RXC ISR preempting it, TCA0 LVL1 preempting USART0_RXC and running the full receive-timeout chain through `process_command_response` → `scan_result_received` → `dali_start_background_scan` → `dali_queue_command` → `dali_transmitting_state_enter`) consumes **≈ 100 bytes** of stack. Worst-case with safety margin for ISR register saves is ≈ 115 bytes, leaving ≈ 60 bytes of headroom against the 175-byte budget. Re-check `build/*.su` after any change that materially deepens a frame.

### Flash Budget

| Item | Bytes |
|------|-------|
| ATtiny1604 flash | 16384 |
| Program (.text + .rodata + .data init) | 9492 |
| **Free** | **6892** (~42 %) |

The build uses no `printf`, `malloc`, or floating-point — `static const` tables (CRC, command sequences, state machine) live in flash via the avrxmega3 unified memory map and are accessed with plain LD instructions, no `PROGMEM` needed.

### Concurrency Model

Where shared state is touched from both main context and ISR context, the following mechanisms keep it consistent:

- **DALI command queue (`commandQ`, `q_size`)** — `dali_queue_command()` wraps the size-check / slot-write / post-increment in `ATOMIC_BLOCK(ATOMIC_RESTORESTATE)`, and atomically claims the "start TX" role by clearing `dali_idle` in the same block. This prevents a TCA0/TCB0 ISR (running e.g. `scan_result_received`) from compacting the queue between main's read of `q_size` and its slot write.
- **Background scan state (`scan_index`, `scan_err_count`)** — `dali_start_background_scan()` is called from both the RTC_PIT ISR and the TCA0 callback chain (`scan_result_received`); the reset + initial queue is wrapped in `ATOMIC_BLOCK` so the two callers can't interleave.
- **Streaming TX register reads** — the DRE ISR reads each `dali_holding_regs` word inside `ATOMIC_BLOCK` (TCA0 LVL1 can preempt the DRE ISR; the 16-bit load needs to be uninterruptible to avoid a torn high/low pair).
- **Modbus error reporting** — `processSetSingleHoldingRegisterCommand` and `processSetSingleCoilCommand` branch on `daliResponseType` after `waitForDaliResponse()` and emit a Modbus exception when the DALI batch did not complete OK. Group-bank writes that change zero bits are reported as a no-op success.
- **Broadcasts** — `inputIsValidModbusCommand()` only accepts deviceId 0 for write-style functions (`0x05`, `0x06`, `0x0F`); broadcast reads are dropped to avoid bus-collision responses.
