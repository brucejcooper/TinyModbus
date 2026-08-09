# TinyModbus DALI Gateway

An ATtiny1604-based Modbus RTU to DALI gateway. It acts as a Modbus slave device and puts DALI frames on the wire on behalf of a master.

> ## The gateway is a stateless pipe
>
> **It caches nothing about the bus.** No arc levels, no status, no presence or
> dirty bitmaps, no group masks, no commissioning state, no scan cursors. The
> master submits opaque commands to an 8-slot queue; the gateway executes them
> in accept order and reports each slot's outcome. That is the whole contract.
>
> This is deliberate — "no cache of a cache": **the ballasts are the
> authority, the controller holds the one and only cache, the gateway holds
> none.** The consequences are worth stating plainly —
>
> - **A gateway reboot is consequence-free.** There is no state to lose, so the
>   class of bug where a rebooted gateway served zeroed registers and poisoned
>   the master's cache is structurally impossible, not merely mitigated.
> - **Board replacement loses nothing.** Swap the hardware, and the controller
>   re-establishes everything by querying the ballasts.
> - **Anything reading gateway state returns ILLEGAL DATA
>   ADDRESS** — see the register map below. A client still reading those
>   addresses is a version mismatch and should fail loudly rather than receive
>   plausible zeros.
>
> The one feature that may ever grow back gateway-side is an unsolicited-frame
> event ring for DALI-2 input devices, which needs the gateway to notice frames
> the master did not ask for.

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

Commissioning is **driven by the controller** over the command queue; this firmware only puts the frames on the wire. Out of scope for this firmware: 24-bit forward frames (DALI-2 control-device addressing), memory-bank reads, and DT1/DT8 device-type extensions.

---

## Architectural Model

The firmware is a **queue executor**. There is no work selector, no
reconciliation cache and no background polling: the only thing that ever causes
a DALI frame is a command the master put in a slot.

```
Modbus master                     gateway                        DALI bus
  FC23 write slot k  ─────────►  mbq_slots: slot k = PENDING
                                       │
  FC23/FC03 read status ◄──────  slot statuses (8 registers)
                                       │
                                 main loop: start_next_dali_work()
                                       │  lowest accept-sequence PENDING slot
                                       ▼
                                 dali_sync: unpack [addr,data,flags,dtr0]
                                       │                            ──────►
                                       ▼                            ◄──────
                                 completion (ISR): mbq_complete(slot,
                                       status, response8)
```

**Slot lifecycle** — `idle → pending → executing → complete | failed`. The
master reads a command's fate directly from its slot; there is no id ledger and
nothing to reconstruct after a disruption. A slot the master believes is active
reading back `idle` means the gateway rebooted ("idle means gone").

**Execution order** is the gateway-assigned accept sequence: the slot stamped
first executes first, wrap-aware over the small live window.

**Multi-frame atomicity.** `dali_busy` is held across a whole command, so a
`QF_DTR0` pair (`0xA3<dtr0>` then the frame) goes out back-to-back with nothing
interleaved. Commissioning — which is a long sequence of interdependent frames —
achieves the same exclusivity from the *controller* side by submitting one frame
at a time and admitting nothing else for that slave.

**Ordering guarantee (acceptance-critical).** `mbq_complete` writes `response8`
BEFORE `status`, under the TCB0+ATOMIC fence, and `mbq_read_status` fences each
slot's read. A single-register Modbus read is atomic, so a master can never see
a terminal status paired with a stale response byte.

**Bus-fault handling.** If the DALI line is held asserted past the escape
timeout (200 TE, ~83 ms), the in-flight slot is completed `failed` with a
bus-fault code rather than abandoned — otherwise it would sit `executing`
forever and the master would wait on a command that can never finish.

### What moved to the controller, and why

| was in the gateway | now |
|---|---|
| `block0` arc/status cache | controller queries `QUERY STATUS` / `QUERY ACTUAL LEVEL` |
| present + dirty bitmaps | controller's presence model, with debounced evidence rules |
| group masks + ADD/REMOVE fan-out | controller diffs membership and emits the frames itself |
| per-ballast brightness bank | controller emits DAPC frames |
| boot scan, rescan cursor, scan trigger | controller's boot interrogation + background sweep |
| commissioning state machine | controller drives INITIALISE…TERMINATE as queued frames |

Size effect of the strip: **flash 15474 → 10014 bytes (−35 %)**, **SRAM 532 →
171 bytes (−68 %)**. `g_regs` no longer exists — its only fields were the four
caches above.

---

## DALI State Machine

The firmware implements a five-state bit-level machine that drives all DALI bus activity. Every state transition is interrupt-driven — the main loop simply sleeps until a Modbus command arrives or a queued command is ready to dispatch.

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

### Work selection

There is one source of DALI work: the command queue. Each main-loop pass, if the
bus is idle and no command is in flight, `start_next_dali_work()` takes the
lowest accept-sequence PENDING slot, marks it EXECUTING and dispatches it.

The sync-flag table, the rotating reconciliation cursor, the periodic rescan and
the priority chain that arbitrated between them are **not present**. The
gateway does no polling of its own, so the bus carries exactly the frames the
controller asked for and nothing else — which is also why bus utilisation is now
a property the controller can reason about and tune.

### Multi-Master Behaviour

The firmware detects transmissions from other DALI masters on the bus: if a rising edge arrives on RX while in IDLE, the state machine yields by entering WAIT_FOR_IDLE. It waits for the other master's transaction (forward frame + optional backward frame) to complete and the bus to settle for 22 Te before re-asserting control. The firmware does not implement active collision detection during its own transmissions.

---

## Modbus Interface

> ⚠️ **This section is OUT OF DATE as of queue v4 (task 08) and is being
> rewritten.** Everything below describing the command queue — the FC23
> read/write pattern, the `0x1000` submit window, the `0x1010` slot status block
> and the per-slot lifecycle — describes the RETIRED v2 design. Those function
> codes and address ranges now return ILLEGAL FUNCTION / ILLEGAL DATA ADDRESS.
>
> **The queue is now one function code, FC 0x41**, carrying an optional enqueue
> plus an optional ack, with a gateway-side response queue retained until acked.
> `QUEUE-PROTOCOL.md` is the normative specification and `src/fc41.h` is the wire
> format; a new diagnostic block lives at FC04 `0x0320` (exactly 8 registers).
> Do not build a client against the register map below.
>
> The sysinfo, version, diagnostic and config registers are unaffected.

### Connection

Modbus RTU, RS-485, **9600 baud, 8N1**.

Slave address, baud rate, and rescan interval all live in USERROW so they survive a chip erase + reflash. The factory-fresh (post-erase) USERROW value of `0xFF` maps to address 1 / 9600 baud / 64 s rescan. Writes to holding registers 0x1F01 / 0x1F02 / 0x1F03 trigger a USERROW page-erase-write; the runtime baud/slave switch is deferred until the FC06/FC10 response we are about to send has cleared the wire, so the master sees the response at the old config and the next request at the new one.

### Function Codes

| Code | Name | Description |
|------|------|-------------|
| 0x03 | Read Holding Registers | Read per-ballast level cache, group masks, the queue status block, and USERROW config registers (max 16 regs per request) |
| 0x04 | Read Input Registers | slot status block, version / diag / sysinfo |
| 0x06 | Write Single Holding Register | Update per-ballast level cache, group mask, or an action/config register (0x1F00+) |
| 0x10 | Write Multiple Holding Registers | Write a contiguous holding block — including the command-queue command block (0x1000–0x100F) |
| 0x17 | Read/Write Multiple Registers | Submit queue commands (write range) and collect status/results (read range) in one transaction — see "Raw Command Queue" |

Coils (FC01 / FC05 / FC0F) are not supported: per-ballast on/off / level changes go through the per-ballast brightness bank (0x0000-0x003F) which arms the asynchronous DALI work selector and ACKs immediately; broadcast / group / specialty DALI sends go through the **raw command queue** (command block 0x1000–0x100F via FC10/FC23).  Frames using the retired function codes get an ILLEGAL_FUNCTION exception reply.

### Register map

The map is deliberately small. **Nothing exposes gateway state**; those
addresses return ILLEGAL DATA ADDRESS (0x02) — a client reading them
addresses is a version mismatch and should fail loudly rather than get zeros.

| Address | FC | Width | Content |
|---|---|---|---|
| `0x1000–0x100F` | 10, 17(w) | ≤16 regs | **Command block** (master-write). 8 slots × 2 regs; `0x1000 + 2*slot` = one 32-bit opaque command. A write covers 1–8 *contiguous* slots and is **all-or-nothing**: if any targeted slot is active the whole write is rejected `DEVICE_BUSY (0x06)`. The write count is in REGISTERS — `n_slots = wr_count / 2`. |
| `0x1010–0x1017` | 03, 04, 17(r) | 8 regs | **Slot status block**, one register per slot, served identically in holding and input space (FC23's read range is holding-only, so it targets this). `[status:8 \| response8:8]`; status = `0 idle / 1 pending / 2 executing / 3 complete / 4 failed`; response8 = the backward-frame answer on complete (0 for a no-answer command) or the failure code on failed. Any 1–8 contiguous slots may be read. |
| `0x0200–0x020F` | 04 | 16 regs | Version string `DALI v<maj>.<min> <serial>`. Read exactly 16. |
| `0x0300–0x030F` | 04 | 16 regs | RX diagnostics: `reg0` rx_overflow, `reg1` frame_error, `reg2` crc_error, `reg3` frames_ok, `reg4` last_fail_len, `reg5–12` the last CRC-failed frame. Zeroed on reset. Read exactly 16. |
| `0x0310–0x0313` | 04 | 4 regs | Sysinfo: `reg0–1` uptime_s (32-bit BE), `reg2` boot_count (since cold power-on), `reg3` reset_reason (`RSTCTRL.RSTFR`: bit0 PORF, 1 BORF, 2 EXTRF, 3 WDRF, 4 SWRF, 5 UPDIRF). No EEPROM — uptime is RAM, boot_count is magic-guarded `.noinit`, reset_reason is hardware-latched. Read exactly 4. |
| `0x1F02` | 03, 06 | 1 reg | Modbus slave address (1–247). USERROW-persisted, applied on reboot. |
| `0x1F03` | 03, 06 | 1 reg | Baud index: 0 = 9600, 1 = 19200, 2 = 38400. USERROW-persisted, applied after the ACK. |
| `0x1F05` | 06 | 1 reg | System reset. Write `0xA5A5` to force a software reset once the ACK has left the wire. Any other value → ILLEGAL DATA ADDRESS. |

**Not implemented — all return ILLEGAL DATA ADDRESS:**

| Address | Was |
|---|---|
| `0x0000–0x003F` | per-ballast status\|brightness (the arc-level bank). The controller emits DAPC frames through the queue instead. |
| `0x0040–0x00FF` | per-ballast config banks. |
| `0x0100–0x0103` | present bitmask |
| `0x0104–0x0107` | dirty bitmask |
| `0x0110–0x014F` | group membership masks |
| `0x1F00` | SCAN_TRIGGER |
| `0x1F01` | RESCAN_INTERVAL |
| `0x1F04` | COMMISSION trigger/status |

Coils (FC01/FC05/FC0F) are not supported and return ILLEGAL FUNCTION.

#### Function codes

| Code | Name | Use |
|---|---|---|
| 0x03 | Read Holding Registers | slot status block, USERROW config (max 16 regs) |
| 0x04 | Read Input Registers | slot status block, version / diag / sysinfo |
| 0x06 | Write Single Holding Register | USERROW config, system reset |
| 0x10 | Write Multiple Holding Registers | the command block |
| 0x17 | Read/Write Multiple Registers | **the normal path** — submit commands (write range) and collect status (read range) in one transaction |

The DALI address byte inside a command record can be any valid DALI address byte — short address, group address, broadcast (0xFE), or special command (e.g. 0xA3 SET_DTR0, or the commissioning specials the controller uses to drive INITIALISE…TERMINATE). The gateway does not interpret it, and there is no cache to invalidate: what a command *means* is entirely the controller's business.

### Raw Command Queue

The gateway holds an **8-slot, self-describing command queue**. Each slot has its own command register pair and its own status register, and the **slot is the command's identity** — there is no controller-assigned id, no result ring, and no id sequence to keep in step. A master writes a command into a free slot and polls that slot's status until it reaches a terminal state. The queue itself is **DALI-agnostic**: it carries an opaque 32-bit command and returns an 8-bit response; the meaning of those bytes is the DALI adapter's contract, below. This replaced the task-04 id-tagged FIFO (submit window + result ring + `accepted_id`/`completed_id`), whose id ledger the master had to track and re-baseline across reboots — the whole class of reboot/glitch recovery-corruption bugs that fought is gone because nothing is inferred from an id sequence.

**Slot lifecycle** — each slot's `status` byte walks: `0 idle` → `1 pending` (a command is written and queued) → `2 executing` (on the DALI bus now) → `3 complete` / `4 failed` (terminal, holding a response). A terminal slot stays terminal until the master overwrites it with a new command (which returns it to `pending`).

**Command block** (holding 0x1000–0x100F, master-write; FC10 or FC23 write range) — slot `k`'s command is 2 registers at `0x1000 + 2*k`, an opaque 32-bit value:

```
reg0 (0x1000 + 2*k):  [byte0 | byte1]
reg1 (0x1001 + 2*k):  [byte2 | byte3]
```

A write covers **1–8 contiguous slots** (register count 2/4/…/16) and is **all-or-nothing**: if **any** targeted slot is currently ACTIVE (pending or executing), the gateway rejects the *entire* write with exception **0x06 (SLAVE_DEVICE_BUSY)** and changes nothing. `idle` and collected-terminal (complete/failed) slots are writable — writing a terminal slot reuses it. Because the FC23 write executes before its read, a rejected submit and a successful one are both distinguishable in the same round trip (the 0x06 exception, or the fresh slot statuses in the read-back). The gateway executes accepted commands in a wrap-aware **accept-sequence** order (the order slots were filled), independent of slot index.

**Status block** (0x1010–0x1017, one register per slot, served identically under FC03 and FC04):

```
0x1010 + k:  [status:8 | response8:8]
```

`status` is the lifecycle value above; `response8` is meaningful only in a terminal state. The gateway writes `response8` **before** `status` under a fence a Modbus status read cannot straddle, so a read that sees a terminal `status` always sees the matching `response8` (never a stale one).

**FC23 (0x17) pattern:** the write executes **before** the read, so one FC23 both submits new commands (write range = the command block, 0x1000) and reads back **all 8 slot statuses** (read range = the status block, 0x1010, holding-only) — the master reconciles its view of every slot against the live truth in the same round trip. A read-only FC23 (write count 0) is a pure status poll.

**Gateway reboot:** a reset clears every slot to `idle`. A master that believed a slot was active therefore reads it back `idle` — the unambiguous **"idle means gone"** signal, with no boot-count or id-ledger comparison needed. (See the controller's `mbq_client.c` for the master-side reconciliation and the standalone protocol spec `controller/docs/modbus-command-queue.md`.)

#### DALI adapter (the payload contract)

The 32-bit opaque command is packed by the DALI adapter as four bytes:

```
byte0  addr    DALI forward-frame address byte (any: short / group / broadcast / special)
byte1  data    DALI forward-frame data byte
byte2  flags   bit0 QF_TWICE  — send the forward frame twice, no answer (SET/INITIALISE/RANDOMISE)
               bit1 QF_DTR0   — emit 0xA3<dtr0> back-to-back immediately before the frame (atomic DTR0+SET pair)
               bit2 QF_ANSWER — open the backward-frame window; response8 = the 8-bit answer
               bits 3–7 reserved 0
byte3  dtr0    value preloaded into DTR0 when QF_DTR0 is set; ignored otherwise
```

`response8` in a terminal slot: on **complete**, the DALI backward-frame answer (or 0 for a no-answer command); on **failed**, a failure code — `0x01` NAK (no/short backward frame where one was expected), `0x02` COLLISION (garbled backward frame — ≥1 gear answered), `0x03` BUSFAULT (transport-level fault). COLLISION is distinct from NAK so the controller's any-on probe can read a collision as "≥1 on" and silence (NAK) as "none on".

A `QF_DTR0` command's two frames land back-to-back with nothing interleaved (the slot holds the bus for the pair). After a level-changing frame the gateway marks the target ballast dirty for a priority re-query (group frames fan out to members via the group masks). A queued command **preempts the gateway's idle-time polling** — it runs within ~1–2 DALI frame times of arrival.

### Commissioning

`MBREG_COMMISSION` (0x1F04) and the gateway's binary-search state machine are
**removed**. The controller drives the whole IEC 62386-102 sequence —
INITIALISE, RANDOMISE, SEARCHADDRH/M/L, COMPARE, PROGRAM SHORT ADDRESS, VERIFY,
WITHDRAW, TERMINATE — as ordinary queued frames. No tight inter-step timing is
involved: the gear's INITIALISE window is 15 minutes against a run of tens of
seconds.

Two behaviours the gateway implementation discovered on hardware are preserved
in the controller port, and are easy to get wrong if it is ever reimplemented:

- **COMPARE is anyone-pulls-low.** ANY decoded response means "at least one gear
  matched"; only a clean no-answer means "none". A collided/garbled backward
  frame is a match, not an error.
- **After a bit-0 COMPARE says "no"**, the gear's SEARCHADDR registers hold the
  bit-0-cleared value while the master's copy has bit 0 restored. The search
  address must be **re-sent** before PROGRAM, or PROGRAM addresses nothing and
  VERIFY fails.

See `controller/src/dali_commission.c`.

### Write propagation and fade tracking

The gateway reports one thing per command: that slot's terminal status and
response byte. It does not track fades, does not re-poll, and has no dirty bit
to raise — those were properties of the cache it no longer has.

**Fade tracking is the controller's job.** A DAPC lands immediately but the
level ramps, so the controller re-reads `QUERY STATUS` while bit 4 (FADE
RUNNING) is set and only then trusts the level. See `dali_state.c`.

### Multi-Master Operation

The firmware is designed to operate as the sole DALI master on its bus. It can detect when another master is transmitting (rising edge on the RX pin) and will yield rather than collide — but it does not decode traffic it did not initiate, and it keeps no register state that could go stale.

If another DALI master changes ballast levels, the **controller's** cache is what drifts, and it re-converges on its background sweep (one ballast per `dali_sweep_interval_ms`, default 10 s) or on the next command's post-command verification. Tune that interval controller-side if an installation genuinely has a second master.

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
make configure SLAVE_ADDR=42 BAUD_INDEX=2
```

`BAUD_INDEX` is `0=9600 / 1=19200 / 2=38400`. `make configure` rewrites all 32 USERROW bytes — the four config bytes plus 28 bytes of `0xFF`.

## Known Limitations

These are residual issues that the current firmware does not address. None are runtime-fatal; each has a documented workaround or expected impact.

### Modbus

- **Inter-byte timeout conflated with inter-frame.**  Modbus §2.5.1.1 wants a 1.5-char inter-byte gap to declare a frame malformed and a separate 3.5-char inter-frame silence to mark frame end.  This firmware uses a single threshold sized for 3.5-char silence ({10, 6, 4} TCA0 ticks at 9600/19200/38400), so bytes arriving 2–3 char apart are silently accepted as part of one frame and rely on the CRC check to reject.  Any master that streams its bytes contiguously is unaffected.
- **FC10 partial execution on register-map holes.**  The holding-register map has gaps at 0x0100–0x010F (FC04-only range), 0x1002–0x100A (action regs retired in subtask 4/5), 0x100B–0x100D (DTR setters retired in subtask 9), and 0x100E–0x100F.  A multi-register write spanning a gap aborts on the first invalid address with `ILLEGAL_ADDRESS`, leaving earlier registers in the same FC10 already written and dispatched.  Workaround: do not write across the holes.
- **FC04 0x0200 returns an ASCII version string, not a register address space.**  Reading exactly 16 input registers from address 0x0200 yields the 32-byte string `"DALI v<maj>.<min> <serial>"`.  Other lengths at this address fall through to the standard register-range check and return `ILLEGAL_ADDRESS`.
- **`frame_is_valid` recognises the unsupported coil function codes (FC01 / FC05 / FC0F) for size-check purposes, and the broadcast filter still admits FC05 / FC0F.**  This is intentional: well-formed coil frames pass frame validation, reach the FC dispatcher, and receive a graceful `ILLEGAL_FUNCTION` exception reply (or — for a broadcast-addressed FC05 / FC0F — are silently absorbed) rather than being dropped at the size check.  Candidate for future tightening: removing the FC01 / FC05 / FC0F cases from `frame_is_valid` and from the broadcast filter would drop those frames at validation time instead.  Nothing breaks if it stays as-is; the current shape gives masters that probe for coil support a clearer "function not supported" answer.

### DALI

- **No multi-master collision detection during our own TX.**  If another master begins driving the bus while we are already transmitting, the resulting collision is not detected; the receiving ballast simply sees a malformed frame and ignores it.
- **Receiver dead-band.**  Half-bit and full-bit pulse classification has a ±83 µs tolerance window (`DALI_HALF_BIT_USECS / 5`); pulses falling between the half-bit and full-bit ranges (~500–750 µs) are reported as `PULSE_IN_MIDDLE` and discarded.  At spec-compliant DALI bit timing this never fires.
- **A queued DTR0-first command holds the bus for its pair.**  A command-queue record with the `send-DTR0-first` flag issues two frames (`0xA3<dtr0>` then the frame, e.g. a config DTR0+SET pair) as one atomic unit, so DTR0 can't be clobbered between them.  Other DALI work waits behind the pair (~2 frame times).  Single-frame commands (DAPC level, group-bit changes, most raw commands) are unaffected — ≤ ~50 ms each.  (The old 6-send `SYNC_WRITE_MINMAX/POWER/FADE` sequences were retired with the config banks.)

### Hardware / runtime

- **Stack budget — comfortable again after the config-bank retirement.**  Static allocation is **532 B** of the 1024 B SRAM (`.data + .bss + .noinit`, per `avr-size`), leaving **~492 B** for the stack.  The queue-v2 8-slot command + status arrays (`mbq_slots`) add ~58 B, but retiring the config banks (block1/2/3 + the 14 B `_reserved` anchor hole) freed ~398 B, a net ~340 B reduction from the pre-task-04 872 B.  Worst-case nested-ISR depth (TCA0 LVL0 chain + TCB0 LVL1 chain + main thread mid-`modbus_process`) is ~104 B, so headroom is now ~388 B (was ~10–30 B).  The largest contributor at main level is the `snprintf` in the FC04 version-string path (`modbus_process` reports 49 B `dynamic,bounded` in its `.su` file).  These `.su` figures are per-frame, not cumulative — for a hard peak-stack number run a call-graph stack analyzer.  If a new ISR vector is added, check the `.su` files (`-fstack-usage`) first.
- **RS-485 DE pin floats briefly at reset.**  `main` drives PA4 low immediately, but there is a ~50–200 µs window after reset (before that store executes) where the pin is high-Z.  Subtask 1 enabled the BOD fuse at 2.6 V, narrowing the window by holding the chip in reset until Vcc is stable; an external pull-down on PA4 on the next PCB revision is the residual recommended fix.
- **`USERROW` write halts the CPU.**  A `MBREG_SLAVE_ADDR` / `BAUD_INDEX` write performs a NVMCTRL page-erase-write that suspends CPU execution for ~3.4 ms.  Triggered by Modbus writes the firmware schedules so the halt lands in the master's silence window before our response, but a DALI frame in flight at that exact moment will be corrupted.  Avoid changing config during heavy DALI traffic.
