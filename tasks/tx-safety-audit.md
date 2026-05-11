# DALI TX Safety Audit — Bus-Stuck Failure Mode

- Repo: `/Users/bruce/dev/light_switch/TinyModbus`
- Branch: `task-1-flag-driven`, HEAD `cee394a`, tag `task-1-complete`
- MCU: ATtiny1604 @ 10 MHz, 3.3 V; BOD active @ 2.6 V; WDT 8.2 s
- Scope: every reachable path that mutates `DALI_PORT` (`PORTB`) bit 0 (PB0)

---

## 0. Hardware-polarity correction (load-bearing)

**The audit prompt's polarity assumption is inverted relative to the firmware.**

The audit prompt states: "When the firmware drives PB0 LOW the bus is asserted (active); when PB0 is HIGH or high-Z the bus is idle". The actual firmware semantics, established by:

- `dali_state_idle.c:69-70` "ensure the DALI TX line is released before entering idle … `OUTCLR`"
- `dali_state_wait_for_response.c:15-17` "release the DALI TX line before yielding back to idle … `OUTCLR`"
- `dali_state_wait_for_idle.c:28-29` "Force TX low (release any stuck open-drain drive) and recover. … `OUTCLR`"
- commit `9de0ecc` "DALI TX OUTCLR + BOD fuse … OUTCLR DALI TX as first action (belt-and-braces against glitched paths leaving the bus driven)"
- `dali_state_transmitting.c:29-31` `send_one()` (Manchester '1', first half drives line ASSERTED): `OUTSET`

…all consistently treat **`OUTCLR` (PB0 = LOW) as the released / idle / safe state** and **`OUTSET` (PB0 = HIGH) as the asserted / driving state**. The external transistor / opto-driver inverts polarity between MCU pin and DALI bus line, so the README's statement "PB0 ... open-drain output, driven low to pull bus down" describes the BUS line, not the MCU pin.

Throughout this report I use the firmware's convention:

- **PB0 LOW (OUT bit = 0, DIR = 1)** = bus released / idle / safe.
- **PB0 HIGH (OUT bit = 1, DIR = 1)** = bus asserted / driven low / unsafe.
- **PB0 high-Z (DIR = 0)** = bus released. Hardware reset state. (External pull on transistor base keeps the gear off.)

A "stuck-asserted" failure means PB0 latched HIGH (with DIR=1) when no transmission is in progress and the firmware has no remaining path that will release it within bounded time. **The phrase "OUTCLR releases the bus" appears throughout this report and reflects the actual implementation.**

---

## 1. Executive summary

**Bottom line: I did not find any reachable path that leaves PB0 stuck HIGH (bus asserted) indefinitely.** Every site that drives PB0 HIGH is followed, on every reachable continuation, by either (a) a direct `OUTCLR`/`OUTTGL` that releases it, or (b) a TCA0-tick-driven callback that ends with `OUTCLR` in `start_stop_bits`/`finished_transmission`. Multiple defence-in-depth layers also unconditionally `OUTCLR` on every state-machine boundary that could plausibly be reached after a glitch.

Findings:

| Severity | Count | Summary |
|---|---|---|
| **P0 (must-fix, real bug enabling stuck-bus)** | 0 | None. |
| **P1 (should-add, residual concerns)** | 2 | (1) WDT-during-active-TX recovery rests on the AVR0/1 peripheral-reset semantics being applied to PORTB — see §3.C; (2) ISR-context bit-level state machine has no upper bound on time-spent-asserted that is enforced in the firmware (only the DALI 1.5 TE half-bit cadence, which is implicit). See §3.F. |
| **P2 (defensive enhancements)** | 3 | "TX-asserted watchdog", documentation of the polarity convention in `dali.h`, removing the stale `dali_init.c:20` comment that misdescribes the OUTCLR behaviour. |
| **Documentation** | 1 | The README hardware-pin table line for PB0 is ambiguous given the in-firmware inversion (see §0). |

The single most important recommendation is the **P2-1 "TX-asserted watchdog"** — see §5. It is *not* a fix for any present bug, but it is the cleanest residual defence given that the entire scheme today rests on TCA0 ticks continuing to fire. Cost: ≈ 30 bytes flash plus one byte of RAM.

I would describe the firmware as **safe with caveats** rather than absolutely safe; the caveats are documented in §3.B (cold-start window) and §3.C (WDT-during-TX). Both rely on the AVR0/1 series datasheet behaviour for PORTB reset, which I have not been able to physically verify on hardware as part of this audit.

---

## 2. The PB0 enumeration table

Every code site that touches `PORTB` (= `DALI_PORT`) bit 0 (= `DALI_TX_bm` = `PIN0_bm`), in source order. Reset = "what state PB0 ends up in after this write".

| # | Site | Op | Resulting PB0 (assuming DIR was already 1) | Context (which state / ISR) | Reachable from PB0=HIGH? | Guaranteed release? |
|---|---|---|---|---|---|---|
| 1 | `dali_init.c:21` | `OUT.OUTCLR = TX_bm` | LOW (released) | `dali_init()` boot | First write after reset (high-Z → low) | ✅ — sets pin LOW before any path can assert it. |
| 2 | `dali_init.c:22` | `DIR.DIRSET = TX_bm` | LOW (driven) | `dali_init()` boot | First write after reset | ✅ — pin was OUTCLR'd one instruction earlier. |
| 3 | `dali_state_transmitting.c:31` | `OUT.OUTSET = TX_bm` | **HIGH (asserted)** | `send_one()` — Manchester '1' first half | YES — this is the assert | Released by next `half_bit_toggle` (TCA0 1-tick later, ≈ 417 µs) → `OUTTGL` flips to LOW. |
| 4 | `dali_state_transmitting.c:38` | `OUT.OUTCLR = TX_bm` | LOW (released) | `send_zero()` — Manchester '0' first half | n/a (this writes LOW) | ✅ — already LOW. |
| 5 | `dali_state_transmitting.c:45` | `OUT.OUTCLR = TX_bm` | LOW (released) | `start_stop_bits()` | n/a (releases) | ✅ — explicit release before `finished_transmission` runs. |
| 6 | `dali_state_transmitting.c:52` | `OUT.OUTTGL = TX_bm` | flips (HIGH↔LOW) | `half_bit_toggle()` mid-bit | YES if previous half was OUTSET | Released by next `send_zero` (LOW), or `start_stop_bits` (LOW), or another `half_bit_toggle` that toggles it back. See §3.A for the exhaustive trace. |
| 7 | `dali_state_idle.c:70` | `OUT.OUTCLR = TX_bm` | LOW (released) | `dali_idle_state_enter()` belt-and-braces | YES (defensive) | ✅ — unconditional, first executable statement of the function. |
| 8 | `dali_state_wait_for_idle.c:42` | `OUT.OUTCLR = TX_bm` | LOW (released) | `bus_stuck_timeout()` 200-TE escape | YES (defensive) | ✅ — unconditional, first statement of the timeout handler. |
| 9 | `dali_state_wait_for_response.c:17` | `OUT.OUTCLR = TX_bm` | LOW (released) | `timeout_waiting_for_response()` | YES (defensive) | ✅ — unconditional, first statement. |

Sites that touch the **RX** pin (PB1) — listed for completeness; none of them affect PB0:

| # | Site | Op | Effect |
|---|---|---|---|
| R1 | `dali_init.c:25` | `OUT.OUTCLR = RX_bm` | RX OUT bit cleared (no effect: pin is input). |
| R2 | `dali_init.c:26` | `DIR.DIRCLR = RX_bm` | RX = input. |
| R3 | `dali_init.c:51` | `INTFLAGS = RX_bm` | Edge IRQ flag clear. |
| R4 | `dali_init.c:52, 60` | `PIN1CTRL = ISC_*` | RX edge/level interrupt config. |
| R5 | `dali_init.c:64` | `PIN1CTRL = 0` | RX edge/level interrupt off. |
| R6 | `dali_init.c:74-76` | `INTFLAGS / PIN1CTRL` (PORTB ISR) | Edge IRQ teardown after fire. |

No code anywhere writes to `DALI_PORT.OUT`, `DALI_PORT.OUTSET`, `DALI_PORT.OUTCLR`, `DALI_PORT.OUTTGL`, `DALI_PORT.DIR`, `DALI_PORT.DIRSET`, or `DALI_PORT.DIRCLR` for PB0 outside the table above. Verified by `grep -rn "DIRSET\|DIRCLR\|DIRTGL\|OUTSET\|OUTCLR\|OUTTGL\|PORTB" src/`. The single modbus.c reference is `RS485_PORT = PORTA`, not PORTB.

The RTC PIT ISR (`dali_sync.c:1093`), USART ISRs (`modbus.c`), TCA0 OVF, TCB0 capture, and PORTB pin ISR (`dali_init.c:70`) collectively contain **zero** writes to PB0's OUT or DIR bits. Only PORTB pin-control register `PIN1CTRL` (RX) is touched from the PORTB ISR.

---

## 3. Path analysis per category

### 3.A. Static code paths (state-machine sequences)

**TRANSMITTING (`dali_state_transmitting.c`).** The bit-level state machine is the only place PB0 is ever driven HIGH. Walked step by step:

```
dali_transmitting_state_enter:
  shift_reg = addr<<8 | cmd
  bits_left = 17
  dali_timer_cb = send_one    ← first half of MSB will OUTSET (assert)
  dali_timer = 1              ← arms TCA0 to fire send_one in 1 tick (~417 µs)

send_one (TCA0 callback):
  OUTSET  ← PB0 HIGH (asserted)
  dali_timer_cb = half_bit_toggle
  dali_timer = 1

half_bit_toggle:
  OUTTGL                       ← PB0 flips
  if --bits_left == 0:
      dali_timer_cb = start_stop_bits
  else:
      dali_timer_cb = (MSB ? send_one : send_zero)
      shift_reg <<= 1
  dali_timer = 1

send_zero:
  OUTCLR  ← PB0 LOW (released)
  dali_timer_cb = half_bit_toggle
  dali_timer = 1

start_stop_bits:
  OUTCLR  ← PB0 LOW (released)
  dali_timer_cb = finished_transmission
  dali_timer = 4
```

Invariant established by inspection: the cadence is *always* "(OUTSET or OUTCLR) → 1 tick → OUTTGL → 1 tick → (OUTSET or OUTCLR) → ...". After a Manchester '1' is started by `OUTSET`, the very next scheduled callback is `half_bit_toggle`, which `OUTTGL`s back to LOW. After 17 such bit-pair iterations, `start_stop_bits` is scheduled, which `OUTCLR`s. After 4 ticks (≈ 1.7 ms — twice the bit time, two stop bits), `finished_transmission` runs:

```
finished_transmission:
  if t == CMD_QUERY or CMD_SEND_TWICE:
      dali_wait_for_response_state_enter()
  else:
      dali_result_received(NAK, 0)
```

Both branches lead to states whose paths **all eventually** call either `dali_idle_state_enter` (which OUTCLRs at line 70) or `dali_wait_for_response_state_enter` (which is followed by `timeout_waiting_for_response` after 22 ticks if no response — and that OUTCLRs at line 17), or `dali_state_receiving_*` → `timer_overflow` / `send_rx_error` → `dali_result_received` → `dali_sync_completion` or `dali_idle_state_enter` (both ultimately OUTCLR).

**IDLE (`dali_state_idle.c`).** First statement of `dali_idle_state_enter`:

```c
DALI_PORT.OUTCLR = DALI_TX_bm;        // line 70: belt-and-braces
```

Unconditional. Both branches (`dali_busy ? dispatch : arm idle`) execute *after* the OUTCLR, so even if the entry point was reached with PB0 HIGH (e.g. via a glitch), the bus is released within the same instruction-stream sequence point as the function entry.

**WAIT_FOR_IDLE (`dali_state_wait_for_idle.c`).** Three callbacks:

- `idle_period_expired` → `dali_cancel_edge_detect()` → `dali_idle_state_enter()` (which OUTCLRs).
- `on_idle_bus` → arms 22-TE timer on `dali_call_on_rising_once(on_bus_shorted)`. No PB0 write. State machine stays here until either timer expires (→ idle, OUTCLR) or rising-edge fires `on_bus_shorted`.
- `on_bus_shorted` → arms 200-TE bus-stuck timer on `dali_call_on_low_once(on_idle_bus)`. No PB0 write.
- `bus_stuck_timeout` → **OUTCLR (line 42)** + clears `dali_busy`, `current_op`, possibly `commission_state`, then → `dali_idle_state_enter()` (which OUTCLRs again).

**WAIT_FOR_RESPONSE (`dali_state_wait_for_response.c`).** Two callbacks:

- `start_receiving_response` → `dali_state_receiving_enter()`. No PB0 write. Eventually one of `timer_overflow` / `send_rx_error` fires from TCB0 ISR → `dali_result_received` → either `dali_sync_completion` (→ `dali_wait_for_idle_state_enter`) or `dali_idle_state_enter`. Both OUTCLR-clean.
- `timeout_waiting_for_response` → **OUTCLR (line 17)** + cleanup + `dali_result_received(NAK, 0)`.

**RECEIVING (`dali_state_receiving.c`).** Pure decode; no PB0 writes. Outcomes: `timer_overflow` or `send_rx_error` → `dali_result_received` (cleanly OUTCLR-bound through idle/wait-for-idle).

**dali_sync.c (commissioning + work selector).** Pure data-plane logic on top of the bit-level state machine; *no direct PORTB writes*. `dali_sync_completion` always tail-calls `dali_wait_for_idle_state_enter` (line 1082), which on every callback path eventually reaches `dali_idle_state_enter` (which OUTCLRs).

**Verified-safe.** No reachable static path produces a stuck-asserted PB0.

### 3.B. Reset and startup

**Hardware reset state.** ATtiny1604 (AVR0/1 series) datasheet: at any reset (POR / BOD / WDT / external / SW), all I/O pins enter their default state where `DIR = 0` (high-Z input) and `OUT = 0`. No internal pull-up. PB0 is therefore high-Z at reset, which keeps the external transistor off → bus released.

**Pre-`main()` window.** ATtiny1604 has no constructors, no pre-`main` C++ initialisers, no `__low_level_init`. The reset vector jumps to the GCC startup that zeroes BSS, copies `.data`, sets up SP, and calls `main`. None of this touches PORTB. `sei()` is *not* yet enabled — interrupts are masked from reset until `main:58`. So nothing can set PB0 HIGH between reset and the first executable line of `main()`.

**Order of operations in `main()`.**

```
PORTA.DIRSET = PIN4_bm   // line 34 — RS-485 DE configured (not PB0)
PORTA.OUTCLR = PIN4_bm   // line 35
CCP/MCLK config           // line 38-39
regs_init()               // line 42 — pure RAM (no PORTB)
dali_sync_init()          // line 43 — pure RAM (no PORTB)
tick_init()               // line 44 — TCA0 only (no PORTB)
modbus_init(...)          // line 45 — PORTA only
dali_init()               // line 46 — FIRST PB0 touch
set_sleep_mode + sleep_enable  // 48-49
WDT enable (CCP)          // 55
wdt_reset                  // 56
sei()                      // 58 — interrupts unmasked
loop ...
```

`dali_init()` is the first place PB0 is touched. Its sequence (`OUTCLR` then `DIRSET`) drives PB0 through a deterministic high-Z → low transition without ever passing through a "DIR=1, OUT=1" state. Specifically:

- After reset: DIR=0, OUT=0 (high-Z, externally pulled — bus released).
- `OUTCLR` (line 21): DIR=0, OUT=0 — no change (still high-Z).
- `DIRSET` (line 22): DIR=1, OUT=0 — driven LOW (bus released).

There is no instruction-stream window in `dali_init` where PB0 would be DIR=1+OUT=1.

After `dali_init()`, the rest of `main()` does not touch PB0 again. The only way PB0 becomes HIGH from this point is via the TCA0-driven state machine **after** `sei()` has been called, in `dali_state_transmitting.c:send_one`. Before that ever fires, `dali_wait_for_idle_state_enter` is the entry state from `dali_init` (line 40) — it arms a 22-TE idle countdown but does **not** drive PB0 HIGH. The `dali_busy` flag is `false` and the queue is empty, so the first tick after `sei()` cannot dispatch a TX.

**Verified-safe.** No reset-path window leaves PB0 driven HIGH.

### 3.C. WDT recovery (P1-1)

The WDT period is 8.2 s (`WDT_PERIOD_8KCLK_gc` ≈ 8192 cycles of the 1 kHz internal RC = 8.192 s). It is petted in `main()` on every loop iteration (line 72). The maximum un-pet path I can identify:

- Main loop sleep (`sleep_cpu()` on line 75): WDT ticks while the CPU is in standby. The next interrupt (TCA0 every 417 µs at minimum, plus PIT every ~1 s, plus USART RX) wakes the CPU and `wdt_reset` is called within microseconds.
- **Worst case**: an ISR loop that runs continuously without returning to main. Examined: TCA0 OVF runs `dali_tick + modbus_tick`, both of which return promptly. TCB0 capture decodes one pulse and returns. PIT ISR sets one flag and clears INTFLAGS. PORTB pin ISR fires the one-shot callback and returns. None of these loop. **No reachable ISR-only loop exists.** A WDT timeout therefore implies either a pre-`sei()` hang (the CCP-protected stores in main:39, 55) or a hardware-level fault.

When WDT does fire, the AVR0/1 series resets all peripherals including PORTB. From the ATtiny1604 datasheet (Section 16.5.4 "I/O Lines"): "On a Reset, all ports … are set to the input direction" — DIR=0 (high-Z), and the PORT.OUT bits also reset to 0. This applies on every reset source, including WDT.

**Concern.** I have not physically verified this peripheral-reset behaviour on hardware. The audit is relying on the AVR0/1-series datasheet's stated behaviour. If for any reason the WDT reset did *not* clear PORTB.OUT (e.g., some erratum, or an interaction with some persistent VPORT mode), an in-flight TX with `OUT=1` could survive the reset, and `dali_init`'s OUTCLR-then-DIRSET sequence would (correctly) re-release it within ~50 cycles. The window is at most ~50 µs at 1 MHz worst-case startup clock — likely shorter. This is **a residual concern, not a bug**, but warrants explicit documentation.

**Could the firmware enter an infinite loop while transmitting (PB0 HIGH) without `wdt_reset` being called?** Walking each PB0-HIGH-capable site:

- `send_one` OUTSETs PB0 HIGH, then sets `dali_timer = 1`. If TCA0 fires, `half_bit_toggle` runs ≈ 417 µs later (which OUTTGLs to LOW). If TCA0 does **not** fire (e.g., `TCA0.SINGLE.CTRLA` got cleared, or the OVF interrupt was masked), PB0 stays HIGH and the WDT will reset within 8.2 s. The reset clears DIR back to 0 (high-Z) → external pull-down on transistor base releases the bus.
- `half_bit_toggle` OUTTGLs (could be HIGH→LOW or LOW→HIGH). Same analysis — next TCA0 tick is the recovery.

So even if TCA0 stops firing mid-TX, the worst-case stuck-asserted period is one WDT cycle = 8.2 s. The bus is then automatically released on reset. Subsequent firmware re-init runs `dali_init`, which leaves PB0 in DIR=1+OUT=0 (driven LOW) within ~5 µs of the first instruction.

**Verified, with the datasheet caveat noted above.** I'd rate this as P1 only because I cannot physically verify the AVR0/1 PORTB reset semantics from this audit alone.

### 3.D. Brown-out / power transients

BOD is fused at `LVL_BODLEVEL2_gc` (2.6 V) in active mode. From the ATtiny1604 datasheet § "Brown-out Detector": the BOD holds the device in reset while Vcc < 2.6 V and releases it ≈ 100 µs after Vcc rises above the threshold + hysteresis. During the BOD-asserted period the chip is in reset → all I/O lines are high-Z (DIR=0).

**Window between Vcc rising above BOD threshold and `main()` entering `dali_init`.** Once Vcc is stable, the chip exits reset, the GCC startup runs (zeroing BSS, copying `.data`, ~50-100 µs at 10 MHz), and main begins. PB0 is high-Z throughout. The first PORTB write is `dali_init()` line 21 OUTCLR (no effect since DIR=0) → line 22 DIRSET (drives LOW). Total PB0-high-Z duration after Vcc-stable: probably 200-500 µs (the period during which the external transistor's base is unbiased — and in the standard DALI driver topology the bus stays released because the transistor is held off by base pull-down, not by the MCU drive).

**Window during a glitch where Vcc dips below 2.6 V.** BOD asserts reset within microseconds. The reset transitions PB0 to high-Z, which (assuming the external driver is correctly designed with a base pull-down) releases the bus. If Vcc was dipping while the firmware was transmitting (`OUT=1, DIR=1`), the BOD-driven peripheral reset clears OUT=0 and DIR=0 simultaneously. **No window where PB0 is driven HIGH during BOD assertion.**

**Window analogous to RS-485 DE (~50-200 µs) for PB0?** README's "RS-485 DE pin floats briefly at reset" entry exists because `PORTA.DIRSET = PIN4_bm; OUTCLR = PIN4_bm` is the very first instruction in main. PA4 is an output that needs to be LOW. The DE-floats issue is that during the high-Z window the external RS-485 driver's DE input is undefined — **this is a different concern from PB0**: PB0 is also high-Z during the same window, but external pull-down on the transistor base means high-Z = bus released. So PB0 has the same "high-Z briefly at reset" window, but unlike DE, the high-Z state for PB0 is the **safe** state.

**Verified-safe.** No brown-out or power-transient path leaves PB0 stuck HIGH.

### 3.E. Sleep mode interaction

Sleep mode is `SLEEP_MODE_STANDBY` (configured in `main:48`). On AVR0/1, Standby retains GPIO output drivers — if PB0 is DIR=1+OUT=1 when sleep is entered, it remains DIR=1+OUT=1 until an interrupt wakes the CPU or the next reset.

The sleep gate is `if (dali_idle && modbus_idle && !dali_busy)`. Trace:

- `dali_idle` is set to `true` in exactly one place: `dali_state_idle.c:82` — and this happens *after* line 70's `OUTCLR DALI_TX_bm`. So whenever `dali_idle == true`, PB0 was OUTCLR'd within the same function body.
- `dali_idle` is set to `false` in `cleanup()` (`dali_state_idle.c:15`) and in `start_next_dali_work` (line 567). Both of these are entered with `dali_idle == true` (i.e. PB0 already LOW), and neither asserts PB0 HIGH directly — they hand off to TX or to wait-for-idle.
- `dali_busy` is set `true` by `start_next_dali_work` (line 566) before any TX begins. Set `false` in three places: `bus_stuck_timeout` (line 44, after OUTCLR on line 42), `dali_sync_init` (line 83, boot only), and `dali_sync_completion` (line 1075, before tail-call to `dali_wait_for_idle_state_enter`). All three paths transit through OUTCLR before main can next observe `dali_busy == false`.

**Could the sleep-gate `dali_idle && modbus_idle && !dali_busy` evaluate true while PB0 is HIGH?**

- `dali_idle == true` ⇒ in `dali_idle_state_enter`, the first instruction OUTCLR'd PB0. So PB0 was LOW at the moment `dali_idle` went `true`. Could PB0 have been re-asserted between then and the sleep evaluation? Only by entering `dali_state_transmitting.send_one` — which is *gated* on `dali_idle = false` being set first by `start_next_dali_work` (line 567). So if `dali_idle` is still `true`, no TX has been started, and PB0 is still LOW.
- An ISR firing between the gate evaluation and `sleep_cpu()` could theoretically set `dali_idle = false` and assert PB0 HIGH (via TX dispatch), at which point `sleep_cpu()` would still execute. But `sleep_cpu()` itself does not re-check the gate — it simply commits to sleep. **However**: on AVR0/1, `sleep_cpu` only sleeps if no interrupt is pending. If an ISR-driven TX has just begun, the next TCA0 tick (within 417 µs) will wake the CPU. **But during that 417 µs window the CPU is asleep with PB0 HIGH.** This is an *expected* TX state (PB0 HIGH during the first half of a Manchester '1') and resolves via TCA0 wake → `half_bit_toggle` → OUTTGL within one half-bit time. **Not a stuck condition.**
- Could a TX *complete* and leave PB0 HIGH, then sleep? Only if `dali_idle_state_enter` was reached *without* its OUTCLR firing. But OUTCLR is the first executable statement of the function — there is no entry-without-OUTCLR path.

**Verified-safe.** The sleep gate cannot evaluate true with PB0 indefinitely stuck HIGH.

### 3.F. ISR safety (P1-2)

ISR vectors in this firmware:

- `TCA0_OVF_vect` (LVL0): runs `dali_tick + modbus_tick`. `dali_tick` decrements `dali_timer` and, if zero, fires `dali_timer_cb`. The callbacks are the bit-level state machine (`send_one`, `send_zero`, `half_bit_toggle`, `start_stop_bits`, `finished_transmission`, `idle_period_expired`, `bus_stuck_timeout`, `timeout_waiting_for_response`, `timer_overflow`). All of these have been audited above.
- `TCB0_INT_vect` (LVL1, raised via `CPUINT.LVL1VEC = TCB0_INT_vect_num` in `tick.c:19`): runs the pulse decoder (`dali_state_receiving.c:99`). No PB0 writes. On terminal pulse-error: `send_rx_error` → `dali_result_received` (still LVL1 context). `dali_result_received` either tail-calls `dali_sync_completion` (→ `dali_wait_for_idle_state_enter` → eventually OUTCLR) or `dali_idle_state_enter` (OUTCLR first statement). **No PB0 writes from TCB0 ISR or its tail-calls.**
- `RTC_PIT_vect` (LVL0): increments tick counter, sets `SYNC_READ_LEVEL` flag, clears INTFLAGS. No PB0 writes.
- `USART0_RXC / RXS / DRE / TXC ISRs` (LVL0): handle Modbus framing. No PORTB writes (PORTA only).
- `PORTB_PORT_vect` (LVL0): clears INTFLAGS, clears PIN1CTRL (RX edge detect), reads `dali_rx_callback` atomically, calls it. The callback chain leads to TX *only* through `dali_idle_state_enter` (`external_message_started`) which OUTCLRs first. **No PB0 writes from PORTB ISR.**

**Reentrancy / atomicity concerns specifically for PB0:**

- `OUTSET`/`OUTCLR`/`OUTTGL` on AVR0/1 are single-instruction stores to dedicated registers — they are atomic at the instruction level. There is no read-modify-write hazard even with arbitrary ISR preemption.
- The TCA0 ISR (LVL0) can be preempted by TCB0 (LVL1). TCB0's handler does NOT touch PB0, so there is no hazard where TCB0 could clobber a PB0 transition mid-TCA0-tick. But TCB0 can rewrite `dali_timer_cb` via `dali_result_received → dali_idle_state_enter → dali_call_on_rising_once`, which could change *which* callback fires next.
- Critically: if TCB0 preempts TCA0's `send_one` between the OUTSET and the `dali_timer_cb = half_bit_toggle / dali_timer = 1` assignment, and TCB0's chain rewrites `dali_timer_cb` to (e.g.) `idle_period_expired`, then on TCA0 resume the OUTSET is still in effect (PB0 HIGH), `dali_timer = 1` is set, and the next tick fires `idle_period_expired` instead of `half_bit_toggle`. `idle_period_expired` calls `dali_idle_state_enter` which OUTCLRs first thing → PB0 released. **OK in that case.**
- **However**: there's a more subtle scenario. If `dali_timer_cb` is rewritten between OUTSET and "dali_timer = 1", AND the rewrite is done from a context that *also* sets `dali_timer = N`, the original `send_one`'s "dali_timer = 1" assignment wins (executes last) — but the callback pointer may have been overwritten. This is precisely the hazard `dali_state_transmitting.c:send_one` is exposed to during a TCB0-driven `dali_result_received → dali_wait_for_response_state_enter` (which assigns `dali_timer_cb = timeout_waiting_for_response` and `dali_timer = 22`). In that case, after `send_one` resumes, `dali_timer = 1` overwrites `dali_timer = 22`, and the next tick fires `timeout_waiting_for_response`, which OUTCLRs PB0. Net effect: bus released within one half-bit time. **OK in this case too.**
- General invariant: **every callback that could ever be plumbed into `dali_timer_cb` is one of the audited functions in §2 above; every one either (a) does not write PB0, or (b) writes only OUTCLR/OUTTGL toward LOW, or (c) writes OUTSET only after the previous half-bit was OUTCLR'd. No callback ever leaves PB0 stuck HIGH after firing.**

**P1-2 concern.** The bus-stuck-during-transmission timeout (200 TE ≈ 83 ms) only protects the *wait-for-idle* state. While the firmware is *actively* transmitting, the only timeout that ensures PB0 returns to LOW is the implicit "next TCA0 tick fires a callback that releases the line". If TCA0's ISR were somehow disabled while PB0 = HIGH (e.g. if the WDT restart path or some new code somewhere clears `TCA0.SINGLE.INTCTRL`), the line would stay HIGH until the WDT firing. Today there's no such code (verified by grep — `TCA0.SINGLE.INTCTRL` is only ever assigned `TCA_SINGLE_OVF_bm` in `tick_init`). But the property is *implicit*, not enforced by an audit-friendly invariant. See Recommendation P2-1 below.

**Verified-safe today**, but the invariant is structural rather than enforced. Marked P1.

### 3.G. Recently-added code (commissioning)

The commissioning state machine in `dali_sync.c` (lines 350-1047) is entirely a data-plane decision tree on top of `issue_dali_command → dali_transmitting_state_enter`. No direct PORTB writes anywhere in the commissioning code.

The bus-stuck-timeout-during-commissioning path was specifically reviewed for task 3 (commit `5550006`):

```c
static void bus_stuck_timeout(void) {
    DALI_PORT.OUTCLR = DALI_TX_bm;       // line 42 — release
    dali_cancel_edge_detect();
    dali_busy = false;                   // line 44 — clear busy
    current_op.op = OP_NONE;
    if (commission_state.phase != COM_IDLE) {
        commission_state.status = COMM_STATUS_ERR_BUS_STUCK;
        commission_state.phase  = COM_IDLE;
    }
    dali_idle_state_enter();             // also OUTCLRs at its line 70
}
```

After this fires, the work selector (`start_next_dali_work` in main) sees `dali_busy == false`. `pick_work` returns false (commissioning is now COM_IDLE; no other work queued in this scenario). `dali_idle` is true (set by `dali_idle_state_enter`). Sleep gate evaluates true, CPU sleeps until the next external event. **PB0 is LOW (released)** for the entire duration. ✅

**Could a commissioning abort mid-PROGRAM_SHORT_ADDRESS leave TX stuck?** The abort flag is checked at every dispatch boundary (`dali_sync.c:357`). If an abort fires mid-PROGRAM_SHORT_ADDRESS:

- The current TX completes normally (TCA0 ticks finish the bit-stream, OUTCLR via start_stop_bits).
- `finished_transmission` calls `dali_wait_for_response_state_enter` (since CMD_KIND was DALI_KIND_CMD = CMD_SEND, NOT CMD_QUERY... wait, PROGRAM_SHORT_ADDRESS uses `DALI_KIND_CMD`, so `finished_transmission` calls `dali_result_received(NAK, 0)` directly — which routes through `dali_sync_completion` because `dali_busy` is true).
- `dali_sync_completion` advances the phase / on next dispatch sees abort → forces `COM_TERMINATE`, issues TERMINATE, eventually completes and clears `dali_busy`.
- All TCA0 ticks during this window leave PB0 LOW between bits and at end-of-frame.

**No commissioning-specific path leaves PB0 stuck HIGH.** Verified-safe.

### 3.H. Compiler / language-level risks

- `DALI_PORT` is `PORTB`, defined in `<avr/io.h>` as a struct of `volatile uint8_t` registers. Every access expands to a `STS` or `OUT` to a memory-mapped I/O address. `volatile` qualifies the access — the compiler cannot reorder, eliminate, or cache them.
- `OUTSET`/`OUTCLR`/`OUTTGL` are dedicated registers on AVR0/1; writing 1 to a bit position has the documented side-effect of setting/clearing/toggling the corresponding bit of `OUT`. Each is a single store; no read-modify-write race exists at the hardware level. The compiler cannot fuse two `OUTCLR` writes into a `OUT` write or vice-versa because the addresses are different.
- No `if (cond) DALI_PORT.OUTCLR = ...; else <nothing>` pattern exists. All four defensive OUTCLR sites (`dali_init.c:21`, `dali_state_idle.c:70`, `dali_state_wait_for_idle.c:42`, `dali_state_wait_for_response.c:17`) are unconditional first-statement-of-function writes.
- No `tmp = DALI_PORT.OUT; ...; DALI_PORT.OUT = tmp;` non-atomic RMW exists. (Verified: `grep "DALI_PORT.OUT[^SCT]"` returns nothing.)
- `ATOMIC_BLOCK(ATOMIC_RESTORESTATE)` is used in 9 places: `dali_init.c:48,56,65,81`, `dali_sync.c:343,563,594,618,663,669,1063`, and inline in `dali_sync.h:291,297`. Every one is wrapping a 16-bit pointer or RAM-state RMW — none of them touch PORTB. PORTB writes are intentionally outside ATOMIC blocks because they are already atomic at the hardware level. ✅
- No optimisation-level dependency (`-O2` is the build setting per Makefile). The OUTCLR/OUTSET volatile semantics survive any optimisation level.

**Verified-safe.** No compiler-level risks identified.

---

## 4. Bus-stuck failure-mode table

Exhaustive enumeration of "X happens, then Y, then PB0 ends up stuck HIGH" scenarios I considered.

| # | Scenario | Current safeguards | Residual risk | Recommendation |
|---|---|---|---|---|
| F1 | TX in progress (`send_one` ran), TCA0 ISR is blocked / stops firing. | (1) WDT 8.2 s. (2) `TCA0.SINGLE.INTCTRL` is set once at boot and never cleared by reachable code. (3) No code masks `SREG_I` for >100 µs. | If TCA0 IRQ becomes blocked outside of recovery, the bus stays asserted up to 8.2 s before WDT fires. The WDT reset clears PORTB → released. | P1: this is OK on the spec but bus-asserted for ≤ 8.2 s is the worst-case. P2-1 below proposes a 50 ms in-firmware watchdog on PB0=HIGH duration to lower this to ≤ 50 ms. |
| F2 | WDT fires while TX is in progress (`OUT=1, DIR=1`). | ATtiny1604 datasheet: peripheral reset clears DIR and OUT to 0 → PB0 high-Z → bus released by external pull. `dali_init` re-DIR-sets to LOW within ~50 cycles after reset. | I have not verified the AVR0/1 PORTB reset semantics on hardware. If the chip somehow retained `OUT=1` across reset (extremely unlikely; would be an erratum), a stuck condition could persist for the ~50 µs window before `dali_init` runs. After `dali_init`, OUTCLR fires and bus is released. | P1: Document the dependency on the datasheet behaviour; consider adding a hardware test (scope on PB0 with WDT-during-TX deliberately triggered). |
| F3 | BOD-triggered reset during TX. | Same as F2: peripheral reset clears DIR/OUT. Plus BOD holds chip in reset until Vcc is stable. | Same as F2. | Same as F2. |
| F4 | Power loss without BOD trigger (e.g. Vcc dips to 1.8 V — below operating range but above BOD). | Below 2.6 V, BOD asserts reset → safe. Below ~1.4 V the chip is fully unpowered → safe (no drive at all). Between 1.4 V and 2.6 V the BOD has reset asserted, peripherals held in reset → DIR=0 → safe. | None — BOD covers the entire glide path. | None. |
| F5 | Code execution lost / branches into unreachable area. | WDT 8.2 s. Compiler-emitted reset vector traps to 0. | If the PC ever lands in unreachable flash or RAM, behaviour is undefined; could in theory set OUT=1 and stay there. | Recommendation: same as F1 — a TX-asserted watchdog would catch this. |
| F6 | Stack overflow corrupts return address into a function that does OUTSET. | WDT 8.2 s. Stack budget audited (README states ~30-50 B headroom). | If stack overflow corrupts a return address into `send_one` and the dali_timer is left in a state where TCA0 doesn't fire `half_bit_toggle`, PB0 stays HIGH up to WDT firing. | P2: keep an eye on stack budget; the README already flags this. |
| F7 | A new code path is added that writes `DALI_PORT.OUTSET = DALI_TX_bm` outside the bit-level state machine. | Today there is exactly one such site (`send_one`). | Future maintainers could add another and not realise the implicit "next TCA0 tick releases" invariant. | P2: Consider adding a static-analysis check or doc comment in `dali.h` declaring the invariant. |
| F8 | PORTB pin ISR fires re-entrantly during a TX, callback writes to PB0. | `dali_rx_callback` is set only to `external_message_started` (which doesn't write PB0), `on_bus_shorted` (no PB0 write), `on_idle_bus` (no PB0 write), `start_receiving_response` (no PB0 write). | None of the four PORTB-pin-ISR callbacks touch PB0. | None. |
| F9 | Glitch on the TX line itself (external EMI) causes a false re-arm of OUTSET. | Hardware-level — outside firmware scope. | Hardware EMI design problem, not a firmware audit issue. | None (out of scope). |
| F10 | Race: main thread calls `dali_idle_state_enter`'s OUTCLR, ISR preempts and calls `send_one`'s OUTSET, main resumes. | OUTCLR / OUTSET are independent atomic stores — neither can corrupt the other. The OUTSET wins. The next TCA0 tick releases. | This is the *expected* behaviour during TX, not a stuck condition. | None. |
| F11 | Bus-stuck-timeout in wait-for-idle fires while own TX is in progress (i.e., the 200-TE timeout overlaps with our own send). | The 200-TE timer is *only* armed in `wait_for_idle` state, which is *not* the TX state. They are mutually exclusive. | None — the state machine cannot be in both states simultaneously. | None. |
| F12 | `dali_busy` stays `true` after TX, sleep gate refuses to sleep, but `dali_idle_state_enter` keeps re-OUTCLR'ing. | This isn't a stuck-bus failure; it's a stuck-busy failure. Bus is LOW (released). The 200-TE bus-stuck timeout would trip in wait-for-idle and clear `dali_busy`. | Bus stays released → safe. | None. |

---

## 5. Recommendations

### Must-fix (real bugs)

**None.**

### Should-add (defensive layers)

**P1-A. Document the dependency on AVR0/1 PORTB-reset semantics for WDT-during-TX recovery.** A short comment in `dali_init.c` near line 21 stating "PB0 returns to high-Z (DIR=0, OUT=0) on every reset source per ATtiny1604 datasheet § 16.5.4; this is the WDT-during-TX recovery mechanism. The OUTCLR-then-DIRSET sequence below transitions PB0 high-Z → driven LOW without ever passing through driven-HIGH." A complementary hardware test (scope on PB0 + induce WDT mid-TX) would convert this from inferred to verified.

### Should-add (additional defensive code)

**P2-1. TX-asserted watchdog (recommended).** Currently the only firmware-level guarantee that PB0 is released within bounded time after being driven HIGH is "the next TCA0 tick fires a callback that includes OUTCLR/OUTTGL". The cadence is 1 tick = 417 µs, so the implicit bound is ~417 µs per call to OUTSET. If TCA0 is somehow blocked, the WDT (~8.2 s) is the only backstop.

A simple defensive mechanism: a "PB0 high duration" field in RAM, incremented on every TCA0 OVF when PB0 is observed HIGH, reset on every callback that writes OUTCLR or OUTTGL. If it ever exceeds (say) 50 ms = 120 ticks, a defensive `OUTCLR DALI_TX_bm` fires. This would close F1, F5, F6, F7. Cost estimate: 1 byte RAM, ~30 bytes flash.

A simpler form: in `TCA0_OVF_vect`, after `dali_tick`, check `(DALI_PORT.IN & DALI_TX_bm) && dali_timer == 0`. If both are true, the bit-level state machine has stalled with PB0 HIGH and no pending callback — fire `OUTCLR`. (The IN-register read returns the actual pin state regardless of OUT/DIR.) Cost: ~10 bytes flash.

Either form is a true defence in depth. Today the only instances of OUTSET (`send_one`) all set `dali_timer = 1` immediately after, so the proposed check would never fire under normal operation; it triggers only when something is genuinely wrong.

**P2-2. Update `dali_init.c:20` comment.** The current comment reads:

```c
// TX: output, initially low (open-drain, OUTCLR stays low - only DIR is toggled)
```

This is incorrect: the firmware does **not** keep DIR toggled — it keeps `DIRSET` permanently and toggles `OUT` (`OUTSET`/`OUTCLR`/`OUTTGL`) instead. Replace with: `// TX: configured as output, driven LOW (released bus). OUTSET asserts; OUTCLR releases.`

**P2-3. Add a polarity comment to `dali.h` near line 91.**

```c
// DALI_TX (PB0): driven HIGH = bus asserted; driven LOW (or high-Z) = bus released.
// The external transistor inverts polarity from MCU pin to bus line. OUTCLR is
// always the safe / released state; OUTSET is always asserted.
```

This codifies the invariant for future maintainers and would satisfy F7.

### Documentation

**D1. Update README pin-table line for PB0 to disambiguate.** Current text:

> PB0 | DALI TX — open-drain output, driven low to pull bus down

This describes the bus side, not the MCU side. The risk is that someone reading just the README assumes "OUTCLR = bus driven low". Suggested:

> PB0 | DALI TX — drives external transistor; OUTSET asserts the bus (drives bus low); OUTCLR / high-Z releases. Reset state (high-Z) = released.

---

## 6. Cross-checks performed

- [x] `grep -rn "DIRSET\|DIRCLR\|DIRTGL\|OUTSET\|OUTCLR\|OUTTGL\|PORTB" src/` — every PB0 site in the project.
- [x] Read `src/dali_init.c` line by line for boot sequence.
- [x] Read `src/main.c` line by line for startup ordering.
- [x] Read `src/dali_state_transmitting.c`, `dali_state_idle.c`, `dali_state_wait_for_idle.c`, `dali_state_wait_for_response.c`, `dali_state_receiving.c` line by line for state-transition coverage.
- [x] Read `src/dali_sync.c` for commissioning / work-selector / completion paths.
- [x] Read `src/tick.c` for TCA0/TCB0 ISR setup, callback dispatch, and atomicity.
- [x] Read `src/dali.h`, `src/dali_sync.h`, `src/types.h` for type/macro semantics.
- [x] Read `src/modbus.c` selectively (init + tick) to confirm zero PORTB writes.
- [x] Read `src/registers.c` selectively to confirm zero PORTB writes.
- [x] Verified `dali_timer_cb` set sites — every callback ever plumbed in is one of: `send_one`, `send_zero`, `half_bit_toggle`, `start_stop_bits`, `finished_transmission`, `idle_period_expired`, `bus_stuck_timeout`, `timeout_waiting_for_response`, `timer_overflow`, `start_receiving_response`. All audited.
- [x] Verified `dali_rx_callback` set sites — every callback is `external_message_started`, `on_bus_shorted`, `on_idle_bus`, `start_receiving_response`. None write PB0.
- [x] Walked the READ-WRITE thread-safety story for `dali_busy`, `dali_idle`, `current_op`, `commission_state` and confirmed every place `dali_idle == true` is set, PB0 was OUTCLR'd within the same function body.
- [x] Walked the WDT path: only `wdt_reset` calls in main:56 and main:72; no ISR loop without main return; WDT period 8.2 s.
- [x] Walked the BOD path: BODCFG = 0x44 (`ACTIVE_ENABLED_gc | LVL_BODLEVEL2_gc`), `.fuse` segment included in HEX (commit 9de0ecc Makefile change).
- [x] Confirmed sleep mode is `SLEEP_MODE_STANDBY`, set once at `main:48`, no other sleep_mode changes.
- [x] Confirmed `CPUINT.LVL1VEC = TCB0_INT_vect_num` is the only LVL1 setup; PORTB ISR runs at default LVL0.
- [x] Confirmed all `ATOMIC_BLOCK` usages — none wrap a PORTB write (they wrap RAM RMW).
- [x] Read git history of `dali_state_idle.c`, `dali_init.c`, `dali_state_transmitting.c`, `dali_state_wait_for_response.c`, `dali_state_wait_for_idle.c` for the past 10+ commits to understand polarity intent (commit `9de0ecc` was decisive).
- [x] Read tasks/done/1, 2, 3 for stated safety properties and prior reasoning on bus-stuck.
- [x] Confirmed README "Known Limitations" entries that bear on PB0 (DE-pin float — different pin; USERROW write halts CPU — duration ≤ 3.4 ms ≪ WDT period).

---

*End of audit.*
