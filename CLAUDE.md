# TinyModbus Project Notes

## Toolchain

Always use the project-local AVR toolchain in `avr/bin/` (e.g. `avr/bin/avr-gcc`, `avr/bin/avr-nm`, `avr/bin/avr-size`). Never use system-installed AVR tools from `/opt/homebrew` or anywhere else - they are out of date and produce incorrect results for this project.

The Makefile already sets `PATH := $(shell pwd)/$(AVR_GCC_DIR)/bin:$(PATH)` so `make build` uses the correct tools automatically. For manual invocations, prefix with `avr/bin/` or set PATH accordingly.

## RAM budget

ATtiny1604 has 1 KB SRAM. Static data (BSS + data + COMMON) must stay well under
that to leave room for the stack (~150 bytes minimum). Currently **153 B**
(task 08), leaving 871 B — the gateway caches nothing about the bus, which is
what keeps it small. Key consumers:

- `buf` (modbus.c): 40 bytes (`MAX_PKT_SZ = 16*2+8`), which limits a single
  FC03/FC04 read to 16 registers (`MAX_REG_RD`).
- The queue core (cq.c): 37 bytes — command queue 12 (`CMDQ_DEPTH` × 6),
  response queue 8 (`RESPQ_DEPTH` × 2), diagnostic counters 12, cursors 5.
- RX diagnostic counters + the last failed frame (modbus.c), and the sysinfo
  values (sysinfo.c).

Deepest measured stack frame is `modbus_process` at 48 B (`-fstack-usage`), and
the worst nested-ISR chain adds ~35 B, so real worst-case use is ~110 B against
the 871 B available.

Packed enums (`__attribute__((packed))`) on `dali_kind_t`, `dali_step_t`,
`tm_err_t` and `tm_fault_site_t` force 1-byte storage instead of the AVR's
default 2-byte int-sized enum. The Modbus version string (FC04 `0x0200`) is
built on demand directly into `buf` rather than kept in a static buffer.

## Diagnostics

### FC04 blocks are EXACT-LENGTH reads

Asking for any other count returns ILLEGAL DATA ADDRESS rather than a short
read, so a client built against a different map fails loudly. `registers.h` is
authoritative; this table exists because the controller's `mbq` stats parser and
its own CLAUDE.md mirror it, and all of them must move together.

| block | address | count | contents |
|---|---|---|---|
| version | `0x0200` | 16 | `"DALI v<maj>.<min> <20-hex-serial>"` |
| RX diagnostics | `0x0300` | 16 | overflow, frame err, CRC err, frames ok, last failed frame, internal faults |
| sysinfo | `0x0310` | 4 | uptime (32-bit), boot count, reset reason |
| queue counters | `0x0320` | 8 | accepts, a_full, executes, results, acks, stalls, q_discard, spare |

Queue counters (task 08, `QUEUE-PROTOCOL.md` §6) are 16-bit wrapping. Two of
them read in **opposite directions** and it matters:

- **`a_full` non-zero is a canary**, not traffic. A_FULL is unreachable while the
  master honours its in-flight cap, so any non-zero value means the master's
  accounting is wrong. Investigate it.
- **`stalls` non-zero is normal.** It counts backpressure *episodes* (rising
  edge, not polls — the arbiter is polled thousands of times a second) and is
  simply the rule working whenever the master stops acking.

### Clang false positives

Clang (macOS) will report false-positive errors for AVR-specific attributes (`__progmem__`, `__AVR_TINY_PM_BASE_ADDRESS__`, ISR attributes, EEPROM macros). These are harmless - the code targets avr-gcc, not clang.


## Power supply — 12 V → 5 V synchronous buck

The logic rail is **5 V** from a synchronous buck. A buck is ~90 % efficient
regardless of output, which is why an LDO is not an option here: dropping 12 V
to logic level linearly dissipates enough to reach thermal shutdown under a
sustained DALI scan, and the collapse fires the ATtiny's BOD (presents as
`RSTFR BORF`). A bigger LDO makes it worse and a bulk cap does not help a
thermal trip.

5 V also lets the ATtiny run **20 MHz** (needs V_DD ≥ 4.5 V) and gives the
THVD1450 stronger RS-485 drive.

**Regulator: MP2331H** (`MP2331HGTL-Z`, **LCSC C6992878**, SOT583-8) — 24 V/2 A, 1.2 MHz,
**COT** control, V_FB 0.8 V, internal FETs (no catch diode). Chosen over the MP1584EN:
synchronous (no external Schottky), fixed-frequency (no RFREQ resistor), fewer BOM lines,
cheaper ($1.80 vs $3.26), smaller; 24 V/2 A is ample for the ~0.1 A load. (The first pick,
MP2359, is NRND.)

**Power-supply BOM (12 V → 5 V).** `V_OUT = 0.8·(1 + R1/R2) = 4.99 V`.

All LCSC codes below are verified in-stock via the `jlcpcb` MCP and match the
KiCad PCB BOM (`pcb/MikroeAdapter/.../BOM-MikroeAdapter.csv`). PCB refs in ().

| Ref (PCB) | Value | Rating | Footprint | LCSC |
|---|---|---|---|---|
| U1 | MP2331HGTL-Z buck | 24 V / 2 A | SOT-583-8 | **C6992878** ⚠ |
| D2 | SS34 Schottky (reverse-polarity) | 40 V / 3 A | SMA | **C8678** |
| D1 | SMBJ13A input TVS (after D2) | 13 V standoff / ~21.5 V clamp | SMB | **C19077567** |
| L1 | FNR4030S4R7MT, 4.7 µH ±20% | 3.2 A I_sat, 2.3 A I_rms, shielded | 4×4×3 mm | **C167874** |
| C1,C2,C10 | 22 µF 25 V X7R ×3 (1 input + 2 output) | 25 V (see note) | 1210 | **C2918511** |
| C6,C8,C9 | 0.1 µF 50 V X7R (HF bypass ×3) | 50 V | 0402 | **C1525** |
| C3 (Cbst) | 1 µF 50 V X5R | 50 V | 0603 | **C15849** |
| R4 (Rbst) | 20 Ω 1% (series with Cbst) | — | 0402 | **C5140751** |
| R1 | 40.2 kΩ 1% (FB top) | — | 0402 | **C25893** |
| R2 | 7.68 kΩ 1% (FB bottom) | — | 0402 | **C25919** |
| C5 (Cff) | 10 pF **C0G/NP0** (across R1) | 50 V | 0402 | **C32949** |

⚠ **U1 stock is low (~29 at JLCPCB)** — order early / keep MP2315 (24 V/3 A sync) as a
fallback. **Cin is 25 V, not 50 V:** 22 µF/1210 isn't stocked above 25 V, so at 12 V bias
it derates to ~12 µF effective — still ample for this low-ripple input. The PCB merges
Cin+Cout onto one part (3× C2918511). **R4 fix:** an earlier BOM used C2004 (a 4× 0402
*array*) for the 20 Ω — corrected to the single-resistor C5140751.
**COT note:** the 10 pF C0G Cff is the loop's ripple injection (ceramic Cout has ~no
ESR) — keep it, and re-tune if L1 changes; don't add output capacitance to "fix" jitter.
**L1 = 4.7 µH** was chosen over the datasheet's 3.3 µH for ~30 % lower ripple current
(peak ~0.76 A vs I_sat 3.2 A); since it deviates from the validated value, **scope the
SW/V_OUT for COT jitter** when the board is first brought up. KiCad symbol/footprint for
L1 and U1 come from `easyeda2kicad --full --lcsc_id=<C167874|C6992878>` (matches the
JLCPCB land pattern; the stock `Changjiang_FTX2016…` footprints are a different, smaller
series and do NOT fit the 4×4 FNR4030).

**Input protection.** Series **SS34** off the connector for reverse polarity (~0.4 V
drop, negligible at ~0.2 A buck input; the buck's wide V_IN swallows it). Optional
**SMBJ13A** TVS *after* the diode for positive transients (clamps ~21 V < the 24 V max).

**12 V supply sizing.** Rail feeds 2× gateway logic (~0.05 A each via the buck) + the
Waveshare 32CH relay board (0.3 W idle / **11.7 W all-on ≈ 0.98 A**). Worst case ~1.1 A.
A **1.25 A** supply is adequate for this install: boot is all-off and relays are switched
**one at a time**, so there's no inrush/step load, and **DALI is separately powered** (not
on this rail). Use 2 A only if all 32 relays will be held on continuously.

**Firmware follow-ups — MUST land together with the 5 V / 20 MHz board:**
1. `-DF_CPU=20000000` in the Makefile.
2. Drop the ÷2 clock prescaler in `main.c` (`CLKCTRL.MCLKCTRLB = 0`; the osc is already
   20 MHz base, currently divided to 10 MHz by `PDIV_2X | PEN`).
3. **Raise the BOD fuse to ~4.2 V (BODLEVEL7)** — it's at 2.6 V now, which is *unsafe* at
   20 MHz (V_DD must be ≥ 4.5 V). Without this the MCU runs out-of-spec on a sagging rail.
4. **Re-verify the DALI bit timing** (TCB0 capture/compare counts) on the scope after the
   F_CPU change. USART baud and `_delay_ms` scale with F_CPU automatically; the DALI
   timing is precise and must be checked.
5. DALI Click 2 → 5 V jumper; confirm no 3.3 V-only nets remain.

**KiCad parts.** `easyeda2kicad --full --lcsc_id=C6992878` pulls the symbol + footprint +
3D model matching the exact land pattern JLCPCB assembles.

## Tasks

- the user will format tasks for claude to execute in the `tasks` directrory
- When executing tasks, create an agent team to process them more effectively
- Tear down team member agents and explicitly kill them when they are no longer needed
- For every coder team member, also fire up a reviewer, who works with the coder to review changes and make suggestions.  The reviewer's job is to make sure that what they write works, is in line with the task, and doesn't drift too far from what the main intent of the system is
- Tasks are performed on a git branch.  your team may check work into this branch at sensible checkpoints. For examle, if the work is broken down into sub-tasks, check in as each subtask is completed, with description.
- When tasks are completed and signed off by the user, move the task to a `done` subdirectory of the tasks directy.  Mark the work as done, and make sure the document is updated with what was changed during the change.  Check any changes into git, and make sure the comment includes informaiton on what was done.  The branch should be ready for a pull request at this time.
- We store our code on a forgejo repostiory.  Use the forgejo mcp tool to create pull requests


## Developer environment
- the user's machine has Tmux installed, and they will be running claude from tmux.  Make appropriate use of tmux panes where necessary
 
