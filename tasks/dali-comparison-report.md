# TinyModbus vs the wider DALI ecosystem — research report

This report compares the TinyModbus firmware (an ATtiny1604 Modbus-RTU-to-DALI gateway,
exposing 64 short-addressed control gear over a Modbus holding/input register map) against:

1. The published behaviour of IEC 62386-101/-102 (DALI v1 + DALI-2 ed.2) and selected
   device-type extensions (DT1 emergency, DT8 colour, D4i parts 250/251/252/253).
2. Open-source DALI master/library implementations on AVR, ESP32, Cortex-M and
   higher-level Python.
3. Commercial Modbus-DALI gateways (Lunatone DALI 4Net / DALI-2 IoT4, Domat R091,
   Intesis MBS-DAL, Helvar 436).
4. Vendor application notes covering bit-level timing (Microchip AN1465A/AN1487,
   Silicon Labs AN1220, Texas Instruments MSPM0 DALI guide).

The report is research-only. It deliberately stops at *inventory* of gaps and weaknesses;
recommendations on which gaps to close are the maintainer's call.

---

## Source inventory and citation map

| Topic | Reference |
|---|---|
| IEC 62386 part list | https://www.dali-alliance.org/standards/IEC62386.html |
| DALI-2 (ed.2) deltas | DiiA Technical Note 1: https://www.dali-alliance.org/data/downloadables/5/4/1711_technical-note-dali-2-the-new-standard.pdf |
| Wikipedia DALI overview | https://en.wikipedia.org/wiki/Digital_Addressable_Lighting_Interface |
| DALI command opcodes | python-dali, https://github.com/sde1000/python-dali/blob/master/dali/gear/general.py |
| DALI emergency (DT1) opcodes | python-dali emergency module, https://github.com/sde1000/python-dali/blob/master/dali/gear/emergency.py |
| DALI colour (DT8) opcodes | python-dali colour module, https://github.com/sde1000/python-dali/blob/master/dali/gear/colour.py |
| Memory bank 0 layout | Beckhoff InfoSys, https://infosys.beckhoff.com/content/1033/tcplclib_tc3_dali/6940982539.html ; python-dali memory module |
| D4i / Parts 250-253 | Inventronics primer, https://www.inventronics-co.com/wp-content/uploads/2020/07/DALI-DALI-2-and-D4i-Technical-Primer.pdf |
| Bit-level timing (vendor app notes) | Silicon Labs AN1220 https://www.silabs.com/documents/public/application-notes/an1220-efr32-dali.pdf ; Microchip AN1487 https://ww1.microchip.com/downloads/en/Appnotes/01487A.pdf ; TI MSPM0 https://software-dl.ti.com/msp430/esd/MSPM0-SDK/latest/docs/english/middleware/dali/doc_guide/doc_guide-srcs/MSPM0_DALI_Users_Guide.html |
| Multi-master collision resolution | Beckhoff InfoSys, https://infosys.beckhoff.com/content/1033/tcplclib_tc3_dali/12346807435.html |
| Arduino/AVR DALI master libs | hubsif/arduino-dali https://github.com/hubsif/arduino-dali ; paultbarrett/bitsDALI https://github.com/paultbarrett/bitsDALI ; NabiyevTR/DALI https://github.com/NabiyevTR/DALI ; edinburghhacklab/dali https://github.com/edinburghhacklab/dali ; davideloba/daliMaster https://github.com/davideloba/daliMaster |
| ESP32 DALI master | Spectoda DALI for ESP32 https://github.com/Spectoda/DALI-Lighting-Interface-ESP32 ; ESPEnlightDALI https://github.com/karnigen/ESPEnlightDALI ; Tasmota DALI driver https://tasmota.github.io/docs/DALI/ |
| Zephyr DALI | PR #88128, https://github.com/zephyrproject-rtos/zephyr/pull/88128 |
| Lunatone Modbus gateway (4Net) | Datasheet https://www.lunatone.com/wp-content/uploads/2018/03/22176666_DALI_4Net_EN_D0053.pdf ; Modbus examples PDF https://www.lunatone.com/wp-content/uploads/2021/01/DALI-4Net-Zugriff-ueber-Modbus-Beispielsammlung.pdf |
| Lunatone DALI-2 IoT4 | Modbus manual https://www.lunatone.com/wp-content/uploads/2025/03/22176625_DALI-2-IoT4_Modbus_Manual_EN_M0028.pdf |
| Domat R091 Modbus DALI | https://domat.blob.core.windows.net/cms/ContentItems/1482_r091/TyvNIw/domat_R091_en.pdf |
| Intesis (HMS) MBS-DAL gateway | https://www.hms-networks.com/p/inmbsdal0640500-dali-2-to-modbus-tcp-server-gateway |
| Helvar 436 BACnet/Modbus gateway | https://helvar.com/product/436-gateway/ |
| Loytec L-DALI | https://www.loytec.com/products/dali |

Where a PDF would not decode through the WebFetch path I extracted text locally with
pypdf 6.11.0 against the cached binary (transcripts in
`~/.claude/projects/.../tool-results/`). All numeric opcodes quoted below have been
cross-checked between the python-dali source and at least one second source (Wikipedia,
Beckhoff, or vendor app note).

---

## Section A — IEC 62386-102 spec coverage gaps

### A.1 The complete reference command set (control gear, 16-bit forward frame)

The 16-bit forward frame is `YAAA AAAS DDDD DDDD`:

- `Y = 0` → short address, `Y = 1` → group address (`1AAA AAAS`) or broadcast
  (`1111 111S` = 0xFE/0xFF), as documented at
  https://en.wikipedia.org/wiki/Digital_Addressable_Lighting_Interface
- `S = 0` → DDDDDDDD is a Direct Arc Power Control level 0–255
- `S = 1` → DDDDDDDD is a standard command opcode 0x00–0xFF
- Special commands replace the address byte entirely with one of the slots
  0xA1, 0xA3, 0xA5, … 0xC9 (LSB always 1 ⇒ command, all upper bits literal).

The full per-opcode list below is taken directly from
[python-dali `gear/general.py`](https://github.com/sde1000/python-dali/blob/master/dali/gear/general.py)
and matches the IEC 62386-102 ed.1/ed.2 numbering. **TinyModbus status** is annotated:
`✓ direct` = wired through a per-ballast register, `✓ raw` = reachable through the
0x1000 / 0x1001 raw-command channel only, `✗` = blocked by current implementation
(typically because the firmware has no DTR1/DTR2 path or no commissioning state).

#### A.1.1 Indirect-arc-power / step / scene commands (data byte, S=1)

| Opcode | Command | TinyModbus | Notes |
|---|---|---|---|
| 0x00 | OFF | ✓ raw | Reachable via 0x1000. Per-ballast bank does not bind it directly. |
| 0x01 | UP | ✓ raw | 200 ms fade up, no response. |
| 0x02 | DOWN | ✓ raw | |
| 0x03 | STEP UP | ✓ raw | |
| 0x04 | STEP DOWN | ✓ raw | |
| 0x05 | RECALL MAX LEVEL | ✓ raw | |
| 0x06 | RECALL MIN LEVEL | ✓ raw | |
| 0x07 | STEP DOWN AND OFF | ✓ raw | |
| 0x08 | ON AND STEP UP | ✓ raw | |
| 0x09 | ENABLE DAPC SEQUENCE | ✓ raw | DALI-2 addition. |
| 0x0A | GO TO LAST ACTIVE LEVEL | ✓ raw | DALI-2 (Note 2.2 in DiiA Tech Note 1). |
| 0x0B | CONTINUOUS UP | ✓ raw | DALI-2 ed.2 added. |
| 0x0C | CONTINUOUS DOWN | ✓ raw | DALI-2 ed.2 added. |
| 0x10–0x1F | GO TO SCENE 0..15 | ✓ raw | |

#### A.1.2 Configuration commands (require SEND-TWICE, 0x20–0x4F)

| Opcode | Command | TinyModbus | Notes |
|---|---|---|---|
| 0x20 | RESET | ✓ raw | |
| 0x21 | STORE ACTUAL LEVEL IN DTR0 | ✓ raw | Spec command 21. Useful for "save current to power-on". |
| 0x22 | SAVE PERSISTENT VARIABLES | ✗ | DALI-2 ed.2 added. Forces NV write. Not exposed; reachable via raw send. |
| 0x23 | SET OPERATING MODE | ✗ | DALI-2 ed.2. Toggles to manufacturer-specific mode. Reachable via raw. |
| 0x24 | RESET MEMORY BANK | ✗ | DALI-2 ed.2. Reachable via raw. |
| 0x25 | IDENTIFY DEVICE | ✗ | DALI-2 ed.2 — visible blink. Reachable via raw. |
| 0x2A | SET MAX LEVEL | ✓ direct | FC03/FC06 0x0040–0x007F (high byte). |
| 0x2B | SET MIN LEVEL | ✓ direct | FC03/FC06 0x0040–0x007F (low byte). |
| 0x2C | SET SYSTEM FAILURE LEVEL | ✓ direct | FC03/FC06 0x0080–0x00BF (high byte). |
| 0x2D | SET POWER ON LEVEL | ✓ direct | FC03/FC06 0x0080–0x00BF (low byte). |
| 0x2E | SET FADE TIME | ✓ direct | FC03/FC06 0x00C0–0x00FF. |
| 0x2F | SET FADE RATE | ✓ direct | FC03/FC06 0x00C0–0x00FF. |
| 0x30 | SET EXTENDED FADE TIME | ✗ | DALI-2 ed.2 *headline feature*. Allows 100 ms…16 min fades. Not exposed. |
| 0x40+s | SET SCENE 0..15 | ✗ direct, ✓ raw | No per-ballast scene bank — must be done via the raw-command channel and a separately staged DTR0. |
| 0x50+s | REMOVE FROM SCENE 0..15 | ✗ direct, ✓ raw | Same. |
| 0x60+g | ADD TO GROUP 0..15 | ✓ direct | Driven by the FC03 0x0110–0x014F group-mask path. |
| 0x70+g | REMOVE FROM GROUP 0..15 | ✓ direct | Same. |
| 0x80 | SET SHORT ADDRESS | ✗ | Used during commissioning. TinyModbus has no commissioning chain, so this is reachable only through raw + DTR0 + INITIALISE machinery. |
| 0x81 | ENABLE WRITE MEMORY | ✗ | Required gate before WRITE MEMORY LOCATION. |

#### A.1.3 Query commands (0x90–0xFF) — backward frame expected

| Opcode | Query | TinyModbus | Notes |
|---|---|---|---|
| 0x90 | QUERY STATUS | ✓ direct | Cached as the high byte of FC03 0x0000–0x003F. |
| 0x91 | QUERY CONTROL GEAR PRESENT | ✗ | TinyModbus derives presence from the *level read* round-robin, not this query. |
| 0x92 | QUERY LAMP FAILURE | ✗ | Bit 1 of QUERY STATUS is cached, but the standalone query is not exposed. |
| 0x93 | QUERY LAMP POWER ON | ✗ direct | Bit 2 of QUERY STATUS suffices for most installations. |
| 0x94 | QUERY LIMIT ERROR | ✗ direct | Bit 3 of QUERY STATUS. |
| 0x95 | QUERY RESET STATE | ✗ direct | Bit 5 of QUERY STATUS. |
| 0x96 | QUERY MISSING SHORT ADDRESS | ✗ direct | Bit 6. Not used because we don't commission. |
| 0x97 | QUERY VERSION NUMBER | ✗ | Useful — reports the IEC 62386-102 version of the device. Reachable via raw. |
| 0x98 | QUERY CONTENT DTR0 | ✗ | Reachable via raw. Useful for verifying DTR0 handshake. |
| 0x99 | QUERY DEVICE TYPE | ✗ | Returns the DT (0=fluo, 1=emergency, 6=LED, 7=switching, 8=colour, 0xFF=multi). **Required** to know which device-type extensions a ballast supports. |
| 0x9A | QUERY PHYSICAL MINIMUM | ✗ | The lowest level the gear can produce. Lets a master expose a real lower-bound to the user. |
| 0x9B | QUERY POWER FAILURE | ✗ | Bit 7 of status. |
| 0x9C | QUERY CONTENT DTR1 | ✗ | We don't use DTR1 internally. |
| 0x9D | QUERY CONTENT DTR2 | ✗ | We don't use DTR2 internally. |
| 0x9E | QUERY OPERATING MODE | ✗ | DALI-2 ed.2. |
| 0x9F | QUERY LIGHT SOURCE TYPE | ✗ | DALI-2 ed.2 (LED, halogen, fluo, emergency, etc.). |
| 0xA0 | QUERY ACTUAL LEVEL | ✓ direct | Low byte of FC03 0x0000–0x003F. |
| 0xA1 | QUERY MAX LEVEL | ✓ direct | High byte of FC03 0x0040–0x007F. |
| 0xA2 | QUERY MIN LEVEL | ✓ direct | Low byte of FC03 0x0040–0x007F. |
| 0xA3 | QUERY POWER ON LEVEL | ✓ direct | High byte of FC03 0x0080–0x00BF. |
| 0xA4 | QUERY SYSTEM FAILURE LEVEL | ✓ direct | Low byte of FC03 0x0080–0x00BF. |
| 0xA5 | QUERY FADE TIME / FADE RATE | ✓ direct | FC03 0x00C0–0x00FF. |
| 0xA6 | QUERY MANUFACTURER SPECIFIC MODE | ✗ | Reserved per IEC 62386-102. The TinyModbus header (`dali.h`) explicitly warns "DALI opcode 0xA6 is reserved by IEC 62386-102 - never issue it." That comment is correct in not issuing it, but the spec opcode at 0xA6 is QUERY MANUFACTURER SPECIFIC MODE — it is *defined*, not *reserved* — so the comment in `dali.h` is technically slightly off. (See python-dali `general.py` lines 857–865 and DiiA Tech Note 2.1.) The decision not to issue it from internal scans is still correct. |
| 0xA7 | QUERY NEXT DEVICE TYPE | ✗ | DALI-2 ed.2. Used to walk multiple device-type extensions on a multi-DT gear. |
| 0xA8 | QUERY EXTENDED FADE TIME | ✗ | DALI-2 ed.2. |
| 0xAA | QUERY CONTROL GEAR FAILURE | ✗ | DALI-2 ed.2. |
| 0xB0+s | QUERY SCENE LEVEL 0..15 | ✗ | We don't expose scenes at all. |
| 0xC0 | QUERY GROUPS 0–7 | ✓ direct | Cached via the group-mask path. |
| 0xC1 | QUERY GROUPS 8–15 | ✓ direct | Same. |
| 0xC2 | QUERY RANDOM ADDRESS H | ✗ | Commissioning. |
| 0xC3 | QUERY RANDOM ADDRESS M | ✗ | Commissioning. |
| 0xC4 | QUERY RANDOM ADDRESS L | ✗ | Commissioning. |
| 0xC5 | READ MEMORY LOCATION | ✗ | **Major gap.** With DTR0=offset, DTR1=bank, this is the only way to read GTIN, firmware version, serial number, hardware version, etc. Reachable via raw, but the master has to drive the 3-step DTR1+DTR0+QUERY sequence itself. |
| 0xFF | QUERY EXTENDED VERSION NUMBER | ✗ | DALI-2 ed.2. The version number of the currently-enabled device-type extension. |

#### A.1.4 Special commands (replace address byte; opcodes 0xA1–0xC9 in the *address* byte)

| Address byte | Special command | TinyModbus | Notes |
|---|---|---|---|
| 0xA1 | TERMINATE | ✗ | Cancels INITIALISE early. |
| 0xA3 | DTR0 (load) | ✓ direct (internal) + ✓ raw | Used internally by SYNC_WRITE_MINMAX/POWER/FADE. README documents `0xA3VV` writes through 0x1000 to load DTR0. |
| 0xA5 | INITIALISE | ✗ | First step of commissioning. Sent twice within 100 ms. Starts a 15-min window in which 0xA7..0xC9 work. |
| 0xA7 | RANDOMISE | ✗ | Triggers fresh 24-bit random address. Sent twice within 100 ms. |
| 0xA9 | COMPARE | ✗ | Backward frame YES (0xFF) if any device's random < SEARCHADDR. |
| 0xAB | WITHDRAW | ✗ | Pull a device out of the comparison set. |
| 0xAD | PING | ✗ | DALI-2 ed.2. Per DiiA Technical Note 1, "PING command defined, but to be ignored by control gear (allows detection of single masters)" — used by control devices to detect each other on multi-master buses. |
| 0xB1 | SEARCHADDRH | ✗ | Set high byte of search address. |
| 0xB3 | SEARCHADDRM | ✗ | Mid byte. |
| 0xB5 | SEARCHADDRL | ✗ | Low byte. |
| 0xB7 | PROGRAM SHORT ADDRESS | ✗ | Assign 0AAAAAA1 to the lowest-RANDOM device that has not WITHDRAWn. |
| 0xB9 | VERIFY SHORT ADDRESS | ✗ | YES backward frame if data byte == device's short address. |
| 0xBB | QUERY SHORT ADDRESS | ✗ | Returns the short address of the lowest-RANDOM remaining device. |
| 0xC1 | ENABLE DEVICE TYPE | ✗ | Required gate before any DT-specific 0xE0–0xFF command. |
| 0xC3 | DTR1 (load) | ✓ raw | Reachable as a raw `0xC3VV` send (README documents this). |
| 0xC5 | DTR2 (load) | ✓ raw | Reachable as `0xC5VV`. |
| 0xC7 | WRITE MEMORY LOCATION | ✗ | Persisted manufacturer/luminaire info path. |
| 0xC9 | WRITE MEMORY LOCATION NO REPLY | ✗ | DALI-2 ed.2. |

#### A.1.5 Categorisation of the gap

Counting from the tables above:

- **DAPC + step/recall + scene-recall (0x00–0x1F):** 100 % reachable (15 of 15).
- **Configuration commands (0x20–0x80):** 7 of 13 reachable directly through the
  per-ballast bank, the rest reachable through the raw channel. The DALI-2-only
  additions (0x22, 0x23, 0x24, 0x25, 0x30) are all unreachable except via the raw
  channel.
- **Query commands (0x90–0xFF):** 9 of ~30 are cached. The rest are *only* reachable
  via raw — and `0xC5` READ MEMORY LOCATION specifically requires a multi-step
  sequence (DTR1+DTR0+QUERY) that the master has to drive itself, because the
  firmware has no native DTR1 plumbing.
- **Special commands (0xA1–0xC9):** 3 of 14 reachable (DTR0/1/2 loads). Every other
  special command — the full commissioning chain — is unreachable except as a
  master-driven raw sequence. Even then, the mandatory "twice within 100 ms" rule
  for INITIALISE / RANDOMISE is *not* enforced by the firmware (the
  RAW_CMD_TWICE channel uses the SET-style 22 Te interval, not the 100 ms
  re-send window — see Section C below).

### A.2 Memory bank coverage

Per python-dali `dali/memory/info.py` and Beckhoff InfoSys, IEC 62386-102 mandates
memory bank 0:

| Offset | Field |
|---|---|
| 0x00 | Last accessible memory area offset |
| 0x02 | Last accessible memory bank number |
| 0x03–0x08 | GTIN (6 bytes) |
| 0x09–0x0A | Firmware version (major, minor) |
| 0x0B–0x12 | Identification number (8 bytes — globally unique with GTIN) |
| 0x13–0x14 | Hardware version (major, minor) |
| 0x15 | IEC 62386-101 version |
| 0x16 | IEC 62386-102 version |
| 0x17 | IEC 62386-103 version |
| 0x18 | Logical control device unit count |
| 0x19 | Logical control gear unit count |
| 0x1A | Index number of this logical unit |

Memory bank 1 is reserved for luminaire data (mandated by IEC 62386-102 ed.2 and
extended by Part 251 for D4i). D4i further specifies banks **202** (energy: active
power and active energy), **203** (diagnostics) and **204** (further diagnostics) —
see the Inventronics primer.

**TinyModbus status:** none of memory bank 0/1/202/203/204 is read. The only thing
in this address class that the master can do is *raw-issue* the four-step sequence
manually:
- `0x1000 ← 0xC3 nn` (DTR1 = bank number)
- `0x1000 ← 0xA3 nn` (DTR0 = offset)
- `0x1000 ← AAAA C500` (READ MEMORY LOCATION) — answer in FC04 0x0108
- repeat the third step; the device auto-increments the offset on each read.

The Lunatone DALI-2 IoT4 manual (page 15) demonstrates exactly this pattern over its
Modbus interface, using the same DTR1+DTR0+0xC5 dance. So in principle our raw channel
can do it too — but with no native cache.

### A.3 DALI-2 (ed.2) deltas summary

From DiiA Tech Note 1 §2.1 and Tridonic's whitepaper:

| Delta | Coverage in TinyModbus |
|---|---|
| Extended fade time (100 ms — 16 min, opcode 0x30) | ✗ even via raw cleanly (would need DTR0 staging) |
| Variable `lastActiveLevel` + GO TO LAST ACTIVE LEVEL (0x0A) | ✓ raw |
| WRITE MEMORY LOCATION NO REPLY (0xC9) | ✗ |
| RESET MEMORY BANK (0x24) | ✗ |
| SAVE PERSISTENT VARIABLES (0x22) | ✗ |
| QUERY LIGHT SOURCE TYPE (0x9F) | ✗ |
| PING (0xAD) — control device only, control gear ignores | n/a (we are a control device, not a control gear; we don't *send* PING, we don't *answer* PING) |
| Bus-powered units, multi-master timing, 24-bit frames in Part 103 | ✗ — TinyModbus only encodes 16-bit forward frames |
| Operating modes (0x23) | ✗ |
| Test sequences in Part 101 (compliance) | n/a (we are not certifying) |

Two of these — extended fade time (0x30) and the new query commands (0x9F, 0xAA,
0xFF) — would be useful additions even for a non-certified gateway. The rest mostly
matter only for DALI-2 *certification*, which TinyModbus is not pursuing.

### A.4 Device-type extension coverage (DT0–DT8+)

| DT | Spec part | Application | TinyModbus |
|---|---|---|---|
| DT0 | -201 | Fluorescent (legacy) | n/a |
| DT1 | -202 | Self-contained emergency lighting | ✗ — see Section B |
| DT2 | -203 | HID lamps | n/a |
| DT5 | -206 | Conversion to 0–10 V | n/a |
| DT6 | -207 | LED modules | implicit DT0/baseline behaviour only — no DT6-specific commands |
| DT7 | -208 | Switching function | ✗ |
| DT8 | -209 | Colour control (Tc, RGBWAF, xy) | ✗ — see Section B |

---

## Section B — Feature gaps versus reference implementations

This section answers, per feature: which open-source / commercial implementations
support it, and where TinyModbus sits.

### B.1 Commissioning (RANDOMISE / COMPARE / WITHDRAW / PROGRAM SHORT ADDRESS)

| Project | Commissioning support |
|---|---|
| python-dali (`dali/sequences.py::Commissioning`) | Full: binary-search COMPARE chain, address pool selection, dry-run, clash detection. |
| hubsif/arduino-dali | Full: `commission()` + `commission_tick()` state-machine ticker; `RANDOMISE`/`COMPARE`/`PROGRAMSHORT` states explicit. |
| Microchip AN1487 (control gear side) | Full: `IdentifyReceiveFrame()` handles all 258–270 special commands; explicit 15-minute INITIALISE window timer. |
| Tasmota DALI driver | Full: `DaliScan` performs sequential address assignment. |
| Domat R091 (commercial) | Full: web-UI driven. "Total readdressing", "Address ballast without short address", "Change address", "Delete short address" all exposed. |
| Lunatone 4Net / IoT4 | Full: addressing done via the manufacturer's DALI Cockpit tool talking to the gateway out-of-band (not via Modbus directly, but the gateway is responsible for the commissioning loop). |
| TinyModbus | **None.** Documented limitation: "This firmware does not implement commissioning — it assumes devices are already addressed before connecting them to the bus." Master can in principle drive a commissioning state machine through the 0x1000 raw channel, but: (a) the SEND-TWICE timing is wrong (see C.4), (b) there is no `WAIT_FOR_IDLE` short-circuit because the special commands don't expect a backward frame, (c) the firmware does not synthesise the 24-bit search-address comparison logic, so the master would have to round-trip COMPARE 24 times per device. |

The closest peer that does *not* support commissioning is **paultbarrett/bitsDALI**
on ATmega328P — it is roughly the same scale as TinyModbus and explicitly does not
implement commissioning. **NabiyevTR/DALI** also omits it. So this gap is not
unique to TinyModbus, but every implementation aimed at being a "complete" DALI master
(arduino-dali, python-dali, Microchip, Tasmota, every commercial gateway reviewed)
includes it.

### B.2 Memory bank reads

| Project | Memory bank reads |
|---|---|
| python-dali (`dali/memory/`) | Full bank 0 + bank 1 + extension banks (202/203/204) decoded into typed accessors. |
| Lunatone 4Net / IoT4 | Documented worked example for bank 0 reads in the Modbus PDF (page 15 of the IoT4 manual): three Modbus writes (DTR1=bank, DTR0=offset, 0xC5 READ MEMORY LOCATION), then read DALI answer back. Multi-byte sequential reads exploit the DALI auto-increment of DTR0. |
| Domat R091 | Reachable through the generic raw-frame TX/RX path (registers 516+); not surfaced as typed registers. |
| hubsif/arduino-dali | Has `CMD_WRITE_MEM_LOC` and `CMD_WRITE_MEM_LOC_NOREPLY` send paths but no canned read path. |
| TinyModbus | No native support. Reachable through the raw channel by the same three-write pattern Lunatone documents. |

The version block (FC04 0x0200) we *do* expose is the **gateway's own** version + serial,
not the ballast's bank 0. There is no register class for "read memory bank N offset K
of ballast X".

### B.3 DALI-2 / DT8 colour control

| Project | DT8 |
|---|---|
| python-dali (`dali/gear/colour.py`) | Full: SetTemporaryX/YCoordinate, SetTemporaryColourTemperature, SetTemporaryRGBDimLevel, SetTemporaryWAFDimLevel, Activate, all of the QueryColour* family. Opcodes 0xE0–0xFF (with `EnableDeviceType(8)` first). |
| hubsif/arduino-dali (fork) | Partial: the fork notes "added DALI Commands (DaliCmdExtendedDT8)". |
| Tasmota DALI driver | Full DT8 RGBWAF integrated with the Tasmota Light controls; "RGBWAF color control using standard Tasmota light controls". |
| Lunatone 4Net / IoT4 | Full — DT8 RGB worked example in the Modbus manual: write DTR0/1/2 for R/G/B, then issue the DT8 SET TEMPORARY DIM LEVEL + ACTIVATE pair with the device-type byte 8 prepended. |
| Domat R091 | Reachable via the variable-length raw TX channel (16/24/25-bit frames supported via `dali TX No.of bits` field). |
| TinyModbus | **No native support.** Reachable only by master-driven raw + DTR0/1/2 staging on the 0x1000 channel; there is no register class for "set Tc of ballast 0x12 to 4000 K" or "set RGB of ballast 0x12 to (255, 128, 0)". The firmware does not even know the concept of *enabling* a device type before issuing 0xE0–0xFF commands — the master must remember to send `0xC1 08` before each colour command, and many ballasts re-disable DT after a single command, requiring re-enable per command. |

### B.4 DALI-2 / DT7 / DT1 emergency

| Project | Emergency (DT1, IEC 62386-202) |
|---|---|
| python-dali (`dali/gear/emergency.py`) | Full: 0xE0–0xFE — Rest, Inhibit, ReLightResetInhibit, StartFunctionTest, StartDurationTest, StopTest, ResetFunctionTestDoneFlag, ResetDurationTestDoneFlag, ResetLampTime, StoreDTRAsEmergencyLevel, StoreTestDelayTimeHigh/LowByte, StoreFunctionTestInterval, StoreDurationTestInterval, StoreTestExecutionTimeout, StoreProlongTime, StartIdentification, plus the full Query family (battery charge, test timing, duration test result, lamp emergency time, lamp total operation time, emergency level/min/max, rated duration, emergency mode, features, failure status, status). |
| Beckhoff TC3 DALI library | Full: function/duration test initiation, status feedback (battery health, lamp condition, test results). |
| Lunatone, Domat, Helvar gateways | Full — emergency drivers handled the same way as any other DT: ENABLE DEVICE TYPE 1, then 0xE0–0xFE. |
| TinyModbus | **No native support.** Reachable only through the raw channel + master-driven `0xC1 01` ENABLE DEVICE TYPE 1 priming, which is *not* a SEND-TWICE command but the firmware's RAW_CMD_TWICE path will send it twice anyway, with the wrong inter-send timing. |

### B.5 D4i — energy reporting and diagnostics (Parts 251/252/253)

D4i mandates memory banks 1 (luminaire info, extended), 202 (active power + active
energy in real time), and 253-format counters (driver overall failure, light source
overall failure, output power limitation, thermal derating/shutdown, lifetime counters,
operating-time timers). See the Inventronics primer.

| Project | D4i support |
|---|---|
| python-dali | Memory access framework supports arbitrary banks; bank 202 / 203 decoders are in development per project notes. |
| Lunatone IoT4 | "Cyclic polling of status and actual level" + sensor-value registers (6100–7123) for DALI-2 input devices; D4i power/energy reads are achievable through the generic memory-bank read pattern. |
| TinyModbus | No support. The gateway has 1 KB of SRAM total and 16 KB of flash on an ATtiny1604; even decoding a single bank-202 reading per ballast (active power = 4 bytes, active energy = 4 bytes per ballast = 512 B for 64 ballasts) would dominate the RAM budget. |

### B.6 Self-test

Distinct from emergency self-test (DT1, B.4 above), the firmware does not run any
gateway-side self-test (e.g. periodic broadcast→broadcast bus integrity check, watchdog
ping, RX-edge-detected-while-TX collision audit). It does have:
- A watchdog (8.2 s) to recover from main-loop hang.
- BOD at 2.6 V to make the reset boundary clean.
- A `WAIT_FOR_IDLE` 200 Te safety escape if the bus never settles.

That is roughly equivalent to the `paultbarrett/bitsDALI` and `NabiyevTR/DALI` peer
group. **No** open-source AVR-class implementation we found instruments the bus
itself with periodic health probes.

### B.7 Energy reporting

The gateway has no energy-meter facility. The DALI-side energy data lives in bank
202 of D4i drivers, which we don't read (B.2). There is no per-ballast Modbus
"active power" or "active energy" register, because the firmware never queries that
data. Compare:
- Lunatone IoT4 has 6100–7123 sensor-value registers (24-bit) but those are for DALI-2
  *input device* sensors (light, temperature, humidity, pressure, AQI, motion), not
  for DALI-side gear energy.
- The Domat R091 has no energy registers either.
- A dedicated DALI-2 energy logger like Lunatone DALI-2 IoT (the cloud variant) does
  pull bank 202.

So this gap matches the commercial gateway median, not the cloud-product top end.

### B.8 Scene control

The TinyModbus firmware has *no* scene support whatsoever beyond the GO TO SCENE
0x10–0x1F broadcast that masters can issue through the raw channel. There is no:
- per-ballast SET SCENE register (0x40+s configuration command),
- per-ballast scene-cache (the python-dali / Lunatone / Domat gateways all expose
  scene-storage as either a register block or a config sub-page).

Domat R091 explicitly has a "Scene configuration" web page that reads/writes scenes
through the SET SCENE / REMOVE FROM SCENE / QUERY SCENE LEVEL paths.

### B.9 Multi-master coexistence

| Project | Multi-master coexistence |
|---|---|
| Beckhoff DALI library | Full collision detection, collision avoidance via priorities (1–5), break-sequence (1.2–1.4 ms) and recover-sequence (4.0–4.6 ms) — DALI-2 ed.2 multi-master timing. |
| Domat R091 | "DALI bus multimaster controller (according to EN 60929 ed. 4:2011 Annex E, static priority 4, setting time 16 ms, retry timeout 300 ms)" — explicit priority + retry. |
| Tasmota | "Connecting several Tasmota DALI gateways to the same bus is supported." |
| TinyModbus | **Yield-only.** Detects "rising edge on RX in IDLE → another master is transmitting", enters WAIT_FOR_IDLE, waits for 22 Te of bus quiet, then resumes. Does **not** detect collisions during *its own* transmissions; does not implement DALI-2 priorities; does not implement break/recover. |

This is the intended design for a "polite single-master" gateway. It is honest about
this in the README ("designed to operate as the sole DALI master on its bus").

---

## Section C — Robustness / bit-level concerns

This section compares TinyModbus's implementation against vendor app notes
(Microchip AN1465A/AN1487, Silicon Labs AN1220, TI MSPM0) and other AVR
implementations on the four robustness axes that DALI implementations historically
get wrong.

### C.1 Bit-level encoding / decoding architecture

| Project | Approach | Tolerance bias |
|---|---|---|
| Silicon Labs AN1220 (EFR32 Series 1) | TX via SPI-as-Manchester (USART in SPI mode, 2× bit rate, lookup table) + DMA. RX via TIMER input capture + PRS-driven DMA sampling at ½ Te then 1 Te per bit. Designed so that "the DALI_RX_PIN capture is always in the middle of every bit, [eliminating] error due to drifting." | Hardware-assisted; minimum CPU. |
| Silicon Labs AN1220 (EFR32xG24) | EUSART hardware-Manchester directly. | True hardware. |
| TI MSPM0 | "Collision detection/avoidance for multi-controller scenarios." Implementation uses peripherals + EEPROM persistence. | Hardware peripheral. |
| Microchip AN1487 + AN1465A | TX bit-bang with hardware timer; RX edge-detect with capture timer. | Bit-banged. |
| hubsif/arduino-dali | TimerOne for TX; PinChangeInterrupt for RX. Pure ISR-based bit-bang. | Bit-banged. |
| paultbarrett/bitsDALI | ATmega328P timer; Cosino-derived. | Bit-banged. |
| NabiyevTR/DALI | Timer 2 (ATmega328) / Timer 1 (ATtiny85), 6 samples per pulse, "100% clock speed difference" tolerance claim. | Oversampled bit-bang — *more tolerant* than TinyModbus's 2-sample classification. |
| TinyModbus | TCA0 half-bit-clock callback (TX) + TCB0 frequency-measurement-mode pulse-width capture (RX) + AVR event system tying the RX pin into TCB0. Pulse classification: short ≈ ½ Te ±20 %, long ≈ 1 Te ±20 %, intermediate dropped. | Bit-banged but with hardware capture; well-architected for a 10 MHz core. |

**Strength:** the TCB0 frequency-measurement path is the same architecture Silicon Labs
recommends — capture pulse width directly in hardware so that a tail-end CPU stall
cannot lose an edge. It is genuinely better than the per-edge-ISR approach of
hubsif/arduino-dali and bitsDALI on the same bit-rate budget.

**Weakness:** the ±20 % classification window means the firmware accepts pulse widths
333–500 µs as "short" and 666–1000 µs as "long". The *spec* (per Pico Tech, Microchip
AN1487, IEC 60929 / 62386-101) calls for half-bit 400–433.3 µs (≈ ±4 %) and full-bit
800–866.7 µs (≈ ±4 %). Our classifier is **deliberately looser** than spec-compliant
sub-tolerance — it will accept transmissions that a strict DALI-2 receiver would
reject. That is actually a *robustness* win against badly-behaved ballasts and known
edge deviations of up to 30 µs (per Nordic devzone discussion). It is also a small
*compliance* loss for a bus in Master mode: a real DALI-2 control device is supposed
to enforce timing, and the README's "Receiver dead-band" Known Limitation calls this
out.

### C.2 Collision detection during own-TX

This is the single biggest gap relative to a multi-master-rated gateway:

- The Beckhoff TC3 DALI library description: "If a DALI device sends a high level but
  the bus is pulled low by another device, a collision is detected."
- Silicon Labs and Microchip both describe gear-side collision-during-RANDOMISE (the
  gear sees its own bit not arriving on the bus and treats the response as YES).
- Domat R091 implements EN 60929 priority 4, setting-time 16 ms, retry-timeout 300 ms.
- TinyModbus README, "Known Limitations / DALI": "**No multi-master collision
  detection during our own TX.** If another master begins driving the bus while we are
  already transmitting, the resulting collision is not detected; the receiving ballast
  simply sees a malformed frame and ignores it."

Cost-of-fix: TCB0 is already capturing edges in WAIT_FOR_RESPONSE / RECEIVING. To do
own-TX collision detection we would have to keep TCB0 running during TRANSMITTING
and compare the captured RX edges against the expected TX waveform. This is exactly
the symmetric mechanism Silicon Labs's AN1220 enables on the EFR32 series 2 (EUSART
hardware-Manchester loops back its TX through the RX path for native collision
detection). Without that, *every* commissioning use-case is fragile because
RANDOMISE and INITIALISE specifically rely on every ballast pulling YES (0xFF)
simultaneously and the controller observing the collision-merged result.

### C.3 22 Te idle-gap enforcement

The DALI standard requires:
- ≥ 22 Te idle between two consecutive forward frames (bus settle time).
- 7–22 Te between forward and backward frames.

TinyModbus has two relevant gaps documented in the README's Known Limitations:

1. **`WAIT_FOR_IDLE` skipped after a backward-frame NAK.** The state machine goes
   directly back to IDLE, on the basis that the 22 Te NAK timeout already has
   provided the gap. Silicon Labs AN1220 confirms this is acceptable: "if no backward
   frame has been started within 22TE, this is interpreted as 'no answer'" — and the
   22 Te timer has already elapsed. So the README's "structural rather than enforced"
   self-criticism is honest but not actually a robustness bug.
2. **Inter-byte/inter-frame enforcement on the *Modbus* side** (separate; see Section D).

### C.4 SET/INITIALISE/RANDOMISE re-send window — 100 ms vs 22 Te

This is the most consequential pre-existing inconsistency that the comparison
surfaces:

- IEC 62386-102 specifies that **configuration commands (0x20–0x80, special INITIALISE
  0xA5, special RANDOMISE 0xA7) must be sent twice within 100 ms** to take effect.
- The TinyModbus internal SYNC_WRITE_MINMAX/POWER/FADE flow drives DTR0 + SET_X twice;
  README's "Known Limitations / DALI" says: "Multi-step write sequences hold the bus
  for ~300 ms. SYNC_WRITE_MINMAX, SYNC_WRITE_POWER, and SYNC_WRITE_FADE each issue
  four logical steps … atomically, because DTR0 is shared across all ballasts." The
  300 ms aggregate is for *six* hardware sends; each pair of SET sends is back-to-back
  with one inter-frame gap, ≈ 50 ms, so each send-twice pair lands well inside 100 ms. ✓
- The RAW_CMD_TWICE path uses the same 22 Te inter-send gap. For SET commands ✓.
- For INITIALISE / RANDOMISE: the spec requires the *same command* sent twice within
  100 ms. The 22 Te (≈ 9 ms) gap easily falls inside 100 ms, so timing-wise it works.
  However, the spec also says "no other commands addressing the control gear can be
  sent between the two successive INITIALISE/RANDOMISE." TinyModbus's RAW_CMD_TWICE
  satisfies this (it sends the same frame twice with no interleaved work).

So we are actually compliant on this axis for the commissioning special commands —
the issue is just that RAW_CMD_TWICE does not *enforce* the 100 ms upper bound, it
relies on the bit-level state machine being faster than 100 ms (it is — single
back-to-back forward frames with 22 Te gap take ≈ 41 ms).

### C.5 DTR atomicity across set sequences

This is a TinyModbus **strength**:

- DTR0 is shared across all ballasts on the bus. If two masters interleave a `DTR0=A`
  + `SET_MAX_LEVEL` sequence, each master's SET_MAX_LEVEL can latch the wrong DTR0
  value.
- TinyModbus runs SYNC_WRITE_MINMAX/POWER/FADE atomically — once started, the work
  selector will not interleave a different ballast's DTR write or a raw command.
- Many AVR-class implementations (paultbarrett/bitsDALI, NabiyevTR/DALI) do not have a
  work scheduler at all; DTR safety is the *master's* problem.
- python-dali's `dali/sequences.py` handles this through Python coroutines, which is
  the same pattern but at the wire-protocol level.
- Lunatone's documentation explicitly notes that for DT8 RGB (which needs DTR0 + DTR1 +
  DTR2 + a DT8 command, four frames) "since 3 DTRs need to be written it cannot be
  included into one registry write" — i.e. the master is responsible for atomicity.
  TinyModbus's atomic 4-step SET sequence is *better* than Lunatone's "good luck,
  master".

### C.6 NAK vs no-response disambiguation

DALI ballasts have three observable response classes:
- **YES**: backward frame data byte 0xFF (per IEC 62386-102 — confirmed in multiple
  vendor app notes including Silabs AN1220 §2.3.3).
- **8-bit data**: backward frame data byte 0x00–0xFE.
- **No backward frame** within 22 Te: silence ⇒ either "no" or "device not present".

TinyModbus collapses (silence) into `DALI_RESPONSE_NAK`. This is correct — the spec
says silence == "no" — but the **TinyModbus retry policy** (one retry on first NAK,
mark absent on second consecutive NAK) is more pessimistic than some implementations:
- python-dali leaves retry policy to the caller.
- Microchip AN1487 has no retry.
- Lunatone: doesn't expose retries.

For an installation where ballasts can be slow to respond (≈ 9 ms response window is
tight for some old gear), two-NAK eviction is OK. The README acknowledges this with
the SYNC_RETRY_PENDING bit.

### C.7 Edge-timing tolerance / capacitive ringing

Several commercial gateways and the Beckhoff library mention "capacitive ringing" or
"poor interface design causes overall interface to fail to meet protocol conditions."
TinyModbus's RX is opto-coupled, so the firmware sees inverted RX. The
WAIT_FOR_IDLE state already restarts the 22 Te countdown if the bus goes high mid-wait
(README documents this). This is correct behaviour for ringing.

---

## Section D — Modbus surface concerns

This section compares the *Modbus register map* of TinyModbus to the four commercial
Modbus-DALI gateways we reviewed in detail.

### D.1 Comparative register surface

| Feature class | TinyModbus | Lunatone IoT4 | Lunatone 4Net | Domat R091 | Intesis MBS-DAL |
|---|---|---|---|---|---|
| Per-ballast brightness (cached, R/W) | 0x0000–0x003F (FC03) | 9000–9063 (R only — actual level + short address) | – | 95–158 (intensity, R/W) | yes |
| Per-ballast status (cached) | 0x0000–0x003F high byte (FC03) | 9100–9163 (extended status + DALI status) | – | 31–94 LSB+MSB (status + comm flags) | yes |
| Per-ballast min/max | 0x0040–0x007F | not exposed as register | – | per-ballast via raw block | not visible |
| Per-ballast power-on / sys-fail | 0x0080–0x00BF | not exposed as register | – | per-ballast via raw block | not visible |
| Per-ballast fade rate / time | 0x00C0–0x00FF | not exposed as register | – | per-ballast via raw block | not visible |
| Per-group group masks | 0x0110–0x014F | not exposed as register | – | not exposed as register | not visible |
| Group intensity (write group level) | – via raw only | – | via raw only | 160–175 (group intensity) | yes |
| Broadcast intensity | – via raw only | – | via raw only | 176 (analogue broadcast) | yes |
| Bit on/off control of ballasts | – | – | – | 177–180 (bit control of ballasts 0–63) | – |
| Bit on/off control of groups | – | – | – | 181 | – |
| Scene config (per ballast × scene) | – | – | – | per-ballast scene-config web (not Modbus-mapped) | – |
| Raw forward-frame channel | 0x1000 (single), 0x1001 (twice) | reg 100 (write) + reg 101 (read), supports 16/24/25-bit DALI | reg 100 (write) + reg 101 (read) | regs 6–17 (8 blocks of TX) and 516+ (16/24/25-bit blocks) | – |
| Async raw result polling | 0x0108 (read-clears) | reg 101, FC23 read+write atom | reg 101 | per-block status byte D1 (0x55 OK / 0x02 bus err / 0x03 damaged / 0x04 short / 0x05 busy) | – |
| Presence bitmap | 0x0100–0x0103 (FC04) | implicit in reg 9000 (short address = 255 means absent) | – | not surfaced | – |
| Dirty bitmap (change notification) | 0x0104–0x0107 (FC04, read-clears) | – | – | – | – |
| Bus integrity flags | – | – | implicit (status byte 0x77) | reg 3 status: bit 4 short-circuit, bit 3 FIFO full, bit 5 power supply disabled | – |
| Energy registers (per-ballast active power / energy) | – | – | – | – | – |
| Sensor-value registers (DALI-2 input devices) | – | 6100–7123 | – | – | yes (sensors) |
| Operating-hours / lifetime counters | – | – | – | – | – |
| Polling-interval config | RESCAN_INTERVAL @ 0x1011 | reg 1 (polling enable per line) | – | reg 159 (status loop time) | – |
| Slave-address config persisted | SLAVE_ADDR @ 0x1012 (USERROW) | – (Ethernet IP-based) | – (Ethernet) | – (Ethernet) | – |
| Baud-rate config persisted | BAUD_INDEX @ 0x1013 (USERROW) | – | – | – | – |
| Send-twice flag per send | (always twice via 0x1001) | bit 5 of "Control" byte in reg 100 frame | bit 5 of "Control" byte | reg 4 send-twice mask per block | – |
| DALI priority field | – | – | – | bits 1–3 of "TX flags" per block | – |
| Send-with-DTR flag | – (must master-stage DTR) | bit 4 of Control + reg 100 byte 8 (DTR auto-staged) | bit 4 of Control | – | – |
| Device-type prefix flag | – | bit 3 of Control + reg 100 byte 10 | bit 3 of Control | – | – |
| Set-actual-level-to-DTR flag | – | bit 2 of Control | bit 2 of Control | – | – |

### D.2 Master-friendly features TinyModbus lacks

Reviewing the matrix, the **gateway-quality-of-life** registers other gateways
expose but TinyModbus does not:

1. **Group-level / broadcast-level write registers.** Domat R091's `group intensity 0..15`
   and `analogue broadcast value` registers (160–176) accept a single 8-bit level and
   issue a DALI group/broadcast DAPC. TinyModbus only exposes this through the raw
   channel (`0x1000 ← 0x80nn` for group 0 DAPC, etc.) — slightly more friction.
2. **Bit-on/off-control of ballasts** as a 64-bit bitmap (Domat regs 177–180). For a
   typical "switch a row of 16 lights" use-case, this is one Modbus write versus 16
   per-ballast writes. TinyModbus's per-ballast bank requires 16 separate FC06s (or
   one FC10 of 16 regs).
3. **Scene-storage register block.** Per-ballast × per-scene level (Domat web UI; not
   Modbus-surface, so this is *not* a clear gap against any reviewed product, but it
   is a clear gap against python-dali and against best-of-breed gateways).
4. **Send-twice + send-with-DTR + send-with-DT-prefix as flags on a single Modbus
   write.** Lunatone exposes Control byte bits 2–5 to do this in one Modbus
   transaction. TinyModbus's master must stage DTR0/1/2 manually and then issue the
   command, which is 2–4 Modbus writes per logical operation. This is the Lunatone
   "we send the prep frame for you" convenience layer.
5. **DALI priority field on raw sends.** Domat blocks 8–15 carry a 3-bit priority. We
   have no concept of priority because we are single-master. (Not a defect, but a
   spec-side feature commercial gateways implement.)
6. **24-bit and 25-bit forward frames.** Domat block can specify 16-, 24-, or 25-bit
   frames; the 24-bit frames are required for IEC 62386-103 control-device
   communications (e.g. talking to DALI-2 input devices). TinyModbus is hard-wired to
   16-bit frames. This means we cannot, even via the raw channel, talk to a DALI-2
   input device — sensors, push-buttons, light sensors all need 24-bit frames.
7. **Bus diagnostics register.** Domat reg 3 has bit 4 = "DALI bus shortcircuit
   detected, bus power is off"; bit 3 = "DALI FIFO full"; bit 1 = "init mode". A
   Modbus master can detect a wire fault directly. TinyModbus has no equivalent
   register; the only signal of bus trouble is "all ballasts NAK during scan, mark all
   absent". A short-circuit doesn't produce edges, so it never triggers an error in
   our state machine — the bus just looks idle forever and the master eventually sees
   the present-bitmap drop to zero.
8. **Energy / operating-hours registers.** Already covered in B.5/B.7.

### D.3 Master-friendly features TinyModbus *does* provide that the comparison set does not

For honesty, not all asymmetry runs against us:

1. **Dirty bitmap with read-clear semantics (FC04 0x0104–0x0107).** None of the
   commercial gateways reviewed has this primitive. The Lunatone IoT4 manual instead
   leans on "polling configuration" (reg 1) — ballasts are polled and the master
   re-reads. The dirty bitmap lets the master poll a single 8-byte FC04 read and
   immediately know which ballast slots changed since last poll. This is a real
   advantage for low-bandwidth Modbus RTU.
2. **Per-ballast cached config block (min/max/power/fade) as native registers.** The
   commercial gateways either don't expose these (Lunatone) or only expose them
   through a raw channel + the master driving the queries (Domat does so). TinyModbus
   exposes them as first-class typed FC03 registers.
3. **USERROW persistence of slave address + baud rate.** Most RTU peripherals lose
   the slave address on chip-erase. The TinyModbus USERROW pattern survives
   re-flashing. This is novel and useful.
4. **Atomic SET sequences (DTR0 + SET twice + DTR0 + SET twice).** As noted in C.5,
   we are *better* on this axis than the commercial gateways' "you stage DTR yourself"
   model.

---

## Section E — Honest weakness ranking

This is the maintainer-facing summary. Each entry is ranked by `severity ×
user-likelihood`. Severity is "what breaks if this happens", user-likelihood is "what
fraction of installations realistically encounter this".

### Top 10 weaknesses

#### 1. No commissioning support — assumes pre-addressed ballasts
- **Description:** Cannot RANDOMISE / COMPARE / PROGRAM SHORT ADDRESS on a fresh,
  unconfigured bus.
- **Evidence:** Every reviewed open-source library aimed at "complete master" has it
  (python-dali, hubsif/arduino-dali, Tasmota, edinburghhacklab/dali, plus all five
  commercial gateways). Only minimal ATmega-class libs (bitsDALI, NabiyevTR/DALI,
  daliMaster) skip it.
- **User-likelihood:** High *for first install of a new bus*; low for incumbent
  installations.
- **Severity:** High — installations cannot use TinyModbus as their only DALI tool;
  they need a separate commissioning device. The README is honest about this.
- **Fix-effort estimate:** Large. Needs (a) DTR0/1/2 staging in the work selector
  with proper 100 ms-window enforcement, (b) full INITIALISE / RANDOMISE / COMPARE /
  WITHDRAW / SEARCHADDRH/M/L / PROGRAM SHORT ADDRESS / VERIFY SHORT ADDRESS /
  TERMINATE state machine, (c) collision-during-COMPARE handling (§C.2), (d) ≥ 1 KB
  flash budget, (e) at least the binary-search bookkeeping in RAM.

#### 2. No DALI-2 input devices (no 24-bit forward frames)
- **Description:** Cannot talk to DALI-2 sensors, push-buttons, or any device defined
  in IEC 62386-103 / 301–304. The state machine encodes only 16-bit forward frames.
- **Evidence:** DiiA Tech Note 1 §2.2: "24-bit frame format defined" by Part 103 for
  control devices. Lunatone IoT4 supports it as "Mode 4 = send eDALI (25-bit)" /
  "Mode 6 = 3-byte DALI (24-bit)". Domat R091 has `dali TX No.of bits` field
  selectable between 16 / 24 / 25.
- **User-likelihood:** Medium-high in any modern installation that mixes control
  gear and DALI-2 input devices on the same bus.
- **Severity:** Medium — TinyModbus is positioned as a control-gear gateway, and
  DALI-2 input devices typically have separate gateways or in-luminaire controllers
  in practice. But anyone wanting to read a wall switch over the DALI bus is blocked.
- **Fix-effort estimate:** Medium — `bits_left` initialisation, the shift register,
  TX state machine, and the receive-side decoder all assume 16-bit forward / 8-bit
  backward. Generalising to N-bit frames is a few hundred lines and a slightly larger
  shift register.

#### 3. No native memory-bank reads
- **Description:** Cannot read GTIN, firmware version, hardware version, identification
  number, IEC 62386-101/-102/-103 versions, or any D4i bank 1/202/203/204 data of the
  ballasts.
- **Evidence:** Beckhoff/Lunatone/python-dali expose this directly. Microchip AN1487
  treats memory banks as part of the "core control gear" feature set.
- **User-likelihood:** Medium — asset management, post-failure forensics, predictive
  maintenance.
- **Severity:** Medium — workaround exists (master drives DTR1+DTR0+0xC5 sequence
  through the raw channel as Lunatone documents on page 15 of the IoT4 manual), but
  it is awkward and 3 Modbus writes per memory location read.
- **Fix-effort estimate:** Small if exposed as a single "read bank K offset O of
  ballast X, return bytes B0..Bn" register-block; medium if cached per ballast.

#### 4. No DT8 colour control
- **Description:** Cannot set Tc / RGB / xy / RGBWAF on DT8 ballasts.
- **Evidence:** python-dali colour module covers opcodes 0xE0–0xFF + necessary
  ENABLE_DEVICE_TYPE 8 + DTR1/DTR2 staging. Lunatone IoT4 has a worked example
  (page 14 of the manual).
- **User-likelihood:** High in colour-temperature-tunable office / hospitality
  installations; low in pure white-light commercial.
- **Severity:** High *for the use-case*. There is no graceful workaround through the
  raw channel (master must remember to ENABLE_DEVICE_TYPE 8 before every command, and
  stage three DTRs).
- **Fix-effort estimate:** Medium. Exposes 1–3 new register blocks (Tc value,
  RGB triplet, xy coordinates) and a DT8 work-selector slot that does the 4-step
  sequence atomically.

#### 5. No DT1 emergency lighting
- **Description:** Cannot run function tests, duration tests, query battery state,
  or read failure status for emergency drivers.
- **Evidence:** python-dali emergency module has full opcode coverage.
- **User-likelihood:** Medium — emergency lighting is required by code in many
  jurisdictions, but most installations use a separate dedicated emergency-test
  controller.
- **Severity:** High *for the use-case*: legal/code requirement is to test
  periodically; manual testing is the alternative.
- **Fix-effort estimate:** Medium; same shape as DT8 — register block + work-selector
  + atomic ENABLE_DEVICE_TYPE 1 priming. Memory bank decoding + counters add
  RAM cost (RAM is already tight per the README).

#### 6. No collision detection during own TX
- **Description:** If another master begins transmitting while we are in
  TRANSMITTING, our own frame is corrupted and we don't know.
- **Evidence:** Beckhoff: "DALI-2 includes collision avoidance, collision detection
  and collision resolution." Domat: priority + retry-timeout. README:
  "**No multi-master collision detection during our own TX.**"
- **User-likelihood:** Low if the install is single-master (the documented design);
  high if any other master is on the same bus (Tasmota, Lunatone, Helvar etc.).
- **Severity:** Low *under the documented usage pattern* (sole master), High otherwise.
- **Fix-effort estimate:** Medium — keep TCB0 capturing during TRANSMITTING and
  compare captured edges to the expected TX waveform; on mismatch, abort + 1.2–1.4 ms
  break sequence + 4.0–4.6 ms recover sequence + retransmit.

#### 7. No DALI-side energy / D4i diagnostics
- **Description:** No bank 202 (active power, active energy) or bank 253 (failure
  flags, lifetime counters, operating timers) read path.
- **Evidence:** Inventronics primer lists the D4i mandatory feature set; Lunatone
  exposes nothing for it via Modbus either, so this is at *parity* with mid-tier
  commercial gateways but below what dedicated cloud loggers do.
- **User-likelihood:** Low to medium in current installs (D4i drivers are still rolling
  out); growing.
- **Severity:** Low — purely advisory data; no control-path impact.
- **Fix-effort estimate:** Conditional on #3 (memory-bank read primitive existing),
  small. RAM cost for caching 64 ballasts × 8 B is ≈ 512 B which is *prohibitive* on
  the 1 KB SRAM target — a per-bank-on-demand read makes more sense.

#### 8. Limited Modbus group/broadcast control surface
- **Description:** No native group-DAPC register or broadcast-DAPC register; master
  has to stage `0x1000 ← 0x80nn` for "group 0 DAPC level nn", `0x1000 ← 0xFEnn` for
  broadcast DAPC.
- **Evidence:** Domat R091 regs 160–175 (group intensity), reg 176 (broadcast),
  177–181 (bit on/off masks). One Modbus write each.
- **User-likelihood:** Medium — common in scene-style "all the lights at 50 %"
  building automation patterns.
- **Severity:** Low — workaround works fine, it's just one extra Modbus write.
- **Fix-effort estimate:** Small. Add a 16-register group-DAPC bank at e.g.
  0x0050–0x005F and a broadcast-DAPC register at 0x1004. Compatible with the existing
  work-selector path.

#### 9. No scene control (storage / recall) surface
- **Description:** GO TO SCENE works (raw channel), but SET SCENE / REMOVE FROM SCENE
  / QUERY SCENE LEVEL are all master-driven raw + DTR0 staging. Per-ballast scene
  cache is absent.
- **Evidence:** python-dali, Domat web UI, Beckhoff library all expose scene config.
- **User-likelihood:** Medium-high in any building automation use-case.
- **Severity:** Medium — workaround exists but it is master-driven, four-frame, and
  involves raw-channel orchestration.
- **Fix-effort estimate:** Medium-large for full per-ballast × 16-scene cache
  (1024 bytes RAM, prohibitive); small for "stateless recall + raw set sequence helper".

#### 10. Receiver pulse-classification window slightly looser than spec
- **Description:** Pulse classifier uses ±20 % windows; spec is ≈ ±4 %.
- **Evidence:** README "Receiver dead-band" Known Limitation; spec quoted in
  Pico Tech / Microchip AN1487.
- **User-likelihood:** Low — most modern ballasts hit ±5 %.
- **Severity:** Very low — the wider window is more *forgiving* of out-of-spec
  ballasts. Only matters in compliance auditing or edge-case timing-attack scenarios.
- **Fix-effort estimate:** Trivial constant change in `dali_state_receiving.c`. The
  trade is robustness vs. compliance.

### Honourable mentions (not in top 10 but worth noting)

- **No own-side bus-fault detection (short-circuit, power loss)** — Domat reg 3
  exposes this directly; we don't surface it. (Severity low — workaround = check
  presence-bitmap.)
- **`dali.h` comment on opcode 0xA6 is technically wrong** — the comment says
  "reserved by IEC 62386-102 - never issue it"; the spec defines it as
  QUERY MANUFACTURER SPECIFIC MODE. The decision *not* to issue it from internal
  scans is correct, but the rationale documented in the comment is off. (Severity
  trivial; cosmetic.)
- **No PING (0xAD) handling on receive** — relevant only on multi-master buses where
  another control device sends PING. We yield on rising edge, so we won't transmit
  during PING, but we don't decode it either. (Severity trivial under documented
  single-master usage.)
- **Inter-byte vs inter-frame Modbus timeout conflation** — already documented in
  README "Known Limitations / Modbus". Not DALI-side.
- **FC04 0x0200 ASCII version block is non-standard** — works only at exactly 16
  registers from 0x0200; other lengths return ILLEGAL_ADDRESS. Documented; not a DALI
  concern.

---

## Section F — Where TinyModbus is *better* than the comparison set

For balance:

1. **Dirty-bitmap with read-clear semantics (FC04 0x0104–0x0107).** Genuinely novel
   primitive against the four commercial Modbus gateways reviewed. Cuts master poll
   bandwidth from "re-read all 64 status bytes every cycle" to "read 8 bytes, see
   which 16-ballast slots changed, re-read just those slots".
2. **Atomic SET sequences across DTR0 sharing.** Better than Lunatone's "your problem
   to coordinate".
3. **USERROW persistence of slave address + baud.** Survives chip-erase and
   re-flash. Most AVR-class peripherals do not have this; commercial gateways have
   different (Ethernet/web-config) persistence which is not directly comparable.
4. **TCB0 frequency-measurement RX path.** Hardware pulse-width capture is closer to
   the Silicon Labs AN1220 hardware-assisted approach than to the
   pure-edge-ISR-and-polling approach of hubsif/arduino-dali, paultbarrett/bitsDALI,
   NabiyevTR/DALI.
5. **Round-robin level rescan via PIT cursor instead of burst.** Avoids the
   bus-saturation pulse that traditional "rescan all 64 every N seconds" gateways
   produce. Domat acknowledges this is a real concern: "These requests bring
   substantial load to the bus, so it is recommended to keep the loop time as long
   as possible." TinyModbus's design *natively* solves this.
6. **Predictive caching with explicit dirty-state.** The combination of "cache write
   immediately, ACK Modbus immediately, dispatch DALI asynchronously, signal
   completion via dirty bit" is genuinely state-of-the-art for low-latency Modbus
   masters. Lunatone IoT4's "polling configuration" (reg 1) is a coarser approximation.
7. **Strict 1 KB SRAM, ≤ 16 KB flash budget achieved.** Comparable AVR-class
   implementations (paultbarrett/bitsDALI on ATmega328P with 2 KB SRAM, edinburghhacklab/dali
   on full Arduino) target larger devices. ATtiny1604 is genuinely tight.

---

## Appendix — quick category dashboard

```
Category                           | TinyModbus | python-dali | Lunatone IoT4 | Domat R091 | Tasmota
---------------------------------- | ---------- | ----------- | ------------- | ---------- | -------
Commissioning                       | ✗          | ✓           | ✓ (DALI Cockpit) | ✓        | ✓
Memory bank reads                   | ✗ native   | ✓           | ✓ (worked ex.)  | raw only   | ?
DT6 (LED)                           | implicit   | ✓           | ✓             | ✓          | ✓
DT7 (switching)                     | ✗          | partial     | ✓             | ✓          | ?
DT8 (colour)                        | ✗ native   | ✓           | ✓             | ✓ raw      | ✓
DT1 (emergency)                     | ✗ native   | ✓           | ✓             | ✓ raw      | ?
D4i bank 252 energy                 | ✗          | partial     | partial       | ✗          | ✗
D4i bank 253 diagnostics            | ✗          | partial     | partial       | ✗          | ✗
24-bit DALI-2 input device frames   | ✗          | ✓           | ✓             | ✓          | partial
Multi-master collision detection    | yield-only | n/a (lib)   | ✓             | ✓          | ✓
Native scene cache                  | ✗          | ✓           | ✗             | web only   | ?
Group/broadcast Modbus convenience  | ✗          | n/a         | partial       | ✓          | n/a (MQTT)
Per-ballast min/max/power/fade regs | ✓          | ✓ via API   | ✗             | ✗          | n/a
Dirty bitmap                        | ✓          | n/a         | ✗             | ✗          | n/a
USERROW persistence                 | ✓          | n/a         | n/a           | n/a        | n/a
Atomic DTR0 sequences               | ✓          | ✓ (await)   | master's job  | master's job | master's job
```

(`?` = not investigated. `n/a` = the project does not present a feature in a way
that a `✓ / ✗` answer is meaningful — e.g. python-dali is a library, not a gateway,
so "group convenience" is not part of its surface.)

---

End of report.
