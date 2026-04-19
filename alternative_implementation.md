# DALI/Modbus Bridge Firmware — Implementation Briefing

## Target Hardware
- **MCU**: ATtiny1604 (AVR 0-Series, 8-bit)
- **Clock**: 20MHz internal oscillator (no prescaler)
- **Flash**: 16KB
- **RAM**: 2KB
- **Toolchain**: avr-gcc, AVR-libc

## Overview
Firmware for an RS-485/Modbus RTU slave that bridges to a DALI lighting
control bus. A Modbus master sends commands (set brightness, toggle, up/down
etc.) which are queued and executed sequentially on the DALI bus. DALI
ballast state is maintained in an in-memory register map and exposed as
Modbus input registers. Background tasks handle fade monitoring and
periodic bus scanning.

---

## Pin Assignments
| Pin  | Function            | Notes                                      |
|------|---------------------|--------------------------------------------|
| PA6  | DALI TX             | Open-drain, DIRSET=LOW, DIRCLR=Hi-Z        |
| PA7  | DALI RX             | Input capture via event system → TCB0      |
| PB3  | USART0 RX           | Modbus RS-485 receive                      |
| PB2  | USART0 TX           | Modbus RS-485 transmit                     |
| PB0  | RS-485 DE           | HIGH = TX driver enabled                   |

---

## Hardware Resources
| Peripheral | Usage                                                        |
|------------|--------------------------------------------------------------|
| TCA0       | Shared 208µs system tick (prescaler /8, PER=519)            |
| TCB0       | DALI RX edge capture (prescaler /2 = 10MHz, input capture)  |
| USART0     | Modbus RTU 8N1                                               |
| EVSYS      | PA7 → ASYNCCH0 → TCB0 input capture                         |
| EEPROM     | Persist Modbus slave address and baud rate                   |

### TCA0 tick dispatch order (ISR calls in this order every 208µs):
1. `dali_tick()` — most timing-critical
2. `modbus_tick()`
3. `bgtasks_tick()`

---

## Protocol Details

### DALI
- 1200 baud, Manchester encoded, open-drain bus (wired-AND)
- **Manchester**: logic 1 = HIGH→LOW transition at bit centre
                  logic 0 = LOW→HIGH transition at bit centre
- **Forward frame**: 1 start bit (1) + 8 address bits + 8 command bits +
                     2 stop bits (0) = 19 bits = 38 half-bits
- **Backward frame**: ballast responds 7ms min after stop bits, 8 data bits
- **NAK**: no response within 22ms = no ballast present
- **Collision**: edge timing outside valid Manchester windows
- **Open-drain TX**: DIRSET=drive LOW, DIRCLR=release (Hi-Z, bus pulled high)
- **Half-bit period**: 416µs = 2 TCA0 ticks
- **Inter-frame gap**: minimum 9.17ms between forward frames

### DALI timing constants (TCB0 at 10MHz, 0.1µs/tick)
```c
#define DALI_TCB_HALF_MIN  3744u   // 374µs
#define DALI_TCB_HALF_MAX  4576u   // 458µs
#define DALI_TCB_FULL_MIN  7497u   // 750µs
#define DALI_TCB_FULL_MAX  9163u   // 916µs
#define DALI_NAK_TICKS      106u   // 22ms at 208µs/tick
```

### DALI address byte encoding
```c
#define DALI_SHORT_ADDR(n)  (uint8_t)(((n) & 0x3F) << 1)
#define DALI_GROUP_ADDR(n)  (uint8_t)(0x80 | (((n) & 0x0F) << 1))
#define DALI_BROADCAST      (uint8_t)(0xFF)
#define DALI_ADDR_CMD(addr) (uint8_t)((addr) | 0x01)  // set bit 0 for command
```

### DALI commands used
```c
#define DALI_CMD_OFF                  0x00
#define DALI_CMD_UP                   0x01
#define DALI_CMD_DOWN                 0x02
#define DALI_CMD_STEP_UP              0x03
#define DALI_CMD_STEP_DOWN            0x04
#define DALI_CMD_RECALL_MAX           0x05
#define DALI_CMD_RECALL_MIN           0x06
#define DALI_CMD_GOTO_LAST_ACTIVE     0x0A
#define DALI_CMD_QUERY_STATUS         0x90
#define DALI_CMD_QUERY_ACTUAL_LEVEL   0xA0
#define DALI_CMD_QUERY_MIN_LEVEL      0xA1
#define DALI_CMD_QUERY_MAX_LEVEL      0xA2
#define DALI_CMD_QUERY_POWER_ON_LEVEL 0xA3
#define DALI_CMD_QUERY_FAIL_LEVEL     0xA4
#define DALI_CMD_QUERY_FADE_RATE      0xA5
#define DALI_CMD_QUERY_FADE_TIME      0xA6
#define DALI_CMD_QUERY_GROUP_0_7      0xC0
#define DALI_CMD_QUERY_GROUP_8_15     0xC1
```

### DALI status byte bits (response to 0x90)
```c
#define DALI_STATUS_BALLAST_OK         0x01  // 0=fault, 1=ok
#define DALI_STATUS_LAMP_FAILURE       0x02
#define DALI_STATUS_LAMP_ON            0x04
#define DALI_STATUS_LIMIT_ERROR        0x08
#define DALI_STATUS_FADE_RUNNING       0x10  // key: drives bg poll
#define DALI_STATUS_RESET_STATE        0x20
#define DALI_STATUS_MISSING_SHORT_ADDR 0x40
#define DALI_STATUS_POWER_FAILURE      0x80
```

### Modbus RTU
- 8N1, configurable baud (9600/19200/38400), RS-485 half-duplex
- Slave address: configurable 1–247, default 1, persisted to EEPROM
- **3.5-character silence rule** enforced for both frame detection
  and partial-packet timeout (same timer, same threshold)
- Silence thresholds at 208µs/tick:
  - 9600 baud:  18 ticks
  - 19200 baud:  9 ticks
  - 38400 baud:  5 ticks
- **RS-485 DE sequencing**: assert DE → load first byte → DRE ISR loads
  subsequent bytes → TXC ISR deasserts DE (ensures last stop bit completes)
- **Function codes**: FC03 (read holding), FC04 (read input),
  FC06 (write single), FC10/0x10 (write multiple)
- **CRC**: CRC-16, polynomial 0x8005, reflected (standard Modbus)

---

## Register Map

### Input Registers — FC04 (read-only)

All paired registers store high byte first (big-endian), matching Modbus
wire format. Blocks can be memcpy'd directly into response buffers.

#### Ballast blocks (index = short address 0–63)
| Address Range   | High Byte    | Low Byte         | DALI source     |
|-----------------|--------------|------------------|-----------------|
| 0x0000–0x003F   | Status       | Current brightness | 0x90 / 0xA0   |
| 0x0040–0x007F   | Min level    | Max level        | 0xA1 / 0xA2     |
| 0x0080–0x00BF   | Power-on lvl | Sys failure lvl  | 0xA3 / 0xA4     |
| 0x00C0–0x00FF   | Fade rate    | Fade time        | 0xA5 / 0xA6     |

#### Group registers (index = group 0–15)
| Address Range   | High Byte         | Low Byte              |
|-----------------|-------------------|-----------------------|
| 0x0100–0x010F   | ORed member status | Last commanded level  |
| 0x0110–0x014F   | Group membership bitmask (4 regs × 16 groups, 64-bit) |

Group membership layout: 4 consecutive registers per group, big-endian
uint16_t[4], where bit n of the 64-bit value = ballast n is a member.

#### System registers
| Address  | Content                                      |
|----------|----------------------------------------------|
| 0x0150   | Scan status: 0=idle 1=scanning 2=complete    |
| 0x0151   | Ballasts present, addr 0–15  (bitmask)       |
| 0x0152   | Ballasts present, addr 16–31                 |
| 0x0153   | Ballasts present, addr 32–47                 |
| 0x0154   | Ballasts present, addr 48–63                 |
| 0x0155   | Active fades, addr 0–15                      |
| 0x0156   | Active fades, addr 16–31                     |
| 0x0157   | Active fades, addr 32–47                     |
| 0x0158   | Active fades, addr 48–63                     |
| 0x0159   | Dirty since last read, addr 0–15             |
| 0x015A   | Dirty since last read, addr 16–31            |
| 0x015B   | Dirty since last read, addr 32–47            |
| 0x015C   | Dirty since last read, addr 48–63            |

**Dirty flag behaviour**: bit set when bg poller detects a change in
block 0 (brightness or status). Atomically cleared when FC04 reads
that dirty register. Allows smart clients to poll only changed ballasts.

### Holding Registers — FC03/FC06/FC10 (read/write)

Action registers (0x1002–0x1009) are write-only by convention —
reads return 0x0000. Each write immediately enqueues a DALI command.

| Address  | Function            | Value format                              |
|----------|---------------------|-------------------------------------------|
| 0x1000   | Raw DALI command    | [15:8]=DALI addr byte  [7:0]=cmd byte     |
| 0x1001   | Direct arc (DAPC)   | [15:8]=DALI addr byte  [7:0]=level        |
| 0x1002   | Go to last active   | [7:0]=DALI addr byte  (0x0A command)      |
| 0x1003   | Off                 | [7:0]=DALI addr byte  (0x00 command)      |
| 0x1004   | Up                  | [7:0]=DALI addr byte  (0x01, repeatable)  |
| 0x1005   | Down                | [7:0]=DALI addr byte  (0x02, repeatable)  |
| 0x1006   | Step up             | [7:0]=DALI addr byte  (0x03 command)      |
| 0x1007   | Step down           | [7:0]=DALI addr byte  (0x04 command)      |
| 0x1008   | Recall max          | [7:0]=DALI addr byte  (0x05 command)      |
| 0x1009   | Recall min          | [7:0]=DALI addr byte  (0x06 command)      |
| 0x1010   | Scan trigger        | write 0x0001 to start full bus scan       |
| 0x1011   | Poll interval       | 100ms units, default 5 (500ms)            |
| 0x1012   | Modbus slave addr   | 1–247, persisted to EEPROM                |
| 0x1013   | Baud rate index     | 0=9600 1=19200 2=38400, persisted         |

---

## In-Memory Data Structures

### register_map_t (698 bytes)
```c
typedef struct {
    // Ballast blocks — direct memcpy into Modbus response
    ballast_brightness_status_t block0[64];  // 128 bytes
    ballast_min_max_t           block1[64];  // 128 bytes
    ballast_levels_t            block2[64];  // 128 bytes
    ballast_fade_t              block3[64];  // 128 bytes
    // Groups
    group_status_t  group_status[16];        //  32 bytes
    uint8_t         group_mask[16][8];       // 128 bytes (64-bit per group)
    // System
    uint16_t scan_status;
    uint16_t present[4];                     // 64-bit bitmask, big-endian
    uint16_t fading[4];
    uint16_t dirty[4];
    // Config
    uint16_t poll_interval;                  // default 5
    uint8_t  mb_slave_addr;                  // default 1
    uint8_t  baud_index;                     // default 0
} register_map_t;

extern register_map_t g_regs;
```

### Paired register structs (high byte = Modbus high byte)
```c
typedef struct { uint8_t status;     uint8_t brightness; }
    ballast_brightness_status_t;
typedef struct { uint8_t min;        uint8_t max;        }
    ballast_min_max_t;
typedef struct { uint8_t power_on;   uint8_t sys_failure;}
    ballast_levels_t;
typedef struct { uint8_t fade_rate;  uint8_t fade_time;  }
    ballast_fade_t;
typedef struct { uint8_t status;     uint8_t brightness; }
    group_status_t;
```

### cmd_entry_t (command queue entries)
```c
typedef enum { CMD_DIRECT_ARC, CMD_SEND, CMD_SEND_TWICE, CMD_QUERY }
    cmd_type_t;
typedef enum { REG_NONE, REG_BRIGHTNESS_STATUS, REG_MIN_MAX,
               REG_LEVELS, REG_FADE } reg_target_t;
typedef enum { REG_BYTE_HIGH, REG_BYTE_LOW } reg_byte_t;

typedef struct {
    cmd_type_t   type;
    uint8_t      dali_addr;    // encoded DALI address byte
    uint8_t      dali_cmd;     // command byte or arc level
    uint8_t      ballast_idx;  // 0–63, or 0xFF for broadcast
    reg_target_t reg_target;   // where to store query response
    reg_byte_t   reg_byte;     // which byte in the pair
} cmd_entry_t;
```

### Command queue
```c
#define CMD_QUEUE_SIZE 16  // power of 2
typedef struct {
    cmd_entry_t entries[CMD_QUEUE_SIZE];
    uint8_t     head;
    uint8_t     tail;
} cmd_queue_t;
extern cmd_queue_t g_cmd_queue;
```

---

## RAM Budget
| Allocation                    | Bytes |
|-------------------------------|-------|
| register_map_t                |   698 |
| Modbus RX ring buffer         |   128 |
| Modbus TX buffer              |   256 |
| Modbus frame assembly         |    64 |
| Command queue (16 × 5 bytes)  |    82 |
| DALI TX/RX working buffers    |    32 |
| Background task state         |    24 |
| Misc globals / tick counters  |    32 |
| Stack (ISR nesting)           |   256 |
| **Total**                     |**1572**|
| **Headroom from 2KB**         | **476**|

---

## Module Files and Responsibilities

### `types.h`
All shared enums and structs. No dependencies. Include everywhere.
Key types: `dali_result_t`, `cmd_type_t`, `reg_target_t`, `reg_byte_t`,
`cmd_entry_t`, `mb_result_t`, `sys_result_t`.

### `registers.h` / `registers.c`
- Global `register_map_t g_regs` instance
- `regs_init()` — zero and set defaults
- `regs_read_input(addr, *val)` — single FC04 read
- `regs_read_holding(addr, *val)` — single FC03 read
- `regs_read_input_bulk(addr, count, *buf)` — memcpy-optimised bulk read
- `regs_write_holding(addr, val)` — validate and write (no DALI side effects)
- `regs_update_block0/1/2/3(idx, hi, lo)` — update paired registers
- `regs_update_brightness(idx, val)` / `regs_update_status(idx, val)`
- `regs_apply_status(idx, dali_status)` — updates block0 status byte,
  sets/clears fading bitmask, sets dirty bit
- `regs_set_present(idx, bool)` — maintains present bitmask
- `regs_set_fading(idx, bool)` — maintains fading bitmask
- `regs_read_clear_dirty(word_idx)` — atomic read+clear of dirty word
- `regs_update_group_mask(ballast_idx, bits, high)` — group membership
- `regs_rebuild_group_status(group_idx)` — OR all member status bytes

### `cmd_queue.h` / `cmd_queue.c`
- Ring buffer, head/tail uint8_t, size 16 (power of 2)
- `cmdq_init()`, `cmdq_empty()`, `cmdq_full()`, `cmdq_count()`
- `cmdq_push(entry*)` → `SYS_ERR_QUEUE_FULL` if full
- `cmdq_peek(entry*)` → false if empty
- `cmdq_pop()`
- Convenience: `cmdq_push_arc()`, `cmdq_push_cmd()`,
  `cmdq_push_cmd_twice()`, `cmdq_push_query()`

### `dali.h` / `dali.c`
- Non-blocking state machine: IDLE → TX → RX_WAIT → RX_ACTIVE → COMPLETE
- `dali_init()` — pins, TCA0 period (519), TCB0 input capture,
  EVSYS PA7→ASYNCCH0→TCB0
- `dali_state()`, `dali_busy()`
- `dali_start_send(addr, cmd)` — no response expected
- `dali_start_arc(addr, level)` — DAPC, no response
- `dali_start_query(addr, cmd)` — expects backward frame
- `dali_get_result(*response)` — call when state==COMPLETE, resets to IDLE
- `dali_tick()` — called from TCA0 ISR every 208µs,
  advances TX by 1 half-bit every 2nd tick
- **TX**: 38 half-bits packed into `uint8_t tx_buf[5]`, MSB first.
  Each logical bit → 2 half-bits: 1→{1,0}, 0→{0,1}. Open-drain:
  level 1 = DIRCLR (Hi-Z), level 0 = DIRSET (drive LOW)
- **RX**: TCB0 captures both edges (toggle `TCB_EDGE_bm` in ISR).
  Measures gap since last edge. half-gap=mid-bit transition,
  full-gap=bit-boundary transition, outside range=collision.
  Assembles 8-bit response MSB first.
- **NAK**: if no edge within 106 ticks (22ms) after TX complete → NAK
- **Collision detection**: gap outside [HALF_MIN,HALF_MAX] and
  [FULL_MIN,FULL_MAX] → `DALI_COLLISION`

#### ISRs owned by dali.c:
- `TCB0_INT_vect` — edge capture, calls `rx_process_edge()`

### `tick.h` / `tick.c`
- `tick_init()` — configures TCA0: prescaler /8, PER=519, overflow interrupt
- `tick_count()` — returns volatile uint16_t tick counter (disable
   interrupts before reading outside ISR context)
- `TCA0_OVF_vect` ISR: increments counter, calls `dali_tick()`,
  `modbus_tick()`, `bgtasks_tick()` in that order

### `modbus.h` / `modbus.c`
- `modbus_init()` — USART0 at configured baud, DE pin as output LOW,
  enable RXC interrupt
- `modbus_tick()` — called from TCA0 ISR: increments silence counter,
  when threshold reached sets state=FRAME_READY and disables RXC ISR
  temporarily
- `modbus_process()` — called from main loop:
  1. Returns immediately if state != FRAME_READY
  2. Drains ring buffer into frame workspace
  3. Validates CRC — sends exception 0x01 if bad
  4. Checks slave address — silently ignores if not ours
  5. Dispatches on function code:
     - FC04: calls `regs_read_input_bulk()`, builds response
     - FC03: calls `regs_read_holding()`, builds response
     - FC06: calls `regs_write_holding()`, then dispatch table lookup,
       calls `cmdq_push_*()` as appropriate, builds echo response
     - FC10: multi-register write, same as FC06 per register
     - Unknown FC: exception 0x01
  6. Appends CRC, asserts DE, starts TX via DRE interrupt
- **USART RXC ISR**: pushes byte to ring buffer, resets silence counter.
  If ring buffer full: set overflow error flag, discard byte.
- **USART DRE ISR**: pops next byte from TX buffer. When exhausted,
  disables DRE interrupt, enables TXC interrupt.
- **USART TXC ISR**: deasserts DE pin, re-enables RXC interrupt,
  sets state=IDLE.
- `modbus_crc16(buf, len)` — CRC-16/Modbus (poly 0x8005 reflected)

#### Modbus dispatch table (in modbus.c)
```c
typedef struct {
    uint16_t        mb_reg;
    cmd_type_t      cmd_type;
    uint8_t         dali_cmd;       // 0xFF = use low byte of written value
    bool            addr_from_high; // true=[15:8], false=[7:0]
    bool            marks_fading;
} reg_dispatch_t;

static const reg_dispatch_t dispatch_table[] = {
    { 0x1000, CMD_SEND,       0xFF, true,  false },  // raw cmd
    { 0x1001, CMD_DIRECT_ARC, 0xFF, true,  true  },  // DAPC
    { 0x1002, CMD_SEND,       0x0A, false, true  },  // GoToLast
    { 0x1003, CMD_SEND,       0x00, false, true  },  // Off
    { 0x1004, CMD_SEND,       0x01, false, true  },  // Up
    { 0x1005, CMD_SEND,       0x02, false, true  },  // Down
    { 0x1006, CMD_SEND,       0x03, false, true  },  // StepUp
    { 0x1007, CMD_SEND,       0x04, false, true  },  // StepDown
    { 0x1008, CMD_SEND,       0x05, false, true  },  // RecallMax
    { 0x1009, CMD_SEND,       0x06, false, true  },  // RecallMin
};
```

### `bg_tasks.h` / `bg_tasks.c`

#### Fade poller task
- Runs every `g_regs.poll_interval × 100ms` (default 500ms)
- Iterates `g_regs.fading[0..3]` bitmasks
- For each set bit: enqueues two queries:
  1. `cmdq_push_query(addr, 0x90, idx, REG_BRIGHTNESS_STATUS, REG_BYTE_HIGH)`
  2. `cmdq_push_query(addr, 0xA0, idx, REG_BRIGHTNESS_STATUS, REG_BYTE_LOW)`
- `regs_apply_status()` will clear fading bit when FADE_RUNNING clears

#### Bus scan task (state machine, not a simple loop)
Scans addresses 0–63 sequentially. Between each address, yields to
main loop (one address per bgtasks_run() call) so DALI bus and Modbus
remain responsive during scan.

Scan sequence per address:
1. `cmdq_push_query(DALI_SHORT_ADDR(i)|1, 0x90, i, REG_BRIGHTNESS_STATUS,
    REG_BYTE_HIGH)` — query status
2. On DALI_OK → `regs_set_present(i, true)`, then enqueue full data fetch:
   - brightness (0xA0), min (0xA1), max (0xA2), power-on (0xA3),
     fail (0xA4), fade rate (0xA5), fade time (0xA6),
     groups 0–7 (0xC0), groups 8–15 (0xC1)
3. On DALI_NAK → `regs_set_present(i, false)`, zero block0..3 for that idx
4. After all 64: set `g_regs.scan_status = SCAN_COMPLETE`

Triggered by: power-on (automatic), or Modbus write 0x0001 to 0x1010.

---

## Main Loop Dispatch Logic

loop:
modbus_process()           // handle any complete Modbus frame
collect_dali_result()      // if DALI_STATE_COMPLETE: store result,
//   call regs_apply_status(), cmdq_pop()
if !dali_busy():
if dispatch_needs_repeat and within deadline:
re-send head of queue (CMD_SEND_TWICE second pass)
elif !cmdq_empty():
dispatch_next_command()
bgtasks_run()              // run any due background tasks









### CMD_SEND_TWICE handling
- First send: set `dispatch_needs_repeat=true`,
  record `deadline = tick_count() + 240` (240 × 208µs ≈ 50ms)
- Second send: check deadline not passed, then send again, pop queue
- If deadline passed before second send: pop queue, reset flag (give up)

---

## Initialisation Order (main.c)
```c
_PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, 0);  // 20MHz, no prescaler
regs_init();      // zero register map, set defaults
cmdq_init();      // zero queue
dali_init();      // configure PA6/PA7, TCB0, EVSYS (no interrupts yet)
modbus_init();    // configure USART0, PB0 DE pin (no interrupts yet)
tick_init();      // configure and START TCA0 (ticks begin)
bgtasks_init();   // register tasks, trigger initial scan
sei();            // open interrupt gate last
```

---

## Key Implementation Rules

1. **ISRs must be short**: No blocking, no malloc, no function calls except
   to the module's own tick function or ring buffer push/pop.

2. **No 64-bit arithmetic in ISRs**: group_mask is uint8_t[8] specifically
   to avoid this on AVR.

3. **Open-drain TX discipline**: `OUTCLR` is set once at init and never
   touched again. Only `DIRSET`/`DIRCLR` toggle the pin state.

4. **TCB0 edge toggling**: Toggle `TCB0.EVCTRL ^= TCB_EDGE_bm` at the
   start of every `TCB0_INT_vect` ISR to alternate between rising and
   falling edge capture.

5. **Modbus DE timing**: Deassert DE only in TXC ISR (not DRE ISR) to
   ensure the last stop bit is fully transmitted before releasing the bus.

6. **Register bulk reads are memcpy-safe**: The paired register structs are
   stored high-byte-first in the same order as the Modbus register map.
   `regs_read_input_bulk()` can use `memcpy` for aligned block reads.

7. **Dirty flag is atomic**: `regs_read_clear_dirty()` must disable
   interrupts around the read-and-clear to prevent a bg_task update
   between the read and the clear.

8. **EEPROM writes**: Only on Modbus write to 0x1012 or 0x1013. Use
   AVR-libc `eeprom_update_byte()` (skips write if value unchanged).

9. **Modbus silence timer resets on every received byte** (in RXC ISR).
   Frame is considered complete only after silence threshold ticks with
   no new byte received.

10. **DALI inter-frame gap**: Enforce minimum 9.17ms (45 ticks) between
    consecutive DALI forward frames. Track last TX completion tick in
    dali.c; `dali_start_send/arc/query()` returns DALI_BUSY if gap not met.