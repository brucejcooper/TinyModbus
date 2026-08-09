# Quick scratchpad for poking at a connected gateway over Modbus RTU, without
# the controller in the way. Not a regression test suite — the tests are
# `make test` (host) and the controller's `soak` CLI (hardware). This is for
# "is the gateway alive and what do its counters say".
#
# Needs a USB-RS-485 adapter on the bus. Adjust PORT/BAUD/SLAVE below; 19200 is
# canonical for this installation (see CLAUDE.md — 38400 works but is not clean
# against the ATtiny's internal oscillator).
#
#   python3 -m pip install pymodbus
#   python3 test.py
#
# Every block below is an EXACT-LENGTH read. Asking for a different count
# returns ILLEGAL DATA ADDRESS rather than a short read — that is deliberate,
# so a master built against the wrong layout fails loudly.

from pymodbus.client import ModbusSerialClient
import struct

PORT  = '/dev/tty.usbserial-XXXXXXXX'   # your RS-485 adapter
BAUD  = 19200
SLAVE = 3

client = ModbusSerialClient(PORT, baudrate=BAUD)
client.connect()


def regs(addr, count):
    r = client.read_input_registers(addr, count=count, slave=SLAVE)
    if r.isError():
        raise SystemExit(f"read {addr:#06x} x{count} failed: {r}")
    return r.registers


# Version string — FC04 0x0200, exactly 16 registers (32 bytes).
version = (b''.join(struct.pack('>H', v) for v in regs(0x0200, 16))
           .split(b"\x00")[0].decode('utf8'))
print("version:", version)

# Sysinfo — FC04 0x0310, exactly 4. reset_reason is the ATtiny RSTFR low byte.
up_hi, up_lo, boots, reset_reason = regs(0x0310, 4)
RSTFR = ["PORF", "BORF", "EXTRF", "WDRF", "SWRF", "UPDIRF"]
causes = [n for i, n in enumerate(RSTFR) if reset_reason & (1 << i)] or ["none"]
print(f"uptime: {(up_hi << 16) | up_lo} s   boots: {boots}   "
      f"reset: {'|'.join(causes)} (0x{reset_reason:04x})")

# RX diagnostics — FC04 0x0300, exactly 16. reg13/reg14 are the gateway's own
# invariant breaks; a healthy gateway reads 0 for its whole uptime.
d = regs(0x0300, 16)
print(f"rx: overflow={d[0]} frame_err={d[1]} crc_err={d[2]} ok={d[3]}   "
      f"internal_faults={d[13]} first_fault=0x{d[14]:04x}")

# Queue counters (v4) — FC04 0x0320, exactly 8. See QUEUE-PROTOCOL.md §6.
#   accepts == executes == results == acks in steady state; any gap is work
#   in flight or a result the master has not collected.
c = regs(0x0320, 8)
print(f"queue: accepts={c[0]} a_full={c[1]} executes={c[2]} results={c[3]} "
      f"acks={c[4]} stalls={c[5]} discards={c[6]}")
if c[1]:
    print("  ! a_full non-zero — the master's pacing estimate ran early")
if c[6]:
    print("  ! discards non-zero — malformed FC 0x41 frames reached us")
# stalls is NOT a warning: it is the backpressure rule doing its job, and a
# client that honours its in-flight cap cannot provoke it at all (§0).

client.close()
