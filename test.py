# Quick scratchpad for poking at a connected gateway over Modbus RTU.
# Not a regression test suite — just a hand-running probe.  Adjust the
# serial port / baud / slave address to match your setup before running.

from pymodbus.client import ModbusSerialClient
import struct

PORT  = '/dev/tty.usbserial-B001AG81'
BAUD  = 9600
SLAVE = 3

client = ModbusSerialClient(PORT, baudrate=BAUD)
client.connect()

# Version string lives at FC04 0x0200, exactly 16 registers (32 bytes).
res = client.read_input_registers(0x0200, count=16, slave=SLAVE)
encoded = (b''.join(struct.pack('>H', v) for v in res.registers)
           .split(b"\x00")[0]
           .decode('utf8'))
print("version:", encoded)

# Presence + dirty bitmaps are packed at FC04 0x0100-0x0107 (8 regs total).
res = client.read_input_registers(0x0100, count=8, slave=SLAVE)
present = res.registers[0:4]
dirty   = res.registers[4:8]
print("present:", [f'0x{w:04x}' for w in present])
print("dirty:  ", [f'0x{w:04x}' for w in dirty])

# Cached per-ballast brightness/status for ballast 0 lives at FC03 0x0000.
res = client.read_holding_registers(0x0000, count=1, slave=SLAVE)
status, brightness = res.registers[0] >> 8, res.registers[0] & 0xFF
print(f"ballast 0: status=0x{status:02x} brightness={brightness}")

client.close()
