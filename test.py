from pymodbus.client import ModbusSerialClient
from time import sleep
import struct

client = ModbusSerialClient('/dev/tty.usbserial-B001AG81', baudrate=9600)
client.connect()
# client.write_coils(0, [True, True, True, True, False, False, False, False, True, False, True, False, True, False, True, False, True, True, True, True, False, False, False, False, True, False, True, False, True, False, True, False], slave=1)

deviceId = 3
addr = 0


def ser_to_hex(vals):
    out = ""
    
    for v in vals:
        out = out + hex(v)[2:] 
    return out
    
    
    client.read
    

res = client.read_input_registers(0, count=32, slave=deviceId)
encoded = (b''.join([struct.pack('>H', v) for v in res.registers]).split(b"\x00")[0]).decode('utf8')

print(encoded)

# while True:
    
    
    
#     # res = client.read_holding_registers(addr, count=1, slave=deviceId)
#     # level = res.registers[0]
#     # print("Current level is ", level)
#     # if level == 0:
#     #     client.write_register(addr, 254, slave=deviceId)
#     # else:
#     #     client.write_register(addr, 0, slave=deviceId)
#     sleep(1)
    

client.close()