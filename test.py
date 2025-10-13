from pymodbus.client import ModbusSerialClient
from time import sleep

client = ModbusSerialClient('/dev/tty.usbserial-B001AG81', baudrate=9600)
client.connect()
# client.write_coils(0, [True, True, True, True, False, False, False, False, True, False, True, False, True, False, True, False, True, True, True, True, False, False, False, False, True, False, True, False, True, False, True, False], slave=1)

while True:
    
    res = client.read_holding_registers(0, count=32, slave=1)
    print("result is ", res)
    
    sleep(1)
    

client.close()