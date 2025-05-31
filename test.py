from pymodbus.client import ModbusSerialClient
from time import sleep

client = ModbusSerialClient('/dev/tty.usbserial-AB0LYXNO', baudrate=9600)
client.connect()
# client.write_coils(0, [True, True, True, True, False, False, False, False, True, False, True, False, True, False, True, False, True, True, True, True, False, False, False, False, True, False, True, False, True, False, True, False], slave=1)

while True:
    print("On")
    client.write_coil(8, True, slave=1)
    sleep(0.5)
    print("Off")
    client.write_coil(8, False, slave=1)
    sleep(0.5)


client.close()