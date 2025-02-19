import serial
import time
ser1 = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)
#ser2 = serial.Serial("/dev/serial1", 115200, timeout=1)

while True:
    bytes_waiting = ser1.in_waiting
    print(f"bytes avail = {bytes_waiting}")
    time.sleep(1)
    

print("both sensors activated")
if ser1.is_open:
    print("Seroal1 port is open")
else:
    print("failed to open 1")

ser1.close()
#ser2.close()
