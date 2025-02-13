import serial
import time

# open serial port tf mini ususes 115200 baud rate
ser = serial.Serial("/dev/serial0", 115200, timeout=2)

print("Starting LiDAR sensor test...")

try:
    while True:
        if ser.in_waiting >= 9:
            data = ser.read(9) # Read 9 bytes
            if data[0] == 0x59 and data[1] == 0x59: #Check packet header
                distance = data[2] + (data[3] << 8) #calc distance in cm
                print(f"Distance: {distance} cm ")
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\nStopping test.")
    ser.close