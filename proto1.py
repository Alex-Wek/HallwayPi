import time
import serial

import neopixel
import time
import keyboard
import board


# LED strip configuration
LED_COUNT = 35         # Number of LEDs in your strip
LED_PIN = board.D18    # GPIO pin connected to the LED strip (must support PWM)
LED_BRIGHTNESS = 0.5   # Brightness (0.0 to 1.0)
LED_ORDER = neopixel.GRB  # Color order of the LEDs

# Initialize the LED strip
strip = neopixel.NeoPixel(LED_PIN, LED_COUNT, brightness=LED_BRIGHTNESS, auto_write=False, pixel_order=LED_ORDER)

TOTAL_DISTANCE = 54
UNIT_DISTANCE = TOTAL_DISTANCE / LED_COUNT


# ser = serial.Serial("/dev/ttyUSB1", 115200)
ser = serial.Serial("/dev/serial0", 115200, timeout=1)
#ser = serial.Serial("/dev/serial0", 115200)
# ser = serial.Serial("COM12", 115200)

#sensor2_port = "/dev/ttyUSB0"
#ser = serial.Serial(sensor2_port, 115200, timeout=1)

# we define a new function that will get the data from LiDAR and publish it
def read_data():
    
    counter = ser.in_waiting # count the number of bytes of the serial port
        #print(counter)
    if counter > 8:
        bytes_serial = ser.read(9)
        ser.reset_input_buffer()

        if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59: # this portion is for python3
            print("Printing python3 portion")            
            distance = bytes_serial[2] + bytes_serial[3]*256 # multiplied by 256, because the binary data is shifted by 8 to the left (equivalent to "<< 8").                                              # Dist_L, could simply be added resulting in 16-bit data of Dist_Total.
            strength = bytes_serial[4] + bytes_serial[5]*256
            temperature = bytes_serial[6] + bytes_serial[7]*256
            temperature = (temperature/8) - 256
            print("Distance:"+ str(distance))
            print("Strength:" + str(strength))
            if temperature != 0:
                print("Temperature:" + str(temperature))
            ser.reset_input_buffer()
            return distance

 
if __name__ == "__main__":
    try:
        while True:
            if ser.isOpen() == False:
                ser.open()
            distance = read_data()
            if distance is not None:
                led_location = int(distance // UNIT_DISTANCE)
                print(led_location)
                strip.fill((0,0,0))
                if led_location < LED_COUNT:
                    print("got herre")
                    strip[led_location] = (255,0,0)
                    strip.show()

            
    except KeyboardInterrupt(): # ctrl + c in terminal.
        if ser != None:
            ser.close()
            print("program interrupted by the user")

