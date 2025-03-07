import time
import serial

import neopixel
import time
import keyboard
import board
import threading


# LED strip configuration
LED_COUNT = 35         # Number of LEDs in your strip
LED_PIN = board.D18    # GPIO pin connected to the LED strip (must support PWM)
LED_BRIGHTNESS = 0.5   # Brightness (0.0 to 1.0)
LED_ORDER = neopixel.GRB  # Color order of the LEDs

#shouldnt do this but making distance global
distance = 0
# Initialize the LED strip
strip = neopixel.NeoPixel(LED_PIN, LED_COUNT, brightness=LED_BRIGHTNESS, auto_write=False, pixel_order=LED_ORDER)

TOTAL_DISTANCE = 57
UNIT_DISTANCE = TOTAL_DISTANCE / LED_COUNT
colours = {"off" : (0,0,0),"red": (100,30,90), "orange": (0,120,120), "yellow": (),"green" : (255,0,0),"blue": (), "indigo": (), "violet": ()}
NEW_TARGET = False

# ser = serial.Serial("/dev/ttyUSB1", 115200)
ser = serial.Serial("/dev/serial0", 115200, timeout=1)
#ser = serial.Serial("/dev/serial0", 115200)
# ser = serial.Serial("COM12", 115200)

#sensor2_port = "/dev/ttyUSB0"
#ser = serial.Serial(sensor2_port, 115200, timeout=1)

# we define a new function that will get the data from LiDAR and publish it



def shift_led(index):
    for i in range(1,index):
        strip[i] = strip[i+1]
    for i in range(LED_COUNT - 1,index, -1):
        strip[i] = strip[i-1]
    
def my_rainbow(colours, block_size, index, sleep_time):
    NEW_TARGET = False
    block_num = 0
    colour_index = 0
    while not NEW_TARGET:
        shift_led(index)
        if index > 0:
            strip[index-1] = colours[colour_index]
        if index < LED_COUNT:
            strip[index+1] = colours[colour_index]
        strip[index] = (255,255,255)
        block_num += 1
        if block_num >= block_size:
            colour_index = (colour_index + 1) % len(colours)
            block_num = 0
        strip.show()
        time.sleep(sleep_time)
        
        
def read_data():
    
    counter = ser.in_waiting # count the number of bytes of the serial port
        #print(counter)
    if counter > 8:
        bytes_serial = ser.read(9)
        ser.reset_input_buffer()

        if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59: # this portion is for python3
            # print("Printing python3 portion")            
            distance = bytes_serial[2] + bytes_serial[3]*256 # multiplied by 256, because the binary data is shifted by 8 to the left (equivalent to "<< 8").                                              # Dist_L, could simply be added resulting in 16-bit data of Dist_Total.
            strength = bytes_serial[4] + bytes_serial[5]*256
            temperature = bytes_serial[6] + bytes_serial[7]*256
            temperature = (temperature/8) - 256
            # print("Distance:"+ str(distance))
            # print("Strength:" + str(strength))
            # if temperature != 0:
                #print("Temperature:" + str(temperature))
            ser.reset_input_buffer()
            return distance

def sensor_thread():
    prev_distance = 0
    global distance
    
    while True:
        print("working")
        if not ser.is_open:
            ser.open()  # Open the serial port if it's closed
        if distance is not None:
            prev_distance = distance    
            distance = read_data()  # Read distance from the sensor
            if distance is not None and abs(distance - prev_distance) > 1:
                NEW_TARGET = True
        time.sleep(0.1)

def t_test1():
    for i in range(10):
        print(f"t1: {i}")
    
def t_test2():
    for i in range(10):
        print(f"t2: {i}")

if __name__ == "__main__":
    thread1 = threading.Thread(target=sensor_thread)

    thread1.start()
    try:
        while True:

            if distance is not None and distance <= TOTAL_DISTANCE:
                # Map distance to LED index
                led_location = int(distance // UNIT_DISTANCE)
                #led_location = max(0, min(LED_COUNT - 1, led_location))  # Clamp to valid range
                print(f"Target LED: {led_location}")
                my_rainbow([colours["red"], colours["orange"], colours["green"]], 5, 15, 0.1)

            time.sleep(0.01)  # Small delay to avoid overwhelming the sensor

    except KeyboardInterrupt:
        # Turn off all LEDs on exit
        strip.fill((0, 0, 0))
        strip.show()
        if ser.is_open:
            ser.close()
        print("Program interrupted by the user")

    except KeyboardInterrupt:
        # Turn off all LEDs on exit
        strip.fill((0, 0, 0))
        strip.show()
        if ser.is_open:
            ser.close()
        print("Program interrupted by the user")
