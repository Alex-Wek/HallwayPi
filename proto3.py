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

def ripple_rainbow(index):
    down_index = index - 1
    up_index = index + 1
    down_distance = down_index
    up_distance = LED_COUNT - up_index
    while down_index >= 0 and up_index < LED_COUNT:
        if down_index == 0:
            down_index = index - 1
        if up_index == LED_COUNT:
            up_index = index + 1
        strip.fill((0,0,0))
        strip[down_index] = colours["green"]
        strip[up_index] = colours["green"]
        down_index -= 1
        up_index += 1
        strip.show()
        time.sleep(0.1)        

        
        
        
def led_down(index, block_size, sleep_time, colours):
    j = 0
    train = 1
    train_max = 1
    #index will always equal the distance from start
    while index > 0:
        print("index:")
        print(index)
        strip.fill((0,0,0))
        
        for i in range(len(colours)):
            c = colours[i]
        
        
        
        for i in range(block_size):
            print("this is i")
            print(i)
            j = i
            #if index - i < 0:
                #print("gothere")
                #block_size -= 1
                #j+=1
            print(10 - i)
            strip[index - j] = colours["green"]
        index -= 1
        train += 1
        strip.show() 
        time.sleep(0.1)  
         

def rainbow_train(colours, block_size, index, sleep_time):
    total_length = index
    train_size = 1
    
    while not NEW_TARGET:
        print("hello")
        train_size = min(train_size, total_length)
        start = (index - train_size + 1) % total_length
        
        strip.fill((0,0,0))
        colour_index = 0
        for i in range(train_size):
            insert_index = (start + i) % total_length
            strip[insert_index] = colours[colour_index]
            
            if(i + 1) % block_size == 0:
                colour_index = (colour_index + 1) % len(colours)
        
        strip.show()
        if train_size < total_length:
            train_size += 1
        time.sleep(sleep_time)

import time
import time

def rainbow_train2(colours, block_size, index, sleep_time):
    total_length = len(strip)  # Get LED strip length
    train_size = 1  # Initial size of the train
    
    while not NEW_TARGET:  # Runs until a new hand position is detected
        strip.fill((0, 0, 0))  # Clear the strip
        
        start = index  # Use the given hand position as the start
        colour_index = 0  # Start with first color
        
        for i in range(train_size):
            insert_index = (start - i) % total_length  # Move left and wrap around
            strip[insert_index] = colours[colour_index]  # Assign color
            
            if (i + 1) % block_size == 0:  # Change color after block size
                colour_index = (colour_index + 1) % len(colours)
        
        strip.show()  # Update LEDs

        index = (index - 1) % total_length  # Move the train left, wrapping around
        
        if train_size < total_length:
            train_size += 1  # Expand train size until it fills the strip
        
        time.sleep(sleep_time)


def shift_led(index):
    for i in range(1,index):
        strip[i] = strip[i+1]
    for i in range(LED_COUNT - 1,index, -1):
        strip[i] = strip[i-1]
    
def my_rainbow2(colours, block_size, index, sleep_time):
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
        
        
    
        
def my_rainbow(colours, block_size, index, sleep_time):
    train_size = 1
    start = index
    
    
    while not NEW_TARGET:
        strip.fill(0,0,0)
        curIndex = index
        colourIndex = 0
        curBlock = 0
        
        strip[index] = (255,255,255)
        
        for i in range(train_size):
            curIndex = index - curBlock
            strip[curIndex] = colours[colourIndex]
            curBlockSize += 1
            if curBlockSize > block_size:
                colourIndex = colour_index + 1 % len(colours)
                curBlockSize = 0
        strip.show()
        
            
            
        
        
    
        
        
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
        curent_distance = 0
        prev_distance = 0
        current_time = 0
        prev_time = 0
        while True:
            if not ser.is_open:
                ser.open()  # Open the serial port if it's closed

            distance = read_data()  # Read distance from the sensor
            if distance is not None and distance <= TOTAL_DISTANCE:
                
                # Map distance to LED index
                led_location = int(distance // UNIT_DISTANCE)
                #led_location = max(0, min(LED_COUNT - 1, led_location))  # Clamp to valid range
                print(f"Target LED: {led_location}")

                # Create a ripple effect at the target index
                #ripple_effect(led_location)
                #ripple_rainbow(led_location)
                #led_down(led_location , 4)
                
                #rainbow_train2([colours["red"], colours["orange"], colours["green"]], 5, led_location, 0.1)
                my_rainbow2([colours["red"], colours["orange"], colours["green"]], 5, 15, 0.1)

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
