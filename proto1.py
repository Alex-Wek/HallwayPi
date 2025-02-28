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

TOTAL_DISTANCE = 57
UNIT_DISTANCE = TOTAL_DISTANCE / LED_COUNT
colours = {"off" : (0,0,0),"green" : (255,0,0)}


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

def set_color(color):
    """Set all LEDs to the same color."""
    for i in range(LED_COUNT):
        strip[i] = color
    strip.show()
#######################################################
def led_track(d,c):
    if d is not None and d <= TOTAL_DISTANCE:
        led_location = int(d // UNIT_DISTANCE)
        strip.fill(colours["off"])
        strip[led_location] = c
        strip.show()

######################################################
def wheel(pos):
    """Generate rainbow colors across 0-255 positions."""
    if pos < 85:
        return (255 - pos * 3, pos * 3, 0)
    elif pos < 170:
        pos -= 85
        return (0, 255 - pos * 3, pos * 3)
    else:
        pos -= 170
        return (pos * 3, 0, 255 - pos * 3)
        
def ripple_effect(target_index):
    """Create a rainbow ripple effect radiating from the target index."""
    # Set the target LED to white
    strip[target_index] = (255, 255, 255)
    strip.show()

    # Ripple outward from the target
    for i in range(1, LED_COUNT):
        # Calculate the ripple positions
        left = target_index - i
        right = target_index + i

        # Stop if the ripple reaches the ends of the strip
        if left < 0 and right >= LED_COUNT:
            break

        # Fade the ripple color
        fade = 255 - (i * 20)  # Adjust fade speed
        if fade < 0:
            fade = 0

        # Set the ripple colors
        if left >= 0:
            strip[left] = wheel((target_index - left) * 255 // LED_COUNT)
            strip[left] = tuple(int(c * fade / 255) for c in strip[left])
        if right < LED_COUNT:
            strip[right] = wheel((right - target_index) * 255 // LED_COUNT)
            strip[right] = tuple(int(c * fade / 255) for c in strip[right])

        # Update the strip
        strip.show()
        #time.sleep(0.05)  # Adjust ripple speed

    # Fade out the ripple
    for i in range(255, -1, -1):
        for led in range(LED_COUNT):
            if strip[led] != (255, 255, 255):  # Don't fade the white LED
                strip[led] = tuple(int(c * i / 255) for c in strip[led])
        strip.show()
        time.sleep(0.01)  # Adjust fade-out speed
    
 
if __name__ == "__main__":
    try:
        while True:
            if ser.isOpen() == False:
                ser.open()
            distance = read_data()
            if distance is not None and distance < TOTAL_DISTANCE:
                #led_track(distance, colours["green"])
               
                led_location = int(distance // UNIT_DISTANCE)
                print(led_location-1)
                ripple_effect(led_location)


            
    except KeyboardInterrupt(): # ctrl + c in terminal.
        set_color((0,0,0))
        if ser != None:
            ser.close()
            print("program interrupted by the user")

