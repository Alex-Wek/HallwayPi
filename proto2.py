import time
import serial
import neopixel
import board

# LED strip configuration
LED_COUNT = 35         # Number of LEDs in your strip
LED_PIN = board.D18    # GPIO pin connected to the LED strip (must support PWM)
LED_BRIGHTNESS = 0.5   # Brightness (0.0 to 1.0)
LED_ORDER = neopixel.GRB  # Color order of the LEDs

# Initialize the LED strip
strip = neopixel.NeoPixel(LED_PIN, LED_COUNT, brightness=LED_BRIGHTNESS, auto_write=False, pixel_order=LED_ORDER)

TOTAL_DISTANCE = 57  # Maximum distance in cm
UNIT_DISTANCE = TOTAL_DISTANCE / LED_COUNT  # Distance per LED

# Serial configuration for TFMini-S sensor
ser = serial.Serial("/dev/serial0", 115200, timeout=1)

def read_data():
    """Read distance data from the TFMini-S sensor."""
    counter = ser.in_waiting  # Count the number of bytes in the serial buffer
    if counter > 8:
        bytes_serial = ser.read(9)
        ser.reset_input_buffer()

        if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59:  # Check for valid data header
            distance = bytes_serial[2] + bytes_serial[3] * 256  # Calculate distance
            strength = bytes_serial[4] + bytes_serial[5] * 256  # Signal strength (unused here)
            temperature = bytes_serial[6] + bytes_serial[7] * 256  # Temperature (unused here)
            temperature = (temperature / 8) - 256  # Convert temperature to Celsius
            print(f"Distance: {distance} cm, Strength: {strength}, Temperature: {temperature}°C")
            return distance
    return None

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
            return

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
        #time.sleep(0.02)  # Adjust ripple speed (reduced for faster updates)

    # Fade out the ripple
    #for i in range(255, -1, -1):
        #for led in range(LED_COUNT):
            #if strip[led] != (255, 255, 255):  # Don't fade the white LED
                #strip[led] = tuple(int(c * i / 255) for c in strip[led])
        #strip.show()
        #time.sleep(0.005)  # Adjust fade-out speed (reduced for faster updates)

if __name__ == "__main__":
    try:
        while True:
            if not ser.is_open:
                ser.open()  # Open the serial port if it's closed

            distance = read_data()  # Read distance from the sensor
            if distance is not None and distance <= TOTAL_DISTANCE:
                # Map distance to LED index
                led_location = int(distance // UNIT_DISTANCE)
                led_location = max(0, min(LED_COUNT - 1, led_location))  # Clamp to valid range
                print(f"Target LED: {led_location}")

                # Create a ripple effect at the target index
                ripple_effect(led_location)

            time.sleep(0.01)  # Small delay to avoid overwhelming the sensor

    except KeyboardInterrupt:
        # Turn off all LEDs on exit
        strip.fill((0, 0, 0))
        strip.show()
        if ser.is_open:
            ser.close()
        print("Program interrupted by the user")
