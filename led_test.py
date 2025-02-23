import board
import neopixel
import time

# LED strip configuration
LED_COUNT = 30         # Number of LEDs in your strip
LED_PIN = board.D18    # GPIO pin connected to the LED strip (must support PWM)
LED_BRIGHTNESS = 0.5   # Brightness (0.0 to 1.0)
LED_ORDER = neopixel.GRB  # Color order of the LEDs

# Initialize the LED strip
strip = neopixel.NeoPixel(LED_PIN, LED_COUNT, brightness=LED_BRIGHTNESS, auto_write=False, pixel_order=LED_ORDER)
print("first check complete")
def set_color(color):
    """Set all LEDs to the same color."""
    for i in range(LED_COUNT):
        strip[i] = color
    strip.show()

def animate():
    """Simple animation example."""
    for i in range(LED_COUNT):
        strip[i] = (255, 0, 0)  # Red
        strip.show()
        time.sleep(0.1)
        strip[i] = (0, 0, 0)    # Off

try:
    while True:
        print("in the loop")
        # Example: Set all LEDs to green
        set_color((0, 255, 0))
        time.sleep(1)

        # Example: Run an animation
        animate()
        time.sleep(1)

except KeyboardInterrupt:
    # Turn off all LEDs on exit
    set_color((0, 0, 0))
