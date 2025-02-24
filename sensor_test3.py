import neopixel
import time
import keyboard

# LED strip configuration
LED_COUNT = 35         # Number of LEDs in your strip
LED_PIN = board.D18    # GPIO pin connected to the LED strip (must support PWM)
LED_BRIGHTNESS = 0.5   # Brightness (0.0 to 1.0)
LED_ORDER = neopixel.GRB  # Color order of the LEDs

# Initialize the LED strip
strip = neopixel.NeoPixel(LED_PIN, LED_COUNT, brightness=LED_BRIGHTNESS, auto_write=False, pixel_order=LED_ORDER)

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
        
def keyboardLED(color, index):
    event = keyboard.read_event()
    if event.event_type == keyboard.KEY_DOWN:
        if event.name == "d" and index > 0:
            index-=1
            print("go right")
        elif event.name == "a" and index < LED_COUNT:
            index+=1
            print("go left")
        strip.fill((0,255,50))
        strip[index] = color
        strip.show()
    return index
try:
    event = keyboard.read_event()
    while True:
        print(event)
      

except KeyboardInterrupt:
    # Turn off all LEDs on exit
    set_color((0, 0, 0))
