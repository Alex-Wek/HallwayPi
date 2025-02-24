import board
import neopixel
import time
import keyboard
import random

# LED strip configuration
LED_COUNT = 35         # Number of LEDs in your strip
LED_PIN = board.D18    # GPIO pin connected to the LED strip (must support PWM)
LED_BRIGHTNESS = 0.5   # Brightness (0.0 to 1.0)
LED_ORDER = neopixel.GRB  # Color order of the LEDs

bg = (200,40,200)
target = (40,255,0)
index = 0

# Initialize the LED strip
strip = neopixel.NeoPixel(LED_PIN, LED_COUNT, brightness=LED_BRIGHTNESS, auto_write=False, pixel_order=LED_ORDER)

strip.fill(bg)
strip.show()

try:
	
	while True:
		index = random.randint(0,LED_COUNT -1)
		strip[index] = target
		strip.show()
		time.sleep(0.3)

	
