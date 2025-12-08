# encoder_edge_probe.py
from gpiozero import Button
from time import sleep, time

PIN = 27  # BCM (GPIO27 = physical pin 13)
enc = Button(PIN, pull_up=True, bounce_time=None)  # start with no debounce

def pressed():
    print(f"{time():.3f} PRESS")

def released():
    print(f"{time():.3f} RELEASE")

enc.when_pressed = pressed     # active -> with pull_up=True, 'pressed' = LOW
enc.when_released = released   # back to HIGH
print("Rotate wheel slowly; CTRL+C to exit")
try:
    while True:
        sleep(0.1)
except KeyboardInterrupt:
    pass
