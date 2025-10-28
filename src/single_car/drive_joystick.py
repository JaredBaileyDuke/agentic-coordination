#!/usr/bin/env python3
import math
import pygame
from gpiozero import Robot, Motor

# --- Robot motor setup (your pins) ---
robot = Robot(
    left=Motor(forward=24, backward=23, enable=18, pwm=True),
    right=Motor(forward=25, backward=5,  enable=19, pwm=True)
)

# --- Joystick mapping from your output ---
AXIS_THROTTLE = 1   # forward/back on right stick Y
AXIS_TURN     = 2   # turn on right stick X

# --- Tunables ---
DEADZONE  = 0.12     # ignore tiny noise near center
MAX_SPEED = 0.9      # overall cap (0..1)
EXPONENT  = 1.0      # 1.0 = linear; 2.0 = softer near center
LOOP_HZ   = 50
STOP_BTN  = 1        # B
QUIT_BTN  = 9        # Start
BOOST_BTN = 0        # A -> temporary full power

def shape(x, dead=DEADZONE, exp=EXPONENT):
    """Deadzone + curve; preserves sign. Input/Output in [-1..1]."""
    if abs(x) < dead:
        return 0.0
    s = (abs(x) - dead) / (1.0 - dead)  # rescale to 0..1 after deadzone
    s = max(0.0, min(1.0, s)) ** exp
    return math.copysign(s, x)

def clamp(x, lo=-1.0, hi=1.0):
    return max(lo, min(hi, x))

def main():
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        raise RuntimeError("No joystick detected.")

    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"Using joystick: {js.get_name()}")

    clock = pygame.time.Clock()
    try:
        while True:
            for _ in pygame.event.get():
                pass

            # Proportional inputs from sticks
            raw_throttle = -js.get_axis(AXIS_THROTTLE)  # invert so up = +forward
            raw_turn     =  js.get_axis(AXIS_TURN)

            v = shape(raw_throttle)   # forward/back speed fraction
            w = shape(raw_turn)       # turning fraction

            # Differential mix (arcade drive)
            left  = clamp(v - w)
            right = clamp(v + w)

            # Scaling & buttons
            scale = MAX_SPEED
            if js.get_button(BOOST_BTN):
                scale = 1.0
            if js.get_button(STOP_BTN):
                left = right = 0.0
                scale = 0.0
            if js.get_button(QUIT_BTN):
                print("Quit button pressed. Stopping.")
                break

            robot.value = (left * scale, right * scale)

            # (Optional) telemetry every ~0.2s
            # if pygame.time.get_ticks() % 200 < 20:
            #     print(f"v={v:+.2f} w={w:+.2f}  L={left*scale:+.2f} R={right*scale:+.2f}")

            clock.tick(LOOP_HZ)

    except KeyboardInterrupt:
        print("KeyboardInterrupt — stopping.")
    finally:
        robot.stop()
        pygame.quit()

if __name__ == "__main__":
    main()
