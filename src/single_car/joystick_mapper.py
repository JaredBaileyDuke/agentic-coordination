#!/usr/bin/env python3
"""
joystick_mapper.py
Discover your joystick's axis / button / hat indices with pygame.
- Move sticks, pull triggers, press buttons, click D-pad (hat).
- Press the controller's 'Start/Options' (or hold any button 3s) to quit.
"""

import time
import pygame

# Print only when values change beyond these:
AXIS_EPS = 0.02       # smaller = more sensitive printing
AXIS_ROUND = 2        # round axes to 2 decimals for readability

def main():
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        raise RuntimeError("No joystick detected. Plug it in and try again.")

    js = pygame.joystick.Joystick(0)
    js.init()

    name = js.get_name()
    n_axes = js.get_numaxes()
    n_buttons = js.get_numbuttons()
    n_hats = js.get_numhats()

    print("=" * 60)
    print(f"Using: {name}")
    print(f"Axes   : {n_axes}")
    print(f"Buttons: {n_buttons}")
    print(f"Hats   : {n_hats}")
    print("- Move a stick, squeeze a trigger, press buttons, use D-pad.")
    print("- Values will print only when they change.")
    print("- Press 'Start/Options' to quit.")
    print("=" * 60)

    # Track last values to print only on change
    last_axes = [0.0] * n_axes
    last_buttons = [0] * n_buttons
    last_hats = [(0, 0)] * n_hats

    # Try to guess a common quit button (Start/Options)
    # If your pad doesn't have BTN_START on index 9, you’ll still be able to quit
    # by holding any button for ~3 seconds (fallback below).
    GUESS_QUIT_BUTTON = 9 if n_buttons > 9 else None

    any_button_down_since = None
    clock = pygame.time.Clock()

    try:
        while True:
            for event in pygame.event.get():
                pass  # we just poll the device directly below

            # --- Axes ---
            for i in range(n_axes):
                val = js.get_axis(i)
                if abs(val - last_axes[i]) >= AXIS_EPS:
                    last_axes[i] = val
                    print(f"AXIS[{i:>2}] = {round(val, AXIS_ROUND)}")

            # --- Buttons ---
            for i in range(n_buttons):
                val = js.get_button(i)
                if val != last_buttons[i]:
                    last_buttons[i] = val
                    state = "DOWN" if val else "UP"
                    print(f"BUTTON[{i:>2}] {state}")

            # Quit on guessed Start/Options
            if GUESS_QUIT_BUTTON is not None and last_buttons[GUESS_QUIT_BUTTON]:
                print("Start/Options detected → quitting.")
                break

            # Fallback quit: hold ANY button for ~3 seconds
            if any(last_buttons):
                if any_button_down_since is None:
                    any_button_down_since = time.time()
                elif time.time() - any_button_down_since > 3.0:
                    print("Button held ~3s → quitting.")
                    break
            else:
                any_button_down_since = None

            # --- Hats (D-pad) ---
            for i in range(n_hats):
                val = js.get_hat(i)  # tuple (x, y), each in {-1, 0, +1}
                if val != last_hats[i]:
                    last_hats[i] = val
                    print(f"HAT[{i:>2}] = {val}")

            clock.tick(120)  # fast enough to feel responsive without spamming
    finally:
        pygame.quit()

if __name__ == "__main__":
    main()
