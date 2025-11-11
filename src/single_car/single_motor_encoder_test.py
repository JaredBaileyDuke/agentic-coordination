# single_motor_encoder_test_right.py
from gpiozero import Robot, Motor, Button
from time import sleep, time
import threading
from threading import Event

# ======= USER SETTINGS =======
ENCODER_PIN = 17           # <-- BCM pin for RIGHT encoder signal
PULSES_PER_REV = 20
GEAR_RATIO = 1.0
PRINT_INTERVAL = 0.5
TEST_SPEEDS = [0.3, 0.5, 0.7]
STEP_TIME = 3.0
# =============================

# After your Option A swap (names match physical sides)
robot = Robot(
    left=Motor(forward=25, backward=5, enable=19, pwm=True),   # physical LEFT
    right=Motor(forward=24, backward=23, enable=18, pwm=True)  # physical RIGHT
)

# Encoder input (open-collector encoders usually need pull_up=True)
encoder = Button(ENCODER_PIN, pull_up=True, bounce_time=0.001)

tick_count = 0
last_print_time = time()
last_print_ticks = 0
drive_direction = 0  # -1 backward, 0 stop, +1 forward
lock = threading.Lock()
stop_event = Event()

def on_tick():
    global tick_count
    with lock:
        tick_count += 1

encoder.when_pressed = on_tick   # rising only; leave when_released unset to avoid warning

def set_direction(sign: int):
    global drive_direction
    drive_direction = sign

def telemetry_loop():
    global last_print_time, last_print_ticks
    while not stop_event.is_set():
        now = time()
        if now - last_print_time >= PRINT_INTERVAL:
            with lock:
                ticks_now = tick_count
            dt = max(now - last_print_time, 1e-6)
            d_ticks = ticks_now - last_print_ticks
            ticks_per_sec = d_ticks / dt

            revs_per_sec_motor = ticks_per_sec / max(PULSES_PER_REV, 1e-9)
            revs_per_sec_wheel = revs_per_sec_motor / max(GEAR_RATIO, 1e-9)
            rpm = revs_per_sec_wheel * 60.0
            signed_rpm = rpm * drive_direction

            print(f"[{now:.2f}] ticks_total={ticks_now} Δ={d_ticks} | "
                  f"ticks/s={ticks_per_sec:6.2f} | RPM={rpm:6.2f} | "
                  f"signedRPM={signed_rpm:6.2f} | dir="
                  f"{'FWD' if drive_direction>0 else 'REV' if drive_direction<0 else 'STOP'}")

            last_print_time = now
            last_print_ticks = ticks_now
        sleep(0.02)

telemetry_thread = threading.Thread(target=telemetry_loop)
telemetry_thread.start()

def drive_right(speed, direction, duration_s):
    """Drive ONLY the RIGHT side; keep left stopped to isolate one wheel."""
    set_direction(direction)

    # explicitly stop the LEFT side so only right moves
    robot.left_motor.stop()
    rm = robot.right_motor

    if direction > 0:
        rm.forward(speed=speed)
    elif direction < 0:
        rm.backward(speed=speed)
    else:
        rm.stop()

    sleep(duration_s)
    rm.stop()
    set_direction(0)

try:
    print("\n=== Single Motor + Encoder Sanity Test (RIGHT side) ===")
    print("Left motor will remain STOPPED.\n")

    # Ensure left is stopped
    robot.left_motor.stop()

    # Forward steps
    for sp in TEST_SPEEDS:
        print(f"FORWARD at {int(sp*100)}% for {STEP_TIME}s")
        drive_right(sp, +1, STEP_TIME)

    # Stop to settle
    print("STOP for 2s")
    drive_right(0.0, 0, 2.0)

    # Backward steps
    for sp in TEST_SPEEDS:
        print(f"REVERSE at {int(sp*100)}% for {STEP_TIME}s")
        drive_right(sp, -1, STEP_TIME)

    print("STOP for 2s")
    drive_right(0.0, 0, 2.0)

except KeyboardInterrupt:
    print("KeyboardInterrupt: stopping...")
finally:
    set_direction(0)
    robot.stop()
    stop_event.set()
    telemetry_thread.join(timeout=2.0)
    encoder.close()
    print("Done.")
