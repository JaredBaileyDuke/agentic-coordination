from gpiozero import Robot, Motor, Button
from time import sleep, time
import threading

# ======= USER SETTINGS (edit these to match your hardware) =======
ENCODER_PIN = 17          # GPIO17 (BCM)
PULSES_PER_REV = 20       # <-- change to your encoder's counts-per-rev (CPR)
GEAR_RATIO = 1.0          # <-- if encoder is on motor shaft and wheel is geared, set gear ratio
PRINT_INTERVAL = 1.0      # seconds between telemetry prints
# ================================================================

# Encoder input (use pull_up=True for open-collector encoders)
encoder = Button(ENCODER_PIN, pull_up=True, bounce_time=0.001)

# Define robot with left and right motors
# NOTE: Swapped forward/backward pins so robot.forward() is truly forward.
# If only ONE side was reversed on your robot, swap pins only for that side.
robot = Robot(
    left=Motor(forward=24, backward=23, enable=18, pwm=True),   # swapped
    right=Motor(forward=25, backward=5, enable=19, pwm=True)    # swapped
)

# Shared counters and state
tick_count = 0
last_print_time = time()
last_print_ticks = 0
drive_direction = 0   # -1 backward, 0 stopped, +1 forward
lock = threading.Lock()

def on_tick():
    global tick_count
    with lock:
        tick_count += 1

# Count rising edges
encoder.when_pressed = on_tick
encoder.when_released = None

def set_direction(sign: int):
    global drive_direction
    drive_direction = sign

def print_telemetry_loop():
    global last_print_time, last_print_ticks
    try:
        while True:
            now = time()
            if now - last_print_time >= PRINT_INTERVAL:
                with lock:
                    ticks_now = tick_count
                dt = max(now - last_print_time, 1e-6)
                d_ticks = ticks_now - last_print_ticks
                ticks_per_sec = d_ticks / dt

                revs_per_sec_motor = ticks_per_sec / PULSES_PER_REV
                revs_per_sec_wheel = revs_per_sec_motor / GEAR_RATIO
                rpm = revs_per_sec_wheel * 60.0
                signed_rpm = rpm * drive_direction

                print(f"[{now:.2f}] Ticks: total={ticks_now} Δ={d_ticks} "
                      f"| ticks/s={ticks_per_sec:.2f} | RPM={rpm:.2f} | signedRPM={signed_rpm:.2f} "
                      f"| dir={'FWD' if drive_direction>0 else 'REV' if drive_direction<0 else 'STOP'}")

                last_print_time = now
                last_print_ticks = ticks_now
            sleep(0.01)
    except KeyboardInterrupt:
        pass

# Start telemetry in background
telemetry_thread = threading.Thread(target=print_telemetry_loop, daemon=True)
telemetry_thread.start()

try:
    while True:
        print("Forward at 70% speed")
        set_direction(+1)
        robot.forward(speed=0.7)
        sleep(5)

        print("Backward at 70% speed")
        set_direction(-1)
        robot.backward(speed=0.7)
        sleep(5)

except KeyboardInterrupt:
    print("Stopping...")
finally:
    set_direction(0)
    robot.stop()
