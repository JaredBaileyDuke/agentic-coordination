from gpiozero import Robot, Motor, Button
from time import sleep, time
import threading

# ==== USER SETTINGS ====
LEFT_ENC_PIN  = 27       # BCM
RIGHT_ENC_PIN = 17       # BCM
PULSES_PER_REV = 20
GEAR_RATIO = 1.0
PRINT_INTERVAL = 1.0
# =======================

# Encoders (pull_up=True is typical for open-collector)
enc_left  = Button(LEFT_ENC_PIN,  pull_up=True, bounce_time=0.001)
enc_right = Button(RIGHT_ENC_PIN, pull_up=True, bounce_time=0.001)

robot = Robot(
    left=Motor(forward=24, backward=23, enable=18, pwm=True),
    right=Motor(forward=25, backward=5,  enable=19, pwm=True)
)

# Shared state
lock = threading.Lock()
tick_L = 0
tick_R = 0
last_print_time = time()
last_L = 0
last_R = 0
drive_sign = 0  # +1 forward, -1 reverse, 0 stop

def on_tick_left():
    global tick_L
    with lock: tick_L += 1

def on_tick_right():
    global tick_R
    with lock: tick_R += 1

enc_left.when_pressed  = on_tick_left
enc_right.when_pressed = on_tick_right
enc_left.when_released = None
enc_right.when_released = None

def set_sign(s):  # convenience
    global drive_sign
    drive_sign = s

def telemetry_loop():
    global last_print_time, last_L, last_R
    try:
        while True:
            now = time()
            if now - last_print_time >= PRINT_INTERVAL:
                with lock:
                    L = tick_L
                    R = tick_R
                dt = max(now - last_print_time, 1e-6)
                dL = L - last_L
                dR = R - last_R

                tpsL = dL / dt
                tpsR = dR / dt

                rps_m_L = tpsL / PULSES_PER_REV
                rps_m_R = tpsR / PULSES_PER_REV
                rps_w_L = rps_m_L / GEAR_RATIO
                rps_w_R = rps_m_R / GEAR_RATIO

                rpm_L = rps_w_L * 60.0
                rpm_R = rps_w_R * 60.0

                print(f"[{now:.2f}] L: ticks={L} d={dL} tps={tpsL:.1f} RPM={rpm_L:.1f} | "
                      f"R: ticks={R} d={dR} tps={tpsR:.1f} RPM={rpm_R:.1f} | dir={'FWD' if drive_sign>0 else 'REV' if drive_sign<0 else 'STOP'}")

                last_print_time = now
                last_L, last_R = L, R
            sleep(0.01)
    except KeyboardInterrupt:
        pass

import threading
threading.Thread(target=telemetry_loop, daemon=True).start()

try:
    while True:
        print("Forward at 70%")
        set_sign(+1)
        robot.forward(0.7)
        sleep(5)

        print("Backward at 70%")
        set_sign(-1)
        robot.backward(0.7)
        sleep(5)
except KeyboardInterrupt:
    pass
finally:
    set_sign(0)
    robot.stop()
