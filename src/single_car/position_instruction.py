# drive_to_pose_dualwheel.py
# Two-phase controller:
#   Phase 1: Drive to (x, y)
#   Phase 2: Rotate to heading theta_deg
#
# Works with single-channel encoders, odometry, and PWM motors.

from gpiozero import Robot, Motor, Button
from time import sleep, time
import math
import threading
import sys

# ================== USER SETTINGS ==================
robot = Robot(
    left=Motor(forward=25, backward=5, enable=19, pwm=True),
    right=Motor(forward=24, backward=23, enable=18, pwm=True)
)

ENCODER_L_A_PIN = 27
ENCODER_R_A_PIN = 17

WHEEL_DIAMETER_L_M = 0.06670
WHEEL_DIAMETER_R_M = 0.06701
GEAR_RATIO = 1.0
PULSES_PER_REV = 20
EDGES_PER_PULSE = 1

WHEEL_BASE_M = 0.135

DT = 0.02
PRINT_INTERVAL = 0.25

MAX_PWM = 0.90
MIN_PWM = 0.20       # reduced for smoother behavior
CREEP_PWM = 0.25     # lower creep for fine control
KICK_SPEED = 0.80
KICK_TIME  = 0.10

# Position gain and heading gain
K_RHO   = 1.0       # move speed proportional to distance
K_ALPHA = 2.0       # turning while driving

RHO_TOL_M = 0.10     # stop driving within 10 cm
TH_TOL_DEG = 10.0    # stop rotating within 10°

MAX_RUNTIME_S = 60.0

TRIM_L = 0.92
TRIM_R = 1.00

# ===================================================

def clamp(v, lo, hi):
    return max(min(v, hi), lo)

def wrap_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

class SimpleEncoder:
    def __init__(self, pin, pull_up=True, bounce_time=0.001):
        self.btn = Button(pin, pull_up=pull_up, bounce_time=bounce_time)
        self.ticks = 0
        self.lock = threading.Lock()
        self.btn.when_pressed = self._on_edge

    def _on_edge(self):
        with self.lock:
            self.ticks += 1

    def read(self):
        with self.lock:
            return self.ticks

    def reset(self):
        with self.lock:
            self.ticks = 0

    def close(self):
        self.btn.close()

def pwm_from_velocity(v_left, v_right, v_scale=1.0):
    def to_pwm(u, trim):
        u *= v_scale
        u = clamp(u, -1.0, 1.0)
        if abs(u) < 1e-6:
            return 0.0
        sign = 1.0 if u > 0 else -1.0
        mag = MIN_PWM + (MAX_PWM - MIN_PWM) * abs(u)
        return sign * mag * trim
    return to_pwm(v_left, TRIM_L), to_pwm(v_right, TRIM_R)

# ===================================================
# MAIN TWO-PHASE CONTROLLER
# ===================================================

def main():
    # -------------------------------------------------------
    # Parse input
    # -------------------------------------------------------
    if len(sys.argv) < 4:
        print("Usage: python drive_to_pose_dualwheel.py x y theta_deg")
        return

    goal_x = float(sys.argv[1])
    goal_y = float(sys.argv[2])
    goal_th_deg = float(sys.argv[3])
    goal_th = wrap_pi(math.radians(goal_th_deg % 360.0))

    # New run → reset PWM cache
    main._last_pwm = (0.0, 0.0)

    # -------------------------------------------------------
    # Setup encoders + precompute ticks-per-meter
    # -------------------------------------------------------
    encL = SimpleEncoder(ENCODER_L_A_PIN)
    encR = SimpleEncoder(ENCODER_R_A_PIN)

    circL = math.pi * WHEEL_DIAMETER_L_M
    circR = math.pi * WHEEL_DIAMETER_R_M
    ticks_per_rev = PULSES_PER_REV * EDGES_PER_PULSE * GEAR_RATIO
    tpmL = ticks_per_rev / circL
    tpmR = ticks_per_rev / circR

    x = 0.0
    y = 0.0
    th = 0.0

    sleep(0.2)
    encL.reset(); encR.reset()
    lastL = 0; lastR = 0

    print(f"Driving to ({goal_x:.3f}, {goal_y:.3f}), then rotate to {goal_th_deg:.1f}°")

    phase = 1  # 1 = translate, 2 = rotate

    start = time()
    last_print = start

    # -------------------------------------------------------
    # BEGIN CONTROL LOOP
    # -------------------------------------------------------
    try:
        while True:

            # TIMEOUT SAFETY
            if time() - start > MAX_RUNTIME_S:
                print("Timeout.")
                break
            
            # ---------------------------------------
            # ENCODER + ODOMETRY UPDATE
            # ---------------------------------------
            currL = encL.read()
            currR = encR.read()

            dL_ticks = currL - lastL
            dR_ticks = currR - lastR
            lastL = currL
            lastR = currR

            dL = dL_ticks / tpmL
            dR = dR_ticks / tpmR

            pwmL_cmd, pwmR_cmd = main._last_pwm
            signL = 1 if pwmL_cmd >= 0 else -1
            signR = 1 if pwmR_cmd >= 0 else -1

            dL_signed = dL * signL
            dR_signed = dR * signR

            ds = (dR_signed + dL_signed) * 0.5
            dth = (dR_signed - dL_signed) / WHEEL_BASE_M

            if abs(dth) < 1e-6:
                x += ds * math.cos(th)
                y += ds * math.sin(th)
            else:
                R_icc = ds / dth
                th_new = th + dth
                x += R_icc * (math.sin(th_new) - math.sin(th))
                y -= R_icc * (math.cos(th_new) - math.cos(th))
                th = th_new

            th = wrap_pi(th)

            # ---------------------------------------
            # ERROR CALCULATION
            # ---------------------------------------
            dx = goal_x - x
            dy = goal_y - y
            rho = math.hypot(dx, dy)
            target_heading = math.atan2(dy, dx)
            heading_err = wrap_pi(goal_th - th)
            heading_err_deg = abs(math.degrees(heading_err))

            # ---------------------------------------
            # PHASE 1: DRIVE TO POSITION
            # ---------------------------------------
            if phase == 1:

                if rho <= RHO_TOL_M:
                    print("Reached position. Proceeding to rotation phase.")
                    robot.stop()
                    sleep(0.15)
                    phase = 2
                    continue

                # steering angle
                alpha = wrap_pi(target_heading - th)

                v = K_RHO * rho
                w = K_ALPHA * alpha

                # scale down near goal
                v_scale = 1.0
                if rho < 0.30: v_scale = 0.6
                if rho < 0.15: v_scale = 0.4

                vL = v - (WHEEL_BASE_M * 0.5) * w
                vR = v + (WHEEL_BASE_M * 0.5) * w

                vmax = max(abs(vL), abs(vR), 1e-6)
                vL /= vmax
                vR /= vmax

                pwmL, pwmR = pwm_from_velocity(vL, vR, v_scale=v_scale)

                # creep near end
                if rho < 0.15:
                    pwmL = math.copysign(max(abs(pwmL), CREEP_PWM), pwmL)
                    pwmR = math.copysign(max(abs(pwmR), CREEP_PWM), pwmR)

            # ---------------------------------------
            # PHASE 2: ROTATE TO HEADING
            # ---------------------------------------
            else:
                if heading_err_deg <= TH_TOL_DEG:
                    print("Rotation complete.")
                    robot.stop()
                    sleep(0.15)
                    break

                turn_speed = 0.35
                if heading_err > 0:
                    pwmL = -turn_speed
                    pwmR = +turn_speed
                else:
                    pwmL = +turn_speed
                    pwmR = -turn_speed

            # ---------------------------------------
            # SEND MOTOR COMMANDS
            # ---------------------------------------
            if pwmL >= 0: robot.left_motor.forward(abs(pwmL))
            else:         robot.left_motor.backward(abs(pwmL))

            if pwmR >= 0: robot.right_motor.forward(abs(pwmR))
            else:         robot.right_motor.backward(abs(pwmR))

            main._last_pwm = (pwmL, pwmR)

            # ---------------------------------------
            # TELEMETRY
            # ---------------------------------------
            if time() - last_print >= PRINT_INTERVAL:
                print(f"[phase {phase}] x={x:.3f}, y={y:.3f}, th={math.degrees(th):.1f}° | "
                      f"rho={rho:.2f}, heading_err={heading_err_deg:.1f}° | "
                      f"pwmL={pwmL:.2f}, pwmR={pwmR:.2f}")
                last_print = time()

            sleep(DT)

    finally:
        robot.stop()
        encL.close()
        encR.close()
        print("Done.")

# ===================================================

if __name__ == "__main__":
    main()
