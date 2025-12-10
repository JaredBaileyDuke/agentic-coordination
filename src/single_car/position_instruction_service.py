# drive_to_pose_dualwheel.py
# Go to (x, y, theta_deg) using dual-wheel encoders (BCM numbering) + odometry.
#  - Uses your per-wheel diameter calibration
#  - Uses a unicycle "go-to-pose" controller (polar form) with velocity -> wheel PWM mapping
#  - Works with single-channel encoders by inferring sign from commanded wheel direction

from gpiozero import Robot, Motor, Button
from time import sleep, time
import math
import threading
import sys

# ================== USER SETTINGS ==================
# Motor wiring (names match physical sides)
robot = Robot(
    left=Motor(forward=25, backward=5, enable=19, pwm=True),   # physical LEFT
    right=Motor(forward=24, backward=23, enable=18, pwm=True)  # physical RIGHT
)

# Encoder pins (A channels)
ENCODER_L_A_PIN = 27  # LEFT encoder A (BCM)
ENCODER_R_A_PIN = 17  # RIGHT encoder A (BCM)

# Wheel + encoder characteristics (from your calibration)
WHEEL_DIAMETER_L_M = 0.06670     # LEFT effective diameter (meters)
WHEEL_DIAMETER_R_M = 0.06701     # RIGHT effective diameter (meters)
GEAR_RATIO = 1.0                 # wheel_rev = motor_rev / GEAR_RATIO
PULSES_PER_REV = 20              # pulses per motor-shaft rev (per channel)
EDGES_PER_PULSE = 1              # 1=rising only; set 2 if counting rising+falling

# Robot geometry (tune this!)
WHEEL_BASE_M = 0.135             # distance between wheel contact centers (meters)

# Control loop + speeds
DT = 0.02                        # control loop period (s)
PRINT_INTERVAL = 0.25            # telemetry print period (s)

# Motion scaling: map (v, w) -> wheel PWM.
# These are *PWM limits*, not m/s or rad/s. We'll scale "virtual" velocities into [-1, 1].
MAX_PWM = 0.90                   # hard cap for |wheel PWM|
MIN_PWM = 0.40                   # deadband helper (helps overcome static friction)
CREEP_PWM = 0.40                 # used when near goal
KICK_SPEED = 0.80                # brief higher speed “kick” to break static friction
KICK_TIME  = 0.12                # seconds

# Pose controller gains (polar go-to-pose: ρ, α, β)
# Stability rules: k_rho > 0, k_alpha > k_rho, k_beta < 0
K_RHO   = 1.20
K_ALPHA = 2.70
K_BETA  = -0.60

# Goal tolerances
RHO_TOL_M = 0.02                 # 2 cm position tolerance
TH_TOL_DEG = 3.0                 # within 3 degrees for final orientation

# Safety/timeouts
MAX_RUNTIME_S = 60.0             # bailout if something goes wrong

# Optional trims to equalize sides (your left slightly short -> +1.05%)
TRIM_L = 0.9200
TRIM_R = 1.0000
# ===================================================


def clamp(x, lo, hi):
    return hi if x > hi else lo if x < lo else x


def wrap_pi(angle_rad):
    """Wrap angle to (-pi, pi]."""
    a = (angle_rad + math.pi) % (2 * math.pi) - math.pi
    # Map -pi to +pi for consistency
    return a if a != -math.pi else math.pi


class SimpleEncoder:
    """Single-channel edge counter (A only). Direction inferred from commanded wheel sign."""
    def __init__(self, pin, pull_up=True, bounce_time=0.001):
        self.btn = Button(pin, pull_up=pull_up, bounce_time=bounce_time)
        self.ticks = 0
        self.lock = threading.Lock()
        self.btn.when_pressed = self._on_edge  # rising only
        if EDGES_PER_PULSE == 2:
            self.btn.when_released = self._on_edge

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
    """
    Map "virtual wheel velocities" (unitless) into PWM [-1, 1] with deadband handling.
    v_scale lets you globally scale aggressiveness (e.g., reduce near goal).
    """
    def to_pwm(u, trim):
        u *= v_scale
        # Saturate
        u = clamp(u, -1.0, 1.0)
        # Deadband lift
        if abs(u) < 1e-6:
            return 0.0
        sign = 1.0 if u > 0 else -1.0
        mag = MIN_PWM + (MAX_PWM - MIN_PWM) * min(abs(u), 1.0)
        return sign * mag * trim

    return to_pwm(v_left, TRIM_L), to_pwm(v_right, TRIM_R)


def main():
    # Parse goal from CLI or prompt
    # Usage: python drive_to_pose_dualwheel.py x y theta_deg
    if len(sys.argv) >= 4:
        goal_x = float(sys.argv[1])
        goal_y = float(sys.argv[2])
        goal_th_deg = float(sys.argv[3])
    else:
        print("Enter goal pose as: x y theta_deg  (meters, meters, degrees; 0/360° is +x)")
        txt = input("Target (x y theta_deg) > ").strip()
        parts = txt.split()
        if len(parts) != 3:
            print("Invalid input. Example: 1.0 0.0 0")
            return
        goal_x, goal_y, goal_th_deg = map(float, parts)
    goal_th = math.radians(goal_th_deg % 360.0)
    goal_th = wrap_pi(goal_th)  # to (-pi, pi]

    # Encoders
    encL = SimpleEncoder(ENCODER_L_A_PIN)
    encR = SimpleEncoder(ENCODER_R_A_PIN)

    # Precompute ticks-per-meter each side
    circL = math.pi * WHEEL_DIAMETER_L_M
    circR = math.pi * WHEEL_DIAMETER_R_M
    ticks_per_rev = PULSES_PER_REV * EDGES_PER_PULSE * GEAR_RATIO
    tpmL = ticks_per_rev / circL
    tpmR = ticks_per_rev / circR

    print(f"\n=== Go to (x, y, theta_deg) — pigpio encoders & calibration ===")
    print(f"Goal: x={goal_x:.3f} m, y={goal_y:.3f} m, θ={goal_th_deg:.1f}°")
    print(f"LEFT : D={WHEEL_DIAMETER_L_M*1000:.2f} mm | C={circL:.5f} m | ticks/m={tpmL:.2f}")
    print(f"RIGHT: D={WHEEL_DIAMETER_R_M*1000:.2f} mm | C={circR:.5f} m | ticks/m={tpmR:.2f}")
    print(f"Wheel base: {WHEEL_BASE_M:.3f} m\n")

    # Odometry state (start assumed at origin, facing +x)
    x = 0.0
    y = 0.0
    th = 0.0  # radians, 0 = +x direction

    # Reset encoder counters
    sleep(0.2)
    encL.reset(); encR.reset()

    # Helper to track last tick counts for odom deltas
    lastL = 0
    lastR = 0

    # Start with a static-friction kick in the general direction of the first move
    # We'll aim towards (goal_x, goal_y)
    vec0 = math.atan2(goal_y - y, goal_x - x)
    heading_err0 = wrap_pi(vec0 - th)
    # brief in-place turn kick if initial bearing way off
    if abs(heading_err0) > math.radians(30):
        left_pwm = -KICK_SPEED if heading_err0 > 0 else KICK_SPEED
        right_pwm = KICK_SPEED if heading_err0 > 0 else -KICK_SPEED
        robot.left_motor.value = left_pwm
        robot.right_motor.value = right_pwm
        sleep(KICK_TIME * 0.7)
        robot.stop()

    # brief forward kick to get rolling
    robot.left_motor.forward(KICK_SPEED)
    robot.right_motor.forward(KICK_SPEED)
    sleep(KICK_TIME)
    robot.stop()

    start = time()
    last_print = start

    # Controller loop
    try:
        while True:
            now = time()
            # Read encoder ticks
            currL = encL.read()
            currR = encR.read()

            dL_ticks = currL - lastL
            dR_ticks = currR - lastR
            lastL = currL
            lastR = currR

            # Convert to *unsigned* distances
            dL = dL_ticks / tpmL
            dR = dR_ticks / tpmR

            # Infer direction from last commanded wheel values (we'll cache them)
            # We store last commanded PWM to sign the odometry
            # If no cache yet, assume forward
            if not hasattr(main, "_last_pwm"):
                main._last_pwm = (0.0, 0.0)
            pwmL_cmd, pwmR_cmd = main._last_pwm
            signL = 1.0 if pwmL_cmd >= 0 else -1.0
            signR = 1.0 if pwmR_cmd >= 0 else -1.0

            dL_signed = dL * signL
            dR_signed = dR * signR

            # Differential-drive odometry
            ds = 0.5 * (dR_signed + dL_signed)
            dth = (dR_signed - dL_signed) / WHEEL_BASE_M
            # Integrate pose (exact arc update)
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

            # Compute polar errors to goal
            dx = goal_x - x
            dy = goal_y - y
            rho = math.hypot(dx, dy)                   # distance to goal
            path_heading = math.atan2(dy, dx)          # direction to goal
            alpha = wrap_pi(path_heading - th)         # bearing error
            beta  = wrap_pi(goal_th - th - alpha)      # heading error at goal

            # Check termination
            th_err_deg = abs(math.degrees(wrap_pi(goal_th - th)))
            if rho <= RHO_TOL_M and th_err_deg <= TH_TOL_DEG:
                robot.stop()
                break

            # Controller (unicycle)
            # v: forward "virtual" speed, w: angular "virtual" speed
            v = K_RHO * rho
            w = K_ALPHA * alpha + K_BETA * beta

            # Scale down when close to goal
            v_scale = 1.0
            if rho < 0.15:
                v_scale = 0.6
            if rho < 0.07:
                v_scale = 0.45

            # Convert (v, w) to left/right "virtual" wheel velocities (unitless)
            # We don't have meters/sec calibration here, just relative splitting:
            vL = v - (WHEEL_BASE_M * 0.5) * w
            vR = v + (WHEEL_BASE_M * 0.5) * w

            # Normalize by a soft max so that max(|vL|, |vR|) ≈ 1 at high demand
            vmax = max(abs(vL), abs(vR), 1e-6)
            vL /= vmax
            vR /= vmax

            # Map to PWM with trims + deadband
            pwmL, pwmR = pwm_from_velocity(vL, vR, v_scale=v_scale)

            # If we're very close, creep with a small minimum to keep moving
            if rho < 0.10:
                def creep(p):
                    if p == 0.0:
                        return 0.0
                    return math.copysign(max(abs(p), CREEP_PWM), p)
                pwmL = creep(pwmL)
                pwmR = creep(pwmR)

            # Command motors and cache
            if pwmL >= 0:
                robot.left_motor.forward(abs(pwmL))
            else:
                robot.left_motor.backward(abs(pwmL))

            if pwmR >= 0:
                robot.right_motor.forward(abs(pwmR))
            else:
                robot.right_motor.backward(abs(pwmR))

            main._last_pwm = (pwmL, pwmR)

            # Telemetry
            if (now - last_print) >= PRINT_INTERVAL:
                print(f"x={x:5.3f} m, y={y:5.3f} m, th={math.degrees(th):6.2f}° | "
                      f"ρ={rho:4.2f} m, α={math.degrees(alpha):6.2f}°, β={math.degrees(beta):6.2f}° | "
                      f"pwmL={pwmL:+.2f}, pwmR={pwmR:+.2f}")
                last_print = now

            # Timeout safety
            if (now - start) > MAX_RUNTIME_S:
                print("Timeout reached; stopping.")
                break

            sleep(DT)

        # Stop & settle
        robot.stop()
        sleep(0.20)

        # Final report
        currL = encL.read(); currR = encR.read()
        # rough traveled distances (unsigned)
        distL = currL / tpmL
        distR = currR / tpmR
        avg  = 0.5 * (distL + distR)
        print(f"\nReached ~goal. "
              f"final pose: x={x:.3f} m, y={y:.3f} m, th={math.degrees(th):.1f}° "
              f"| wheel dists (unsigned): L={distL:.3f} m, R={distR:.3f} m, avg={avg:.3f} m "
              f"in {time()-start:.2f}s.")

    except KeyboardInterrupt:
        print("\nKeyboardInterrupt: stopping...")
    finally:
        robot.stop()
        encL.close()
        encR.close()
        print("Done.")


if __name__ == "__main__":
    main()
