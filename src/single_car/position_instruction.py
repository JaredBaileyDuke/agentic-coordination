# drive_to_pose_twophase.py
# Two-phase go-to-pose using dual-wheel encoders + odometry:
#   PHASE 1: Navigate to (x, y) position (ignoring final heading)
#   PHASE 2: Rotate in place to desired theta
#
# Uses your per-wheel diameter calibration and unicycle controller

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
MAX_PWM = 0.90                   # hard cap for |wheel PWM|
MIN_PWM = 0.40                   # deadband helper (helps overcome static friction)
CREEP_PWM = 0.40                 # used when near goal
KICK_SPEED = 0.80                # brief higher speed "kick" to break static friction
KICK_TIME  = 0.12                # seconds

# PHASE 1: Position controller gains (ρ, α only - no β since we don't care about final heading yet)
K_RHO_P1   = 1.20
K_ALPHA_P1 = 2.70

# PHASE 2: Rotation controller gain
K_THETA_P2 = 1.80                # proportional gain for in-place rotation

# Goal tolerances
RHO_TOL_M = 0.08                 # position tolerance for Phase 1 (meters)
TH_TOL_DEG = 8.0                 # heading tolerance for Phase 2 (degrees)

# Safety/timeouts
MAX_RUNTIME_PHASE1_S = 45.0      # bailout for Phase 1
MAX_RUNTIME_PHASE2_S = 15.0      # bailout for Phase 2

# Optional trims to equalize sides
TRIM_L = 0.9200
TRIM_R = 1.0000
# ===================================================


def clamp(x, lo, hi):
    return hi if x > hi else lo if x < lo else x


def wrap_pi(angle_rad):
    """Wrap angle to (-pi, pi]."""
    a = (angle_rad + math.pi) % (2 * math.pi) - math.pi
    return a if a != -math.pi else math.pi


class SimpleEncoder:
    """Single-channel edge counter (A only). Direction inferred from commanded wheel sign."""
    def __init__(self, pin, pull_up=True, bounce_time=0.001):
        self.btn = Button(pin, pull_up=pull_up, bounce_time=bounce_time)
        self.ticks = 0
        self.lock = threading.Lock()
        self.btn.when_pressed = self._on_edge
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
    """
    def to_pwm(u, trim):
        u *= v_scale
        u = clamp(u, -1.0, 1.0)
        if abs(u) < 1e-6:
            return 0.0
        sign = 1.0 if u > 0 else -1.0
        mag = MIN_PWM + (MAX_PWM - MIN_PWM) * min(abs(u), 1.0)
        return sign * mag * trim

    return to_pwm(v_left, TRIM_L), to_pwm(v_right, TRIM_R)


def update_odometry(x, y, th, dL_signed, dR_signed):
    """Update pose using differential drive odometry."""
    ds = 0.5 * (dR_signed + dL_signed)
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
    return x, y, th


def command_motors(pwmL, pwmR):
    """Command motors with given PWM values."""
    if pwmL >= 0:
        robot.left_motor.forward(abs(pwmL))
    else:
        robot.left_motor.backward(abs(pwmL))
    
    if pwmR >= 0:
        robot.right_motor.forward(abs(pwmR))
    else:
        robot.right_motor.backward(abs(pwmR))


def phase1_navigate_to_position(goal_x, goal_y, encL, encR, tpmL, tpmR):
    """
    PHASE 1: Navigate to (x, y) position using position controller.
    Returns final pose (x, y, th) when position is reached.
    """
    print("\n=== PHASE 1: Navigate to position ===")
    print(f"Target: x={goal_x:.3f} m, y={goal_y:.3f} m")
    
    # Initialize pose
    x, y, th = 0.0, 0.0, 0.0
    lastL, lastR = 0, 0
    last_pwm = (0.0, 0.0)
    
    # Initial kick toward goal
    vec0 = math.atan2(goal_y - y, goal_x - x)
    heading_err0 = wrap_pi(vec0 - th)
    
    if abs(heading_err0) > math.radians(30):
        left_pwm = -KICK_SPEED if heading_err0 > 0 else KICK_SPEED
        right_pwm = KICK_SPEED if heading_err0 > 0 else -KICK_SPEED
        robot.left_motor.value = left_pwm
        robot.right_motor.value = right_pwm
        sleep(KICK_TIME * 0.7)
        robot.stop()
    
    robot.left_motor.forward(KICK_SPEED)
    robot.right_motor.forward(KICK_SPEED)
    sleep(KICK_TIME)
    robot.stop()
    
    start = time()
    last_print = start
    
    while True:
        now = time()
        
        # Read encoders and update odometry
        currL, currR = encL.read(), encR.read()
        dL_ticks, dR_ticks = currL - lastL, currR - lastR
        lastL, lastR = currL, currR
        
        dL = dL_ticks / tpmL
        dR = dR_ticks / tpmR
        
        pwmL_cmd, pwmR_cmd = last_pwm
        signL = 1.0 if pwmL_cmd >= 0 else -1.0
        signR = 1.0 if pwmR_cmd >= 0 else -1.0
        
        dL_signed = dL * signL
        dR_signed = dR * signR
        
        x, y, th = update_odometry(x, y, th, dL_signed, dR_signed)
        
        # Compute errors
        dx = goal_x - x
        dy = goal_y - y
        rho = math.hypot(dx, dy)
        path_heading = math.atan2(dy, dx)
        alpha = wrap_pi(path_heading - th)
        
        # Check if position reached
        if rho <= RHO_TOL_M:
            robot.stop()
            print(f"Position reached: x={x:.3f} m, y={y:.3f} m, th={math.degrees(th):.1f}°")
            break
        
        # Position controller (no beta term)
        v = K_RHO_P1 * rho
        w = K_ALPHA_P1 * alpha
        
        # Scale down when close
        v_scale = 1.0
        if rho < 0.15:
            v_scale = 0.6
        if rho < 0.07:
            v_scale = 0.45
        
        # Convert to wheel velocities
        vL = v - (WHEEL_BASE_M * 0.5) * w
        vR = v + (WHEEL_BASE_M * 0.5) * w
        
        vmax = max(abs(vL), abs(vR), 1e-6)
        vL /= vmax
        vR /= vmax
        
        pwmL, pwmR = pwm_from_velocity(vL, vR, v_scale=v_scale)
        
        # Creep when very close
        if rho < 0.10:
            def creep(p):
                if p == 0.0:
                    return 0.0
                return math.copysign(max(abs(p), CREEP_PWM), p)
            pwmL = creep(pwmL)
            pwmR = creep(pwmR)
        
        command_motors(pwmL, pwmR)
        last_pwm = (pwmL, pwmR)
        
        # Telemetry
        if (now - last_print) >= PRINT_INTERVAL:
            print(f"x={x:5.3f} m, y={y:5.3f} m, th={math.degrees(th):6.2f}° | "
                  f"ρ={rho:4.2f} m, α={math.degrees(alpha):6.2f}° | "
                  f"pwmL={pwmL:+.2f}, pwmR={pwmR:+.2f}")
            last_print = now
        
        # Timeout
        if (now - start) > MAX_RUNTIME_PHASE1_S:
            print("Phase 1 timeout reached")
            break
        
        sleep(DT)
    
    robot.stop()
    sleep(0.2)
    return x, y, th


def phase2_rotate_to_heading(goal_th, x, y, th, encL, encR, tpmL, tpmR):
    """
    PHASE 2: Rotate in place to desired heading.
    Returns final pose (x, y, th) when heading is reached.
    """
    print("\n=== PHASE 2: Rotate to heading ===")
    print(f"Current heading: {math.degrees(th):.1f}°, Target: {math.degrees(goal_th):.1f}°")
    
    lastL, lastR = encL.read(), encR.read()
    last_pwm = (0.0, 0.0)
    
    # Initial rotation kick
    heading_err = wrap_pi(goal_th - th)
    if abs(heading_err) > math.radians(15):
        left_pwm = -KICK_SPEED * 0.7 if heading_err > 0 else KICK_SPEED * 0.7
        right_pwm = KICK_SPEED * 0.7 if heading_err > 0 else -KICK_SPEED * 0.7
        robot.left_motor.value = left_pwm
        robot.right_motor.value = right_pwm
        sleep(KICK_TIME * 0.5)
        robot.stop()
    
    start = time()
    last_print = start
    
    while True:
        now = time()
        
        # Read encoders and update odometry
        currL, currR = encL.read(), encR.read()
        dL_ticks, dR_ticks = currL - lastL, currR - lastR
        lastL, lastR = currL, currR
        
        dL = dL_ticks / tpmL
        dR = dR_ticks / tpmR
        
        pwmL_cmd, pwmR_cmd = last_pwm
        signL = 1.0 if pwmL_cmd >= 0 else -1.0
        signR = 1.0 if pwmR_cmd >= 0 else -1.0
        
        dL_signed = dL * signL
        dR_signed = dR * signR
        
        x, y, th = update_odometry(x, y, th, dL_signed, dR_signed)
        
        # Compute heading error
        th_err = wrap_pi(goal_th - th)
        th_err_deg = abs(math.degrees(th_err))
        
        # Check if heading reached
        if th_err_deg <= TH_TOL_DEG:
            robot.stop()
            print(f"Heading reached: th={math.degrees(th):.1f}° (error: {th_err_deg:.1f}°)")
            break
        
        # Pure rotation controller
        w = K_THETA_P2 * th_err
        
        # Scale down when close
        w_scale = 1.0
        if th_err_deg < 20:
            w_scale = 0.5
        if th_err_deg < 10:
            w_scale = 0.35
        
        # Differential wheel velocities for rotation (v=0, only w)
        vL = -(WHEEL_BASE_M * 0.5) * w
        vR = (WHEEL_BASE_M * 0.5) * w
        
        vmax = max(abs(vL), abs(vR), 1e-6)
        vL /= vmax
        vR /= vmax
        
        pwmL, pwmR = pwm_from_velocity(vL, vR, v_scale=w_scale)
        
        # Minimum rotation speed to overcome friction
        if th_err_deg > TH_TOL_DEG:
            def min_rotate(p):
                if p == 0.0:
                    return 0.0
                return math.copysign(max(abs(p), CREEP_PWM * 0.8), p)
            pwmL = min_rotate(pwmL)
            pwmR = min_rotate(pwmR)
        
        command_motors(pwmL, pwmR)
        last_pwm = (pwmL, pwmR)
        
        # Telemetry
        if (now - last_print) >= PRINT_INTERVAL:
            print(f"th={math.degrees(th):6.2f}° | err={math.degrees(th_err):+6.2f}° | "
                  f"pwmL={pwmL:+.2f}, pwmR={pwmR:+.2f}")
            last_print = now
        
        # Timeout
        if (now - start) > MAX_RUNTIME_PHASE2_S:
            print("Phase 2 timeout reached")
            break
        
        sleep(DT)
    
    robot.stop()
    sleep(0.2)
    return x, y, th


def main():
    # Parse goal from CLI or prompt
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
    goal_th = wrap_pi(goal_th)
    
    # Setup encoders
    encL = SimpleEncoder(ENCODER_L_A_PIN)
    encR = SimpleEncoder(ENCODER_R_A_PIN)
    
    # Precompute ticks-per-meter
    circL = math.pi * WHEEL_DIAMETER_L_M
    circR = math.pi * WHEEL_DIAMETER_R_M
    ticks_per_rev = PULSES_PER_REV * EDGES_PER_PULSE * GEAR_RATIO
    tpmL = ticks_per_rev / circL
    tpmR = ticks_per_rev / circR
    
    print(f"\n=== Two-Phase Go-to-Pose ===")
    print(f"Goal: x={goal_x:.3f} m, y={goal_y:.3f} m, θ={goal_th_deg:.1f}°")
    print(f"LEFT : D={WHEEL_DIAMETER_L_M*1000:.2f} mm | ticks/m={tpmL:.2f}")
    print(f"RIGHT: D={WHEEL_DIAMETER_R_M*1000:.2f} mm | ticks/m={tpmR:.2f}")
    print(f"Wheel base: {WHEEL_BASE_M:.3f} m")
    
    # Reset encoders
    sleep(0.2)
    encL.reset()
    encR.reset()
    
    overall_start = time()
    
    try:
        # PHASE 1: Navigate to position
        x, y, th = phase1_navigate_to_position(goal_x, goal_y, encL, encR, tpmL, tpmR)
        
        # PHASE 2: Rotate to heading
        x, y, th = phase2_rotate_to_heading(goal_th, x, y, th, encL, encR, tpmL, tpmR)
        
        # Final report
        print(f"\n=== Mission Complete ===")
        print(f"Final pose: x={x:.3f} m, y={y:.3f} m, θ={math.degrees(th):.1f}°")
        print(f"Goal pose:  x={goal_x:.3f} m, y={goal_y:.3f} m, θ={goal_th_deg:.1f}°")
        print(f"Errors: Δx={goal_x-x:.3f} m, Δy={goal_y-y:.3f} m, Δθ={math.degrees(wrap_pi(goal_th-th)):.1f}°")
        print(f"Total time: {time()-overall_start:.2f}s")
        
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt: stopping...")
    finally:
        robot.stop()
        encL.close()
        encR.close()
        print("Done.")


if __name__ == "__main__":
    main()