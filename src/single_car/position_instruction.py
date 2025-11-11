#!/usr/bin/env python3
# Differential drive "go to (x, y, theta_deg)" with robust encoder handling.
# - +x forward, +y left, CCW+ theta
# - pigpio pin factory for reliable high-rate edge capture
# - per-wheel sizes (diameter/CPR/ratio) and per-wheel meters-per-tick
# - built-in straight-line calibration: "calibrate 1.00"
# - ignore odom updates if no ticks this cycle

from time import sleep, time
import math, threading

# ---------- gpiozero with pigpio backend ----------
from gpiozero import Robot, Motor, Button, Device
try:
    from gpiozero.pins.pigpio import PiGPIOFactory
    Device.pin_factory = PiGPIOFactory()   # use pigpio if available
except Exception:
    pass  # fall back to default; still works but may miss pulses at speed

# ====================== USER SETTINGS ======================
# Encoders (BCM)
LEFT_ENCODER_PIN  = 27
RIGHT_ENCODER_PIN = 17

# Motors (BCM)
LEFT_MOTOR_PINS  = dict(forward=24, backward=23, enable=18, pwm=True)
RIGHT_MOTOR_PINS = dict(forward=25, backward=5,  enable=19, pwm=True)

# Chassis geometry
TRACK_WIDTH_M = 0.133

# Wheel / encoder parameters (per wheel)
WHEEL_DIAMETER_L_M = 0.067
WHEEL_DIAMETER_R_M = 0.066
PULSES_PER_REV_L   = 20
PULSES_PER_REV_R   = 20
GEAR_RATIO_L       = 1.0   # wheel_rev = motor_rev / GEAR_RATIO
GEAR_RATIO_R       = 1.0

# Learned/auto settings (start neutral; auto-filled at runtime)
PWM_SCALE_L = 1.0
PWM_SCALE_R = 1.0

# Track last-known wheel directions (+1 forward, -1 reverse, 0 stop)
_last_dir_l = 0.0
_last_dir_r = 0.0

# If one encoder is logically inverted, flip its direction here:
# If both encoders face the same direction physically, flip one here:
ENCODER_DIR_L = +1.0   # set to -1.0 to flip
ENCODER_DIR_R = -1.0   # FLIPPED: encoders likely face same direction

# Controller tuning (gentle)
K_RHO, K_ALPHA, K_THETA = 0.9, 2.0, 1.8
MAX_PWM, MIN_PWM = 0.55, 0.12
V_MAX_ROBOT, W_MAX = 0.35, 2.0  # m/s, rad/s

# Near-goal slowdown
NEAR_POS_M, V_NEAR, W_NEAR = 0.30, 0.20, 1.2
FINAL_POS_TOL_M, FINAL_HEAD_TOL_DEG = 0.03, 5.0

# Command shaping / timing
SLEW_PER_SEC = 1.8
CTRL_DT      = 0.03

# Optional active brake
USE_ACTIVE_BRAKE = True
BRAKE_PWM, BRAKE_TIME_SEC = 0.18, 0.10

# Calibration helper
CALIB_PWM, CALIB_TIME_SEC = 0.25, 2.0
# ==========================================================

def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v
def wrap_to_pi(a):    return (a + math.pi) % (2 * math.pi) - math.pi

def meters_per_tick(diameter_m: float, cpr: float, ratio: float) -> float:
    return (math.pi * diameter_m) / (cpr * ratio)

# Initial theoretical MPT (we'll refine with calibration)
MPT_L = meters_per_tick(WHEEL_DIAMETER_L_M, PULSES_PER_REV_L, GEAR_RATIO_L)
MPT_R = meters_per_tick(WHEEL_DIAMETER_R_M, PULSES_PER_REV_R, GEAR_RATIO_R)

# ---------- Hardware ----------
robot = Robot(left=Motor(**LEFT_MOTOR_PINS), right=Motor(**RIGHT_MOTOR_PINS))
# No debounce; pigpio handles timing well. If you see chatter at *standstill*, you can set bounce_time=0.0001–0.0003
left_enc  = Button(LEFT_ENCODER_PIN,  pull_up=True)
right_enc = Button(RIGHT_ENCODER_PIN, pull_up=True)

_lock = threading.Lock()
lticks = 0
rticks = 0

def _on_lt(): 
    global lticks
    with _lock: lticks += 1
def _on_rt(): 
    global rticks
    with _lock: rticks += 1

left_enc.when_pressed  = _on_lt
right_enc.when_pressed = _on_rt

def set_wheels(wl, wr):
    """Set wheel PWM with scaling. Tracks commanded direction."""
    global _last_dir_l, _last_dir_r
    wl_scaled = wl * PWM_SCALE_L
    wr_scaled = wr * PWM_SCALE_R
    
    # Clamp to valid PWM range
    wl_scaled = clamp(wl_scaled, -1.0, 1.0)
    wr_scaled = clamp(wr_scaled, -1.0, 1.0)
    
    # Set left motor
    if wl_scaled >= 0: 
        robot.left_motor.forward(wl_scaled)
        _last_dir_l = +1.0 if wl_scaled > 1e-3 else 0.0
    else:        
        robot.left_motor.backward(-wl_scaled)
        _last_dir_l = -1.0 if wl_scaled < -1e-3 else 0.0
    
    # Set right motor
    if wr_scaled >= 0: 
        robot.right_motor.forward(wr_scaled)
        _last_dir_r = +1.0 if wr_scaled > 1e-3 else 0.0
    else:        
        robot.right_motor.backward(-wr_scaled)
        _last_dir_r = -1.0 if wr_scaled < -1e-3 else 0.0


def stop_with_brake():
    if not USE_ACTIVE_BRAKE:
        robot.stop(); return
    set_wheels(-BRAKE_PWM, -BRAKE_PWM)
    sleep(BRAKE_TIME_SEC)
    robot.stop()

def pwm_from_v(v_lin, v_max):
    if v_max <= 1e-6: return 0.0
    mag = (abs(v_lin)/v_max) * MAX_PWM
    if mag > 1e-3: mag = max(mag, MIN_PWM)
    return clamp(mag, 0.0, MAX_PWM) * (1.0 if v_lin >= 0 else -1.0)

def slew_limit(target, current, max_step):
    if target > current + max_step: return current + max_step
    if target < current - max_step: return current - max_step
    return target

def vw_to_wheels(v, w, track):
    return v - 0.5*w*track, v + 0.5*w*track

# ---------- Odometry ----------
class Odom:
    def __init__(self):
        self.x = self.y = 0.0
        self.th = 0.0
        self._lt_last = 0
        self._rt_last = 0
    
    def reset(self, x=0.0, y=0.0, th_deg=0.0):
        self.x, self.y, self.th = x, y, math.radians(th_deg)
        with _lock:
            self._lt_last = lticks
            self._rt_last = rticks
    
    def update(self, wl_cmd, wr_cmd):
        """Update odometry based on encoder ticks and commanded wheel directions."""
        with _lock:
            lt = lticks
            rt = rticks
        
        dlt = lt - self._lt_last
        drt = rt - self._rt_last
        self._lt_last = lt
        self._rt_last = rt

        # If no ticks this cycle, don't update pose
        if dlt == 0 and drt == 0:
            return dlt, drt, 0.0, 0.0

        # Use the commanded direction to determine tick sign
        # This is more reliable than _last_dir which may lag
        dir_l = (+1.0 if wl_cmd >= 0 else -1.0) * ENCODER_DIR_L
        dir_r = (+1.0 if wr_cmd >= 0 else -1.0) * ENCODER_DIR_R

        # Calculate wheel distances
        dl = dlt * MPT_L * dir_l
        dr = drt * MPT_R * dir_r

        # Differential drive kinematics
        dc  = 0.5 * (dl + dr)  # center distance
        dth = (dr - dl) / TRACK_WIDTH_M  # heading change

        # Update pose
        if abs(dth) < 1e-6:
            # Straight line motion
            self.x += dc * math.cos(self.th)
            self.y += dc * math.sin(self.th)
        else:
            # Arc motion
            R = dc / dth
            self.x += R * (math.sin(self.th + dth) - math.sin(self.th))
            self.y += -R * (math.cos(self.th + dth) - math.cos(self.th))
        
        self.th = wrap_to_pi(self.th + dth)
        return dlt, drt, dl, dr

# ---------- Controller ----------
def go_to_pose(odom: Odom, xg, yg, thg_deg, timeout_s=60.0, print_every=0.25):
    thg = math.radians(thg_deg)
    t0 = time()
    t_last = 0.0
    wl_cmd = wr_cmd = 0.0

    try:
        while True:
            now = time()
            dlt, drt, dl_m, dr_m = odom.update(wl_cmd, wr_cmd)

            # Calculate errors
            dx, dy = xg - odom.x, yg - odom.y
            rho    = math.hypot(dx, dy)
            alpha  = wrap_to_pi(math.atan2(dy, dx) - odom.th)
            th_err = wrap_to_pi(thg - odom.th)

            # Check if goal reached
            at_pos  = rho <= FINAL_POS_TOL_M
            at_pose = at_pos and abs(math.degrees(th_err)) <= FINAL_HEAD_TOL_DEG
            if at_pose:
                stop_with_brake()
                print(f"Reached: x={odom.x:.3f}, y={odom.y:.3f}, th={math.degrees(odom.th):.1f}°")
                return True

            # Compute control velocities
            v_cap = V_NEAR if rho < NEAR_POS_M else V_MAX_ROBOT
            w_cap = W_NEAR if rho < NEAR_POS_M else W_MAX

            if not at_pos:
                # Drive towards position
                v = clamp(K_RHO * rho, -v_cap, v_cap)
                w = clamp(K_ALPHA * alpha, -w_cap, w_cap)
            else:
                # Only correct heading
                v = 0.0
                w = clamp(K_THETA * th_err, -w_cap, w_cap)

            # Convert to wheel velocities
            v_l, v_r = vw_to_wheels(v, w, TRACK_WIDTH_M)
            target_wl = pwm_from_v(v_l, V_MAX_ROBOT)
            target_wr = pwm_from_v(v_r, V_MAX_ROBOT)

            # Apply slew rate limiting
            step = SLEW_PER_SEC * CTRL_DT
            wl_cmd = slew_limit(target_wl, wl_cmd, step)
            wr_cmd = slew_limit(target_wr, wr_cmd, step)
            set_wheels(wl_cmd, wr_cmd)

            # Print status
            if now - t_last >= print_every:
                print(f"x={odom.x:.3f} m, y={odom.y:.3f} m, th={math.degrees(odom.th):.1f}° | "
                      f"ρ={rho:.3f} m, α={math.degrees(alpha):.1f}°, θ_err={math.degrees(th_err):.1f}° | "
                      f"wl={wl_cmd:.2f}, wr={wr_cmd:.2f} | dlt={dlt}, drt={drt}")
                t_last = now

            # Check timeout
            if now - t0 > timeout_s:
                stop_with_brake()
                print("Timeout before reaching the target.")
                return False

            sleep(CTRL_DT)
    finally:
        robot.stop()

# ---------- Calibration ----------
def calibrate_straight(measured_distance_m: float | None = None):
    global lticks, rticks, PWM_SCALE_L, PWM_SCALE_R, MPT_L, MPT_R
    print("\n--- Calibration: straight drive ---")
    print(f"PWM={CALIB_PWM:.2f} for {CALIB_TIME_SEC:.2f}s; then measure distance and re-run as 'calibrate <meters>'")
    
    with _lock:
        l0, r0 = lticks, rticks
    
    set_wheels(CALIB_PWM, CALIB_PWM)
    sleep(CALIB_TIME_SEC)
    robot.stop()
    sleep(0.2)
    
    with _lock:
        l1, r1 = lticks, rticks
    
    dL, dR = (l1 - l0), (r1 - r0)
    print(f"Ticks: left={dL}, right={dR}")

    # If both wheels ticked, adjust PWM to balance them
    if dL > 0 and dR > 0:
        if dL > dR:
            PWM_SCALE_L = max(0.6, min(1.0, dR / dL))
            PWM_SCALE_R = 1.0
        else:
            PWM_SCALE_R = max(0.6, min(1.0, dL / dR))
            PWM_SCALE_L = 1.0
        print(f"Applied PWM trim: PWM_SCALE_L={PWM_SCALE_L:.3f}, PWM_SCALE_R={PWM_SCALE_R:.3f}")

    # If distance was measured, update meters-per-tick
    if measured_distance_m and measured_distance_m > 0:
        if dL > 0:
            MPT_L = measured_distance_m / dL
            print(f"Updated MPT_L={MPT_L:.6f} m/tick")
        if dR > 0:
            MPT_R = measured_distance_m / dR
            print(f"Updated MPT_R={MPT_R:.6f} m/tick")
    
    print("--- End calibration ---\n")


# ---------- CLI ----------
def parse_goal(s: str):
    s = s.strip().lower()
    if s in ("q","quit","exit"): return None
    if s == "test": return ("test",)
    if s == "motors": return ("motors",)
    if s.startswith("calibrate"):
        parts = s.split()
        dist = float(parts[1]) if len(parts) == 2 else None
        return ("calibrate", dist)
    parts = s.replace(",", " ").split()
    if len(parts) != 3: raise ValueError("Enter: x y theta_deg   (or 'calibrate [m]', 'test', 'motors', or 'q')")
    return float(parts[0]), float(parts[1]), float(parts[2])

def test_motors():
    """Test motors individually to verify they work."""
    print("\n--- MOTOR TEST ---")
    print("Testing each motor individually...\n")
    
    # Test left motor forward
    print("LEFT motor FORWARD (2 sec) - should spin...")
    robot.left_motor.forward(0.5)
    sleep(2)
    robot.stop()
    sleep(1)
    
    # Test left motor backward
    print("LEFT motor BACKWARD (2 sec) - should spin opposite...")
    robot.left_motor.backward(0.5)
    sleep(2)
    robot.stop()
    sleep(1)
    
    # Test right motor forward
    print("RIGHT motor FORWARD (2 sec) - should spin...")
    robot.right_motor.forward(0.5)
    sleep(2)
    robot.stop()
    sleep(1)
    
    # Test right motor backward
    print("RIGHT motor BACKWARD (2 sec) - should spin opposite...")
    robot.right_motor.backward(0.5)
    sleep(2)
    robot.stop()
    sleep(1)
    
    # Test both motors forward
    print("BOTH motors FORWARD (2 sec) - robot should drive straight...")
    robot.forward(0.5)
    sleep(2)
    robot.stop()
    
    print("\n--- END MOTOR TEST ---")
    print("Did the motors spin? If not:")
    print("  1. Check motor driver power supply")
    print("  2. Verify motor driver connections")
    print("  3. Check enable pins (currently 18 and 19)")
    print("  4. Try testing with a multimeter on motor terminals")
    print("  5. Verify motor driver chip isn't overheating/damaged\n")

def test_encoders():
    """Test encoder readings during forward motion."""
    global lticks, rticks
    print("\n--- ENCODER TEST ---")
    print("Driving forward for 3 seconds...")
    print("Watch for tick counts - should see dozens of ticks!\n")
    
    with _lock:
        l0, r0 = lticks, rticks
    
    # Drive forward and monitor ticks in real-time
    robot.forward(0.3)
    for i in range(12):  # 3 seconds at 0.25s intervals
        sleep(0.25)
        with _lock:
            l_now, r_now = lticks, rticks
        dL = l_now - l0
        dR = r_now - r0
        print(f"  {(i+1)*0.25:.2f}s: Left={dL} ticks, Right={dR} ticks")
    
    robot.stop()
    sleep(0.2)
    
    with _lock:
        l1, r1 = lticks, rticks
    
    dL_total = l1 - l0
    dR_total = r1 - r0
    
    print(f"\nFINAL: Left={dL_total} ticks, Right={dR_total} ticks")
    
    if dL_total < 10 and dR_total < 10:
        print("\n⚠️  WARNING: Very few ticks detected!")
        print("Possible issues:")
        print("  1. Check encoder wiring and connections")
        print("  2. Verify GPIO pins (currently L=27, R=17)")
        print("  3. Try changing pull_up=False in encoder setup")
        print("  4. Check encoder disk alignment with sensors")
        print("  5. Verify encoder power supply")
    elif abs(dL_total - dR_total) > 10:
        print(f"\n⚠️  Encoders very unbalanced: {abs(dL_total - dR_total)} tick difference")
        print("  One encoder may be wired incorrectly or failing")
    else:
        print("\n✓ Encoders appear to be working!")
        print(f"  Estimated distance: {(dL_total * MPT_L + dR_total * MPT_R) / 2:.3f} m")
    
    print("--- END TEST ---\n")

def auto_direction_probe():
    """Briefly drive forward and report encoder tick deltas."""
    print("[autodir] Probing encoder directions...")
    
    with _lock:
        l0, r0 = lticks, rticks
    
    # Drive forward briefly
    robot.forward(0.25)
    sleep(0.8)
    robot.stop()
    sleep(0.3)
    
    with _lock:
        dL = lticks - l0
        dR = rticks - r0
    
    print(f"[autodir] Encoder ticks during forward motion: Left={dL}, Right={dR}")
    print(f"[autodir] Current ENCODER_DIR_L={ENCODER_DIR_L:+.0f}, ENCODER_DIR_R={ENCODER_DIR_R:+.0f}")
    if dL < 5 and dR < 5:
        print("[autodir] ⚠️  Very few ticks! Use 'test' command to diagnose")
    else:
        print("[autodir] If robot veers badly, try flipping ENCODER_DIR values in code")


def main():
    print("=== Go to (x, y, theta_deg) — pigpio encoders & calibration ===")
    print("Commands: 'x y th', 'calibrate [m]', 'test', 'motors', 'q'\n")
    
    # Probe encoder directions
    auto_direction_probe()
    
    # Initialize odometry
    odom = Odom()
    odom.reset(0, 0, 0)

    while True:
        try:
            line = input("Target (x y theta_deg) > ")
        except (EOFError, KeyboardInterrupt):
            print("\nExiting.")
            break
        
        if not line.strip():
            continue
        
        try:
            p = parse_goal(line)
        except Exception as e:
            print(f"Input error: {e}")
            continue
        
        if p is None:
            break
        
        if isinstance(p, tuple) and p[0] == "motors":
            test_motors()
            continue
        
        if isinstance(p, tuple) and p[0] == "test":
            test_encoders()
            continue
        
        if isinstance(p, tuple) and p[0] == "calibrate":
            _, dist = p
            calibrate_straight(dist)
            continue
        
        xg, yg, thg = p
        print(f"Going to: x={xg:.3f}, y={yg:.3f}, θ={thg:.1f}°")
        
        # Reset odometry before each goal
        odom.reset(0, 0, 0)
        go_to_pose(odom, xg, yg, thg, timeout_s=60.0)
    
    robot.stop()
    print("Goodbye.")

if __name__ == "__main__":
    try:
        main()
    finally:
        robot.stop()