#!/usr/bin/env python3
"""
Differential-drive "go to (x, y, theta_deg)" for Raspberry Pi using gpiozero.

Frame:
- +x is forward
- +y is left
- theta increases CCW (0° aligned with +x)

Key fixes:
- No debounce on encoder inputs (bounce_time=None) to avoid missing pulses.
- Zero-aware sign for direction inference (cmd_sign): if |PWM| < eps, treat as 0.
- Optional ENCODER_DIR_L/R multipliers to flip an encoder if wired inverse.
- Telemetry prints per-loop tick deltas for each wheel to diagnose counting.
"""

from gpiozero import Robot, Motor, Button
from time import sleep, time
import math
import threading

# ====================== USER SETTINGS (per wheel) ======================
# Encoders (GPIO BCM)
LEFT_ENCODER_PIN  = 27   # left wheel encoder input (BCM)
RIGHT_ENCODER_PIN = 17   # right wheel encoder input (BCM)

# Motors (GPIO BCM)
LEFT_MOTOR_PINS  = dict(forward=24, backward=23, enable=18, pwm=True)
RIGHT_MOTOR_PINS = dict(forward=25, backward=5,  enable=19, pwm=True)

# Chassis geometry
TRACK_WIDTH_M = 0.133    # distance between wheel contact patches (meters)

# --- Wheel/encoder parameters (ALLOW DIFFERENT VALUES PER WHEEL) ---
# Diameter = tire outer diameter in meters (measure with calipers / ruler)
WHEEL_DIAMETER_L_M = 0.067   # e.g., 67 mm left
WHEEL_DIAMETER_R_M = 0.066   # e.g., 66 mm right

# CPR = counts per motor shaft revolution from your encoder (edges you count)
PULSES_PER_REV_L = 20
PULSES_PER_REV_R = 20

# If encoder is on motor shaft and there is a gearbox before the wheel, set ratio.
# wheel_rev = motor_rev / GEAR_RATIO
GEAR_RATIO_L = 1.0
GEAR_RATIO_R = 1.0

# Optional: flip an encoder if it behaves opposite to the commanded direction.
# With single-channel encoders we infer direction from PWM sign; these let you correct wiring quirks.
ENCODER_DIR_L = +1.0   # set to -1.0 if left counts go the wrong way
ENCODER_DIR_R = +1.0   # set to -1.0 if right counts go the wrong way

# Control gains & limits (gentle defaults)
K_RHO    = 0.9
K_ALPHA  = 2.0
K_THETA  = 1.8
MAX_PWM  = 0.55
MIN_PWM  = 0.12
# Robot-level speed caps used for mapping v->PWM (tune to your platform)
V_MAX_ROBOT = 0.35   # m/s (approx straight-line top speed)
W_MAX       = 2.0    # rad/s

# Near-goal slowdown
NEAR_POS_M         = 0.30
V_NEAR             = 0.20
W_NEAR             = 1.2
FINAL_POS_TOL_M    = 0.03       # 3 cm
FINAL_HEAD_TOL_DEG = 5.0        # 5 degrees

# Command shaping / timing
SLEW_PER_SEC = 1.8   # max PWM change per second
CTRL_DT      = 0.03  # control period (s)

# Optional active brake to reduce coasting
USE_ACTIVE_BRAKE = True
BRAKE_PWM        = 0.18  # small reverse pulse
BRAKE_TIME_SEC   = 0.10  # brief

# Calibration drive (used by 'calibrate' CLI helper)
CALIB_PWM       = 0.25   # straight drive PWM for calibration
CALIB_TIME_SEC  = 2.0    # duration of straight drive
# ======================================================================

# ---------- Derived per-wheel meters-per-tick ----------
def _meters_per_tick(diameter_m: float, cpr: float, gear_ratio: float) -> float:
    """
    meters per encoder tick = (π * D) / (CPR * GEAR_RATIO)
    where GEAR_RATIO = motor_rev / wheel_rev (so wheel_rev = motor_rev / GEAR_RATIO)
    """
    return (math.pi * diameter_m) / (cpr * gear_ratio)

METERS_PER_TICK_L = _meters_per_tick(WHEEL_DIAMETER_L_M, PULSES_PER_REV_L, GEAR_RATIO_L)
METERS_PER_TICK_R = _meters_per_tick(WHEEL_DIAMETER_R_M, PULSES_PER_REV_R, GEAR_RATIO_R)

# ---------- Helpers ----------
def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v
def wrap_to_pi(a):    return (a + math.pi) % (2 * math.pi) - math.pi

def cmd_sign(x: float, eps: float = 1e-3) -> float:
    """Return -1, 0, or +1 based on PWM magnitude; zero stays zero."""
    if x > eps:  return  +1.0
    if x < -eps: return  -1.0
    return 0.0

# ---------- Hardware ----------
robot = Robot(left=Motor(**LEFT_MOTOR_PINS), right=Motor(**RIGHT_MOTOR_PINS))

# No debounce: we don't want to drop high-frequency pulses
left_enc  = Button(LEFT_ENCODER_PIN,  pull_up=True, bounce_time=None)
right_enc = Button(RIGHT_ENCODER_PIN, pull_up=True, bounce_time=None)

# Tick counters
_lock = threading.Lock()
_left_ticks_total  = 0
_right_ticks_total = 0

def _on_left_tick():
    global _left_ticks_total
    with _lock:
        _left_ticks_total += 1

def _on_right_tick():
    global _right_ticks_total
    with _lock:
        _right_ticks_total += 1

left_enc.when_pressed = _on_left_tick
right_enc.when_pressed = _on_right_tick
# Note: do NOT set when_released at all (avoids gpiozero warning)

# ---------- Odometry ----------
class Odometry:
    """
    Odometry integrates encoder deltas to track pose (x, y, th).
    Encoders are single-channel; we infer direction from the commanded wheel PWM.
    """
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0  # radians
        self._lt_last = 0
        self._rt_last = 0

    def reset(self, x=0.0, y=0.0, th_deg=0.0):
        self.x, self.y, self.th = x, y, math.radians(th_deg)
        with _lock:
            self._lt_last = _left_ticks_total
            self._rt_last = _right_ticks_total

    def update(self, wl_cmd=0.0, wr_cmd=0.0):
        """
        wl_cmd, wr_cmd are signed PWM cmds (-1..1) used to infer tick signs.
        """
        with _lock:
            lt = _left_ticks_total
            rt = _right_ticks_total
        dlt = lt - self._lt_last
        drt = rt - self._rt_last
        self._lt_last = lt
        self._rt_last = rt

        # Direction inferred from current PWM, with optional per-encoder polarity
        dir_l = cmd_sign(wl_cmd) * ENCODER_DIR_L
        dir_r = cmd_sign(wr_cmd) * ENCODER_DIR_R

        dl = dlt * METERS_PER_TICK_L * dir_l
        dr = drt * METERS_PER_TICK_R * dir_r

        dc  = 0.5 * (dl + dr)
        dth = (dr - dl) / TRACK_WIDTH_M

        if abs(dth) < 1e-6:
            self.x += dc * math.cos(self.th)
            self.y += dc * math.sin(self.th)
        else:
            R = dc / dth
            self.x += R * (math.sin(self.th + dth) - math.sin(self.th))
            self.y += -R * (math.cos(self.th + dth) - math.cos(self.th))
        self.th = wrap_to_pi(self.th + dth)

        # Return raw deltas for telemetry
        return dlt, drt, dl, dr

# ---------- Kinematics / Mapping ----------
def vw_to_wheels(v, w, track):
    """(v,w) robot velocities -> left/right wheel linear velocities (m/s)."""
    return v - 0.5 * w * track, v + 0.5 * w * track

def pwm_from_v(v_lin, v_max):
    """Map desired wheel linear speed to PWM with deadband and cap."""
    if v_max <= 1e-6:
        return 0.0
    mag = (abs(v_lin) / v_max) * MAX_PWM
    if mag > 1e-3:
        mag = max(mag, MIN_PWM)  # overcome stiction
    return clamp(mag, 0.0, MAX_PWM) * (1.0 if v_lin >= 0 else -1.0)

def slew_limit(target, current, max_step):
    """Limit rate of change of PWM commands."""
    if target > current + max_step: return current + max_step
    if target < current - max_step: return current - max_step
    return target

def set_wheels(wl, wr):
    """Apply signed PWM to wheels."""
    if wl >= 0: robot.left_motor.forward(wl)
    else:       robot.left_motor.backward(-wl)
    if wr >= 0: robot.right_motor.forward(wr)
    else:       robot.right_motor.backward(-wr)

def stop_with_brake():
    if not USE_ACTIVE_BRAKE:
        robot.stop(); return
    # Brief reverse pulse to arrest motion
    set_wheels(-BRAKE_PWM, -BRAKE_PWM)
    sleep(BRAKE_TIME_SEC)
    robot.stop()

# ---------- Controller ----------
def go_to_pose(odom: Odometry, x_goal, y_goal, th_goal_deg, *, timeout_s=60.0, print_every=0.25):
    """Drive to (x_goal, y_goal, th_goal_deg). Returns True on success, False on timeout."""
    th_goal = math.radians(th_goal_deg)

    t0 = time()
    t_last = 0.0

    wl_cmd = 0.0
    wr_cmd = 0.0

    try:
        while True:
            now = time()

            # Update odometry; get raw tick deltas for debugging
            dlt, drt, dl_m, dr_m = odom.update(wl_cmd, wr_cmd)

            # Goal errors
            dx = x_goal - odom.x
            dy = y_goal - odom.y
            rho = math.hypot(dx, dy)

            goal_heading = math.atan2(dy, dx)
            alpha = wrap_to_pi(goal_heading - odom.th)
            th_err = wrap_to_pi(th_goal - odom.th)

            at_pos = rho <= FINAL_POS_TOL_M
            at_pose = at_pos and abs(math.degrees(th_err)) <= FINAL_HEAD_TOL_DEG
            if at_pose:
                stop_with_brake()
                print(f"Reached: x={odom.x:.3f}, y={odom.y:.3f}, th={math.degrees(odom.th):.1f}°")
                return True

            # Speed caps (slow down when near)
            v_cap = V_NEAR if rho < NEAR_POS_M else V_MAX_ROBOT
            w_cap = W_NEAR if rho < NEAR_POS_M else W_MAX

            # Control law
            if not at_pos:
                v = clamp(K_RHO * rho, -v_cap, v_cap)
                w = clamp(K_ALPHA * alpha, -w_cap, w_cap)
            else:
                v = 0.0
                w = clamp(K_THETA * th_err, -w_cap, w_cap)

            # Convert (v,w) -> wheel velocities, then to PWM
            v_l, v_r = vw_to_wheels(v, w, TRACK_WIDTH_M)
            target_wl = pwm_from_v(v_l, V_MAX_ROBOT)
            target_wr = pwm_from_v(v_r, V_MAX_ROBOT)

            # Slew-limit the PWM to avoid surges/coasting
            max_step = SLEW_PER_SEC * CTRL_DT
            wl_cmd = slew_limit(target_wl, wl_cmd, max_step)
            wr_cmd = slew_limit(target_wr, wr_cmd, max_step)

            set_wheels(wl_cmd, wr_cmd)

            # Telemetry (now includes tick deltas per loop)
            if now - t_last >= print_every:
                print(
                    f"x={odom.x:.3f} m, y={odom.y:.3f} m, th={math.degrees(odom.th):.1f}° | "
                    f"ρ={rho:.3f} m, α={math.degrees(alpha):.1f}°, θ_err={math.degrees(th_err):.1f}° | "
                    f"wl={wl_cmd:.2f}, wr={wr_cmd:.2f} | "
                    f"dlt={int(dlt)}, drt={int(drt)}"
                )
                t_last = now

            # Timeout guard
            if now - t0 > timeout_s:
                stop_with_brake()
                print("Timeout before reaching the target.")
                return False

            sleep(CTRL_DT)
    finally:
        robot.stop()

# ---------- Calibration helper ----------
def calibrate_straight(measured_distance_m: float | None = None):
    """
    Drives straight for CALIB_TIME_SEC at CALIB_PWM and prints tick counts.
    If you provide measured_distance_m, it will compute suggested MPT values.
    """
    global _left_ticks_total, _right_ticks_total

    print("\n--- Calibration: straight drive ---")
    print(f"Driving straight at PWM={CALIB_PWM:.2f} for {CALIB_TIME_SEC:.2f}s...")

    with _lock:
        lt0 = _left_ticks_total
        rt0 = _right_ticks_total

    set_wheels(CALIB_PWM, CALIB_PWM)
    sleep(CALIB_TIME_SEC)
    robot.stop()
    sleep(0.2)

    with _lock:
        lt1 = _left_ticks_total
        rt1 = _right_ticks_total

    dL_ticks = lt1 - lt0
    dR_ticks = rt1 - rt0
    print(f"Ticks: left={dL_ticks}, right={dR_ticks}")

    if measured_distance_m is not None and measured_distance_m > 0:
        mpt_L = measured_distance_m / max(dL_ticks, 1)
        mpt_R = measured_distance_m / max(dR_ticks, 1)
        print(f"Suggested METERS_PER_TICK_L = {mpt_L:.6f}")
        print(f"Suggested METERS_PER_TICK_R = {mpt_R:.6f}")
    else:
        print("Measure the travel distance (m), then compute:")
        print("  METERS_PER_TICK_L = measured_distance / left_ticks")
        print("  METERS_PER_TICK_R = measured_distance / right_ticks")
    print("--- End calibration ---\n")

# ---------- CLI ----------
def parse_goal(line: str):
    s = line.strip().lower()
    if s in ("q", "quit", "exit"):
        return None
    if s.startswith("calibrate"):
        parts = s.split()
        if len(parts) == 2:
            try:
                dist = float(parts[1])
            except ValueError:
                dist = None
        else:
            dist = None
        return ("calibrate", dist)

    parts = line.replace(",", " ").split()
    if len(parts) != 3:
        raise ValueError("Enter: x y theta_deg   (or 'calibrate [meters]' or 'q' to quit)")
    return float(parts[0]), float(parts[1]), float(parts[2])

def main():
    print("=== Go to (x, y, theta_deg) — per-wheel sizes, robust odom ===")
    print("Frame: +x forward, +y left, CCW+ theta. Units: meters & degrees.")
    print("Commands:")
    print("  x y theta_deg     e.g., 1.0 0 0   or   0.0 0.5 90")
    print("  calibrate [m]     drive straight & print ticks (optionally compute MPT)")
    print("  q                 quit\n")

    odom = Odometry()
    odom.reset(0.0, 0.0, 0.0)

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

        if isinstance(p, tuple) and len(p) == 2 and p[0] == "calibrate":
            _, dist = p
            calibrate_straight(dist)
            continue

        xg, yg, thg = p
        print(f"Going to: x={xg:.3f}, y={yg:.3f}, θ={thg:.1f}°")
        go_to_pose(odom, xg, yg, thg, timeout_s=60.0)

    robot.stop()
    print("Goodbye.")

if __name__ == "__main__":
    try:
        main()
    finally:
        robot.stop()
