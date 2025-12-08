# both_wheels_encoders_test.py
# Tests both wheels and both encoders at once.
# - Works with single-channel encoders (A only) or full quadrature (A+B).
# - Prints per-wheel ticks, ticks/s, RPM, signed RPM, and direction checks.
#
# gpiozero uses BCM numbering.

from gpiozero import Robot, Motor, Button
from time import sleep, time
import threading
from threading import Event
import math

# ======= USER SETTINGS =======
# Motor wiring (you already swapped to match physical sides)
robot = Robot(
    left=Motor(forward=25, backward=5, enable=19, pwm=True),   # physical LEFT
    right=Motor(forward=24, backward=23, enable=18, pwm=True)  # physical RIGHT
)

# Encoder pins (BCM)
ENCODER_L_A_PIN = 27          # LEFT encoder channel A (required)
ENCODER_L_B_PIN = None        # LEFT encoder channel B (optional for quadrature)
ENCODER_R_A_PIN = 17          # RIGHT encoder channel A (required)
ENCODER_R_B_PIN = None        # RIGHT encoder channel B (optional for quadrature)

# Encoder characteristics
PULSES_PER_REV = 20           # datasheet pulses PER CHANNEL per motor-shaft rev (A rising edges)
EDGES_PER_PULSE = 1           # 1 = count rising only, 2 = count rising+falling (single-channel mode)
GEAR_RATIO = 1.0              # wheel_rev = motor_rev / GEAR_RATIO
WHEEL_DIAMETER_M = 0.065      # for m/s readout (optional; adjust to your wheel)

# Telemetry and test plan
PRINT_INTERVAL = 0.5          # seconds
TEST_SPEEDS = [0.30, 0.50, 0.70]
STEP_TIME = 3.0               # seconds each segment (forward/reverse)
STOP_TIME = 2.0               # stop segment between moves

# Start-up friction helper
KICK_SPEED = 0.8              # brief kick speed to overcome static friction
KICK_TIME = 0.15              # seconds
# =============================

# ---- Encoder helper ----
class DualEncoder:
    """
    Handles either single-channel counting (A only) or full quadrature (A+B).
    - Single-channel: counts edges on A; direction sign comes from commanded direction.
    - Quadrature: decodes direction from A+B transitions; sign is true direction.
    """
    def __init__(self, a_pin, b_pin=None, pull_up=True, bounce_time=0.001):
        self.a = Button(a_pin, pull_up=pull_up, bounce_time=bounce_time)
        self.b = Button(b_pin, pull_up=pull_up, bounce_time=bounce_time) if b_pin is not None else None

        self.lock = threading.Lock()
        self.ticks = 0          # total (unsigned) ticks (good for single-channel)
        self.signed_ticks = 0   # signed ticks (only accurate in quadrature mode)
        self.last_state = None  # last (A,B) tuple when doing quadrature

        if self.b is None:
            # Single-channel counting: allow rising+falling if EDGES_PER_PULSE=2
            self.a.when_pressed = self._on_a_edge
            if EDGES_PER_PULSE == 2:
                self.a.when_released = self._on_a_edge
        else:
            # Quadrature decoding: attach to both lines (both edges)
            self.a.when_pressed  = lambda: self._on_quad_edge(a_edge=True)
            self.a.when_released = lambda: self._on_quad_edge(a_edge=True)
            self.b.when_pressed  = lambda: self._on_quad_edge(a_edge=False)
            self.b.when_released = lambda: self._on_quad_edge(a_edge=False)
            # Initialize state
            self.last_state = (int(self.a.is_pressed), int(self.b.is_pressed))

    def close(self):
        self.a.close()
        if self.b:
            self.b.close()

    def _on_a_edge(self):
        # Single-channel: count edges; no true direction
        with self.lock:
            self.ticks += 1

    def _on_quad_edge(self, a_edge: bool):
        # Full quadrature: decode direction from (prev_state -> curr_state)
        curr = (int(self.a.is_pressed), int(self.b.is_pressed))
        with self.lock:
            prev = self.last_state
            self.last_state = curr
            if prev is None:
                return
            # Gray code forward sequence: 00->01->11->10->00 ; reverse is opposite
            # Compute delta using a small lookup (-1, 0, +1)
            delta = self._quad_delta(prev, curr)
            if delta:
                self.signed_ticks += delta
                self.ticks += 1  # total edges seen (unsigned)

    @staticmethod
    def _quad_delta(prev, curr):
        # Map transitions to +1 / -1. Invalid/noise -> 0.
        # States are tuples (A,B) with values 0/1.
        transitions = {
            (0,0):(0,1),(0,1):(1,1),(1,1):(1,0),(1,0):(0,0)  # forward ring
        }
        # If curr is the next in the forward ring
        for s_from, s_to in transitions.items():
            if prev == s_from and curr == s_to:
                return +1
        # If prev is next after curr -> reverse
        for s_from, s_to in transitions.items():
            if curr == s_from and prev == s_to:
                return -1
        return 0

# Instantiate encoders
encL = DualEncoder(ENCODER_L_A_PIN, ENCODER_L_B_PIN)
encR = DualEncoder(ENCODER_R_A_PIN, ENCODER_R_B_PIN)

# Shared state for telemetry
stop_event = Event()
drive_dir = 0   # +1 forward, -1 reverse, 0 stop
last_t = time()
last_L_ticks = 0
last_R_ticks = 0
last_L_signed = 0
last_R_signed = 0
lock_global = threading.Lock()

def counts_to_rpm(delta_counts, dt, quadrature=False):
    """
    Convert count delta to wheel RPM.
    - Single-channel: counts per rev = PULSES_PER_REV * EDGES_PER_PULSE
    - Quadrature:     counts per rev = 4 * PULSES_PER_REV (full 4x decoding)
    """
    if dt <= 0:
        return 0.0
    if quadrature:
        counts_per_motor_rev = 4.0 * PULSES_PER_REV
    else:
        counts_per_motor_rev = float(PULSES_PER_REV * EDGES_PER_PULSE)
    motor_revs = delta_counts / counts_per_motor_rev
    wheel_revs = motor_revs / max(GEAR_RATIO, 1e-9)
    return wheel_revs / dt * 60.0

def telemetry_loop():
    global last_t, last_L_ticks, last_R_ticks, last_L_signed, last_R_signed
    quadL = encL.b is not None
    quadR = encR.b is not None
    circ = math.pi * WHEEL_DIAMETER_M

    while not stop_event.is_set():
        now = time()
        if now - last_t >= PRINT_INTERVAL:
            with encL.lock:
                tL = encL.ticks
                sL = encL.signed_ticks
            with encR.lock:
                tR = encR.ticks
                sR = encR.signed_ticks

            dt = now - last_t
            dL = tL - last_L_ticks
            dR = tR - last_R_ticks
            dsL = sL - last_L_signed
            dsR = sR - last_R_signed

            # RPM (unsigned counts), then assign sign
            rpmL = counts_to_rpm(dL, dt, quadrature=quadL)
            rpmR = counts_to_rpm(dR, dt, quadrature=quadR)

            if quadL:
                # True signed RPM from signed counts
                srpmL = counts_to_rpm(abs(dsL), dt, quadrature=True) * (1 if dsL >= 0 else -1 if dsL <= 0 else 0)
            else:
                srpmL = rpmL * drive_dir

            if quadR:
                srpmR = counts_to_rpm(abs(dsR), dt, quadrature=True) * (1 if dsR >= 0 else -1 if dsR <= 0 else 0)
            else:
                srpmR = rpmR * drive_dir

            vL = (srpmL / 60.0) * circ
            vR = (srpmR / 60.0) * circ

            dir_str = "FWD" if drive_dir > 0 else "REV" if drive_dir < 0 else "STOP"
            print(f"[{now:.2f}] "
                  f"L: Δ={dL:3d} rpm={rpmL:6.2f} sRPM={srpmL:7.2f} v={vL:5.3f} m/s | "
                  f"R: Δ={dR:3d} rpm={rpmR:6.2f} sRPM={srpmR:7.2f} v={vR:5.3f} m/s | "
                  f"dir={dir_str}")

            # Direction sanity (only meaningful in quadrature)
            if drive_dir != 0:
                if quadL and (srpmL * drive_dir < -1e-6):
                    print("WARNING: LEFT encoder direction disagrees with commanded direction.")
                if quadR and (srpmR * drive_dir < -1e-6):
                    print("WARNING: RIGHT encoder direction disagrees with commanded direction.")

            last_t = now
            last_L_ticks, last_R_ticks = tL, tR
            last_L_signed, last_R_signed = sL, sR

        sleep(0.02)

tele_thr = threading.Thread(target=telemetry_loop)
tele_thr.start()

def drive_both(speed, direction, duration_s):
    """
    Drive both wheels with a brief kick to overcome static friction.
    """
    global drive_dir
    drive_dir = direction

    if direction == 0 or duration_s <= 0:
        robot.stop()
        drive_dir = 0
        sleep(max(duration_s, 0))
        return

    # Kick
    if KICK_TIME > 0:
        if direction > 0:
            robot.forward(KICK_SPEED)
        else:
            robot.backward(KICK_SPEED)
        sleep(min(KICK_TIME, max(duration_s - 0.01, 0)))

    # Hold target speed for remainder
    remain = max(duration_s - KICK_TIME, 0)
    if remain > 0:
        if direction > 0:
            robot.forward(speed)
        else:
            robot.backward(speed)
        sleep(remain)

    robot.stop()
    drive_dir = 0

def step_and_check(speed, direction, label):
    # Snapshot ticks before
    with encL.lock: startL = encL.ticks
    with encR.lock: startR = encR.ticks

    print(f"{label} at {int(speed*100)}% for {STEP_TIME}s")
    drive_both(speed, direction, STEP_TIME)

    # Snapshot after
    with encL.lock: endL = encL.ticks
    with encR.lock: endR = encR.ticks

    dL = endL - startL
    dR = endR - startR
    if direction != 0:
        if dL == 0:
            print("WARNING: LEFT produced 0 ticks during this step.")
        if dR == 0:
            print("WARNING: RIGHT produced 0 ticks during this step.")

def main():
    try:
        print("\n=== Both Wheels + Both Encoders Test ===\n")
        robot.stop()
        sleep(0.3)

        # Forward
        for sp in TEST_SPEEDS:
            step_and_check(sp, +1, "FORWARD")

        # Stop segment
        print(f"STOP for {STOP_TIME:.1f}s"); drive_both(0.0, 0, STOP_TIME)

        # Reverse
        for sp in TEST_SPEEDS:
            step_and_check(sp, -1, "REVERSE")

        # Final stop
        print(f"STOP for {STOP_TIME:.1f}s"); drive_both(0.0, 0, STOP_TIME)

    except KeyboardInterrupt:
        print("KeyboardInterrupt: stopping...")
    finally:
        robot.stop()
        stop_event.set()
        tele_thr.join(timeout=2.0)
        encL.close()
        encR.close()
        print("Done.")

if __name__ == "__main__":
    main()
