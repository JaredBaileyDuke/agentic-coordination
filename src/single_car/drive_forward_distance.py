# drive_forward_1m_dualwheel.py
# Drive forward exactly 1.0 m using per-wheel encoder feedback (BCM numbering).

from gpiozero import Robot, Motor, Button
from time import sleep, time
import math
import threading

# ================== USER SETTINGS ==================
# Motor wiring (names match physical sides)
robot = Robot(
    left=Motor(forward=25, backward=5, enable=19, pwm=True),   # physical LEFT
    right=Motor(forward=24, backward=23, enable=18, pwm=True)  # physical RIGHT
)

# Encoder pins (A channels)
ENCODER_L_A_PIN = 27  # LEFT encoder A (BCM)
ENCODER_R_A_PIN = 17  # RIGHT encoder A (BCM)

# Wheel + encoder characteristics
# Effective diameters calibrated from your measurements:
#  - Right: 4.75 rev ≈ 1.000 m -> C = 1/4.75 = 0.21053 m -> D = C/pi = 0.06701 m
#  - Left : 4.80 rev ≈ 1.000 m -> C = 1/4.80 = 0.20833 m -> D = C/pi = 0.06631 m
WHEEL_DIAMETER_L_M = 0.0667     # LEFT effective diameter (meters)
WHEEL_DIAMETER_R_M = 0.06701     # RIGHT effective diameter (meters)
GEAR_RATIO = 1.0                 # wheel_rev = motor_rev / GEAR_RATIO
PULSES_PER_REV = 20              # pulses per motor-shaft rev (per channel)
EDGES_PER_PULSE = 1              # 1=rising only; set 2 if you also count falling edges

# Distance target
DISTANCE_M = 1.0

# Motion profile
CRUISE_SPEED = 0.55              # 0..1
CREEP_SPEED  = 0.28              # 0..1
SLOWDOWN_AT_FRACTION = 0.90      # start creep after 90% of distance

# Start friction helper
KICK_SPEED = 0.80                # brief higher speed “kick”
KICK_TIME  = 0.15                # seconds

# Control + telemetry
DT = 0.02                        # control loop period (s)
PRINT_INTERVAL = 0.25            # telemetry print period (s)

# Straightness correction (gentle P-control on progress error)
KP_STRAIGHT = 0.20               # proportional gain
MAX_SPEED = 1.00
MIN_SPEED = 0.00

# Small trim to equalize sides (left was slightly short). ~+1.05%:
TRIM_L = 0.9200                  # multiply left base speed
TRIM_R = 1.0000                  # multiply right base speed
# ===================================================


class SimpleEncoder:
    """Single-channel edge counter (A only). Direction inferred from command."""
    def __init__(self, pin, pull_up=True, bounce_time=0.001):
        self.btn = Button(pin, pull_up=pull_up, bounce_time=bounce_time)
        self.ticks = 0
        self.lock = threading.Lock()
        self.btn.when_pressed = self._on_edge  # rising only
        if EDGES_PER_PULSE == 2:
            self.btn.when_released = self._on_edge  # rising+falling

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


def clamp(x, lo, hi):
    return hi if x > hi else lo if x < lo else x


def main():
    encL = SimpleEncoder(ENCODER_L_A_PIN)
    encR = SimpleEncoder(ENCODER_R_A_PIN)

    try:
        # Per-wheel ticks-per-meter
        circL = math.pi * WHEEL_DIAMETER_L_M
        circR = math.pi * WHEEL_DIAMETER_R_M
        ticks_per_rev = PULSES_PER_REV * EDGES_PER_PULSE * GEAR_RATIO
        tpmL = ticks_per_rev / circL
        tpmR = ticks_per_rev / circR
        targetL = tpmL * DISTANCE_M
        targetR = tpmR * DISTANCE_M

        print(f"LEFT  wheel: D={WHEEL_DIAMETER_L_M*1000:.2f} mm | C={circL:.5f} m | ticks/m={tpmL:.2f} | target={targetL:.2f}")
        print(f"RIGHT wheel: D={WHEEL_DIAMETER_R_M*1000:.2f} mm | C={circR:.5f} m | ticks/m={tpmR:.2f} | target={targetR:.2f}\n")

        # Reset counters and settle
        sleep(0.2)
        encL.reset(); encR.reset()

        # Kick to overcome static friction
        robot.left_motor.forward(KICK_SPEED)
        robot.right_motor.forward(KICK_SPEED)
        sleep(KICK_TIME)

        # Enter cruise
        robot.left_motor.forward(CRUISE_SPEED * TRIM_L)
        robot.right_motor.forward(CRUISE_SPEED * TRIM_R)

        start = time()
        last_print = start
        doneL = False
        doneR = False
        phase_creep = False

        while True:
            t = time()
            L = encL.read()
            R = encR.read()

            # Completion checks
            if not doneL and L >= targetL:
                doneL = True
                robot.left_motor.stop()
            if not doneR and R >= targetR:
                doneR = True
                robot.right_motor.stop()

            if doneL and doneR:
                break

            # Progress fractions (cap to 1.0)
            fracL = min(L / targetL, 1.0)
            fracR = min(R / targetR, 1.0)
            avg_frac = 0.5 * (fracL + fracR)

            # Creep near the end
            if not phase_creep and avg_frac >= SLOWDOWN_AT_FRACTION:
                phase_creep = True

            base = (CREEP_SPEED if phase_creep else CRUISE_SPEED)
            baseL = base * TRIM_L
            baseR = base * TRIM_R

            # Straightness correction (left minus right progress)
            err = fracL - fracR
            corr = KP_STRAIGHT * err
            spL = clamp(baseL - corr, MIN_SPEED, MAX_SPEED) if not doneL else 0.0
            spR = clamp(baseR + corr, MIN_SPEED, MAX_SPEED) if not doneR else 0.0

            # Command motors
            if not doneL:
                robot.left_motor.forward(spL)
            if not doneR:
                robot.right_motor.forward(spR)

            # Telemetry
            if (t - last_print) >= PRINT_INTERVAL:
                distL = L / tpmL
                distR = R / tpmR
                print(f"[{t - start:5.2f}s] "
                      f"L: ticks={L:5.1f} ({distL:5.3f} m, {fracL*100:5.1f}%) | "
                      f"R: ticks={R:5.1f} ({distR:5.3f} m, {fracR*100:5.1f}%) | "
                      f"base={base:.2f} spL={spL:.2f} spR={spR:.2f}")
                last_print = t

            sleep(DT)

        # Stop & settle
        robot.stop()
        sleep(0.20)

        # Final report
        L = encL.read(); R = encR.read()
        distL = L / tpmL
        distR = R / tpmR
        avg  = 0.5 * (distL + distR)
        print(f"\nReached target. "
              f"L={L:.1f} ticks ({distL:.3f} m) | R={R:.1f} ticks ({distR:.3f} m) | avg={avg:.3f} m "
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
