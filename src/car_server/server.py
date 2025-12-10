import threading
import time
import math
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel

# === Import your robot controller functions ===
# We wrap your long "main()" into a callable so the thread can run it safely.
from src.single_car.position_instruction import robot, SimpleEncoder, wrap_pi, pwm_from_velocity
import sys

app = FastAPI()

# ----------------------------
# GLOBAL STATE
# ----------------------------
motion_thread = None
stop_event = threading.Event()
state_lock = threading.Lock()
is_moving = False


# ----------------------------
# MOVE REQUEST MODEL
# ----------------------------
class MoveRequest(BaseModel):
    x: float
    y: float
    psi: float  # degrees


# ================================================================
# INTERNAL: Drive Function (refactored from your original script)
# ================================================================
def drive_to_pose(x_goal, y_goal, psi_goal_deg):
    """
    A thread-safe wrapper around your existing drive algorithm.
    Runs until the robot reaches the goal OR stop_event is set.
    """

    global is_moving

    # Mark running
    with state_lock:
        is_moving = True

    try:
        # ----------------------
        # (1) Setup from your script
        # ----------------------
        goal_x = x_goal
        goal_y = y_goal
        goal_th = math.radians(psi_goal_deg % 360.0)
        goal_th = wrap_pi(goal_th)

        encL = SimpleEncoder(27)
        encR = SimpleEncoder(17)

        # Constants from your script
        WHEEL_DIAMETER_L_M = 0.06670
        WHEEL_DIAMETER_R_M = 0.06701
        PULSES_PER_REV = 20
        EDGES_PER_PULSE = 1
        GEAR_RATIO = 1.0
        WHEEL_BASE_M = 0.135
        DT = 0.02

        circL = math.pi * WHEEL_DIAMETER_L_M
        circR = math.pi * WHEEL_DIAMETER_R_M
        ticks_per_rev = PULSES_PER_REV * EDGES_PER_PULSE * GEAR_RATIO
        tpmL = ticks_per_rev / circL
        tpmR = ticks_per_rev / circR

        # Odometry
        x = 0.0
        y = 0.0
        th = 0.0
        lastL = 0
        lastR = 0

        # Controller gains
        K_RHO = 1.20
        K_ALPHA = 2.70
        K_BETA = -0.60

        RHO_TOL_M = 0.02
        TH_TOL_DEG = 3.0

        start_time = time.time()

        # ----------------------
        # (2) Main control loop
        # ----------------------
        while not stop_event.is_set():

            # ----------------------
            # Goal test
            # ----------------------
            dx = goal_x - x
            dy = goal_y - y
            rho = math.hypot(dx, dy)
            th_err = abs(math.degrees(wrap_pi(goal_th - th)))

            if rho <= RHO_TOL_M and th_err <= TH_TOL_DEG:
                break

            # ----------------------
            # Encoder update
            # ----------------------
            currL = encL.read()
            currR = encR.read()
            dL_ticks = currL - lastL
            dR_ticks = currR - lastR
            lastL = currL
            lastR = currR

            dL = dL_ticks / tpmL
            dR = dR_ticks / tpmR

            ds = 0.5 * (dR + dL)
            dth = (dR - dL) / WHEEL_BASE_M

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

            # ----------------------
            # Controller
            # ----------------------
            path_heading = math.atan2(dy, dx)
            alpha = wrap_pi(path_heading - th)
            beta = wrap_pi(goal_th - th - alpha)

            v = K_RHO * rho
            w = K_ALPHA * alpha + K_BETA * beta

            vL = v - (WHEEL_BASE_M * 0.5) * w
            vR = v + (WHEEL_BASE_M * 0.5) * w

            vmax = max(abs(vL), abs(vR), 1e-6)
            vL /= vmax
            vR /= vmax

            pwmL, pwmR = pwm_from_velocity(vL, vR, v_scale=1.0)

            # ----------------------
            # Apply motor commands
            # ----------------------
            if pwmL >= 0:
                robot.left_motor.forward(abs(pwmL))
            else:
                robot.left_motor.backward(abs(pwmL))

            if pwmR >= 0:
                robot.right_motor.forward(abs(pwmR))
            else:
                robot.right_motor.backward(abs(pwmR))

            time.sleep(DT)

        # Stop when finished OR canceled
        robot.stop()

        encL.close()
        encR.close()

    finally:
        with state_lock:
            is_moving = False
        robot.stop()


# ================================================================
# API ENDPOINTS
# ================================================================

@app.post("/move")
def move(req: MoveRequest):
    global motion_thread, stop_event, is_moving

    with state_lock:
        if is_moving:
            raise HTTPException(400, "Car is already moving.")

        # Clear any old stop signal
        stop_event.clear()

        # Start background motion thread
        motion_thread = threading.Thread(
            target=drive_to_pose,
            args=(req.x, req.y, req.psi),
            daemon=True
        )
        motion_thread.start()

    return {"status": "started", "goal": req.dict()}


@app.post("/stop")
def stop():
    global stop_event

    stop_event.set()   # Tell drive loop to stop

    robot.stop()

    return {"status": "stopping"}


@app.get("/status")
def status():
    with state_lock:
        return {"moving": is_moving}
