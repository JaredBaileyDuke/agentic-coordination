import threading
import time
import math
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel

# === Import your robot controller functions ===
from src.single_car.position_instruction import robot, SimpleEncoder, wrap_pi, pwm_from_velocity

app = FastAPI()

# ----------------------------
# GLOBAL STATE
# ----------------------------
motion_thread = None
stop_event = threading.Event()
state_lock = threading.Lock()
is_moving = False

# Store last commanded PWM globally so drive_to_pose can access it
last_pwm = {"left": 0.0, "right": 0.0}
pwm_lock = threading.Lock()


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

    global is_moving, last_pwm

    # Mark running
    with state_lock:
        is_moving = True

    encL = None
    encR = None

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
        PRINT_INTERVAL = 0.5

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

        RHO_TOL_M = 0.08
        TH_TOL_DEG = 8.0
        MAX_RUNTIME_S = 60.0

        # Reset last PWM
        with pwm_lock:
            last_pwm = {"left": 0.0, "right": 0.0}

        # Give encoders time to initialize
        time.sleep(0.2)
        encL.reset()
        encR.reset()

        start_time = time.time()
        last_print = start_time

        print(f"Starting motion to x={goal_x:.3f}, y={goal_y:.3f}, psi={psi_goal_deg:.1f}°")

        # ----------------------
        # (2) Main control loop
        # ----------------------
        while not stop_event.is_set():

            now = time.time()

            # ----------------------
            # Encoder update with DIRECTION INFERENCE
            # ----------------------
            currL = encL.read()
            currR = encR.read()
            dL_ticks = currL - lastL
            dR_ticks = currR - lastR
            lastL = currL
            lastR = currR

            dL = dL_ticks / tpmL
            dR = dR_ticks / tpmR

            # *** CRITICAL FIX: Infer direction from last commanded PWM ***
            with pwm_lock:
                pwmL_cmd = last_pwm["left"]
                pwmR_cmd = last_pwm["right"]
            
            signL = 1.0 if pwmL_cmd >= 0 else -1.0
            signR = 1.0 if pwmR_cmd >= 0 else -1.0

            dL_signed = dL * signL
            dR_signed = dR * signR

            # Odometry update
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

            # ----------------------
            # Compute errors
            # ----------------------
            dx = goal_x - x
            dy = goal_y - y
            rho = math.hypot(dx, dy)
            path_heading = math.atan2(dy, dx)
            alpha = wrap_pi(path_heading - th)
            beta = wrap_pi(goal_th - th - alpha)
            
            th_err = abs(math.degrees(wrap_pi(goal_th - th)))

            # ----------------------
            # Goal test
            # ----------------------
            if rho <= RHO_TOL_M and th_err <= TH_TOL_DEG:
                print(f"Goal reached! Final: x={x:.3f}, y={y:.3f}, th={math.degrees(th):.1f}°")
                break

            # Timeout check
            if (now - start_time) > MAX_RUNTIME_S:
                print("Timeout reached, stopping.")
                break

            # ----------------------
            # Controller
            # ----------------------
            v = K_RHO * rho
            w = K_ALPHA * alpha + K_BETA * beta

            # Scale down when close
            v_scale = 1.0
            if rho < 0.15:
                v_scale = 0.6
            if rho < 0.07:
                v_scale = 0.45

            vL = v - (WHEEL_BASE_M * 0.5) * w
            vR = v + (WHEEL_BASE_M * 0.5) * w

            vmax = max(abs(vL), abs(vR), 1e-6)
            vL /= vmax
            vR /= vmax

            pwmL, pwmR = pwm_from_velocity(vL, vR, v_scale=v_scale)

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

            # Store commanded PWM for next iteration's direction inference
            with pwm_lock:
                last_pwm["left"] = pwmL
                last_pwm["right"] = pwmR

            # Telemetry
            if (now - last_print) >= PRINT_INTERVAL:
                print(f"x={x:5.3f} m, y={y:5.3f} m, th={math.degrees(th):6.2f}° | "
                      f"ρ={rho:4.2f} m, α={math.degrees(alpha):6.2f}°, β={math.degrees(beta):6.2f}° | "
                      f"pwmL={pwmL:+.2f}, pwmR={pwmR:+.2f}")
                last_print = now

            time.sleep(DT)

    except Exception as e:
        print(f"Error in drive_to_pose: {e}")
        import traceback
        traceback.print_exc()

    finally:
        # Always stop motors and clean up
        robot.stop()
        
        if encL is not None:
            encL.close()
        if encR is not None:
            encR.close()
        
        with state_lock:
            is_moving = False
        
        with pwm_lock:
            last_pwm = {"left": 0.0, "right": 0.0}
        
        print("Motion thread finished.")


# ================================================================
# API ENDPOINTS
# ================================================================

@app.post("/move")
def move(req: MoveRequest):
    global motion_thread, stop_event, is_moving

    with state_lock:
        if is_moving:
            raise HTTPException(400, "Car is already moving. Call /stop first.")

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

    print("Stop requested via API")
    stop_event.set()   # Tell drive loop to stop
    robot.stop()       # Immediately stop motors

    return {"status": "stopping"}


@app.get("/status")
def status():
    with state_lock:
        return {"moving": is_moving}


@app.on_event("shutdown")
def shutdown_event():
    """Ensure motors stop when server shuts down"""
    print("Server shutting down, stopping motors...")
    stop_event.set()
    robot.stop()