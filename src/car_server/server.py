# src/car_server/server.py

import threading
import time
import math
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel

# === Import your robot controller functions ===
from src.single_car.position_instruction import (
    robot, 
    SimpleEncoder, 
    wrap_pi, 
    phase1_navigate_to_position,
    phase2_rotate_to_heading,
    ENCODER_L_A_PIN,
    ENCODER_R_A_PIN,
    WHEEL_DIAMETER_L_M,
    WHEEL_DIAMETER_R_M,
    PULSES_PER_REV,
    EDGES_PER_PULSE,
    GEAR_RATIO,
    GLOBAL_TIMEOUT_S
)

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
# INTERNAL: Drive Function (now uses position_instruction functions)
# ================================================================
def drive_to_pose(x_goal, y_goal, psi_goal_deg):
    """
    Drives robot to goal pose using the two-phase controller from position_instruction.py
    """
    global is_moving

    # Mark running
    with state_lock:
        is_moving = True

    encL = None
    encR = None

    try:
        # Setup
        goal_th = math.radians(psi_goal_deg % 360.0)
        goal_th = wrap_pi(goal_th)

        encL = SimpleEncoder(ENCODER_L_A_PIN)
        encR = SimpleEncoder(ENCODER_R_A_PIN)

        # Precompute ticks-per-meter
        circL = math.pi * WHEEL_DIAMETER_L_M
        circR = math.pi * WHEEL_DIAMETER_R_M
        ticks_per_rev = PULSES_PER_REV * EDGES_PER_PULSE * GEAR_RATIO
        tpmL = ticks_per_rev / circL
        tpmR = ticks_per_rev / circR

        print(f"\nStarting motion to x={x_goal:.3f}, y={y_goal:.3f}, psi={psi_goal_deg:.1f}°")
        print(f"⏱️ Global timeout: {GLOBAL_TIMEOUT_S:.1f} seconds")

        # Reset encoders
        time.sleep(0.2)
        encL.reset()
        encR.reset()

        overall_start = time.time()

        # PHASE 1: Navigate to position
        x, y, th = phase1_navigate_to_position(
            x_goal, y_goal, encL, encR, tpmL, tpmR, overall_start
        )

        # Check if we should stop (timeout or stop_event)
        if stop_event.is_set():
            print("Stop requested during Phase 1")
            return

        if (time.time() - overall_start) > GLOBAL_TIMEOUT_S:
            print(f"Global timeout reached after Phase 1")
            return

        # PHASE 2: Rotate to heading
        x, y, th = phase2_rotate_to_heading(
            goal_th, x, y, th, encL, encR, tpmL, tpmR, overall_start
        )

        # Check if we should stop
        if stop_event.is_set():
            print("Stop requested during Phase 2")
            return

        # Final report
        elapsed = time.time() - overall_start
        if elapsed > GLOBAL_TIMEOUT_S:
            print(f"Mission stopped (global timeout)")
        else:
            print(f"Mission complete!")

        print(f"Final pose: x={x:.3f} m, y={y:.3f} m, θ={math.degrees(th):.1f}°")
        print(f"Goal pose:  x={x_goal:.3f} m, y={y_goal:.3f} m, θ={psi_goal_deg:.1f}°")
        print(f"Total time: {elapsed:.2f}s")

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

        print(f"Motion thread finished.")


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

    return {"status": "started", "goal": req.dict(), "timeout_seconds": GLOBAL_TIMEOUT_S}


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