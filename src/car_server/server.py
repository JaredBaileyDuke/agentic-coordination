import subprocess
import threading
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import Optional

app = FastAPI()

# Path to your movement script
SCRIPT_PATH = "/home/admin/agentic-coordination/src/single_car/position_instruction.py"

# Global process handle + lock
move_process: Optional[subprocess.Popen] = None
process_lock = threading.Lock()


# ---------------------------
# Pydantic model for /move
# ---------------------------
class MoveRequest(BaseModel):
    x: float
    y: float
    psi: float  # degrees


# ---------------------------
# Endpoint: Move to pose
# ---------------------------
@app.post("/move")
def move_car(req: MoveRequest):
    global move_process

    with process_lock:
        # If already moving, reject or automatically stop (your choice)
        if move_process is not None and move_process.poll() is None:
            raise HTTPException(status_code=400, detail="Car is already moving.")

        # Start a new background process running your script
        cmd = [
            "python3",
            SCRIPT_PATH,
            str(req.x),
            str(req.y),
            str(req.psi)
        ]

        try:
            move_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True
            )
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"Failed to start script: {e}")

    return {"status": "started", "x": req.x, "y": req.y, "psi": req.psi}


# ---------------------------
# Endpoint: Stop motion
# ---------------------------
@app.post("/stop")
def stop_car():
    global move_process

    with process_lock:
        if move_process is None or move_process.poll() is not None:
            return {"status": "idle", "message": "Car was not moving."}

        try:
            move_process.terminate()     # try graceful
            move_process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            move_process.kill()          # force kill if needed
        finally:
            move_process = None

    return {"status": "stopped"}


# ---------------------------
# Endpoint: Query motion status
# ---------------------------
@app.get("/status")
def get_status():
    global move_process

    with process_lock:
        if move_process is None:
            return {"moving": False}

        # poll() returns None if still running
        is_running = move_process.poll() is None
        return {"moving": is_running}
