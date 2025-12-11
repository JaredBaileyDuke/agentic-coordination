import cv2
import cv2.aruco as aruco
import numpy as np
import time

# ================================
# USER SETTINGS
# ================================
CAR_MARKERS = [8, 10]
FLOOR_MARKER = 13

marker_sizes_mm = {
    8: 6.1,   # car 1 marker size
    10: 6.1,  # car 2 marker size
    13: 16.0  # floor marker size
}

CAR_MARKER_HEIGHT_MM = 11.7  # height of car markers above ground

PRINT_RATE_HZ = 5
PRINT_PERIOD = 1.0 / PRINT_RATE_HZ

# ================================
# CAMERA SETUP
# ================================
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise Exception("Could not open webcam")

# Default ArUco dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)

# =======================================================
# IMPORTANT: Camera calibration parameters REQUIRED
# =======================================================
# These MUST be replaced with your calibrated values!
# Using dummy values for now so script runs.
# ------------------------
fx, fy = 800, 800
cx, cy = 640, 360
camera_matrix = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0,  0,  1]], dtype=float)

dist_coeffs = np.zeros((5, 1))  # assuming no distortion

# ================================
# HELPER FUNCTIONS
# ================================

def estimate_pose(corners, marker_length_mm):
    """Returns rvec, tvec for a given marker."""
    marker_length_m = marker_length_mm / 1000.0
    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
        corners,
        marker_length_m,
        camera_matrix,
        dist_coeffs
    )
    return rvec[0][0], tvec[0][0]


def yaw_from_rvec(rvec):
    """Extract yaw angle (psi) in degrees from OpenCV rotation vector."""
    R, _ = cv2.Rodrigues(rvec)

    # yaw = rotation around Z axis
    yaw = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    return yaw


def transform_to_floor_frame(car_t, car_rvec, floor_t, floor_rvec):
    """
    Convert car pose (tvec, rvec) into reference frame of floor marker.
    """
    # Convert rvecs to rotation matrices
    R_car, _ = cv2.Rodrigues(car_rvec)
    R_floor, _ = cv2.Rodrigues(floor_rvec)

    # Transformation camera->floor
    R_cf = R_floor.T
    t_cf = -R_floor.T @ floor_t

    # Transform camera->car into floor frame
    t_car_floor = R_cf @ car_t + t_cf
    R_car_floor = R_cf @ R_car

    # Extract yaw
    yaw_car = np.degrees(np.arctan2(R_car_floor[1, 0], R_car_floor[0, 0]))
    
    return t_car_floor, yaw_car


# ================================
# MAIN LOOP
# ================================
print("Tracking Cars 08 & 10 relative to Floor Marker 13")
print("Running at 5 Hz... Press 'q' to quit.")

last_print = 0

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None:
        ids = ids.flatten()

        # Draw detection
        aruco.drawDetectedMarkers(frame, corners, ids)

        # Store poses
        poses = {}

        for i, marker_id in enumerate(ids):
            if marker_id in marker_sizes_mm:
                rvec, tvec = estimate_pose(corners[i], marker_sizes_mm[marker_id])
                poses[marker_id] = (rvec, tvec)

                # Draw axes for visualization
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.03)

        # We must have the floor marker to get global coordinates
        if FLOOR_MARKER in poses:
            r_floor, t_floor = poses[FLOOR_MARKER]

            # Print at fixed rate
            if time.time() - last_print >= PRINT_PERIOD:
                last_print = time.time()

                print("\n--- Car Positions (relative to Floor Marker 13) ---")

                for car_id in CAR_MARKERS:
                    if car_id in poses:
                        r_car, t_car = poses[car_id]

                        # Convert pose into floor frame
                        t_rel, yaw_deg = transform_to_floor_frame(
                            t_car, r_car,
                            t_floor, r_floor
                        )

                        # Correct Z height because markers are 11.7 mm above ground
                        x = t_rel[0]
                        y = t_rel[1]

                        # Print output
                        print(
                            f"Car {car_id}: Orientation: {yaw_deg:6.2f}°, "
                            f"X Position: {x:7.3f} m, "
                            f"Y Position: {y:7.3f} m"
                        )

    cv2.imshow("ArUco Tracking", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
