# app.py
import streamlit as st
import time
import cv2
import numpy as np
from find_cars import car_state, latest_frame

st.set_page_config(page_title="Car Pose Tracker", layout="wide")

st.title("🚗 Live Car Tracking with ArUco Markers")

placeholder_car1 = st.empty()
placeholder_car2 = st.empty()
placeholder_image = st.empty()

REFRESH_RATE = 1/5  # 5 Hz

while True:
    # --- Display Car 08 ---
    car1 = car_state[8]
    placeholder_car1.markdown(
        f"""
        ### Car 08  
        **Orientation:** {car1['yaw']:.2f}°  
        **X Position:** {car1['x']:.3f} m  
        **Y Position:** {car1['y']:.3f} m  
        """
    )

    # --- Display Car 10 ---
    car2 = car_state[10]
    placeholder_car2.markdown(
        f"""
        ### Car 10  
        **Orientation:** {car2['yaw']:.2f}°  
        **X Position:** {car2['x']:.3f} m  
        **Y Position:** {car2['y']:.3f} m  
        """
    )

    # --- Show Camera Frame ---
    if latest_frame is not None:
        frame_rgb = cv2.cvtColor(latest_frame, cv2.COLOR_BGR2RGB)
        placeholder_image.image(frame_rgb, caption="Live Camera", use_column_width=True)

    time.sleep(REFRESH_RATE)
