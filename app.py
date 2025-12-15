# app.py

import streamlit as st
from src.client.api_call import move_car, stop_car, car_status, set_base_url

st.set_page_config(page_title="Robot Car Controller", layout="wide")

# -----------------------------------------------------
# WORKING CSS FOR LARGE, CENTERED RADIO BUTTONS
# -----------------------------------------------------
st.markdown("""
<style>

    /* Reduce vertical padding around radio widget */
    div[data-testid="stRadio"] {
        margin-top: -5px !important;
        margin-bottom: -10px !important;
        padding: 0 !important;
    }

    /* Center the radio group horizontally */
    div[data-testid="stRadio"] > div {
        display: flex !important;
        justify-content: center !important;
        align-items: center !important;
    }

    /* Force horizontal layout of radio buttons */
    div[data-testid="stRadio"] div[role="radiogroup"] {
        display: flex !important;
        flex-direction: row !important;
        gap: 60px !important; /* spacing between options */
    }

    /* Large, styled label text */
    div[data-testid="stRadio"] label {
        font-family: "Gill Sans", sans-serif !important;
        color: #c1d1d4 !important;
        font-size: 2.2rem !important;
        font-weight: 300 !important;
        line-height: 1.4 !important;
        cursor: pointer !important;
        padding: 8px !important;
    }

    /* Ensure nested elements in the label also inherit font */
    div[data-testid="stRadio"] label * {
        font-family: "Gill Sans", sans-serif !important;
        color: #c1d1d4 !important;
        font-size: 2.2rem !important;
        font-weight: 300 !important;
    }

    /* Enlarge the radio circle itself */
    div[data-testid="stRadio"] input[type="radio"] {
        width: 28px !important;
        height: 28px !important;
        transform: scale(1.8) !important;
        margin-right: 12px !important;
    }

</style>
""", unsafe_allow_html=True)


# -----------------------------------------------------
# CENTERED RADIO BUTTONS
# -----------------------------------------------------
padL, colMid, padR = st.columns([1, 2, 1])

with colMid:
    car_choice = st.radio(
        "",
        ["Car 1", "Car 2"],
        horizontal=True
    )


# -----------------------------------------------------
# UPDATE BASE URL BASED ON SELECTION
# -----------------------------------------------------
if car_choice == "Car 1":
    set_base_url("http://172.28.163.77:8000")
else:
    set_base_url("http://172.28.68.179:8000")

st.markdown("---")


# =====================================================
# MAIN UI LAYOUT
# =====================================================
pad_left, col_left, col_right, pad_right = st.columns([1, 2, 2, 1])

# SLIDERS COLUMN
with col_left:
    st.subheader("🎯 Set Goal Position")

    x = st.slider("X Position (meters)", -1.0, 1.0, 0.0, 0.01)
    y = st.slider("Y Position (meters)", -1.0, 1.0, 0.0, 0.01)
    psi = st.slider("Psi (Yaw Degrees)", 0, 359, 0)


# COMMANDS COLUMN
with col_right:
    st.subheader("🔧 Commands")

    if st.button("➡️ Move Car", use_container_width=True):
        code, resp = move_car(x, y, psi)
        st.session_state["last_response"] = (code, resp)

    if st.button("🛑 Stop Car", use_container_width=True):
        code, resp = stop_car()
        st.session_state["last_response"] = (code, resp)

    if st.button("📡 Car Status", use_container_width=True):
        code, resp = car_status()
        st.session_state["last_response"] = (code, resp)

    st.markdown("---")

    if "last_response" in st.session_state:
        code, resp = st.session_state["last_response"]
        st.code(f"Status: {code}\nResponse:\n{resp}")
    else:
        st.code("No API calls yet.")