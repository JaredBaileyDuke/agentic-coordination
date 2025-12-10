# src/client/api_call.py
import requests

BASE_URL = "http://172.28.163.77:8000"  # default


def set_base_url(url):
    global BASE_URL
    BASE_URL = url


def move_car(x, y, psi):
    url = f"{BASE_URL}/move"
    payload = {"x": x, "y": y, "psi": psi}
    r = requests.post(url, json=payload)
    return r.status_code, r.json()


def stop_car():
    url = f"{BASE_URL}/stop"
    r = requests.post(url)
    return r.status_code, r.json()


def car_status():
    url = f"{BASE_URL}/status"
    r = requests.get(url)
    return r.status_code, r.json()
