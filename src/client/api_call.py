import requests

url = "http://172.28.163.77:8000/move"

payload = {
    "x": 1.0,  # meters
    "y": 0.0,  # meters
    "psi": 0.0  # yaw in degrees
}

response = requests.post(url, json=payload)

print("Status Code:", response.status_code)
print("Response Body:", response.json())