### Joystick service starts on boot

#### To stop the service run:
sudo systemctl stop drive_joystick.service

#### To start the service again:
sudo systemctl start drive_joystick.service

#### See its current status and whether it's enabled to start on boot
sudo systemctl status drive_joystick.service

#### Stop it, disable it for future boots, and prevent accidental starts
sudo systemctl stop drive_joystick.service
sudo systemctl disable drive_joystick.service
sudo systemctl mask drive_joystick.service

position-instruction.service

#### Reload systemd units just in case
sudo systemctl daemon-reload

pigpiod

#### Testing wheels and encoders
python src/single_car/drive_forward_distance.py

### API Call ###
python3 -m uvicorn src.car_server.server:app --host 0.0.0.0 --port 8000
#### Need to know indivdual ip addresses of cars
Plug api addresses into web browser
hostname -I


# Create a service
sudo nano /etc/systemd/system/car_server.service

## Here
[Unit]
Description=FastAPI Car Server
After=network.target

[Service]
Type=simple
User=admin
WorkingDirectory=/home/admin/agentic-coordination
ExecStart=/usr/bin/python3 -m uvicorn src.car_server.server:app --host 0.0.0.0 --port 8000
Restart=always
RestartSec=3

# OPTIONAL: environment variables if you need them
# Environment="PYTHONUNBUFFERED=1"

StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target

## Reload systemd
sudo systemctl daemon-reload

## Start service now
sudo systemctl start car_server.service

## Check that service is running
sudo systemctl status car_server.service

## Enable on boot
sudo systemctl enable car_server.service
