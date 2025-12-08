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
#### Need to know indivdual ip addresses