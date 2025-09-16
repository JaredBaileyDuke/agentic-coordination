from gpiozero import Robot, Motor
from time import sleep

# Define robot with left and right motors
robot = Robot(
    left=Motor(forward=23, backward=24, enable=18, pwm=True),
    right=Motor(forward=5, backward=25, enable=19, pwm=True)
)

try:
    while True:
        print("Forward at 70% speed")
        robot.forward(speed=0.7)   # Speed range: 0.0 to 1.0
        sleep(5)

        print("Backward at 70% speed")
        robot.backward(speed=0.7)
        sleep(5)

except KeyboardInterrupt:
    print("Stopping...")
    robot.stop()

