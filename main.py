"""
Mark 01 - Autonomous GoToGoal robot with L298N driver
"""

import time
import math
import RPi.GPIO as GPIO
from gusbots import encoder, localization, stateControl

# L298N Motor Driver Pins
ENA = 12  # PWM pin for left motor speed
IN1 = 22  # Direction pin 1 for left motor
IN2 = 23  # Direction pin 2 for left motor
ENB = 13  # PWM pin for right motor speed
IN3 = 17  # Direction pin 1 for right motor
IN4 = 27  # Direction pin 2 for right motor

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENB, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

# Create PWM instances
left_pwm = GPIO.PWM(ENA, 1000)  # 1000Hz frequency
right_pwm = GPIO.PWM(ENB, 1000)
left_pwm.start(0)
right_pwm.start(0)


def set_motor_speeds(left, right):  ## Get ftom state control  motor speed output
    # Left motor
    if left >= 0:
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
    else:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
    left_pwm.ChangeDutyCycle(min(abs(left) * 100, 100))

    # Right motor
    if right >= 0:
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
    else:
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)
    right_pwm.ChangeDutyCycle(min(abs(right) * 100, 100))


# Used to manage how fast the main loop runs


# Initialize localization
wheel_radius = 0.0335
wheel_base = 0.222

# Initialize the encoder. Both have 40 ticks per resolution.
left_wheel_encoder = encoder.encoder(24, 11, wheel_radius)
right_wheel_encoder = encoder.encoder(25, 11, wheel_radius)

odo = localization.odometry(left_wheel_encoder, right_wheel_encoder, wheel_base)
stControl = stateControl.stateControl()

last_left_dir = 1
last_right_dir = 1

x = 0
y = 0
theta = 0

# Start in autonomous mode
stControl.input.autonomous = 1

last_reset_time = time.time()
RESET_INTERVAL = 60

# Main program loop
try:
    start_time = time.time_ns()

    while True:
        # Calculate how many ns passed since last read   ## old = 100 , current  150 , interval = 150 - old = 50
        t = time.time_ns()
        dt = t - start_time
        start_time = t

        # Run odometry to update robot location
        odo.step(last_left_dir, last_right_dir)

        if time.time() - last_reset_time > RESET_INTERVAL:
            odo.resetPose()
            left_wheel_encoder.reset()
            right_wheel_encoder.reset()
            last_reset_time = time.time()  # ← MUST reset timer!
            print("Pose and encoders reset")

        # Get pose values
        x, y, theta = odo.getPose()

        # Set inputs for the state machine
        stControl.input.joy_left = 0
        stControl.input.joy_right = 0
        stControl.input.autonomous = 1  # Always autonomous
        stControl.input.x = x
        stControl.input.y = y
        stControl.input.theta = theta
        stControl.input.dt = dt
        stControl.input.el = left_wheel_encoder
        stControl.input.er = right_wheel_encoder
        stControl.input.L = wheel_base

        # Run state machine
        stControl.step()

        # Update motor outputs
        set_motor_speeds(stControl.output.left_motor, stControl.output.right_motor)

        # The encoder can not know the direction of the motor
        last_left_dir = 1 if stControl.output.left_motor >= 0 else -1
        last_right_dir = 1 if stControl.output.right_motor >= 0 else -1

        # Print some info
        theta_d = theta - (2 * math.pi * math.floor((theta + math.pi) / (2 * math.pi)))
        theta_d = theta * 180 / math.pi

        print(
            "L/R:",
            left_wheel_encoder.counter,
            right_wheel_encoder.counter,
            "x:",
            "{:02.3f}".format(x),
            "y:",
            "{:02.3f}".format(y),
            "theta:",
            "{:02.2f}".format(theta_d),
        )

        # Limit to 10 frames per second. (100ms loop)
        time.sleep(0.1)

except KeyboardInterrupt:
    # Press Ctrl+C to exit the application
    pass

# Exiting application (clean up)
set_motor_speeds(0, 0)
left_pwm.stop()
right_pwm.stop()
GPIO.cleanup()
