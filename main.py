import math
import time
from hub import port, button, motion_sensor
import motor
import motor_pair
import color
import app

# Define motor ports
lWheel = port.E
rWheel = port.F

# Robot parameters
wheelRadius = 5.6# cm (For the blue spike prime wheels)
wheelCircumference = 2 * math.pi * wheelRadius
Kp = 2 #see https://fll-pigeons.github.io/gamechangers/gyro_pid.html
DEBUG = False # If you want to see line graphs

# Initialize motor pair
motor_pair.pair(motor_pair.PAIR_1, lWheel, rWheel)


def convertRotationToDistance(degrees):
    return wheelCircumference * (degrees / 360)


def adjustMotorsBasedOnHeading(desiredAngle, currentPosition):
    # Get current heading angle from motion sensor
    currentAngle = motion_sensor.tilt_angles()[0] / 10# Assuming angle is in degrees

    # Calculate error angle
    errorAngle = desiredAngle - currentAngle

    # Calculate duty cycle adjustment based on error angle
    dutyCycleAdjustment = Kp * errorAngle

    # Plot some data
    if DEBUG:
        app.linegraph.plot(color.PURPLE, currentPosition, currentAngle)
        app.linegraph.plot(color.YELLOW, currentPosition, errorAngle)
        app.linegraph.plot(color.BLUE, currentPosition, desiredAngle)

    # Apply duty cycles to motors
    motor_pair.move(motor_pair.PAIR_1, -int(dutyCycleAdjustment), velocity=100)


def followPath(testPath):
    maxDistance = max(testPath.keys())
    motor.reset_relative_position(lWheel, 0)
    motor.reset_relative_position(rWheel, 0)

    position = 0
    prevLeftPosition = 0
    prevRightPosition = 0

    while position < maxDistance:
        currentLeftPosition = motor.relative_position(lWheel)
        currentRightPosition = motor.relative_position(rWheel)
        leftDelta = abs(currentLeftPosition - prevLeftPosition)
        rightDelta = abs(currentRightPosition - prevRightPosition)
        leftDistTravelled = convertRotationToDistance(leftDelta)
        rightDistTravelled = convertRotationToDistance(rightDelta)
        position += (leftDistTravelled + rightDistTravelled) / 2

        # Find closest waypoints
        closestDistances = sorted(testPath.keys(), key=lambda k: abs(k - position))
        pos1 = testPath[closestDistances[0]]

        heading = pos1 # The closest angle to this distance

        # Adjust motors based on current heading
        adjustMotorsBasedOnHeading(heading, position)

        # Update previous positions
        prevLeftPosition = currentLeftPosition
        prevRightPosition = currentRightPosition

        # Delay for control loop
        time.sleep(0.01)

    # Stop motors at the end of the path
    motor.stop(lWheel)
    motor.stop(rWheel)


def get_data():
    data = {}
    position = 0
    prevLeftPosition = 0
    prevRightPosition = 0

    while not button.pressed(button.LEFT):
        currentLeftPosition = motor.relative_position(lWheel)
        currentRightPosition = motor.relative_position(rWheel)
        leftDelta = abs(currentLeftPosition - prevLeftPosition)
        rightDelta = abs(currentRightPosition - prevRightPosition)
        leftDistTravelled = convertRotationToDistance(leftDelta)
        rightDistTravelled = convertRotationToDistance(rightDelta)
        # Calculate distance based on distance
        
        if ((leftDistTravelled + rightDistTravelled) / 2) > 0.5: # Don't overload the limited spike prime memory
            position += (leftDistTravelled + rightDistTravelled) / 2
            data[position] = motion_sensor.tilt_angles()[0] / 10

            prevLeftPosition = currentLeftPosition
            prevRightPosition = currentRightPosition

        # Delay for control loop
        time.sleep(0.01)

    return data


# Main execution
while True:
    app.linegraph.clear_all()
    testPath = get_data()
    time.sleep(2)
    followPath(testPath)
    time.sleep(10)# Repeat as many times as you want
