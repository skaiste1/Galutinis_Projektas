"""my_controller controller."""

import numpy

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

CELL_SIZE = 0.15
KP = 10.0

def rotate(target_dir):
    print("Rotating to direction:", target_dir)
    global leftMotor, rightMotor, timeStep, robot, cur_pos, cur_dir
    global compass
    while(numpy.abs(target_dir - cur_dir) > 0.01):
        angle_error = target_dir - cur_dir
        angle_error = (angle_error + numpy.pi) % (2 * numpy.pi) - numpy.pi
        print("Current direction:", cur_dir*180/numpy.pi, " Target direction    :", target_dir*180/numpy.pi, " Angle error:", angle_error*180/numpy.pi)   

        leftMotor.setVelocity( -KP * angle_error)
        rightMotor.setVelocity(+KP * angle_error)

        robot.step(timeStep)
        cur_pos = numpy.array(gps.getValues())
        cur_dir = (-numpy.arctan2(compass.getValues()[1], compass.getValues()[0]) + numpy.pi/2) % (2 * numpy.pi)
        cur_dir = (cur_dir + numpy.pi) % (2 * numpy.pi) - numpy.pi
    leftMotor.setVelocity (0.0)
    rightMotor.setVelocity(0.0)


def move_to(target_pos):
    global leftMotor, rightMotor, timeStep, robot 
    global gps
    global cur_pos, cur_dir
    print("Moving to position:", target_pos, " from ", cur_pos)

    target_dir = numpy.arctan2(target_pos[1] - cur_pos[1], target_pos[0] - cur_pos[0])
    target_dir = (target_dir + numpy.pi/4)//(numpy.pi/2) * numpy.pi/2  # snap to 90 degrees

    print("Target direction:", target_dir*180/numpy.pi, " from ", cur_dir*180/numpy.pi)

    if(numpy.abs(target_dir - cur_dir) > 0.1):
        rotate(target_dir)

    dist = numpy.linalg.norm(target_pos - cur_pos)
    #print("Distance to target:", dist)
    while(dist > 0.005):

        target_pos_dir = target_pos + CELL_SIZE * numpy.array([numpy.cos(target_dir), numpy.sin(target_dir), 0.0])
        angle_error = numpy.arctan2(target_pos_dir[1] - cur_pos[1],
                                    target_pos_dir[0] - cur_pos[0]) - cur_dir
        angle_error = (angle_error + numpy.pi) % (2 * numpy.pi) - numpy.pi

        print("Current position:", (cur_pos - abs_ref_pos)/CELL_SIZE, " Target position:", (target_pos-abs_ref_pos)/CELL_SIZE, 
              " Distance:", dist, " Target Angle Pos:", (target_pos_dir-abs_ref_pos)/CELL_SIZE, " Current Angle:", cur_dir*180/numpy.pi)

        print("Angle error:", angle_error*180/numpy.pi)

        leftMotor.setVelocity( cruiseVelocity - KP * angle_error)
        rightMotor.setVelocity(cruiseVelocity + KP * angle_error)

        robot.step(timeStep)
        cur_pos = numpy.array(gps.getValues())
        cur_dir = (-numpy.arctan2(compass.getValues()[1], compass.getValues()[0]) + numpy.pi/2) % (2 * numpy.pi)
        cur_dir = (cur_dir + numpy.pi) % (2 * numpy.pi) - numpy.pi
        dist = numpy.linalg.norm(target_pos - cur_pos)
    leftMotor.setVelocity (0.0)
    rightMotor.setVelocity(0.0)


# create the Robot instance.
robot = Robot()

# Get simulation step length.
timeStep = int(robot.getBasicTimeStep())

# Constants of the e-puck motors and distance sensors.
cruiseVelocity = 4.0
num_dist_sensors = 8

# Get left and right wheel motors.
leftMotor = robot.getDevice("left wheel motor")
rightMotor = robot.getDevice("right wheel motor")

# Get frontal distance sensors.
dist_sensors = [robot.getDevice('ps' + str(x)) for x in range(num_dist_sensors)]  # distance sensors
list(map((lambda s: s.enable(timeStep)), dist_sensors))  # Enable all distance sensors

# Disable motor PID control mode.
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))


# Set the initial velocity of the left and right wheel motors.
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

camera = robot.getDevice("camera")
camera.enable(timeStep)

gps = robot.getDevice("gps")
gps.enable(timeStep)

compass = robot.getDevice("compass")
compass.enable(timeStep)

# Step simulation to initialize sensors.
robot.step(timeStep)

abs_ref_pos = numpy.array(gps.getValues())
print("Initial position:", abs_ref_pos)
target_pos = abs_ref_pos.copy()

cur_pos = abs_ref_pos.copy()
cur_dir = numpy.arctan2(compass.getValues()[1], compass.getValues()[0])

# open commands file
commands_file = open("commands.txt", "r")   

while robot.step(timeStep) != -1:
    cur_pos = numpy.array(gps.getValues())
    cur_dir = (-numpy.arctan2(compass.getValues()[1], compass.getValues()[0]) + numpy.pi/2) % (2 * numpy.pi)
    cur_dir = (cur_dir + numpy.pi) % (2 * numpy.pi) - numpy.pi

    # Read next movement from file
    command = commands_file.readline().strip()
    if command == "N":
        print("Moving North")
        target_pos[1] += CELL_SIZE
    elif command == "S":
        print("Moving South")
        target_pos[1] -= CELL_SIZE
    elif command == "E":
        print("Moving East")
        target_pos[0] += CELL_SIZE
    elif command == "W":
        print("Moving West")
        target_pos[0] -= CELL_SIZE
    move_to(target_pos)
