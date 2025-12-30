"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
"""from controller import Robot

# create the Robot instance.
robot = Robot()

# Get simulation step length.
timeStep = int(robot.getBasicTimeStep())

# Constants of the e-puck motors and distance sensors.
cruiseVelocity = 5.0
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
leftMotor.setVelocity(cruiseVelocity)
rightMotor.setVelocity(cruiseVelocity)

camera = robot.getDevice("camera")
camera.enable(timeStep)


near_front = 100
near_side = 90

while robot.step(timeStep) != -1:

    # if(robot.getTime()>10.0):
        # exit(0)

    dist_sensor_values = [g.getValue() for g in dist_sensors]
    
    print(dist_sensor_values)

    if dist_sensor_values[0] > near_front \
       or dist_sensor_values[7] > near_front:
        print('rotate')
        leftMotor.setVelocity (-cruiseVelocity)
        rightMotor.setVelocity( cruiseVelocity)
    elif dist_sensor_values[1]>near_side and dist_sensor_values[1]>dist_sensor_values[6]:
        print('turn left')
        leftMotor.setVelocity ( 0.1*cruiseVelocity)
        rightMotor.setVelocity( 1.2*cruiseVelocity)
    elif dist_sensor_values[6]>near_side:
        print('turn right')
        leftMotor.setVelocity ( 1.2*cruiseVelocity)
        rightMotor.setVelocity( 0.1*cruiseVelocity)
    else:
        print('go')
        leftMotor.setVelocity(cruiseVelocity)
        rightMotor.setVelocity(cruiseVelocity)
    """
        

from enum import Enum
from collections import deque
from controller import Robot
import numpy as np

KP_CENTER = 0.002 
KD_CENTER = 0.0008    

MAX_SPEED = 6.27

SENSOR_FLOOR = 67.0

START_ROTATION_THRESHOLD = 10.0
MOVE_AHEAD_THRESHOLD = 5.0

NUM_SENSORS = 8

class CircularBuffer:
    def __init__(self, size):
        self.buffer = deque(maxlen=size)

    def add(self, value):
        self.buffer.append(value)

    def get_all(self):
        return list(self.buffer)

    def get_average(self):
        if not self.buffer:
            return 0.0
        return np.mean(self.buffer)

    def is_full(self):
        return len(self.buffer) == self.buffer.maxlen

class PDController:
    def __init__(self, dt):
        self.dt = dt
        self.prev_error = None 

        self.d_filter = 0.0
        self.d_alpha = 0.88  

    def control(self, error, Kp, Kd):
        if self.prev_error is None:
            self.prev_error = error

        raw_d = (error - self.prev_error) / self.dt
        self.d_filter = self.d_alpha * self.d_filter + (1 - self.d_alpha) * raw_d
        
        self.prev_error = error

        u = (Kp * error) + (Kd * self.d_filter)
        return u

class Direction(Enum):
    LEFT = 'left'
    RIGHT = 'right'
    FRONT_LEFT = 'front_left'
    FRONT_RIGHT = 'front_right'
    FRONT = 'front'

class MyRob:
    def __init__(self):
        self.robot = Robot()
        self.timeStep = int(self.robot.getBasicTimeStep())
        
        # Motors
        self.leftMotor = self.robot.getDevice("left wheel motor")
        self.rightMotor = self.robot.getDevice("right wheel motor")
        self.leftMotor.setPosition(float('inf'))
        self.rightMotor.setPosition(float('inf'))
        self.driveMotors(0.0, 0.0)

        # Sensors
        self.dist_sensors = [self.robot.getDevice(f'ps{x}') for x in range(NUM_SENSORS)]
        for s in self.dist_sensors:
            s.enable(self.timeStep)

        self.buf_left = CircularBuffer(5)
        self.buf_right = CircularBuffer(5)
        self.buf_front = CircularBuffer(10)

        dt = self.timeStep / 1000.0
        self.steering_pid = PDController(dt)

        print(f"Controller initialized - timestep: {self.timeStep}ms")

    def step(self):
        if self.robot.step(self.timeStep) == -1:
            return -1

        raw_ds = [s.getValue() for s in self.dist_sensors]
        self.ds = [max(0.0, val - SENSOR_FLOOR) for val in raw_ds]

        self.buf_left.add(self.ds[5])
        self.buf_right.add(self.ds[2])
        self.buf_front.add((self.ds[0] + self.ds[7]) / 2)

    def driveMotors(self, leftSpeed, rightSpeed):
        self.leftMotor.setVelocity(max(min(leftSpeed, MAX_SPEED), -MAX_SPEED))
        self.rightMotor.setVelocity(max(min(rightSpeed, MAX_SPEED), -MAX_SPEED))

    def rotate(self):
        if self.buf_right.get_average() > self.buf_left.get_average():
            direction = -1  # Left
        else:
            direction = 1   # Right

        while self.step() != -1:
            turn_speed = MAX_SPEED * 0.5
            self.driveMotors(turn_speed * direction, -turn_speed * direction)

            if self.buf_front.get_average() < MOVE_AHEAD_THRESHOLD:
                break

        self.driveMotors(0, 0)

    def wander(self):
        f = self.buf_front.get_average()

        if f > START_ROTATION_THRESHOLD:
            self.rotate()
            return
        
        l = self.buf_left.get_average()
        r = self.buf_right.get_average()
        
        error = (r - l)
        steering = self.steering_pid.control(error, KP_CENTER, KD_CENTER)

        left_speed = MAX_SPEED - steering
        right_speed = MAX_SPEED + steering
        
        self.driveMotors(left_speed, right_speed)

if __name__ == '__main__':
    myrob = MyRob()
    while myrob.step() != -1:
        myrob.wander()