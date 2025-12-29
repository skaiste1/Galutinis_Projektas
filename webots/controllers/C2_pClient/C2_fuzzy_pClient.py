from enum import Enum
from collections import deque
from controller import Robot
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

MAX_SPEED = 6.27

MAX_SENSOR_DIFF = 20

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

class FuzzyController:
    def __init__(self):
        error = ctrl.Antecedent(np.arange(-MAX_SENSOR_DIFF, MAX_SENSOR_DIFF, 1), 'error')
        
        steer_limit = MAX_SPEED*0.1
        steering = ctrl.Consequent(np.arange(-steer_limit, steer_limit, 0.1), 'steering')

        error['FL'] = fuzz.trimf(error.universe, [-MAX_SENSOR_DIFF, -MAX_SENSOR_DIFF, -5]) 
        error['SL'] = fuzz.trimf(error.universe, [-10, -5, 0])
        error['C']  = fuzz.trimf(error.universe, [-1, 0, 1])       
        error['SR'] = fuzz.trimf(error.universe, [0, 5, 10])      
        error['FR'] = fuzz.trimf(error.universe, [5, MAX_SENSOR_DIFF, MAX_SENSOR_DIFF])    

        steering['HL'] = fuzz.trimf(steering.universe, [steer_limit/2, steer_limit, steer_limit]) 
        steering['SL'] = fuzz.trimf(steering.universe, [0, steer_limit/2, steer_limit])            
        steering['C']  = fuzz.trimf(steering.universe, [-0.5, 0, 0.5])
        steering['SR'] = fuzz.trimf(steering.universe, [-steer_limit, -steer_limit/2, 0])
        steering['HR'] = fuzz.trimf(steering.universe, [-steer_limit, -steer_limit, -steer_limit/2])

        rule1 = ctrl.Rule(error['FL'], steering['HR']) 
        rule2 = ctrl.Rule(error['SL'], steering['SR']) 
        rule3 = ctrl.Rule(error['C'],  steering['C'])  
        rule4 = ctrl.Rule(error['SR'], steering['SL']) 
        rule5 = ctrl.Rule(error['FR'], steering['HL']) 

        center_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5])
        self.u = ctrl.ControlSystemSimulation(center_ctrl)

    def control(self, error):
        clamped_error = max(min(error, (MAX_SENSOR_DIFF)), (-MAX_SENSOR_DIFF))
        
        self.u.input['error'] = clamped_error
        self.u.compute()
        return self.u.output['steering']

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

        self.steering_controller = FuzzyController()

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
        steering = self.steering_controller.control(error)

        left_speed = MAX_SPEED - steering
        right_speed = MAX_SPEED + steering
        
        self.driveMotors(left_speed, right_speed)

if __name__ == '__main__':
    myrob = MyRob()
    while myrob.step() != -1:
        myrob.wander()