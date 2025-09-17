"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

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

near_front = 74
near_side = 71

while robot.step(timeStep) != -1:
    dist_sensor_values = [g.getValue() for g in dist_sensors]
    
    print(dist_sensor_values)

    if dist_sensor_values[0] > near_front \
       or dist_sensor_values[7] > near_front:
        print('rotate')
        leftMotor.setVelocity (-cruiseVelocity)
        rightMotor.setVelocity( cruiseVelocity)
    elif dist_sensor_values[1]>near_side and dist_sensor_values[1]>dist_sensor_values[6]:
        print('turn left')
        leftMotor.setVelocity (-0.1*cruiseVelocity)
        rightMotor.setVelocity( 1.2*cruiseVelocity)
    elif dist_sensor_values[6]>near_side:
        print('turn right')
        leftMotor.setVelocity ( 1.2*cruiseVelocity)
        rightMotor.setVelocity(-0.1*cruiseVelocity)
    else:
        print('go')
        leftMotor.setVelocity(cruiseVelocity)
        rightMotor.setVelocity(cruiseVelocity)




# get the time step of the current world.
#timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#motorR = robot.getDevice('right wheel motor')
#motorL = robot.getDevice('left wheel motor')
#ds = robot.getDevice('dsname')
#ds.enable(timestep)

#motorR.setPosition(float('inf'))
#motorL.setPosition(float('inf'))


# Main loop:
# - perform simulation steps until Webots is stopping the controller
#while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
   # val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
#    motorR.setVelocity(6.0)
#    motorL.setVelocity(2.0)



    #print('comm')

# Enter here exit cleanup code.
