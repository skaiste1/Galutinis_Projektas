"""my_controller controller."""

import numpy
import xml.etree.ElementTree as ET

from controller import Robot, Motor, DistanceSensor, Camera

CELLROWS=7
CELLCOLS=14
CELL_SIZE = 0.15
KP = 10.0
MAX_SPEED = 6.27


class Map():
    def __init__(self, filename):
        try:
            tree = ET.parse(filename)
            root = tree.getroot()
            self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
            i=1
            for child in root.iter('Row'):
               line=child.attrib['Pattern']
               row =int(child.attrib['Pos'])
               if row % 2 == 0:
                   for c in range(len(line)):
                       if (c+1) % 3 == 0:
                           if line[c] == '|':
                               self.labMap[row][(c+1)//3*2-1]='|'
               else:
                   for c in range(len(line)):
                       if c % 3 == 0:
                           if line[c] == '-':
                               self.labMap[row][c//3*2]='-'
               i=i+1
        except Exception as e:
            print(f"Warning: Map load failed ({e}). Continuing.")

class MyRob:
    def __init__(self):
        self.robot = Robot()
        self.timeStep = int(self.robot.getBasicTimeStep())

        self.cruiseVelocity = 4.0
        self.num_dist_sensors = 8

        self.leftMotor = self.robot.getDevice("left wheel motor")
        self.rightMotor = self.robot.getDevice("right wheel motor")
        self.leftMotor.setPosition(float('inf'))
        self.rightMotor.setPosition(float('inf'))
        self.leftMotor.setVelocity(0.0)
        self.rightMotor.setVelocity(0.0)

        self.dist_sensors = []
        for x in range(self.num_dist_sensors):
            sensor = self.robot.getDevice('ps' + str(x))
            if sensor:
                sensor.enable(self.timeStep)
                self.dist_sensors.append(sensor)
            else:
                print(f"Warning: Sensor ps{x} not found!")

        # camera
        self.camera = self.robot.getDevice("camera")
        if self.camera:
            self.camera.enable(self.timeStep)
        
        self.checkpoint_state = 0

        self.gps = self.robot.getDevice("gps")
        if self.gps:
            self.gps.enable(self.timeStep)
        else:
            print("Info: GPS not found on robot. Skipping GPS.")

        self.compass = self.robot.getDevice("compass")
        if self.compass:
            self.compass.enable(self.timeStep)
        else:
            print("Info: Compass not found on robot. Skipping Compass.")

        self.step()

        if self.gps:
            self.abs_ref_pos = numpy.array(self.gps.getValues())
            print("Initial position:", self.abs_ref_pos)
        else:
            self.abs_ref_pos = numpy.array([0, 0, 0]) 
            self.cur_dir = 0

    def step(self):
        self.robot.step(self.timeStep)

        if self.gps:
            self.cur_pos = numpy.array(self.gps.getValues())
        
        if self.compass:
            compass_val = self.compass.getValues()
            if compass_val:
                self.cur_dir = (-numpy.arctan2(compass_val[1], compass_val[0]) + numpy.pi/2) % (2 * numpy.pi)
                self.cur_dir = (self.cur_dir + numpy.pi) % (2 * numpy.pi) - numpy.pi

    def driveMotors(self, leftSpeed, rightSpeed):
        self.leftMotor.setVelocity(max(min(leftSpeed,MAX_SPEED),-MAX_SPEED))
        self.rightMotor.setVelocity(max(min(rightSpeed,MAX_SPEED),-MAX_SPEED))

    def rotate(self, target_dir):
        if not self.compass: return 
        while(numpy.abs(target_dir - self.cur_dir) > 0.01):
            angle_error = target_dir - self.cur_dir
            angle_error = (angle_error + numpy.pi) % (2 * numpy.pi) - numpy.pi
            self.driveMotors(-KP * angle_error, +KP * angle_error)
            self.step()
        self.driveMotors(0.0, 0.0)

    def move_to(self, target_pos):
        if not self.gps or not self.compass: return 
        
        target_dir = numpy.arctan2(target_pos[1] - self.cur_pos[1], target_pos[0] - self.cur_pos[0])
        target_dir = (target_dir + numpy.pi/4)//(numpy.pi/2) * numpy.pi/2 

        if(numpy.abs(target_dir - self.cur_dir) > 0.1):
            self.rotate(target_dir)

        dist = numpy.linalg.norm(target_pos - self.cur_pos)
        while(dist > 0.005):
            target_pos_dir = target_pos + CELL_SIZE * numpy.array([numpy.cos(target_dir), numpy.sin(target_dir), 0.0])
            angle_error = numpy.arctan2(target_pos_dir[1] - self.cur_pos[1],
                                        target_pos_dir[0] - self.cur_pos[0]) - self.cur_dir
            angle_error = (angle_error + numpy.pi) % (2 * numpy.pi) - numpy.pi

            self.driveMotors( self.cruiseVelocity - KP * angle_error,
                             self.cruiseVelocity + KP * angle_error)   
            self.step()
            dist = numpy.linalg.norm(target_pos - self.cur_pos)
        self.driveMotors(0.0, 0.0)

    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))


    def get_color(self):
        if not self.camera: return None
        img = self.camera.getImage()
        if not img: return None
        
        w = self.camera.getWidth()
        h = self.camera.getHeight()
        x = w // 2
        y = h // 2
        try:
            r = self.camera.imageGetRed(img, w, x, y)
            g = self.camera.imageGetGreen(img, w, x, y)
            b = self.camera.imageGetBlue(img, w, x, y)
            

            # print(f"RGB: {r}, {g}, {b}")

            if b > 100 and r < 80 and g < 80: return "BLUE"
            if r > 100 and g > 100 and b < 80: return "YELLOW"
            if r > 100 and g < 80 and b < 80: return "RED"
        except:
            return None
            
        return None
    def turn_around(self):
        print("Wrong direction. Turning around.")
        self.driveMotors(-4.0, 4.0)
        for _ in range(40): self.step()
        self.driveMotors(0, 0)
        self.step()
        self.last_processed_color = None

    def update_checkpoint_state(self):
        color = self.get_color()
        
        if color is None:
            self.last_processed_color = None
            return

        if color == self.last_processed_color:
            return

        self.last_processed_color = color
        
        if self.checkpoint_state == 0 and color == "BLUE":
            print("Checkpoint: BLUE REACHED")
            self.checkpoint_state = 1
        elif self.checkpoint_state == 1 and color == "YELLOW":
            print("Checkpoint: YELLOW REACHED")
            self.checkpoint_state = 2
        elif self.checkpoint_state == 2 and color == "RED":
            print("Checkpoint: RED REACHED ")
            self.checkpoint_state = 0
        elif self.checkpoint_state == 1 and color == "BLUE":
             self.turn_around()
        elif self.checkpoint_state == 2 and color == "YELLOW":
             self.turn_around()
        elif self.checkpoint_state == 0 and color == "RED":
             self.turn_around()

    def run_auto_mode(self):
        
        CRUISE_SPEED = 6.27
        TURN_FACTOR = 0.08

        while self.step() != -1:
            if not self.dist_sensors: break

            ps = [s.getValue() for s in self.dist_sensors]

            # front ps0 ir ps7
            front_obstacle = max(ps[0], ps[7])
            
            # side ps5 and ps2
            left_wall = ps[5]
            right_wall = ps[2]
            
            # diagonals ps6 and ps1
            left_diag = ps[6]
            right_diag = ps[1]

            left_speed = CRUISE_SPEED
            right_speed = CRUISE_SPEED

            if front_obstacle > 150:
                
                if left_wall > right_wall:
                     left_speed = 3.0
                     right_speed = -3.0
                else:
                     left_speed = -3.0
                     right_speed = 3.0
            

            else:
                diff = left_diag - right_diag
                left_speed = CRUISE_SPEED + (diff * TURN_FACTOR)
                right_speed = CRUISE_SPEED - (diff * TURN_FACTOR)

            self.update_checkpoint_state()
            self.driveMotors(left_speed, right_speed)       

""" 
    def run_auto_mode(self):
    #this would get stuck at 470 
        
        CRUISE_SPEED = 6.27  
        TURN_FACTOR = 0.08  

        while self.step() != -1:
            if not self.dist_sensors: break

            ps = [s.getValue() for s in self.dist_sensors]

            # ps7 (left front), ps0 (right front)
            # ps6 (left diagonal), ps1 (right diagonal) 

            front_obstacle = max(ps[0], ps[7])
            left_diag = ps[6]
            right_diag = ps[1]

            left_speed = CRUISE_SPEED
            right_speed = CRUISE_SPEED

  
            if front_obstacle > 150:
                #if wall infront stop and turn where free

                if left_diag < right_diag:
                     left_speed = -3.0
                     right_speed = 3.0
                else:
                     left_speed = 3.0
                     right_speed = -3.0
            

            else:
                # error > 0 means left wall is closer so turn right
                # error < 0 means right wall is closer so turn left

                diff = left_diag - right_diag

                left_speed = CRUISE_SPEED + (diff * TURN_FACTOR)
                right_speed = CRUISE_SPEED - (diff * TURN_FACTOR)

            self.update_checkpoint_state()
            self.driveMotors(left_speed, right_speed)
            
            """

if __name__ == '__main__':
    myrob = MyRob()


    try:
        mapc = Map("../C1_supervisor/C1-lab.xml")
        if hasattr(myrob, 'gps') and myrob.gps:
             myrob.setMap(mapc.labMap)
             myrob.printMap()
    except:
        pass


    myrob.run_auto_mode()