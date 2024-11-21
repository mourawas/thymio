import numpy as np
import math

class ThymioControl:
    def __init__(self):
        self.treshold = 5/360*2*math.pi #angle in radian
        self.pos = None
        self.angle = None
        pass
        
    def move(self, speed):
        # Move the robot
        # speed has two values: the velocity in the direction aligned with the robot's front/back and the direction perpendicular to the robot's front/back
        pass

    def move(self, path, position, angle):
        # Move the robot along the path
        # then remove the heading from the path after moving

        #position and angle of the thymio
        self.angle = angle #remove this line to take the real value of thymio angle
        self.pos = position #remove this line to take the real pos of the thymio

        #calculate the angle that the thymio need
        alpha = math.atan2((self.pos[1] - path[0][1]), (self.pos[0] - path[0][0]))
        #move the robot and if the cell is reach, delete it and restart with the following
        if self.pos != path[0]:
            #change the speed depending the angle of the robot
            if abs(self.angle - alpha) < self.treshold:
                return 0
            else:
                #calculate and send the angle that the thymio need to turn
                return alpha - self.angle
        else:
            #remove the cell from the path when it's reached
            path.remove(path[0])
            alpha = math.atan2((self.pos[1] - path[0][1]), (self.pos[0] - path[0][0]))
            #change the speed depending the angle of the robot
            if abs(self.angle - alpha) < self.treshold:
                return 0
            else:
                #calculate and send the angle that the thymio need to turn
                return alpha - self.angle

    def getMotorSpeeds(self):
        # get the motor speeds
        return motor_left_speed, motor_right_speed

    def getProximity(self):
        # get the proximity sensor readings
        pass

    def getProximity(self, sensor):
        # get the proximity sensor reading for a specific sensor
        pass

    def setMotors(self, left_target, right_target):
        # set the motor speeds
        pass

    def turn(self, angle):
        # turn the robot
        pass