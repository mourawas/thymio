import numpy as np
import math

class ThymioControl:
    def __init__(self):
        pass
        
    def move(self, speed):
        # Move the robot
        # speed has two values: the velocity in the direction aligned with the robot's front/back and the direction perpendicular to the robot's front/back
        pass

    def move(path):
        # Move the robot along the path
        # then remove the heading from the path after moving
        thymioangle = 0 #remove this line to take the real value of thymio angle
        thymiox = 3; thymioy = 3 #remove this line to take the real pos of the thymio

        #pos of the robot (put the real value thymio.x or i don't know what)
        pos = (thymiox, thymioy)
        #calculate the angle that the thymio need
        alpha = math.atan2((pos[1] - path[0][1]), (pos[0] - path[0][0]))
        #move the robot and if the cell is reach, delete it and restart with the following
        if pos != path:
            #change the speed depending the angle of the robot
            if thymioangle == alpha:
                return 0
            else:
                #calculate and send the angle that the thymio need to turn
                return alpha - thymioangle
        else:
            #remove the cell from the path when it's reached
            path.remove(path[0])
            alpha = math.atan2((pos[1] - path[0][1]), (pos[0] - path[0][0]))
            #change the speed depending the angle of the robot
            if thymioangle == alpha:
                return 0
            else:
                #calculate and send the angle that the thymio need to turn
                return alpha - thymioangle

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