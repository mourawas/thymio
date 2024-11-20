import numpy as np
import math
from kalman import Kalman

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
        kalman = Kalman()
        thymioangle = 0 #remove this line to take the real value of thymio angle
        thymiox = 3; thymioy = 3 #remove this line to take the real pos of the thymio

        #pos of the robot (put the real value thymio.x or i don't know what)
        pos = (thymiox, thymioy)
        #calculate the angle that the thymio need
        alpha = math.atan2((pos[1] - path[0][1]), (pos[0] - path[0][0]))
        #move the robot until the cell is reach
        while pos != path:
            #change the speed depending the angle of the robot
            if thymioangle == alpha:
                thymiospeedR = 20 #change this two line to change the value directly on the thymio
                thymiospeedL = 20
            else:
                #calculate the angle that the thymio need to turn
                thymioangle = alpha - thymioangle
                while thymioangle != alpha:
                    if thymioangle > alpha:
                        thymiospeedR = 0
                        thymiospeedL = 20
                    else:
                        thymiospeedR = 20
                        thymiospeedL = 0
                kalman.kalman_update()
                thymiospeedR = 20
                thymiospeedL = 20
            kalman.kalman_update()
        #remove the cell from the path when it's reached
        path.remove(path[0])
        pass

    def getMotorSpeeds(self):
        # Get the motor speeds
        pass

    def getProximity(self):
        # Get the proximity sensor readings
        pass