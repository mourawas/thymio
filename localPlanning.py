import numpy as np
import math

class LocalPlanning:
    def __init__(self, map, thymioControl):
        self.map = map
        self.distances = []
        self.weights = [6, 4, -2, -6, -8]
        self.maxValue = 4300
        self.thresholds = [3200, 3000, 3000, 3000, 3200]
        self.speed = [0, 0]
        self.angles = [-40, -20, 0, 20, 40]*np.pi/180
        self.thymioControl = thymioControl

    # the robot gathers the proximity sensor readings and has to determine if it has to use local planning
    def is_local_planning(self, prox_horizontal):
        for reading, thresh in zip(prox_horizontal, self.thresholds):
            if reading > thresh:
                return True
        return False

    # the robot uses the proximity sensor readings to determine the obstacles and on which side it should turn
    # decides which side to turn based on the sensors with the highest readings
    # it then implements 
    def local_planning(self, prox_horizontal, speed):
        turn = None
        # get the current speed
        self.speed = speed
        
        if prox_horizontal[0] + prox_horizontal[1] > prox_horizontal[3] + prox_horizontal[4]:
            turn = "right"
        elif prox_horizontal[0] + prox_horizontal[1] < prox_horizontal[3] + prox_horizontal[4]:
            turn = "left"

        follow_wall(turn)
        
        return self.speed
        
            
        
