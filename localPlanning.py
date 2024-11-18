import numpy as np
import math

class LocalPlanning:
    def __init__(self, map):
        self.map = map
        self.distances = []
        self.weights = [0.03, 0.05, 0.5, 0.05, 0.03]
        self.maxValue = 4300
        self.thresholds = [3200, 3000, 3000, 3000, 3200]
        self.speed = [0, 0]
        self.angles = [-40, -20, 0, 20, 40]*np.pi/180

    # the robot gathers the proximity sensor readings and has to determine if it has to use local planning
    def is_local_planning(self, prox_horizontal):
        for reading, thresh in zip(prox_horizontal, self.thresholds):
            if reading > thresh:
                return True
        return False

    # the robot uses the proximity sensor readings to determine the obstacles distances and computes the change 
    def local_planning_dumb(self, prox_horizontal, speed):
        # get the current speed
        self.speed = speed
        # iterate over the proximity sensor readings
        for reading, thresh, angle, weight in zip(prox_horizontal, self.thresholds, self.angles, self.weights):
            # if the reading is higher than the threshold, the robot has to change its speed, according to the angle
            if reading > thresh:
                if (angle < 0):
                    # front direction
                    self.speed[0] = self.speed[0] - weight * reading * np.sin(-angle)
                    # side direction
                    self.speed[1] = self.speed[1] - weight * reading * np.cos(angle)
                else:
                    self.speed[0] = self.speed[0] - weight * reading * np.sin(angle)
                    self.speed[1] = self.speed[1] + weight * reading * np.cos(angle)
        
        # speed corresponds to the velocity in the direction aligned with the robot's front/back and the direction perpendicular to the robot's front/back
        return self.speed

    # the robot uses the proximity sensor readings to determine the obstacles distances and computes the change
    def local_planning(self, prox_horizontal, speed):
        turn = None
        # get the current speed
        self.speed = speed
        
        if prox_horizontal[0] + prox_horizontal[1] > prox_horizontal[3] + prox_horizontal[4]:
            turn = "right"
        elif prox_horizontal[0] + prox_horizontal[1] < prox_horizontal[3] + prox_horizontal[4]:
            turn = "left"

        if turn == "right":
            self.speed[1] = self.speed[1] - self.weights[0] * prox_horizontal[0] - self.weights[1] * prox_horizontal[1]
        elif turn == "left":
            self.speed[1] = self.speed[1] + self.weights[3] * prox_horizontal[3] + self.weights[4] * prox_horizontal[4]
        
        return self.speed
    
    def local_planning_map(self, prox_horizontal, speed, map):
        