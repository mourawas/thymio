class LocalPlanning:
    def __init__(self, thymioControl):
        self.__thresholds = [3200, 3000, 3000, 3000, 3200]
        self.__thymioControl = thymioControl

        # tunable parameters
        self.__noDetectionThreshold = 100
        self.__turnAngle = 40
        self.__speedDuringAvoidance = 100

        # ann values
        self.__wl = [6, 9, -10, -9, -6]
        self.__wr = [-6, -9, -10, 9, 6]
        self.__sensorScale = 200
        self.__y = [0,0]
        self.__x = [0,0,0,0,0,0,0]

    # the robot gathers the proximity sensor readings and has to determine if it has to use local planning
    def is_local_planning(self, prox_horizontal):
        for reading, thresh in zip(prox_horizontal, self.__thresholds):
            if reading > thresh:
                return True
        return False

    """
    # the robot uses the proximity sensor readings to determine the obstacles and on which side it should turn
    # decides which side to turn based on the sensors with the highest readings
    # it then implements local avoidance with wall following, since we decided to use convex shapes for the obstacles
    def local_planning_no_ann(self, prox_horizontal):
        turn = None
        
        if prox_horizontal[0] + prox_horizontal[1] > prox_horizontal[3] + prox_horizontal[4]:
            turn = "right"
        elif prox_horizontal[0] + prox_horizontal[1] < prox_horizontal[3] + prox_horizontal[4]:
            turn = "left"
        elif prox_horizontal[0] > prox_horizontal[4]:
            turn = "right"
        elif prox_horizontal[0] < prox_horizontal[4]:
            turn = "left"
        else:
            # if the robot is in front of an obstacle, it will turn right
            turn = "right"

        self._avoid_obstacle(turn)
    
    def _avoid_obstacle(self, turn):
        # if the robot is in front of an obstacle, it will turn right
        if turn == "right":
            self.__thymioControl.move([0, -1])
        else:
            self.__thymioControl.move([0, 1])

    def local_planning_retry(self):
        free = False
        turn = None
        
        if prox_horizontal[0] + prox_horizontal[1] > prox_horizontal[3] + prox_horizontal[4]:
            turn = "right"
        elif prox_horizontal[0] + prox_horizontal[1] < prox_horizontal[3] + prox_horizontal[4]:
            turn = "left"
        elif prox_horizontal[0] > prox_horizontal[4]:
            turn = "right"
        elif prox_horizontal[0] < prox_horizontal[4]:
            turn = "left"
        else:
            # if the robot is in front of an obstacle, it will turn right
            turn = "right"

        # get current speeds
        self.__y = [self.__thymioControl.getMotorSpeeds()]

        while not free:
            prox_horizontal = self.__thymioControl.getProximity()
            if prox_horizontal.max() < self.__noDetectionThreshold:
                # no more detection
                free = True
            else:
                # for all sensors
                for i in range(len(self.__x)):
                    # get and scale inputs
                    self.__x[i] = prox_horizontal[i] // self.__sensorScale
                    
                    # compute outputs of neurons and set motor powers
                    self.__y[0] = self.__y[0] + self.__x[i] * self.__wl[i]
                    self.__y[1] = self.__y[1] + self.__x[i] * self.__wr[i]
    
                    # set motor powers
                    self.__thymioControl.set_motors(self.__y[0], self.__y[1])
        
        # now try to curve back to the original path
        turn_angle = self.__turnAngle if turn == "right" else -self.__turnAngle
        self.__thymioControl.turn(turn_angle)
        
        if (self.is_local_planning(self.__thymioControl.getProximity())):
            self.local_planning(self.__thymioControl.getProximity())
        # else exit from local planning, localize the robot and continue with global planning
    """

    def move(self, path, position, angle):
        # move the robot along the path
        # then remove the heading from the path after moving

        # position and angle of the thymio
        self.__angle = angle

    def local_planning(self, prox_horizontal):
        # if the robot is in front of an obstacle, it will turn right
        wl = self.__speedDuringAvoidance
        wr = self.__speedDuringAvoidance

        for i in range(len(prox_horizontal)):
            wl += prox_horizontal[i] * self.__wl[i] / self.__sensorScale
            wr += prox_horizontal[i] * self.__wr[i] / self.__sensorScale
        
        return wl, wr
