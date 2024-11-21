class LocalPlanning:
    def __init__(self):
        self.__thresholds = [3200, 3000, 3000, 3000, 3200]

        # tunable parameters
        self.__speedDuringAvoidance = 100

        # ann values
        self.__wl = [6, 9, -10, -9, -6]
        self.__wr = [-6, -9, -10, 9, 6]
        self.__sensorScale = 200

    # the robot gathers the proximity sensor readings and has to determine if it has to use local planning
    def is_local_planning(self, prox_horizontal):
        for reading, thresh in zip(prox_horizontal, self.__thresholds):
            if reading > thresh:
                return True
        return False

    def local_planning(self, prox_horizontal):
        # if the robot is in front of an obstacle, it will turn right
        wl = self.__speedDuringAvoidance
        wr = self.__speedDuringAvoidance

        for i in range(len(prox_horizontal)):
            wl += prox_horizontal[i] * self.__wl[i] / self.__sensorScale
            wr += prox_horizontal[i] * self.__wr[i] / self.__sensorScale
        
        return wl, wr
