import math
class ThymioControl:
    def __init__(self):
        self.__pos = []        
        self.__oldPos = []
        self.__angle = None
        self.__oldAngle = None
        self.__oldPoses = []

        self.__step = 1

        # thresholds
        self.__kidnappingThresholdPosition = 20
        self.__kidnappingThresholdAngle = 30 # degrees
        self.__reachedThreshold = 50 # mm

        # constant linear speed
        self.__linearSpeed = 10

        # conversion from rad/s to wheel speed command
        self.__thymioWheelSpeedConversion = 65.5
        # constant proportional parameter for transforming angle into rotational speed
        self.__thymioRotationalSpeedConversion = 0.1
        # conversion from measured distance to mm
        self.__distanceConversion = 10
        # adjustment for the thymio's wheels
        self.__wheelsAdjustment = 1.1
        # cell to mm conversion
        self.__cellToMm = 10

        # max value for angular speed
        self.__maxAngularSpeed = 3.14
        
        # robot geometry
        self.__lenght = 93 # mm, distance between the wheels
        self.__radius = 22 # mm, radius of the wheels

        # kalman filter state
        self.x_cam = 0
        self.y_cam = 0
        self.theta_cam = 0
        self.x_est = 0
        self.y_est = 0
        self.theta_est = 0

    def set_path(self, path):
        self.__path = path
        self.__reduce_path()
        self.__step = 1

    def set_pose(self, position, angle):
        self.__pos = position #* self.__cellToMm
        self.__angle = angle

    def __reduce_path(self):
        # reduce the path by removing the cells that are in a straight line
        i = 0
        while i < len(self.__path) - 2:
            x1, y1 = self.__path[i]
            x2, y2 = self.__path[i + 1]
            x3, y3 = self.__path[i + 2]
            if (x1 - x2) * (y2 - y3) == (y1 - y2) * (x2 - x3):
                self.__path.pop(i + 1)
            else:
                i += 1

    def get_path(self):
        return self.__path

    def move(self, position, angle):
        # move the robot along the path

        #position and angle of the thymio
        self.__angle = angle
        self.__pos = position * self.__cellToMm

        objective = self.__path[self.__step]

        # calculate the distance between the robot and the objective
        x_diff = objective[0] - self.__pos[0]
        y_diff = objective[1] - self.__pos[1]
        distance = math.sqrt(x_diff**2 + y_diff**2) * self.__distanceConversion

        # calculate the angle between the robot and the objective
        # normalize the angle between -pi and pi
        angleDistance = (math.atan2(y_diff, x_diff) - self.__angle + math.pi) % (2 * math.pi) - math.pi

        # move the robot and if the cell is reached, delete it and restart with the following
        if distance < self.__reachedThreshold:
            if self.__step == len(self.__path) - 1:
                print("Destination reached")
                return 0, 0, 0, 0, True
            self.__step += 1

            self.move(self.__pos, self.__angle)
        else:
            # move throwards the next cell in the path
            v, w = self.__linearSpeed, angleDistance * self.__thymioRotationalSpeedConversion
            w = max(min(w, self.__maxAngularSpeed), -self.__maxAngularSpeed)
            wl, wr = self.differentialDrive(v, w)
            
            # find wl and wr with the astolfi controller
        return v, w, wl, wr, False
    
    def differentialDrive(self, v, w):
        wl = self.__thymioWheelSpeedConversion * self.__wheelsAdjustment * (v - self.__lenght * w / 2) / self.__radius
        wr = self.__thymioWheelSpeedConversion * (v + self.__lenght * w / 2) / self.__radius
        return wl, wr
    
    def inverseDifferentialDrive(self, wl, wr):
        w = ((wr - wl/self.__wheelsAdjustment) * self.__radius / self.__thymioWheelSpeedConversion) / self.__lenght
        v = ((wr + wl/self.__wheelsAdjustment) * self.__radius / self.__thymioWheelSpeedConversion) / 2
        return v, w
    
    def amIKidnapped(self):
        # check if the robot is kidnapped
        return math.sqrt(math.pow(self.__oldPos[0] - self.__pos[0]) + math.pow(self.__oldPos[1] - self.__pos[1])) > self.__kidnappingThresholdPosition or abs(self.__oldAngle - self.__angle) > self.__kidnappingThresholdAngle
    
    def update_pose(self, position, angle):
        self.__pos[0] = position[0] * self.__cellToMm
        self.__pos[1] = position[1] * self.__cellToMm
        self.__angle = angle
        self.__oldPos = self.__pos
        self.__oldAngle = self.__angle
    
    def update_last_pose(self):
        self.__oldPoses.append((self.__pos, self.__angle))
        self.__oldPos = self.__pos
        self.__oldAngle = self.__angle
    
    def get_wheel_distance(self):
        return self.__lenght