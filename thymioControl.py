import math
class ThymioControl:
    def __init__(self):
        self.__threshold = 5/360*2*math.pi #angle in radian
        self.__pos = []        
        self.__oldPos = []
        self.__angle = None
        self.__oldAngle = None

        self.__oldPoses = []

        self.__step = 1

        # thresholds
        self.__kidnappingThresholdPosition = 20
        self.__kidnappingThresholdAngle = 30 # degrees
        self.__reachedThreshold = 5 # mm

        # constant linear speed
        self.__linearSpeed = 500

        # conversion from rad/s to wheel speed command
        self.__thymioWheelSpeedConversion = 2
        # constant proportional parameter for transforming angle into rotational speed
        self.__thymioRotationalSpeedConversion = 0.55
        
        # robot geometry
        self.__lenght = 93 # mm, distance between the wheels
        self.__radius = 22 # mm, radius of the wheels

    def set_path(self, path):
        self.__path = path
        self.__step = 1

    def move(self, position, angle):
        # move the robot along the path

        #position and angle of the thymio
        self.__angle = angle
        self.__pos = position

        objective = self.__path[self.__step]

        # calculate the distance between the robot and the objective
        x_diff = objective[0] - self.__pos[0]
        y_diff = objective[1] - self.__pos[1]
        distance = math.sqrt(x_diff**2 + y_diff**2)

        # calculate the angle between the robot and the objective
        angleDistance = self.__radToDeg(math.atan2(y_diff, x_diff))

        # move the robot and if the cell is reached, delete it and restart with the following
        if distance < self.__reachedThreshold:
            if self.__step == len(self.__path) - 1:
                print("Destination reached")
                return 0, 0, 0, 0, True
            self.__step += 1

            self.move(self.__pos, self.__angle)

            """
            v, w = 0, 0
            wl, wr = self.differentialDrive(v, w)
            """
        else:
            # move throwards the next cell in the path
            v, w = self.__linearSpeed, angleDistance * self.__thymioRotationalSpeedConversion
            wl, wr = self.differentialDrive(v, w)
            
            # find wl and wr with the astolfi controller
        return v, w, wl, wr, False
            
    def __radToDeg(self,angle):
        return angle * 180 / math.pi
    
    def differentialDrive(self, v, w):
        wl = self.__thymioWheelSpeedConversion * (v - self.__lenght * w / 2) / self.__radius
        wr = self.__thymioWheelSpeedConversion * (v + self.__lenght * w / 2) / self.__radius
        return wl, wr
    
    def inverseDifferentialDrive(self, wl, wr):
        w = ((wr - wl) * self.__radius / self.__thymioWheelSpeedConversion) / self.__lenght
        v = ((wr + wl) * self.__radius / self.__thymioWheelSpeedConversion) / 2
        return

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

    def getRotationalVelocity(self):
        return 
    
    def amIKidnapped(self):
        # check if the robot is kidnapped
        return math.sqrt(math.pow(self.__oldPos[0] - self.__pos[0]) + math.pow(self.__oldPos[1] - self.__pos[1])) > self.__kidnappingThresholdPosition or abs(self.__oldAngle - self.__angle) > self.__kidnappingThresholdAngle
    
    def update_pose(self, position, angle):
        self.__pos[0] = position[0]
        self.__pos[1] = position[1]
        self.__angle = angle
        self.__oldPos = self.__pos
        self.__oldAngle = self.__angle
    
    def update_last_pose(self):
        self.__oldPoses.append((self.__pos, self.__angle))
        self.__oldPos = self.__pos
        self.__oldAngle = self.__angle