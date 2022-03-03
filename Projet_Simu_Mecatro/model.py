import sys
from time import sleep
import motor
import math
from constants import *


class Model(object):
    """
    Represents the robot's state 
    """

    def __init__(self):
        # Distance between the wheels
        self.l = L
        # Wheel radius
        self.r = R

        self.x = 0
        self.y = 0
        self.theta = 0

        self.x_goal = 0
        self.y_goal = 0
        self.theta_goal = 0

        self.m1 = motor.Motor()
        self.m2 = motor.Motor()

        self.acc = 0
        self.speed_acc = 0
        self.mode = 1

    def __repr__(self):
        s = "current : {} {} {}".format(self.x, self.y, self.theta)
        s = s + "\ngoal    : {} {} {}".format(self.x_goal, self.y_goal, self.theta_goal)
        s = s + "\nmotors    : {} {}".format(self.m1, self.m2)
        s = s + "acc={}, speed_acc={}, mode={}".format(
            self.acc, self.speed_acc, self.mode
        )
        return s
#da = m2_speed
#db = m1_speed
#dp = linear_speed
#dteta = rotational speed
    def ik(self, linear_speed, rotational_speed):
        """Given the linear speed and the rotational speed, 
        returns the speed of the wheels in a differential wheeled robot
        
        Arguments:
            linear_speed {float} -- Linear speed (m/s)
            rotational_speed {float} -- Rotational speed (rad/s)
        
        Returns:
            float -- Speed of motor1 (m/s), speech of motor2 (m/s)
        """
        # TODO
        m1_speed = linear_speed - rotational_speed * (self.l/2)
        m2_speed = linear_speed + rotational_speed * (self.l/2)
        return m1_speed, m2_speed

    def dk(self):
        """Given the speed of each of the 2 motors (m/s), 
        returns the linear speed (m/s) and rotational speed (rad/s) of a differential wheeled robot
        
        Keyword Arguments:
            m1_speed {float} -- Speed of motor1 (m/s) (default: {None})
            m2_speed {float} -- Speed of motor2 (default: {None})
        
        Returns:
            float -- linear speed (m/s), rotational speed (rad/s)
        """
        # TODO

        linear_speed = (self.m1.speed + self.m2.speed)/2
        rotation_speed = (self.m1.speed - self.m2.speed)/self.l

        return linear_speed, rotation_speed

    def update(self, dt):
        """Given the current state of the robot (speeds of the wheels) and a time step (dt), 
        calculates the new position of the robot.
        The speed of the wheels are assumed constant during dt.
        
        Arguments:
            dt {float} -- Travel time in seconds
        """
        # Going from wheel speeds to robot speed
        linear_speed, rotation_speed = self.dk()

        # TODO
        dp = linear_speed * dt
        dteta = rotation_speed * dt
        if rotation_speed == 0 :
            dxr = dp
            dyr = 0
        else :
            dxr = (dp/dteta)*math.sin(dteta)
            dyr = (dp/dteta)*(1-math.cos(dteta))

        # Updating the robot position
        self.x = self.x + dxr * math.cos(self.theta)- dyr * math.sin(rotation_speed*dt) # TODO
        self.y = self.y + dxr * math.sin(self.theta) + dyr * math.cos( rotation_speed*dt) # TODO
        self.theta = self.theta + rotation_speed  # TODO

