import numpy as np
import matplotlib.pyplot as plt
import math
import random
import time
from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.kalman import UnscentedKalmanFilter as UKF

# Problem 2(a) 
# Define the complete system model. You will need to derive and present the mathematical equations describing this robot plant, including the appropriate sensor and actuator models and system dynamics. Note that you will need to amend the state of your robot from the previous lab in order to accommodate this specific collection of sensors. Be sure to clearly dene and describe all variables and equations, and produce illustrative diagrams as necessary.

# Define the map:
# A rectangular environment of length L = 750 mm and width W = 500mm
# Four walls: x = 0, x = L, y = 0, y = W
L = 750     # unit: mm, length of the map
W = 500     # unit: mm, width of the map

# Define the robot shape
r = 25      # unit: mm, radius of the wheels
length = 100     # unit: mm, length of the robot
width = 90      # unit: mm, width of the robot

C = 2 * math.pi * r    # unit: mm, circumference of the wheels

# Maximum values of the robot motion
wmax = 1    # unit: RPS (rotations per second), maximum angular velocity of the wheels
vx_max = vy_max = C * wmax    # unit: mm/s, maxium velocity of the robot in the direction of x and y
wmax_robot = 2 * C * wmax / width      # unit: rad/s, maximum angular velocity of the robot. = max{C * (w1 - w2) / width} = 2 * C * wmax / width (w1, w2 unit: RPS)

# inputs: (w1, w2)
# w1, w2: the angular velocity of the left and right wheel, unit: RPS, range: [-wmax, wmax]
# raw inputs of 
# v1, v2: velocity of the left and right wheel, unit: mm/s
# we: error of angular velocity, [we1~ N(0, 0.05 * wmax)]
v1 = (w1 + we1) * C
v2 = (w2 + we2) * C

# linear and angular velocity of the robot (clockwise motion: positive angular velocity)
v_robot = (v1 + v2 + we1 + we2) / 2
w_robot = (v2 - v1+ we2 - we1) / width

# robot state space S = {s}, s = (x, y, θ). (x, y) is the position of the center point of the robot in the map, θ indicates the direction of the robot

# linear and angular displacement in time = dt (sec)
ds = v_robot * dt
d_theta = w_robot * dt
dx = ds * math.cos(θ + d_theta/2) 
dy = ds * math.sin(θ + d_theta/2) 

# sensor outputs: (d1, d2, θs, rs)
# d1: the distance to a wall in a straight line in front of the robot 
# d2: the distance to a wall in a straight line to the right of the robot
# θs: an absolute bearing indicating the angle of the robot with respect to magnetic north 
# rs: the rotational speed

