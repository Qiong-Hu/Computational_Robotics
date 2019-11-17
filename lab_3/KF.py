import numpy as np
import matplotlib.pyplot as plt
import math
import random
import time
from filterpy.kalman import MerweScaledSigmaPoints
from filterpy.kalman import UnscentedKalmanFilter as UKF

# Problem 2(a) 
# Define the complete system model. You will need to derive and present the mathematical equations describing this robot plant, including the appropriate sensor and actuator models and system dynamics. Note that you will need to amend the state of your robot from the previous lab in order to accommodate this specific collection of sensors. Be sure to clearly define and describe all variables and equations, and produce illustrative diagrams as necessary.

# Define the environment with a map and walls
# A rectangular map of length L = 750 mm and width W = 500mm
# Four walls: x = 0, x = L, y = 0, y = W
L = 750     # unit: mm, length of the map
W = 500     # unit: mm, width of the map

# Define the robot shape
r = 25      # unit: mm, radius of the wheels
length = 100     # unit: mm, length of the robot
width = 90      # unit: mm, width of the robot

C = 2 * math.pi * r    # unit: mm, circumference of the wheels

# Maximum values of the robot motion
wmax = 1    # unit: RPS (rotations per second), maximum input angular velocity of the wheels
wmax_robot = 2 * C * wmax / width      # unit: rad/s, maximum input angular velocity of the robot. wmax_robot = max{C * (w1 - w2) / width} = 2 * C * wmax / width (w1, w2 unit: RPS)

# Inputs: (w1, w2)
# w1, w2: the input angular velocity of the left and right wheel, unit: RPS, range: [-wmax, wmax]
# w1, w2 are controlled by signals from the microcontrollers. There may be slippage between the wheels and the floor or sticking in the gears of the motor, so assume the resulting effective angular speed of each wheel is Gaussian

# v1, v2: velocity of the left and right wheel, unit: mm/s
# we: error of effective angular velocity,  we1, we2 ~ N(0, 0.05 * wmax)
v1 = (w1 + we1) * C
v2 = (w2 + we2) * C

# Effective linear and angular velocity of the robot (angular velocity direction: positive: clockwise, negative: anti-clockwise)
v_robot = (v1 + v2) / 2         # unit: mm/s
w_robot = (v2 - v1) / width     # unit: rad/s

# Actual linear and angular displacement in time = dt (unit: sec)
ds = v_robot * dt
d_theta = w_robot * dt
dx = ds * math.cos(θ + d_theta/2) 
dy = ds * math.sin(θ + d_theta/2) 

# Sensor outputs: (d1, d2, θs, rs)
# d1: the distance to a wall in a straight line in front of the robot 
# d2: the distance to a wall in a straight line to the right of the robot
# θs: an absolute bearing indicating the angle of the robot with respect to magnetic north 
# rs: the rotational speed from an angular rate (gyro) sensor

# Robot state space S = {s}, s = (x, y, θ)
# Coordinate (x, y): the location of the centerpoint of the wheels
# θ: the angle between +X and the direction of the robot, θ ∈ [0, 2 * pi)

s0 = [0, 0, 0]      # Initial state

# Time evolution model
traj = []
traj.append(s0)

# When t -> t+1: x -> x + dx, y -> y + dy, theta -> (theta + d_theta) % 2pi
s = [traj[-1][0] + dx, traj[-1][1] + dy,  (traj[-1][2] + d_theta) % (2 * math.pi)]
traj.append(s)


