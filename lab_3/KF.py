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

# Sensor outputs: (d1, d2, θs)
# d1: the distance to a wall in a straight line in front of the robot
# d2: the distance to a wall in a straight line to the right of the robot
# θs: an absolute bearing indicating the angle of the robot with respect to magnetic north (direction of +X)

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


# Problem 2(b)
# Include realistic noise terms into the model as appropriate, and numerically quantify their parameters.

# Observation (measurement) model
# Define sensor errors: (d1e, d2e, θse), for each sensor output components

# Predict sensor output from robot state
# Steps:
# 1, Calculate the angles between magnetic north (direction of +X) and the line connecting the four corners of the wall and robot
# 2, Divide the intersection point on the wall into four regions, depend on the angle of the laser w.r.t. the angles to the corner
# 3, Calculate the distance between the robot and the intersection in the front of the robot, which is an estimated value of the sensor output with gaussian distribution
# 4, Repeat Step3 to calculate the distance to the right of the robot

def stateToSensor(s):
    # Given: robot state s = (x, y, theta)
    # Return: sensor outputs (d1s, d2s, θs)
    x, y, theta = s[0], s[1], s[2]
    theta_s = theta + theta_se

    # Calculate angle of (x, y) to (0, 0), (0, L), (L, 0), (L, W)
    theta1 = math.atan2(W - y, L - x)             # top-right (L, W)
    theta2 = math.atan2(W - y, -x)                # top-left (0, W)
    theta3 = math.atan2(-y, -x) + 2 * math.pi     # bottom_left (0, 0)
    theta4 = math.atan2(-y, L - x) + 2 * math.pi  # bottom_right (L, 0)

    # Calculate the distance to the nearest wall in front of the robot
    if  theta1 <= theta < theta2: # robot facing top side of the map
        d1 = (W - y) / math.sin(theta)            
    elif theta2 <= theta < theta3: # robot facing left side of the map
        d1 = x / math.cos(math.pi - theta)
    elif theta3 <= theta < theta4:
        d1 = y / math.sin(theta - math.pi)
    else:
        d1 = (L - x) / math.cos(theta)

    # Calculate the distance to the nearest wall to the right of the robot
    theta = theta - math.pi / 2
    if  theta1 <= theta < theta2: # robot facing top side of the map
        d2 = (W - y) / math.sin(theta)            
    elif theta2 <= theta < theta3: # robot facing left side of the map
        d2 = x / math.cos(math.pi - theta)
    elif theta3 <= theta < theta4:
        d2 = y / math.sin(theta - math.pi)
    else:
        d2 = (L - x) / math.cos(theta)

     d1s = d1 + d1e
     d2s = d2 + d2e
     return d1s, d2s, theta_s

