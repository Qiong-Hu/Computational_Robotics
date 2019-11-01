import numpy as np
import matplotlib.pyplot as plt
import math
import random
import time

# Define robot shape
R = 25      # radius of the wheels
L = 100     # length of the robot
W = 90      # width of the robot


# Problem 1(a)

# input (action) space
# A = {a}, a = (w1, w2). w1 and w2 represent angular velocity of wheel 1 and wheel 2. w1, w2 ∈ [-1,1] RPS
D_A = 2     # The dimensionality is 2 

# robot state (configuration) space
# S = {s}, s = (x, y, θ). Coordinate (x, y) is the location of the centerpoint of the wheels, direction θ is the direction of the robot.
D_S = 3     # The dimensionality is 3

# operational space
# O = {o}, o = (vx,vy, w). (vx, vy) is the velocity of the robot, w is the angular speed of the robot
# vx = v cos(θ) = math.pi * R * (w1 + w2) * cos(θ) mm/s
# vy = v sin(θ) = math.pi * R * (w1 + w2) * sin(θ) mm/s
# w = 2 * math.pi * R * (w1 - w2) / W rad/s     # positive: clockwise, negative: anti-clockwise
D_O = 3     # The dimensionality is 3

