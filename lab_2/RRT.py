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

# Problem 1(b)
# Define the (continuous space) system model. Assume a fully observable system, i.e. the state itself is directly accessible with no sensor model.

# state space：a m*n rectangular environment with any direction of robot
m = 1
n = 1

# obstacles : a list to store obstacle states, each obstacle [xi,yi,wi,hi] is a rectangular, xi,yi represents the lower left corner of the obstacle and wi, hi represents the width and height of the obstacle
# obstacles= [[xi,yi,wi,hi],...]

# start state: s0=[xs,ys,θs]
s0=[0, 0, 0]
    
# target state: s1=[xt,yt,θt]
s1=[0, 0, 0]



# Trajectory Planning
# 2(a). Given a set of points V in C-space and a single target point xt, dene and justify a metric that can be used to determine which of the points in V is closest to xt.

#RRT Node
class Node():
    def __init__(self, state):
        self.state = state
        self.parent = None

#determine which of the points in V is closest to xt
def find_closestNode(V, xt):
    """ 
    given a set of RRT nodes <V> in C-space and a single target point <xt>
    return the closest node to the target point inside the set
    """
    Closestnode = V[0]
    weighted_dist = ((V[0][0] - xt[0])/(math.pi * R * 2))**2 +
    				((V[0][1] - xt[1])/(math.pi * R * 2))**2 + 
    				((V[0][2] - xt[2])/(math.pi * R * 4/W))**2

    for point in V:
        dist = ((point[0] - xt[0])/(math.pi * R * 2))**2 + 
        	   ((point[1] - xt[1])/(math.pi * R * 2))**2 + 
        	   ((point[2] - xt[2])/(math.pi * R * 4/W))**2
        if dist < weighted_dist:
            Closestnode = point
            weighted_dist = dist
    return Closestnode


