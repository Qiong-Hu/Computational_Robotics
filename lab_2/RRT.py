import numpy as np
import matplotlib.pyplot as plt
import math
import random
import time

# Define robot shape
R = 25      # unit: mm, radius of the wheels
L = 100     # unit: mm, length of the robot
W = 90      # unit: mm, width of the robot

# Maximum values
wmax = 1    # unit: RPS (rotations per second), maximum angular velocity of the wheels
vx_max = vy_max = 2 * math.pi * R * wmax    # unit: mm/s, maxium velocity of the robot in the direction of x and y
wmax_robot = 4 * math.pi* R * wmax / W      # unit: rad/s, maximum angular velocity of the robot, = max{2 * pi * R * (w1 - w2) / W}

show_animation = True

# Problem 1(a)
# Define rebot model
# input (action) space
# A = {a}, a = (w1, w2). w1 and w2 represent angular velocity of wheel 1 and wheel 2. w1, w2 ∈ [-wmax, wmax] = [-1, 1] RPS
D_A = 2     # The dimensionality is 2 

# robot state (configuration) space
# S = {s}, s = (x, y, θ). Coordinate (x, y) represents the location of the centerpoint of the wheels, θ is the angle between +X and the direction of the robot, θ∈[0,2*pi)
D_S = 3     # The dimensionality is 3

# operational space
# O = {o}, o = (vx,vy, w). (vx, vy) is the velocity of the robot, w is the angular speed of the robot
# v1 and v2 represent velocity of wheel 1 and wheel 2
# v1 = 2 * pi * R * w1 mm/s 
# v2 = 2 * pi * R * w2 mm/s
# v = (v1 + v2) / 2 mm/s
# vx = v cos(θ) = pi * R * (w1 + w2) * cos(θ) mm/s
# vy = v sin(θ) = pi * R * (w1 + w2) * sin(θ) mm/s
# w = 2 * pi * R * (w1 - w2) / W rad/s     # positive: clockwise, negative: anti-clockwise
D_O = 3     # The dimensionality is 3


# Problem 1(b)
# Define the (continuous space) system model.
# Assume a fully observable system, i.e. the state itself is directly accessible with no sensor model.

# Define map
# a m * n rectangular environment
m = 10000   # unit: mm, length of the map
n = 10000   # unit: mm, width of the map

# obstacles: a list to store obstacle states, each obstacle [xi, yi, wi, hi] is a rectangle, (xi, yi) is the coordinate of the lower left-corner of the obstacle, (wi, hi) is the width and height of the obstacle
# obstacles= {obstacle}, obstacle = [xi, yi, wi, hi]

# start state: s0=[x0, y0, θ0]
s0=[0, 0, 0]

# target state: s1=[xt, yt, θt]
s1=[0, 0, 0]


# Trajectory Planning
# Problem 2(a)
# Given a set of points V in C-space and a single target point xt, define and justify a metric that can be used to determine which of the points in V is closest to xt.

# RRT node
class Node():
    def __init__(self, state):
        self.state = state
        self.path = []
        self.parent = None

# determine which of the points in V is closest to xt
def find_closestNode(V, xt):
    # V: a set of RRT points. V = {v}, v = (x, y, theta) in C-space
    # xt: a single target point. xt = (x_t, y_t, theta_t)
    # Return: the closest node to the target point xt inside the set V

    closest_node = V[0]

    # Calculate the weighted distance between a point in set V to the target point xt
    # For delta_x: distance between x and x_t divided by the maximum velocity of the robot vx_max
    # For delta_y: distance between y and y_t divided by the maximum velocity of the robot vy_max
    # For delta_theta: difference between theta and theta_t1 divided by the maximum angular velocity of the robot wmax_robot
    # By dividing by the maximum value of each motion, the weighted distance of three different dimensions that the robot can reach is normalized.
    weighted_dist = ((V[0][0] - xt[0]) / (2 * math.pi * R * wmax)) ** 2 + ((V[0][1] - xt[1]) / (2 * math.pi * R * wmax)) ** 2 + ((V[0][2] - xt[2]) / (4 * math.pi * R * wmax / W)) ** 2

    for point in V:
        dist = ((point[0] - xt[0]) / (2 * math.pi * R * wmax)) ** 2 + ((point[1] - xt[1]) / (2 * math.pi * R * wmax)) ** 2 + ((point[2] - xt[2]) / (4 * math.pi * R * wmax / W)) ** 2
        if dist < weighted_dist:
            closest_node = point
            weighted_dist = dist
    return closest_node


# Problem 2(b)
# Given arbitrary initial robot state xi and target robot state xt (in C-space), generate a smooth achievable trajectory (that matches the metric you defined above) from the xi towards xt lasting 1 second. What are the control inputs for this trajectory?

def generate_trajectory(xi, xt):
    # Given initial and target robot state: xi, xt in C-space.
    # Return: an achievable trajectory

    # a list to store each point in the trajectory within one second
    trajectory = []

    # remaining time within one second, updated after each step
    remain_time = 1

    # end point of each step
    end = [xi[0], xi[1], 0]

    # start-to-goal vector
    diff_x = xt[0] - xi[0]
    diff_y = xt[1] - xi[1]
    diff_dist = math.sqrt(diff_x ** 2 + diff_y ** 2)

    # angle between +x and start-to-goal vector
    angle = math.atan(diff_y / diff_x)

    # update angle of the end point of each step according to the angle difference
    diff_angle = (xi[2] - angle) % math.pi
    if diff_angle <= math.pi / 2:
        end[2] = xi[2] - diff_angle
    else:
        end[2] = xi[2] + (math.pi - diff_angle)
    trajectory.append(end)
    # print(trajectory)
    
    req_rot_time=  diff_angle % (math.pi / 2) / wmax_robot
    remain_time = 1 - req_rot_time
    req_dis_time = diff_dist / (2 * math.pi * R)
    end = [end[0], end[1], end[2]]
    if remain_time < req_dis_time :
        dist = remain_time * 2 * math.pi * R
        end = [xi[0] + dist * math.cos(angle), xi[1] + dist * math.sin(angle), end[2]]
        trajectory.append(end)
        # print(trajectory)
    else:
        end = [xt[0], xt[1], end[2]]
        trajectory.append(end)
        # print(trajectory)
        end = [xt[0], xt[1], end[2]]
        remain_time -= req_dis_time 
        diff_angle = (end[2] - xt[2]) % (2 * math.pi)
        req_dis_time = (diff_angle % math.pi) / wmax_robot
        if remain_time >= req_dis_time:
            trajectory.append(xt)
            # print(trajectory)
        else:
            if 0<= diff_angle <= math.pi:
               end[2] = end[2] - wmax_robot * remain_time
            else:
                end[2] = end[2] + wmax_robot * remain_time
            trajectory.append(end)
            # print(trajectory)
    return trajectory

def plottrajectory(trajectory, ax):
    for point in trajectory:
        # plot center point
        plt.plot(point[0], point[1], 'bo')
        # plot robot, undo
    return ax




