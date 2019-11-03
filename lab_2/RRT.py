import numpy as np
import matplotlib.pyplot as plt
import math
import random
import time

# Define robot shape
R = 25      # unit: mm, radius of the wheels
L = 100     # unit: mm, length of the robot
W = 90      # unit: mm, width of the robot
wmax = 1    # unit: RPS, maximum angular velocity of the wheels
<<<<<<< Updated upstream
=======
m = 10000   # unit: mm, length of grid
n = 10000   # unit: mm, width of grid
>>>>>>> Stashed changes

show_animation = True


# Problem 1(a)
# Define rebot model
# input (action) space
<<<<<<< Updated upstream
# A = {a}, a = (w1, w2). w1 and w2 represent angular velocity of wheel 1 and wheel 2. w1, w2 ∈ [-wmax, wmax] = [-1,1] RPS
=======
# A = {a}, a = (w1, w2). w1 and w2 represent angular velocity of wheel 1 and wheel 2. w1, w2 ∈ [-wmax, wmax] = [-1, 1] RPS
>>>>>>> Stashed changes
D_A = 2     # The dimensionality is 2 

# robot state (configuration) space
# S = {s}, s = (x, y, θ). Coordinate (x, y) represents the location of the centerpoint of the wheels, θ is the angle between +X and the direction of the robot, θ∈[0,2*pi)
D_S = 3     # The dimensionality is 3

# operational space
# O = {o}, o = (vx,vy, w). (vx, vy) is the velocity of the robot, w is the angular speed of the robot
# v1 and v2 represent velocity of wheel 1 and wheel 2
# v1 = 2 * pi * R * w1 mm/s 
# v2 = 2 * pi * R * w2 mm/s
# v = (v1 + v2)/2 mm/s
# vx = v cos(θ) = pi * R * (w1 + w2) * cos(θ) mm/s
# vy = v sin(θ) = pi * R * (w1 + w2) * sin(θ) mm/s
# w = 2 * pi * R * (w1 - w2)/W rad/s     # positive: clockwise, negative: anti-clockwise
D_O = 3     # The dimensionality is 3


# Problem 1(b)
<<<<<<< Updated upstream
# Define the (continuous space) system model
# Assume a fully observable system, i.e. the state itself is directly accessible with no sensor model.

# state space： a m * n rectangular environment with any direction of the robot
=======
# Define the (continuous space) system model.
# Assume a fully observable system, i.e. the state itself is directly accessible with no sensor model.

# state space：a m * n rectangular environment with any direction of the robot
>>>>>>> Stashed changes
m = 1
n = 1

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
    # xt: a single target point. xt = (xt1, yt1, thetat1)
    # Return: the closest node to the target point xt inside the set V

    closest_node = V[0]

    # Calculate the weighted distance between a point in set V to the target point xt
    # For delta_x: distance between x and xt1 divided by the maximum velocity of the robot, vx_max = 2 * pi * R * wmax
    # For delta_y: distance between y and yt1 divided by the maximum velocity of the robot, vy_max = 2 * pi * R * wmax
    # For delta_theta: difference between theta and thetat1 divided by the maximum angular velocity of the robot: wmax_robot = max{2 * pi * R * (w1 - w2)/W} = 4 * pi* R * wmax/W
    # By dividing by the maximum value of each motion, the weighted distance of three different dimensions that the robot can reach are normalized.
<<<<<<< Updated upstream
    weighted_dist = ((V[0][0] - xt[0])/(2 * math.pi * R * wmax))**2 +
                    ((V[0][1] - xt[1])/(2 * math.pi * R * wmax))**2 + 
                    ((V[0][2] - xt[2])/(4 * math.pi * R * wmax/W))**2

    for point in V:
        dist = ((point[0] - xt[0])/(2 * math.pi * R * wmax))**2 +
               ((point[1] - xt[1])/(2 * math.pi * R * wmax))**2 +
               ((point[2] - xt[2])/(4 * math.pi * R * wmax/W))**2
=======
    weighted_dist = ((V[0][0] - xt[0])/(2 * math.pi * R * wmax))**2 + ((V[0][1] - xt[1])/(2 * math.pi * R * wmax))**2 + ((V[0][2] - xt[2])/(4 * math.pi * R * wmax/W))**2

    for point in V:
        dist = ((point[0] - xt[0])/(2 * math.pi * R * wmax))**2 + ((point[1] - xt[1])/(2 * math.pi * R * wmax))**2 + ((point[2] - xt[2])/(4 * math.pi * R * wmax/W))**2
>>>>>>> Stashed changes
        if dist < weighted_dist:
            closest_node = point
            weighted_dist = dist
    return closest_node
<<<<<<< Updated upstream


# Problem 2(b)
# Given arbitrary initial robot state xi and target robot state xt (in C-space), generate a smooth achievable trajectory (that matches the metric you defined above) from the xi towards xt lasting 1 second. What are the control inputs for this trajectory?

def generate_trajectory(xi, xt):
    """
    Given initial and 
    """
    # trajectory
    trajectory = []
    # time remain
    remain_time = 1
    # end point
    end0 = [xi[0],xi[1],0]

    # goal-start vector
    diffx = xt[0]-xi[0]
    diffy = xt[1]-xi[1]
    diffd = np.sqrt(diffx**2+ diffy**2)
    # angle between +x and goal-start vector
    angle = np.arctan2(diffy,diffx)
    diffangle = (xi[2] - angle)%math.pi
    if diffangle<= math.pi/2:
        end0[2] = xi[2] - diffangle
    else:
        end0[2] = xi[2] + (math.pi-diffangle)
    trajectory.append(end0)
    # print(trajectory)

    t0 =  diffangle%(math.pi/2)/(math.pi *R * 4/W)
    remain_time = 1 - t0
    needed_time = diffd/(2 * math.pi * R)
    end1=[xi[0],xi[1],end0[2]]
    if remain_time<needed_time:
        dist = remain_time* 2 * math.pi * R
        end1[0] = xi[0] + dist*np.cos(angle)
        end1[1] = xi[1] + dist*np.sin(angle)
        trajectory.append(end1)
        # print(trajectory)

    else:
        end1[0] = xt[0]
        end1[1] = xt[1]
        trajectory.append(end1)
        # print(trajectory)
        end2 = [xt[0],xt[1],end0[2]]
        remain_time -= needed_time
        diffangle = (end2[2] - xt[2])%(2 * math.pi)
        needed_time = (diffangle % math.pi) / (math.pi *R * 4/W)
        if remain_time>=needed_time:
            trajectory.append(xt)
            # print(trajectory)
        else:
            if 0<= diffangle <= math.pi:
               end2[2] = end2[2] - math.pi *R * 4/W*remain_time
            else:
                end2[2] = end2[2] + math.pi *R * 4/W*remain_time
            trajectory.append(end2)
            # print(trajectory)
    return trajectory

def plottrajectory(trajectory):
    plt.figure()
    for point in trajectory:
        plt.arrow(point[0]+5, point[1]+5, 0.5*np.cos(point[2]),
          0.5*np.sin(point[2]), color='r', width=10)
        plt.plot(point[0], point[1],'bo')
    plt.ylim([-100, 1000])
    plt.xlim([-100, 1000])
    plt.grid()
    plt.show()


# Problem 2(c)
# Given a smooth robot trajectory in C-space and obstacles defined in operational space, determine whether this trajectory is collision-free. 

def plotobstacles(obstacles, ax):
    for obstacle in obstacles:
        rec = plt.Rectangle((obstacle[0],obstacle[1]), obstacle[2], obstacle[3], angle = obstacle[4], color = 'k')
        ax.add_patch(rec)
    return ax

def plotpoint(point):
    plt.arrow(point[0], point[1], 0.5 * np.cos(point[2]), 0.5*np.sin(point[2]), color='r', width=m / 100)
    plt.plot(point[0], point[1],'bo')

