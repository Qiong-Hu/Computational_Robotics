import numpy as np
import matplotlib.pyplot as plt
import math
import random
import time

# Define robot shape
R = 25      # unit: mm, radius of the wheels
L = 100     # unit: mm, length of the robot
W = 90      # unit: mm, width of the robot

C = 2 * math.pi * R    # unit: mm, circumference of the wheels

# Maximum values
wmax = 1    # unit: RPS (rotations per second), maximum angular velocity of the wheels
vx_max = vy_max = C * wmax    # unit: mm/s, maxium velocity of the robot in the direction of x and y
wmax_robot = 2 * C * wmax / W      # unit: rad/s, maximum angular velocity of the robot. = max{C * (w1 - w2) / W} = 2 * C * wmax / W (w1, w2 unit: RPS)

show_animation = True


# Problem 1(a)
# Define rebot model
# input (action) space
# Input = {input}, input = (w1, w2). w1 and w2 represent angular velocity of wheel 1 and wheel 2. w1, w2 ∈ [-wmax, wmax] = [-1, 1] RPS
D_I = 2     # The dimensionality is 2

# w is the angular velocity of the robot. positive: clockwise, negative: anti-clockwise
# w = C * (w1 - w2) / W rad/s
# v1, v2 represent velocity of wheel 1 and wheel 2
# v1 = C * w1 mm/s, v2 = C * w2 mm/s
# v is the velocity of the robot
# v = (vx, vy), |v| = (v1 + v2) / 2 mm/s
# vx = v cos(θ) = C * (w1 + w2) / 2 * cos(θ) mm/s
# vy = v sin(θ) = C * (w1 + w2) / 2 * sin(θ) mm/s

# robot state (configuration) space
# Config = {config}, config = (x, y, θ). Coordinate (x, y) represents the location of the centerpoint of the wheels, θ is the angle between +X and the direction of the robot, θ ∈ [0, 2 * pi)
D_C = 3     # The dimensionality is 3

# operational space
# O = {o}, o = (x, y). Coordinate (x, y) represent the map with obstacles
D_O = 2     # The dimensionality is 2


# Problem 1(b)
# Define the (continuous space) system model.
# Assume a fully observable system, i.e. the state itself is directly accessible with no sensor model.

# Define map
# an m * n rectangular environment
m = 5000   # unit: mm, length of the map
n = 5000   # unit: mm, width of the map

# obstacles: a list to store obstacle states
# obstacles= {obstacle}, obstacle = [x, y, w, h, angle]
# each obstacle [x, y, w, h, angle] is a rectangle, (x, y) is the coordinate of the lower-left corner of the obstacle, (w, h) is the width and height of the obstacle, angle is the orientation of the rectangle
# randomly generate 10 obstacle rectangles
obstacles = []
for i in range(10):
    x = random.uniform(0, m)
    y = random.uniform(0, n)
    w = random.uniform(0, m / 10)
    h = random.uniform(0, n / 10)
    angle = random.uniform(0, 180)
    obstacles.append([x, y, w, h, angle])

# start state: s0 = [x0, y0, θ0]
s0 = [0, 0, 0]

# target state: st = [xt, yt, θt]
# randomly generate a target state
x = random.uniform(0, m)
y = random.uniform(0, n)
z = random.uniform(0, 2 * math.pi)
st = [x, y, z]


# Trajectory Planning
# Problem 2(a)
# Given a set of points V in C-space and a single target point xt, define and justify a metric that can be used to determine which of the points in V is closest to xt.

# RRT node
class Node():
    def __init__(self, state):
        self.state = state
        self.path = []
        self.parent = None

# determine which of the points in V is the closest to xt
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
    weighted_dist = ((V[0].state[0] - xt[0]) / vx_max) ** 2 + ((V[0].state[1] - xt[1]) / vy_max) ** 2 + ((V[0].state[2] - xt[2]) / wmax_robot) ** 2

    for point in V:
        dist = ((point.state[0] - xt[0]) / vx_max) ** 2 + ((point.state[1] - xt[1]) / vy_max) ** 2 + ((point.state[2] - xt[2]) / wmax_robot) ** 2
        if dist < weighted_dist:
            closest_node = point
            weighted_dist = dist
    return closest_node


# Problem 2(b)
# Given arbitrary initial robot state xi and target robot state xt (in C-space), generate a smooth achievable trajectory (that matches the metric you defined above) from the xi towards xt lasting one second. What are the control inputs for this trajectory?

# three-step policy: 
# 1, rotate from the initial orientation to the angle that points to (or is back to) the target position
# 2, go straight until arrive at the target position
# 3, rotate until the orientation is the same as the target state
# 4, if one-second time is not enough to accomplish all three steps, stop wherever when running out of time
def generate_trajectory(xi, xt):
    # Given the initial and target robot state: xi, xt in C-space.
    # Return: an achievable trajectory

    # a list to store each point in the trajectory within one second
    trajectory = []
    trajectory.append(xi)

    # control inputs of two wheels
    inputs = []

    # remaining time within one second, updated after each step
    remain_time = 1

    # end point of each step
    end = [xi[0], xi[1], 0]

    # start-to-goal vector
    diff_x = xt[0] - xi[0]
    diff_y = xt[1] - xi[1]
    diff_dist = math.sqrt(diff_x ** 2 + diff_y ** 2)

    # angle between +x and start-to-goal vector, angle ∈ [-pi, pi]
    angle = np.arctan2(diff_y, diff_x)

    # step 1: rotate towards (or back to) the target position
    diff_angle = (xi[2] - angle) % math.pi
    if diff_angle <= math.pi / 2:
        end[2] = xi[2] - diff_angle
    else:
        end[2] = xi[2] + (math.pi - diff_angle)
    trajectory.append(end)
    # print(trajectory)

    # calculate remaining time after step 1
    required_time = diff_angle % (math.pi / 2) / wmax_robot
    remain_time = 1 - required_time
    inputs.append((-wmax, wmax, required_time))

    # calculate required time for step 2
    required_time = diff_dist / C
    end = [end[0], end[1], end[2]]      # update list pointer

    # if not enough time for step 2 (to reach the target after rotation)
    if remain_time < required_time:
        dist = remain_time * C
        end = [xi[0] + dist * math.cos(angle), xi[1] + dist * math.sin(angle), end[2]]
        trajectory.append(end)
        inputs.append((wmax, wmax, remain_time))
        # print(trajectory)

    # if the robot has enough time to reach the target after rotation
    else:
        end = [xt[0], xt[1], end[2]]
        trajectory.append(end)
        inputs.append((wmax, wmax, required_time))
        # print(trajectory)

        # for the remaining time, do the step 3 (rotate to the orientation of the target state)
        end = [xt[0], xt[1], end[2]]
        remain_time -= required_time 
        diff_angle = (end[2] - xt[2]) % (2 * math.pi)
        required_time = (diff_angle % math.pi) / wmax_robot

        # if have enough time for step 3, then arrive at the target state
        if remain_time >= required_time:
            trajectory.append(xt)
            inputs.append((-wmax, wmax, required_time))
            # print(trajectory)

        # if not enough time for step 3
        else:
            if 0 <= diff_angle <= math.pi:
               end[2] = end[2] - wmax_robot * remain_time
            else:
                end[2] = end[2] + wmax_robot * remain_time
            trajectory.append(end)
            inputs.append((-wmax, wmax, remain_time))
            # print(trajectory)
    return trajectory, inputs


# Problem 2(c)
# Given a smooth robot trajectory in Configuration space and obstacles defined in operational space, determine whether this trajectory is collision free

def cross(p1, p2, p3):
    x1 = p2[0] - p1[0]
    y1 = p2[1] - p1[1]
    x2 = p3[0] - p1[0]
    y2 = p3[1] - p1[1]
    return x1 * y2 - x2 * y1

# determine whether two line segments are intersecting
def IsIntersec(p1, p2, p3, p4):
    # (p1, p2) defines line 1, (p3, p4) defines line 2
    # If there is an intersection p0, then p0 is on both lines, so min{p1, p2} <= p0 <= max{p1, p2} and min{p3, p4} <= p0 <= max{p3, p4} are satisfied for both x component and y component
    if (max(p1[0], p2[0]) >= min(p3[0], p4[0]) 
        and max(p3[0], p4[0]) >= min(p1[0], p2[0]) 
        and max(p1[1], p2[1]) >= min(p3[1], p4[1]) 
        and max(p3[1], p4[1]) >= min(p1[1], p2[1])): 

        if(cross(p1, p2, p3) * cross(p1, p2, p4) <= 0
           and cross(p3, p4, p1) * cross(p3, p4, p2) <= 0):
            result = True
        else:
            result = False
    else:
        result = False
    return result

# determine whether two rectangles have intersections
def isCollisionRectangle(Rectangle1, Rectangle2):
    # Rectangle: [[x1, y1], [x2, y2], [x3, y3], [x4, y4]]
    collision = False
    for i in range(4):
        for j in range(4):
            collision = collision or IsIntersec(Rectangle1[i],
                Rectangle1[(i + 1) % 4], Rectangle2[j], Rectangle2[(j + 1) % 4])
    return collision

# determine whether trajectory is collision free.
def isCollisionTrajectory(trajectory, obstacles):
    collision = False
    n = len(trajectory)
    for i in range(n - 1):
        x = trajectory[i][0]
        y = trajectory[i][1]
        z = trajectory[i][2]
        x1 = trajectory[i + 1][0]
        y1 = trajectory[i + 1][1]
        z1 = trajectory[i + 1][2]
        steps = 10
        for j in range(steps):
            xt = j * (x1 - x) / (steps - 1) + x
            yt = j * (y1 - y) / (steps - 1) + y
            zt = j * (z1 - z) / (steps - 1) + z
            # four vertices of the robot
            p1 = [xt + R * math.cos(zt) + W / 2 * math.sin(zt), yt + R * math.sin(zt) - W / 2 * math.cos(zt)]
            p2 = [p1[0] - L * math.cos(zt), p1[1] - L * math.sin(zt)]
            p3 = [p2[0] - W * math.sin(zt), p2[1]+W * math.cos(zt)]
            p4 = [p3[0] + L * math.cos(zt), p3[1]+L * math.sin(zt)]
            for obstacle in obstacles:
                w = obstacle[2]
                h = obstacle[3]
                zeta = obstacle[4] / 180 * math.pi
                # four vertices of the obstacle
                p5 = [obstacle[0],obstacle[1]]
                p6 = [p5[0] - h * math.sin(zeta),p5[1] + h * math.cos(zeta)]
                p7 = [p6[0] + w * math.cos(zeta),p6[1] + w * math.sin(zeta)]
                p8 = [p7[0] + h * math.sin(zeta),p7[1] - h * math.cos(zeta)]
                #whether trajectory is collision free.
                collision = collision or iscollisionRectangle([p1, p2, p3, p4], [p5, p6, p7, p8])
    return collision


# Problem 2(d)
# Combine the above to implement an RRT planner generating a trajectory from a specified initial state to the desired goal region. Visualize the evolution of the RRT.

def RRT(s0, s1, obstacles):
    V = [Node(s0)]
    end = Node(s0)
    while ((end.state[0] - s1[0]) / vx_max) ** 2 + ((end.state[1] - s1[1]) / vy_max) ** 2 + ((end.state[2] - s1[2]) / wmax_robot) ** 2 >=1:     
        # get a random point            
        randompoint=[random.uniform(0,m),random.uniform(0,n),random.uniform(0,2*math.pi)]
        start = find_closestNode(V, randompoint)
        #  generate a trajectory from start to random point
        trajectory = generate_trajectory(start.state, randompoint)
        # if not collision, add it into V
        if not iscollisiontrajectory(trajectory,obstacles):
            end = Node(trajectory[-1])
            end.parent = start
            V.append(end)
        if len(V) > 10000:
            break
    #print(len(V))
    trajectory = [end.state]
    while (end.parent != None):
        trajectory.append(end.parent.state)
        end = end.parent
    return trajectory, V

# For visualization: plot trajectory, obstacles, and points that represent the robot
def plotTrajectory(trajectory, ax):
    for point in trajectory:
        # plot robot
        x = point[0] + R*math.cos(point[2]) + W/2 * math.sin(point[2]) - L * math.cos(point[2])
        y = point[1] + R*math.sin(point[2]) - W/2 * math.cos(point[2]) - L * math.sin(point[2])
        rec = plt.Rectangle((x, y), L, W, angle = point[2]*180/math.pi, color = 'b')
        ax.add_patch(rec)
    return ax

def plotObstacles(obstacles, ax):
    for obstacle in obstacles:
        rec = plt.Rectangle((obstacle[0],obstacle[1]), obstacle[2], obstacle[3], angle = obstacle[4], color = 'k')
        ax.add_patch(rec)
    return ax

def plotPoint(point,ax):
    # plot arrow
    plt.arrow(point[0], point[1], 0.5*np.cos(point[2]),0.5*np.sin(point[2]), color='r', width=m/100)
    # plot center point 
    plt.plot(point[0], point[1],'bo')
    # plot robot 
    x = point[0] + R * math.cos(point[2]) +W / 2*math.sin(point[2]) - L * math.cos(point[2])
    y = point[1] + R * math.sin(point[2]) - W / 2*math.cos(point[2]) - L * math.sin(point[2])
    rec = plt.Rectangle((x, y), L, W, angle = point[2]*180 / math.pi, color = 'b')
    ax.add_patch(rec)
    return ax


# Problem 3(a)
# Run some examples that demonstrate the performance (in terms of computational efficiency, trajectory efficiency, and obstacle avoidance) of your planner as your robot tries to achieve various goals (such as head-in parking and parallel parking between other such parked vehicles). Clearly describe the experiments that were run, the data that was gathered, and the process by which you use that data to characterize the performance of your planner. Include figures; you may also refer to animations uploaded to your git repo.

#test performance of RRT
def RRT_test(s0, s1, obstacles):
    start = time.time()
    trajectory,V = RRT(s0,s1,obstacles)
    end = time.time()
    # Caculate time RRT need to find a trajectory
    print(end - start)
    # Caculate number of nodes need to find a trajectory
    space = len(V)
    print(space)
    #Caculate time robot need to go for a trajectory
    t = 0
    for i in range(len(trajectory) - 1):
        dx = trajectory[i + 1][0] - trajectory[i][0]
        dy = trajectory[i + 1][1] - trajectory[i][1]
        dz = trajectory[i + 1][2] - trajectory[i][2]
        t += math.sqrt(dx ** 2 + dy ** 2) / vx_max + dz / wmax_robot
    print (t)
    #plot figure
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    # plot start and goal
    plotpoint(s0, ax)
    plotpoint(s1, ax)
    # plot trajectory
    plottrajectory(trajectory, ax)
    # plot obstacles
    plotobstacles(obstacles, ax)
    # plot grid
    plt.xlim((-m / 10, m))
    plt.ylim((-n / 10, n))
    plt.grid()
    plt.show()

'''
# For debug and test
x=random.uniform(0,m)
y=random.uniform(0,n)
z=random.uniform(0,2*math.pi)
s0=[0,0,0]
s1=[x,y,z]
obstacles=[]
for i in range(10):
    x=random.uniform(0,m)
    y=random.uniform(0,n)
    w=random.uniform(0,m/10)
    h=random.uniform(0,n/10)
    angle = random.uniform(0,180)
    obstacles.append([x, y, w, h, angle])

RRT_test(s0, s1, obstacles)
'''

# Prob 3(b)
# Run some examples that demonstrate the performance. 

# Prob 3(c)
# Improve on your planner using the RRT* algorithm, and compare to your original RRT planner using the above metrics.
#Caculate time robot need to go for a trajectory
def trajectory_time(trajectory):
    t = 0
    for i in range(len(trajectory) - 1):
        dx = trajectory[i+1][0]-trajectory[i][0]
        dy = trajectory[i+1][1]-trajectory[i][1]
        dz = trajectory[i+1][2]-trajectory[i][2]
        t += math.sqrt(dx**2+dy**2)/vx_max + dz/wmax_robot
    return t

def RRTstar(s0,s1,obstacles):
    V = [Node(s0)]
    end = Node(s0)
    while ((end.state[0] - s1[0])/vx_max) ** 2 + ((end.state[1] - s1[1])/vy_max) ** 2 + ((end.state[2] - s1[2]) / wmax_robot) ** 2 >=1:     
        # get a random point            
        randompoint=[random.uniform(0,m),random.uniform(0,n),random.uniform(0,2*math.pi)]
        start = find_closestNode(V, randompoint)
        #  generate a trajectory from start to random point
        trajectory = generate_trajectory(start.state, randompoint)
        mintime = trajectory_time(trajectory)+start.t
        # if not collision, add it into V
        if not iscollisiontrajectory(trajectory,obstacles):
            end = Node(trajectory[-1])
            for node in V:
                if ((end.state[0] - node.state[0])/vx_max) ** 2 + ((end.state[1] - node.state[1])/vy_max) ** 2 + ((end.state[2] - node.state[2]) / wmax_robot) ** 2 <=1:
                    trajectory = generate_trajectory(node.state, end.state)
                    if trajectory_time(trajectory)+node.t<mintime:
                        start = node
                        mintime = trajectory_time(trajectory)+node.t
            end.parent = start
            end.t = mintime
            V.append(end)
        if len(V)>20000:
            break

    trajectory = [end.state]
    while (end.parent != None):
        trajectory.append(end.parent.state)
        end = end.parent
    return trajectory,V



# Prob 3(d)
# Qualitatively describe some conclusions about the effectiveness of your planner for potential tasks your robot may encounter. For example, what happens to your planner in the presence of process noise, i.e. a stochastic system model? How might you modify your algorithm to better handle noise?
