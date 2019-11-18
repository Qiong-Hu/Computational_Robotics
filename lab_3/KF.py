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
wmax = 1                            # unit: RPS (rotations per second), maximum expected angular velocity of the wheels
wmax_robot = 2 * C * wmax / width   # unit: rad/s, maximum expected angular velocity of the robot. wmax_robot = max{C * (w1 - w2) / width} = 2 * C * wmax / width (w1, w2 unit: RPS)
vmax_robot = wmax * C               # unit: mm/s, maximum expected velocity of the robot

# Inputs: (w1, w2)
# w1, w2: the input angular velocity of the left and right wheel, unit: RPS, range: [-wmax, wmax]
# w1, w2 are controlled by signals from the microcontrollers. 

# Sensor outputs: (d1, d2, theta, omega)
# d1: the distance to a wall in a straight line in front of the robot
# d2: the distance to a wall in a straight line to the right of the robot
# theta: an absolute bearing indicating the angle of the robot with respect to magnetic north (direction of +X) gained from an inertial measurement unit (IMU), unit: rad
# omega: a measurement of the rotational speed from an angular rate (gyro) sensor, unit: rad/s

# Define robot state space S
# S = {s}, s = (x, y, theta, omega)
# (x, y): the location coordinate of the centerpoint of the wheels, x ∈ [0, L], y ∈ [0, W]
# theta: the angle between +X and the orientation of the robot, theta ∈ [0, 2 * pi), unit: rad
# omega: the angular velocity of the robot, omega ∈ [-wmax_robot, wmax_robot], unit: rad/s

s0 = [100, 100, 0, 0]      # Initial state


# Problem 2(b)
# Include realistic noise terms into the model as appropriate, and numerically quantify their parameters.

# Define time step dt
dt = 0.1    # unit: second

# Define process noise
# There may be slippage between the wheels and the floor or sticking in the gears of the motor, so assume the error and the resulting effective angular speed of each wheel is Gaussian
# w1_e, w2_e: error of effective angular velocity of two wheels w1, w2, unit: RPS
w_cov = (0.05 * wmax) ** 2     # w1_e, w2_e ~ N(0, w_cov)

# Define measurement noise
# (d1_e, d2_e, theta_e, omega_e): sensor errors for each sensor output components
# d1_e ~ N(0, d1 * d_cov), d2_e ~ N(0, d2 * d_cov)
d_cov = 0.06 ** 2       # unitless, standard deviation of the laser range sensor VL53L0X is 6%
# theta_e ~ N(0, theta_cov)
theta_cov = (0.1 / 180 * math.pi) ** 2      # unit: rad, standard deviation of the gyroscope on MPU-9250 is 0.1 degree
# omega_e ~ N(omega_bias, omega_cov)
omega_bias = 0      # unit: rad/s
omega_cov = (0.1 / 180 * math.pi / dt) ** 2     # unit: rad/s


# Problem 2(c)
# Create a Kalman Filter based state estimator to take the motor commands and sensor measurements and generate a state estimate. Implement an Extended Kalman Filter (EKF), or an Unscented Kalman Filter (UKF).

# Time evolution (process) model

# Theories of Extended Kalman Filter; we use EKF because of the non-linearity of the problem.
# s(t + dt) = f(s(t), u(t), w(t))
# s(t), s(t + dt): the robot state at time t and t + dt
# u(t): inputs at time t
# w(t): input noises at time t
# Linear approximation: s(t + dt) ≈ F(t) s(t) + W(t) w(t)
# Matrix F(t)[i][j] = df/dx

# Update state from t to t + dt
def stateTimeEvolution(s, inputs, noise = True):
    # Given: current state s = (x, y, theta, omega)
    # Given: inputs from actuators, inputs = (w1, w2), angular velocities of the two wheels
    # Return: updated state s' = (x', y', theta', omega') after evolving for time dt

    w1, w2 = inputs

    # w1_e, w2_e: error of effective angular velocity, ~ N(0, w_cov)
    w_cov = w_cov * noise
    w1_e = np.random.normal(0, math.sqrt(w_cov))
    w2_e = np.random.normal(0, math.sqrt(w_cov))

    # v1, v2: velocity of the left and right wheel, unit: mm/s
    v1 = (w1 + w1_e) * C
    v2 = (w2 + w2_e) * C

    # Effective linear and angular velocity of the robot (angular velocity direction: positive: clockwise, negative: anti-clockwise)
    v_robot = (v1 + v2) / 2         # unit: mm/s
    w_robot = (v2 - v1) / width     # unit: rad/s

    # Linear and angular displacements in time dt
    ds = v_robot * dt
    d_theta = w_robot * dt
    theta = s[2] 
    dx = ds * math.cos(theta + d_theta / 2) 
    dy = ds * math.sin(theta + d_theta / 2)

    # When t -> t + dt: x -> x + dx, y -> y + dy, theta -> (theta + d_theta) % 2pi
    s_new = [s[0] + dx, s[1] + dy, (s[2] + d_theta) % (2 * math.pi), w_robot ]
    return s_new

# Calculate matrix F, dimension of F is 4 x 4
def Fmatrix(s, inputs):
    w1, w2 = inputs
    theta = s[2]
    ds = (w1 + w2) * C * dt / 2
    d_theta = (w2 - w1) * C * dt / width
    F = np.array(
        [[1, 0, -ds * math.sin(theta + d_theta / 2), 0],
         [0, 1, ds * math.cos(theta + d_theta / 2), 0],
         [0, 0, 1, 0],
         [0, 0, 0, 0]])
    return F

# Calculate matrix W, dimension of W is 4 x 2
def Wmatrix(s):
    w1, w2 = inputs
    theta = s[2]
    ds = (w1 + w2) * C * dt / 2
    d_theta = (w2 - w1) * C * dt / width
    z1 = C * dt / 2
    z2 = C * dt / width / 2
    W = np.array([[z1 * math.cos(theta + d_theta / 2) + ds * math.sin(theta + d_theta / 2) * z2, 
        z1 * math.cos(theta + d_theta / 2) + ds * -math.sin(theta + d_theta / 2) * z2 ], 
        [z1 * math.sin(theta+ d_theta / 2) + ds * math.cos(theta + d_theta / 2) * -z2, 
        z1 * math.sin(theta + d_theta / 2) + ds * math.cos(theta + d_theta / 2) * z2], 
        [-2 * z2, 2 * z2], 
        [-C / width, C / width]])
    return W

# Calculate matrix Q
# E[w*wˆT] = Q
Qmatrix = np.array([[w_cov, 0], [0, w_cov]])

# Observation (measurement) model
# TODO: explain theories

# Predict sensor output from robot state
# Steps:
# 1, Calculate the angles between magnetic north (direction of +X) and the line connecting the four corners of the wall and robot
# 2, Divide the intersection point on the wall into four regions, depend on the angle of the laser w.r.t. the angles to the corner
# 3, Calculate the distance between the robot and the intersection in the front of the robot, which is an estimated value of the sensor output with a gaussian distribution
# 4, Repeat Step3 to calculate the distance to the right of the robot

def stateToSensor(s):
    # Given: robot state s = (x, y, theta, omega)
    # Return: sensor outputs (d1_s, d2_s, theta_s, omega_s)

    x, y, theta, omega = s[0], s[1], s[2], s[3]
    theta_e = np.random.normal(0, math.sqrt(theta_cov))
    theta_s = (theta + theta_e) % (2 * math.pi)

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
    theta = (theta - math.pi / 2) % (2 * math.pi)
    if  theta1 <= theta < theta2: # robot facing top side of the map
        d2 = (W - y) / math.sin(theta)            
    elif theta2 <= theta < theta3: # robot facing left side of the map
        d2 = x / math.cos(math.pi - theta)
    elif theta3 <= theta < theta4:
        d2 = y / math.sin(theta - math.pi)
    else:
        d2 = (L - x) / math.cos(theta)

    d1_e = np.random.normal(0, math.sqrt(d_cov)) * d1
    d2_e = np.random.normal(0, math.sqrt(d_cov)) * d2
    omega_e = np.random.normal(omega_bias, math.sqrt(omega_cov))
    d1_s = d1 + d1_e
    d2_s = d2 + d2_e
    omega_s = omega + omega_e
    return [d1_s, d2_s, theta_s, omega_s]

# Calculate matrix H
def Hmatrix(s):
    
    x, y, theta, omega = s[0], s[1], s[2], s[3]

    if  theta1 <= theta < theta2: # robot facing top side of the map
        # d1 = (W - y) / math.sin(theta)
        h00 = 0
        h01 = -1/math.sin(theta)
        h02 = (W - y) * -math.cos(theta)/ math.sin(theta)**2       
    elif theta2 <= theta < theta3: # robot facing left side of the map
        # d1 = x / math.cos(math.pi - theta)
        h00 = 1 / math.cos(math.pi - theta)
        h01 = 0
        h02 = x * -math.sin(math.pi - theta)/ math.cos(math.pi - theta)**2
    elif theta3 <= theta < theta4:
        # d1 = y / math.sin(theta - math.pi)
        h00 = 0
        h01 = -1/ math.sin(theta - math.pi)
        h02 = y * -math.cos(theta - math.pi)/ math.sin(theta - math.pi)**2
    else:
        # d1 = (L - x) / math.cos(theta)
        h00 = -1 / math.cos(theta)
        h01 = 0
        h02 = (L - x)* math.sin(theta)/ math.cos(theta)**2

    # Calculate the distance to the nearest wall to the right of the robot
    theta = (theta - math.pi / 2) % (2 * math.pi)
    if  theta1 <= theta < theta2: # robot facing top side of the map
        #d2 = (W - y) / math.sin(theta)  
        h10 = 0
        h11 = -1/math.sin(theta)
        h12 = (W - y) * -math.cos(theta)/ math.sin(theta)**2          
    elif theta2 <= theta < theta3: # robot facing left side of the map
        #d2 = x / math.cos(math.pi - theta)
        h10 = 1 / math.cos(math.pi - theta)
        h11 = 0
        h12 = x * -math.sin(math.pi - theta)/ math.cos(math.pi - theta)**2
    elif theta3 <= theta < theta4:
        #d2 = y / math.sin(theta - math.pi)
        h10 = 0
        h11 = -1/ math.sin(theta - math.pi)
        h12 = y * -math.cos(theta - math.pi)/ math.sin(theta - math.pi)**2
    else:
        #d2 = (L - x) / math.cos(theta)
        h10 = -1 / math.cos(theta)
        h11 = 0
        h12 = (L - x)* math.sin(theta)/ math.cos(theta)**2

    H = np.array(
        [[h00, h01, h02, 0],
         [h10, h11, h12, 0],
         [0, 0, 1, 0],
         [0, 0, 0, 1]])
    
    return H

# Calculate matrix R
Rmatrix = np.array([[d_cov, 0, 0, 0],
    [0, d_cov, 0, 0],
    [0, 0, theta_cov, 0],
    [0, 0, 0, omega_cov]
    ])

