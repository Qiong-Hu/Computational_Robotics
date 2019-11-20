import numpy as np
import matplotlib.pyplot as plt
import math
import random
import time

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
wmax_robot = 2 * C * wmax / width   # unit: rad/s, maximum expected angular velocity of the robot. wmax_robot = max{C * (w2 - w1) / width} = 2 * C * wmax / width (w1, w2 unit: RPS)
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
# d1_e ~ N(0, d1 * d1 * d_cov), d2_e ~ N(0, d2 * d2 * d_cov)
d_cov = 0.06 ** 2       # unitless, standard deviation of the laser range sensor VL53L0X is 6%, relative d1, d2
# theta_e ~ N(0, theta_cov)
theta_cov = (0.1 / 180 * math.pi) ** 2      # unit: rad, standard deviation of the gyroscope on MPU-9250 is 0.1 degree
# omega_e ~ N(omega_bias, omega_cov)
omega_bias = 0      # unit: rad/s
omega_cov = (0.1 / 180 * math.pi) ** 2     # unit: rad/s, standard deviation of the omega on MPU-9250 is 0.1/dt degree/s, dt is the time step


# Problem 2(c)
# Create a Kalman Filter based state estimator to take the motor commands and sensor measurements and generate a state estimate. Implement an Extended Kalman Filter (EKF), or an Unscented Kalman Filter (UKF).
# We use EKF because of the non-linearity of the problem.

# Time evolution (process) model
# Theories: 
# s(t + dt) = f(s(t), u(t), w(t))
# s(t), s(t + dt): the robot state at time t and t + dt
# u(t): inputs at time t
# w(t): input (process) noise at time t
# Linear approximation: s(t + dt) ≈ F(t) s(t) + W(t) w(t)
# Matrix F(t)[i][j] = df[i]/dx[j]
# Matrix W(t)[i][j] = df[i]/dw[j]
# Process noise covariance Q(t) = E[w(t) * w(t)^T]

# Update state from t to t + dt
def stateTimeEvolution(s, inputs, w_cov = 0):
    # Given: current state s = (x, y, theta, omega)
    # Given: inputs from actuators, inputs = (w1, w2), angular velocities of the two wheels
    # Return: updated state s' = (x', y', theta', omega') after evolving for time dt

    w1, w2 = inputs

    # w1_e, w2_e: error of effective angular velocity, ~ N(0, w_cov)
    w1_e = np.random.normal(0, math.sqrt(w_cov))
    w2_e = np.random.normal(0, math.sqrt(w_cov))

    # v1, v2: velocity of the left and right wheel, unit: mm/s
    v1 = (w1 + w1_e) * C
    v2 = (w2 + w2_e) * C

    # Effective linear and angular velocity of the robot (angular velocity direction: positive: anti-clockwise, negative: clockwise)
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

# Calculate matrix F
# F is the Jacobian matrix of partial derivatives of function "stateTimeEvolution" with respect to state s
# Dimensions: dim(F) = dim(s) * dim(s) = 4 * 4
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

# Calculate matrix W
# W is the Jacobian matrix of partial derivatives of function "stateTimeEvolution" with respect to input noise w
# Dimensions: dim(W) = dim(s) * dim(inputs) = 4 * 2
def Wmatrix(s, inputs):
    w1, w2 = inputs
    theta = s[2]
    ds = (w1 + w2) * C * dt / 2
    d_theta = (w2 - w1) * C * dt / width
    z1 = C * dt / 2
    z2 = C * dt / width / 2
    W = np.array([[z1 * math.cos(theta + d_theta / 2) + ds * math.sin(theta + d_theta / 2) * z2, 
        z1 * math.cos(theta + d_theta / 2) + ds * -math.sin(theta + d_theta / 2) * z2], 
        [z1 * math.sin(theta+ d_theta / 2) + ds * math.cos(theta + d_theta / 2) * -z2, 
        z1 * math.sin(theta + d_theta / 2) + ds * math.cos(theta + d_theta / 2) * z2], 
        [-2 * z2, 2 * z2], 
        [-C / width, C / width]])
    return W

# Calculate matrix Q
# Q is the process noise covariance, Q = E[w*wˆT]
# Dimensions: dim(Q) = dim(w) * dim(w) = 2 * 2
def Qmatrix (w_cov = 0):
    Q = np.array([[w_cov, 0], [0, w_cov]])
    return Q

# Observation (measurement) model
# Theories: 
# y(t) = h(s(t), v(t))
# s(t), y(t): the robot state and the measured state at time t
# v(t): output (measurement) noise at time t
# Linear approximation: y(t) ≈ H(t) s(t) + V(t) v(t)
# Matrix H(t)[i][j] = dh[i]/dx[j]
# Matrix V(t)[i][j] = dh[i]/dv[j]
# Measurement noise covariance R(t) = E[v(t) * v(t)^T]

# Predict sensor output from robot state
# Steps:
# 1, Calculate the angles between magnetic north (direction of +X) and the line connecting the robot and four corners of the wall
# 2, Divide the intersection point on the wall into four regions (up, left, bottom, right), depending on the angle of the laser w.r.t. the angles to the corner
# 3, Calculate the distance between the robot and the intersection in front of the robot, which is an estimated value of the sensor output with a gaussian distribution
# 4, Repeat Step 3 to calculate the distance to the right of the robot

def stateToSensor(s, d_cov = 0, theta_cov = 0, omega_bias = 0, omega_cov = 0):
    # Given: robot state s = (x, y, theta, omega)
    # Return: sensor outputs (d1_s, d2_s, theta_s, omega_s)

    x, y, theta, omega = s[0], s[1], s[2], s[3]
    theta_e = np.random.normal(0, math.sqrt(theta_cov))
    theta_s = (theta + theta_e) % (2 * math.pi)

    # Calculate the angle of the vector pointing from (x, y) to (0, 0), (0, L), (L, 0), (L, W)
    theta1 = math.atan2(W - y, L - x)             # top-right (L, W)
    theta2 = math.atan2(W - y, -x)                # top-left (0, W)
    theta3 = math.atan2(-y, -x) + 2 * math.pi     # bottom-left (0, 0)
    theta4 = math.atan2(-y, L - x) + 2 * math.pi  # bottom-right (L, 0)

    # Calculate the distance to the nearest wall in front of the robot
    if  theta1 <= theta < theta2:           # intersection on the top wall
        d1 = (W - y) / math.sin(theta)            
    elif theta2 <= theta < theta3:          # intersection on the left wall
        d1 = x / math.cos(math.pi - theta)
    elif theta3 <= theta < theta4:          # intersection on the bottom wall
        d1 = y / math.sin(theta - math.pi)
    else:                                   # intersection on the right wall
        d1 = (L - x) / math.cos(theta)

    # Calculate the distance to the nearest wall to the right of the robot
    theta = (theta - math.pi / 2) % (2 * math.pi)
    if  theta1 <= theta < theta2:           # intersection on the top wall
        d2 = (W - y) / math.sin(theta)            
    elif theta2 <= theta < theta3:          # intersection on the left wall
        d2 = x / math.cos(math.pi - theta)
    elif theta3 <= theta < theta4:          # intersection on the bottom wall
        d2 = y / math.sin(theta - math.pi)
    else:                                   # intersection on the right wall
        d2 = (L - x) / math.cos(theta)

    d1_e = np.random.normal(0, math.sqrt(d_cov)) * d1
    d2_e = np.random.normal(0, math.sqrt(d_cov)) * d2
    omega_e = np.random.normal(omega_bias, math.sqrt(omega_cov))
    d1_s = d1 + d1_e
    d2_s = d2 + d2_e
    omega_s = omega + omega_e
    return [d1_s, d2_s, theta_s, omega_s]

# Calculate matrix H
# H is the Jacobian matrix of partial derivatives of function "stateToSensor" with respect to state s
# Dimensions: dim(H) = dim(outputs) * dim(s) = 4 * 4
def Hmatrix(s):
    x, y, theta, omega = s[0], s[1], s[2], s[3]
    
    # Calculate the angle of the vector pointing from (x, y) to (0, 0), (0, L), (L, 0), (L, W)
    theta1 = math.atan2(W - y, L - x)             # top-right (L, W)
    theta2 = math.atan2(W - y, -x)                # top-left (0, W)
    theta3 = math.atan2(-y, -x) + 2 * math.pi     # bottom-left (0, 0)
    theta4 = math.atan2(-y, L - x) + 2 * math.pi  # bottom-right (L, 0)

    # Gradient when calculating the distance to the nearest wall in front of of the robot
    if  theta1 < theta < theta2:           # intersection on the top wall
        # d1 = (W - y) / math.sin(theta)
        h00 = 0
        h01 = -1 / math.sin(theta)
        h02 = -(W - y) * math.cos(theta) / math.sin(theta) ** 2
    elif theta2 < theta < theta3:           # intersection on the left wall
        # d1 = x / math.cos(math.pi - theta)
        h00 = -1 / math.cos(theta)
        h01 = 0
        h02 = -x * math.sin(theta)/ math.cos(theta) ** 2
    elif theta3 < theta < theta4:           # intersection on the bottom wall
        # d1 = y / math.sin(theta - math.pi)
        h00 = 0
        h01 = -1 / math.sin(theta)
        h02 = y * math.cos(theta)/ math.sin(theta) ** 2
    elif theta4 < theta < 2 * math.pi or 0 < theta < theta1: # intersection on the right wall
        # d1 = (L - x) / math.cos(theta)
        h00 = -1 / math.cos(theta)
        h01 = 0
        h02 = (L - x)* math.sin(theta)/ math.cos(theta) ** 2
    else:     # intersection at the corner, gradient function not continuous, set to be 0
        h00 = 0
        h01 = 0
        h02 = 0

    # Gradient when calculating the distance to the nearest wall to the right of the robot
    theta = (theta - math.pi / 2) % (2 * math.pi)
    if  theta1 < theta < theta2:           # intersection on the top wall
        # d2 = (W - y) / math.sin(theta)
        h10 = 0
        h11 = -1 / math.sin(theta)
        h12 = -(W - y) * math.cos(theta) / math.sin(theta) ** 2
    elif theta2 < theta < theta3:           # intersection on the left wall
        # d2 = x / math.cos(math.pi - theta)
        h10 = -1 / math.cos(theta)
        h11 = 0
        h12 = -x * math.sin(theta)/ math.cos(theta) ** 2
    elif theta3 < theta < theta4:           # intersection on the bottom wall
        # d2 = y / math.sin(theta - math.pi)
        h10 = 0
        h11 = -1 / math.sin(theta)
        h12 = y * math.cos(theta)/ math.sin(theta) ** 2
    elif theta4 < theta < 2 * math.pi or 0 < theta < theta1: # intersection on the right wall
        # d2 = (L - x) / math.cos(theta)
        h10 = -1 / math.cos(theta)
        h11 = 0
        h12 = (L - x)* math.sin(theta)/ math.cos(theta) ** 2
    else:     # intersection at the corner, gradient function not continuous, set to be 0
        h10 = 0
        h11 = 0
        h12 = 0

    H = np.array(
        [[h00, h01, h02, 0],
         [h10, h11, h12, 0],
         [0, 0, 1, 0],
         [0, 0, 0, 1]])
    
    return H

# Calculate matrix V
# V is the Jacobian matrix of partial derivatives of function "stateToSensor" with respect to measurement errors v
# Dimensions: dim(V) = dim(outputs) * dim(outputs) = 4 * 4
# Since the matrix V is normally too small to make distinctive difference in measurement update, compared to H, so we omit matrix V here.

# Calculate matrix R
# R is the measurement noise covariance, R = E[v*v^T]
# Dimensions: dim(R) = dim(outputs) * dim(outputs) = 2 * 2
def Rmatrix(s):
    x, y, theta, omega = s[0], s[1], s[2], s[3]
    
    # Calculate the angle of the vector pointing from (x, y) to (0, 0), (0, L), (L, 0), (L, W)
    theta1 = math.atan2(W - y, L - x)             # top-right (L, W)
    theta2 = math.atan2(W - y, -x)                # top-left (0, W)
    theta3 = math.atan2(-y, -x) + 2 * math.pi     # bottom-left (0, 0)
    theta4 = math.atan2(-y, L - x) + 2 * math.pi  # bottom-right (L, 0)

    # Calculate the distance to the nearest wall in front of the robot
    if  theta1 <= theta < theta2:           # intersection on the top wall
        d1 = (W - y) / math.sin(theta)            
    elif theta2 <= theta < theta3:          # intersection on the left wall
        d1 = x / math.cos(math.pi - theta)
    elif theta3 <= theta < theta4:          # intersection on the bottom wall
        d1 = y / math.sin(theta - math.pi)
    else:                                   # intersection on the right wall
        d1 = (L - x) / math.cos(theta)

    # Calculate the distance to the nearest wall to the right of the robot
    theta = (theta - math.pi / 2) % (2 * math.pi)
    if  theta1 <= theta < theta2:           # intersection on the top wall
        d2 = (W - y) / math.sin(theta)            
    elif theta2 <= theta < theta3:          # intersection on the left wall
        d2 = x / math.cos(math.pi - theta)
    elif theta3 <= theta < theta4:          # intersection on the bottom wall
        d2 = y / math.sin(theta - math.pi)
    else:                                   # intersection on the right wall
        d2 = (L - x) / math.cos(theta)

    R = np.array([[d_cov * d1 * d1, 0, 0, 0],
    [0, d_cov * d2 * d2, 0, 0],
    [0, 0, theta_cov, 0],
    [0, 0, 0, omega_cov]
    ])
    return R

# Create Extended Kalman Filter (EKF)
def EKF(s0, P0, inputs, timeUpdate = stateTimeEvolution, measureUpdate = stateToSensor, w_cov = w_cov, d_cov = d_cov, theta_cov = theta_cov, omega_bias = omega_bias, omega_cov = omega_cov, Fmatrix = Fmatrix, Wmatrix = Wmatrix, Qmatrix = Qmatrix, Hmatrix = Hmatrix, Rmatrix = Rmatrix):
    # Given:
        # s0: initial state (x, y, theta, omega)
        # P0: initial covariance matrix of the state, dim(P0) = dim(s) * dim(s) = 4 * 4
        # inputs: a list of control input sequences [(w1, w2)]
        # timeUpdate: process model, s' = f(s, u, w), return state (x, y, theta, omega)
        # measureUpdate: measurement model, y = h(s, v), return outputs (d1, d2, theta, omega)
        # w_cov, d_cov, theta_cov, omega_bias, omega_cov: noise parameters, as defined in Problem 2(b)
        # Fmatrix, Wmatrix, Qmatrix, Hmatrix, Rmatrix: matrices in the form of functions, as defined along with process and measurement model
    # Return:
        # traj: real trajectory of the robot {(x, y, theta, omega)}, generated from inputs and time update function, with process noise
        # obs: observed traces {(d1, d2, theta, omega)}, gained from measurement update function, with measurement noise
        # exp: expected trajectory {(x’, y’, theta’, omega’)}, gained from EKF algorithms, to estimate the real trajectory

    # Number of robot motions, same as the length of the input sequences
    step = len(inputs)

    # Initialization
    traj, exp = [s0], [s0]
    obs =[]
    s = s0
    P = P0

    # Trajectory evolution with time step of dt
    for i in range(step):
        s = traj[i]
        w = inputs[i]
        F = Fmatrix(s, w)
        W = Wmatrix(s, w)
        Q = np.dot(W, Qmatrix(w_cov)).dot(W.T)   # Q(t+1)=W(t)Q(t)W(t)^T

        # Real state
        s1 = timeUpdate(s, w, w_cov)    # s(t+1)=f(s(t),inputs(t),input_noise(t))
        # Get observation for s1
        ob = measureUpdate(s1, d_cov, theta_cov, omega_bias, omega_cov) # y(t)=h(s(t),output_noise(t))
        obs.append(ob)
        traj.append(s1)

        # Time update estimate
        s = exp[i]
        s1 = timeUpdate(s, w, 0)        # s(t+1)=f(s(t),inputs(t),0) without noise
        P = np.dot(F, P).dot(F.T) + Q   # P(t+1)=F(t)P(t)F(t)^T+W(t)Q(t)W(t)^T
        # exp.append(s1)

        # Observation update estimate
        R = Rmatrix(s1)
        H = Hmatrix(s1)
        K = np.dot(H, P).dot(H.T) + R   # K(t)=H(t)P(t)H(t)^T+R
        s2 = list(np.array(s1) + np.dot(P, H.T).dot(np.linalg.inv(K)).dot(np.array(ob) - np.array(measureUpdate(s1,0,0,omega_bias,0))))    # s(t+1)'=s(t+1)+P(t+1)H(t)^T K^-1 (y(t)-h(s(t),0))
        P = P - np.dot(P, H.T).dot(np.linalg.inv(K)).dot(H).dot(P)         # P(t+1)'=P(t+1)-P(t+1)H(t)^T K^-1 H(t) P(t+1)
        exp.append(s2)
    return traj, exp, obs


# Problem 3(a)
# Define and describe several reference trajectories (that in turn generate control input sequences) that capture the abilities and limitations of a state estimator in this environment.

# All reference trajectories are determined by initial state s0 and a list of control input sequences
# Trajectory1: straight forward line in the direction of +X
s01 = [100, 100, 0, 0]
inputs1 = []
for i in range(40):
    inputs1.append([1, 1])

# Trajectory2: straight forward line in the direction of +Y
s02 = [100, 100, math.pi / 2, 0]
inputs2 = []
for i in range(40):
    inputs2.append([1,1])

# Trajectory3: clockwise circular trajectory
s03 = [200, 200, 0, 0]
inputs3 = []
for i in range(40):
    inputs3.append([1, 0])

#trajectory4: anti-clockwise circular trajectory
s04 = [200, 200, 0, 0]
inputs4 = []
for i in range(40):
    inputs4.append([0, 1])

# Trajectory5: straight forward line in the direction of -X
s05 = [400, 400, math.pi, 0]
inputs5 = []
for i in range(40):
    inputs5.append([1, 1])

# Trajectory6: straight forward line in the direction of -Y
s06 = [400, 400, math.pi*3/2, 0]
inputs6 = []
for i in range(40):
    inputs6.append([1, 1])

# Trajectory7: straight forward line in the direction of 45 degree from +X
s07 = [100, 100, math.pi/4, 0]
inputs7 = []
for i in range(40):
    inputs7.append([1, 1])

# Trajectory8: complicated combined trajectory
s08 = [100, 100, 0, 0]
inputs8 = []
for i in range(10):
    inputs8.append([1, -1])
for i in range(10):
    inputs8.append([1, 1])
for i in range(10):
    inputs8.append([-1, 1])
for i in range(10):
    inputs8.append([1, 1])


# Problem 3(b)
# Implement a simulation, including models of your sensor and actuator response (especially including noise), to generate realistic sensor traces given the above control inputs. Present and explain the simulated sensor traces.

# Assign a trajectory (for example, Trajectory1)
s0 = s01
inputs = inputs1

# Initial state with zero covariance
P0 = np.zeros((4,4))

# Obtain sensor observations
traj, exp, obs = EKF(s0, P0, inputs, stateTimeEvolution, stateToSensor, w_cov, d_cov, theta_cov, omega_bias, omega_cov, Fmatrix, Wmatrix, Qmatrix, Hmatrix, Rmatrix)

# Get realistic sensor traces
print(obs)


# Problem 3(c)
# Implement your KF based state estimator on these examples, demonstrating the performance (in terms of accuracy and efficiency) of your computed state estimate over time as the robot is issued commanded input 2 sequences. Consider both perfect knowledge of the initial state as well as no knowledge of the initial state on the same traces. Clearly describe the experiments that were run, the data that was gathered, and the process by which you use that data to characterize the performance of your state estimator. Include figures; you may also refer to animations uploaded to your git repo.

# Plot the trajectory from the given list of robot states {(x, y, theta, omega)}
def plotTrajectory(trajectory, isreal):
    # Given: isreal (boolean value), if the trajectory is a pre-programmed reference trajectory, paint it blue, otherwise paint it red
    if isreal:
        for point in trajectory:
            plt.plot(point[0], point[1], 'bo', markersize = 2)
    else:
        for point in trajectory:
            plt.plot(point[0], point[1], 'ro', markersize = 2)

# Plot the state error in the form of 10%-transparent circles with a radius relative to the standard deviation
def plotstdTrajectory(trajectory, rlist):
    for i in range(len(trajectory)):
        plt.plot(trajectory[i][0], trajectory[i][1], 'go', markersize = 5 * rlist[i * 2], alpha = 0.1)
        plt.plot(trajectory[i][0], trajectory[i][1], 'go', markersize = 5 * rlist[i * 2 + 1], alpha = 0.1)

# Assumptions of the initial state covariance P0
# If the robot has perfect knowledge of the initial state
P0 = np.zeros((4,4))
# If no knowledge of the initial state
P0 = np.array(
        [[750/12, 0, 0, 0],
         [0, 500/12, 0, 0],
         [0, 0, math.pi**2/3, 0],
         [0, 0, 0, C/width/6]])

# Obtain real trajectory and estimated trajectory
traj, exp, obs = EKF(s0, P0, inputs, stateTimeEvolution, stateToSensor, w_cov, d_cov, theta_cov, omega_bias, omega_cov, Fmatrix, Wmatrix, Qmatrix, Hmatrix, Rmatrix)

# Evaluation of the performance of the EKF from the difference between real trajectory and the expected trajectory
def performance(traj, exp):
    n = len(traj)
    sum1 = 0    # Second-order norm of the error between traj and exp
    sum2 = 0    # Standard deviation between traj and exp
    for i in range(n):
        sum1 += math.sqrt((traj[i][0] - exp[i][0]) ** 2 + (traj[i][1] - exp[i][1]) ** 2)
        sum2 += (traj[i][0] - exp[i][0]) ** 2 + (traj[i][1] - exp[i][1]) ** 2
    errormean = sum1 / n
    errorstd = math.sqrt(sum2 / n)
    return errormean, errorstd

# print(performance(traj, exp))

# To visualize and compare the trajectories
plt.xlim([0, L])
plt.ylim([0, W])
plt.grid()
plotTrajectory(traj, True)
plotTrajectory(exp, False)
plt.plot(np.array(traj)[:, 0], np.array(traj)[:, 1], color = 'b', linewidth = 0.5)
plt.plot(np.array(exp)[:, 0], np.array(exp)[:, 1], 'r--', linewidth = 0.5)
plotstdTrajectory(exp, rP)
plt.show()


# Problem 3(d)
# How might you accommodate more realistic system models? For example, the gyro sensor doesn’t have zero-mean additive noise; rather, the mean of the gyro noise is a slowly varying non-zero bias value. The actuator uncertainty (process noise) typically depends on the commanded speed itself. Describe the updates to the system model in these and other cases, and how that would impact your state estimator.


# Problem 3(e)
# Qualitatively describe some conclusions about the effectiveness of your state estimator for potential tasks your robot may encounter, and tradeoffs regarding yours vs other possible state estimation algorithms.

