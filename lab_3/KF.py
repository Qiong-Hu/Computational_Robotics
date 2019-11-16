import numpy as np
import matplotlib.pyplot as plt
import math
import random
import time


# Problem 2(a) 
# Define the complete system model. You will need to derive and present the mathematical equations describing this robot plant, including the appropriate sensor and actuator models and system dynamics. Note that you will need to amend the state of your robot from the previous lab in order to accommodate this specific collection of sensors. Be sure to clearly dene and describe all variables and equations, and produce illustrative diagrams as necessary.

# Define a rectangular environment of length L = 750mm and width W = 500mm
L = 750
W = 500

# walls: x=0, x=L, y=0, y=W

# inputs: (w1, w2), the angular velocity of wheel 1 and wheel 2
# sensor outputs: (d1, d2, θ, w)
# d1: the distance to a wall in a straight line in front of the robot 
# d2: the distance to a wall in a straight line to the right of the robot
# θ: an absolute bearing indicating the angle of the robot with respect to magnetic north 
# w: the rotational speed

# robot state: (x, y, θ)
# time update:

# observation update:




