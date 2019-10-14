import numpy as np
import matplotlib.pyplot as plt

# Defining Actions
STILL = 0
FORWARDS = 1
BACKWARDS = -1
NOT_TURN = 0
TURN_LEFT = -1
TURN_RIGHT = 1

# Define grid world
L = 8
W = 8

# Problem 1(a)
# Create State Space S = {s}, s = (x, y, h). Coordinates (x, y) and heading h
S = []
for x in range(L):
    for y in range (W):
        for h in range(12):
            S.append((x, y, h))

# State Space Size NS
NS = len(S)

# Problem 1(b)
# Create Action Space A = {a}
A = [];
for action in [STILL, FORWARDS, BACKWARDS]:
    if action != STILL:
        for turn in [NOT_TURN, TURN_LEFT, TURN_RIGHT]:
            A.append((action, turn))
    else:
        A.append((STILL, NOT_TURN))

# Action Space Size NA
NA = len(A)

# Problem 1(c)


# Problem 3(a)
# Create and populate a matrix/array that stores the action a = pi0(s) prescribed by the initial policy pi0 when indexed by state s.

policy = {}
for s in S:
    # Get the vector from the state to the goal
    dir_vector = [5-s[0], 7-s[1]]
    
    # already reach goal
    if dir_vector == [0, 0]:
        policy[s] = (STILL, NOT_TURN)
    
    # Compute the move direction
    # heading +x
    if s[2] in [2, 3, 4]:
        action = FORWARDS if (dir_vector[0]>=0 or dir_vector[1]==0) else BACKWARDS
        
    # heading -x
    if s[2] in [8, 9, 10]:
        action = FORWARDS if (dir_vector[0]<=0 or dir_vector[1]==0) else BACKWARDS
        
    # heading +y
    if s[2] in [11, 0, 1]:
        action = FORWARDS if (dir_vector[1]>=0 or dir_vector[0]==0) else BACKWARDS
        
    # heading -y
    if s[2] in [5, 6, 7]:
        action = FORWARDS if (dir_vector[1]<=0 or dir_vector[0]==0) else BACKWARDS
    
    # Compute the turn direction
    # get the vetor angle theta
    theta = np.arctan2(dir_vector[1], dir_vector[0])*180/np.pi
    angle_diff = s[2]*30-(90 - theta)
    if (angle_diff > 0) and (angle_diff < 180):
        turn = TURN_LEFT
    elif (angle_diff == 0) or (angle_diff == 180):
        turn = NOT_TURN
    else:
        turn = TURN_RIGHT
        
    policy[s] = (action, turn)