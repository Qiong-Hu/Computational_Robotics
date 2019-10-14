import numpy as np
# Problem 1(a), Problem 1(b)
# Defining Actions
STILL = 0
FORWARD = 1
BACKWARD = -1
NOT_TURN = 0
TURN_LEFT = -1
TURN_RIGHT = 1

# Create State Space S = {s}, s = (x, y, h). Coordinates (x, y) and heading h
S = []
for x in range(L):
    for y in range (W):
        for h in range(12):
            S.append((x, y, h))

# State Space Size Ns
Ns = len(S)

# Create Action Space A = {a}
A = [];
for action in [STILL, FORWARD, BACKWARD]:
    if action != STILL:
        for turn in [NOT_TURN, TURN_LEFT, TURN_RIGHT]:
            A.append((action, turn))
    else:
        A.append((STILL, NOT_TURN))

# Action Space Size Na
Na = len(A)

# Problem 3(a)
# Create and populate a matrix/array that stores the action a = pi0(s) prescribed by the initial policy pi0 when indexed by state s.

policy = {}
for s in S:
    # Get the vector from the state to the goal
    dir_vector = [3-s[0], 4-s[1]]
    
    # already reach goal
    if dir_vector == [0, 0]:
        policy[s] = (STILL, NO_TURN)
        
    # Compute the move direction
    # heading +x
    if s[2] in [2, 3, 4]:
        move = FWDS if (dir_vector[0]>=0 or dir_vector[1]==0) else BWDS
        
    # heading -x
    if s[2] in [8, 9, 10]:
        move = FWDS if (dir_vector[0]<=0 or dir_vector[1]==0) else BWDS
        
    # heading +y
    if s[2] in [11, 0, 1]:
        move = FWDS if (dir_vector[1]>=0 or dir_vector[0]==0) else BWDS
        
    # heading -y
    if s[2] in [5, 6, 7]:
        move = FWDS if (dir_vector[1]<=0 or dir_vector[0]==0) else BWDS
    
    # Compute the turn direction
    # get the vetor angle theta
    theta = np.arctan2(dir_vector[1], dir_vector[0])*180/np.pi
    angle_diff = s[2]*30-(90 - theta)
    if (angle_diff > 0) and (angle_diff < 180):
        turn = LEFT_TURN
    elif (angle_diff == 0) or (angle_diff == 180):
        turn = NO_TURN
    else:
        turn = RIGHT_TURN
        
    policy[s] = (move, turn)
