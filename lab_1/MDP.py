import numpy as np
import matplotlib.pyplot as plt

# Defining Actions
STILL = 0
FORWARDS = 1
BACKWARDS = -1
motion_list = [STILL, FORWARDS, BACKWARDS]

NOT_TURN = 0
TURN_LEFT = -1
TURN_RIGHT = 1
turn_list = [NOT_TURN, TURN_LEFT, TURN_RIGHT]

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
# Create Action Space A = {a}, a = (motion, turn)
A = [];
for motion in motion_list:
    if motion != STILL:
        for turn in turn_list:
            A.append((motion, turn))
    else:
        A.append((STILL, NOT_TURN))

# Action Space Size NA
NA = len(A)

# Problem 1(c)
# Transform 12 headings to the nearest cardinal direction
def direction(heading):
    dir4 = int((heading + 1) / 3) % 4   # Transform to 4 directions
    # dir4: 0 -> +y (0, 1), 1 -> +x (1, 0), 2 -> -y (0, -1), 3 -> -x (-1, 0)
    x = dir4 % 2 if not dir4 % 2 else 2 - dir4
    y = (dir4 + 1) % 2 if not (dir4 + 1) % 2 else 1 - dir4
    return [x, y]

# A list of new headings with probability after pre-rotating
def err_prob(heading, pe):
    err_headings = []
    for turn in turn_list:
        # The probability of pre-rotating = "pe", and not pre-rotating = "1 - 2 * pe"
        if turn == STILL:
            err_headings.append((heading, 1 - 2 * pe))
        else:
            err_headings.append(((heading + turn) % 12, pe))
    return err_headings

# Transition Probabilities p_sa
# Probability of state s to new state s', with action a, error probability pe
def p_sa(s, a, s_, pe):
    # Current state s = (x, y, h) 
    # Future state s_p = (x_p, y_p, h_p)
    # Action a = (motion, turn)
    # Pre-rotate error pe: If the robot moves, it will first rotate by +1 or -1 (mod 12) with probability "pe" before it moves. It will not pre-rotate with probability 1-2*pe. If motion is still, no error. 

    # Pre-rotate probability validity check
    if (pe < 0 and pe > 0.5):
        return ValueError('Invalid error probability Pe. Pe should be between 0 and 0.5.')

    x, y = s[0], s[1]
    prob = 0
    # Staying still will result in no pre-rotate error. Future state = Current state.
    if a[0] == STILL:
        if s_ == s:
            prob = 1
    # Otherwise, error occurs.
    else:
        for state in err_prob(s[2], pe):
            # Attempting to move off of a grid will result in no linear movement, though rotation portion will still happen.
            x = s[0] + a[0] * direction(state[0])[0]
            if (0 <= x <= L - 1):
                x_p = x
            else: 
                x_p = s[0]

            # Consider directions a[0]: FORWARDS (+1) and BACKWARDS (-1)
            y = s[1] + a[0] * direction(state[0])[1]
            if (0 <= y <= W - 1):
                y_p = y
            else:
                y_p = s[1]
      
            h_p = (state[0] + a[1]) % 12
            s_p = (x_p, y_p, h_p)

            # Check if the "s_" argument is equal to the calculated future states "s_p".
            if s_ == s_p:
                prob = state[1]
                return prob
    return prob

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