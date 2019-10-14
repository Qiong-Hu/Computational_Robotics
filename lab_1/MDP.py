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
# Create Action Space A = {a}, a = (motion, turn)
A = [];
for motion in [STILL, FORWARDS, BACKWARDS]:
    if motion != STILL:
        for turn in [NOT_TURN, TURN_LEFT, TURN_RIGHT]:
            A.append((motion, turn))
    else:
        A.append((STILL, NOT_TURN))

# Action Space Size NA
NA = len(A)

# Problem 1(c)
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

    # Possible heading directions with respective error listed in a dictionary
    H = { }
    # Actions: Left-L (-x), Right-R (+x), Up-U (+y), Down-D (-y). H[heading] = [(Action, heading, probability)] 
    # The probability of pre-rotating = "pe", and not pre-rotating = "1 - 2 * pe"
    prob = 1 - 2 * pe
    H[0] = [('U', 11, pe), ('U', 0, prob), ('U', 1, pe)]
    H[1] = [('U', 0, pe), ('U', 1, prob), ('R', 2, pe)]
    H[2] = [('U', 1, pe), ('R', 2, prob), ('R', 3, pe)]
    H[3] = [('R', 2, pe), ('R', 3, prob), ('R', 4, pe)]
    H[4] = [('R', 3, pe), ('R', 4, prob), ('D', 5, pe)]
    H[5] = [('R', 4, pe), ('D', 5, prob), ('D', 6, pe)]
    H[6] = [('D', 5, pe), ('D', 6, prob), ('D', 7, pe)]
    H[7] = [('D', 6, pe), ('D', 7, prob), ('L', 8, pe)]
    H[8] = [('D', 7, pe), ('L', 8, prob), ('L', 9, pe)]
    H[9] = [('L', 8, pe), ('L', 9, prob), ('L', 10, pe)]
    H[10] = [('L', 9, pe), ('L', 10, prob), ('U', 11, pe)]
    H[11] = [('L', 10, pe), ('U', 11, prob), ('U', 0, pe)]
    # can also use int((heading+1)/3)
    
    # A dictionary for moving directions
    Dir = {'U': [0, 1], 'D': [0, -1], 'L': [-1, 0], 'R': [1, 0]}

    # Staying still will result in no pre-rotate error. Future state = Current state. Otherwise, error occurs.
    x, y = s[0], s[1]
    prob = 0
    if a[0] == STILL:
        if s_ == s:
            prob = 1
    else:
        for state in H[s[2]]:
        # Attempting to move off of a grid will result in no linear movement, though rotation portion will still happen.
            x = s[0] + a[0] * Dir.get(state[0])[0]
            if (0 <= x <= L - 1):
                x_p = x
            else: 
                x_p = s[0]
        
            # Consider directions a[0]: FORWARDS (+1) and BACKWARDS (-1)
            y = s[1] + a[0] * Dir.get(state[0])[1] 
            if (0 <= y <= W - 1):
                y_p = y
            else:
                y_p = s[1]         
      
            h_p = (state[1] + a[1]) % 12
            s_p = (x_p, y_p, h_p)
            
            prob = state[2]

            # Check if the "s_" argument equals to the calculated future states "s_p".
            if s_ == s_p:
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