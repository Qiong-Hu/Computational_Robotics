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

GOAL = (5, 6)

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
    # Next state s_p = (x_p, y_p, h_p)
    # Given state s_: need to compare with s_p to calculate probability
    # Action a = (motion, turn)
    # Pre-rotate error pe: If the robot moves, it will first rotate by +1 or -1 (mod 12) with probability "pe" before it moves. It will not pre-rotate with probability 1-2*pe. If motion is still, no error. 

    # Pre-rotate probability validity check
    if (pe < 0 or pe > 0.5):
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

# Problem 1(d)
# Uses the above function p_sa to return a next state s' given error probability pe, initial state s, and action a. Make sure the returned value s' follows the probability distribution specified by p_sa.
def next_state(s, a, pe):
    # Current state s = (x, y, h)
    # Action a = (move, turn)
    # Error probability pe: probability to pre-rotate when choosing to move. 0 <= pe <= 0.5
    # Return result p: a dictionary contains all possible next states with corresponding probability
    p={}
    for state in S:
        if p_sa(s, a, state, pe) > 0:
            p[state] = p_sa(s, a, state, pe)
    return p

# Problem 2(a)
# Write a function that returns the reward R(s) given input s.
def reward(s):
    # Current state s = (x, y, h)
    # Return: rewards of the current state

    # Border states (Red, marked X)
    if s[0] == 0 or s[0] == L - 1 or s[1] == 0 or s[1] == W - 1:
        return -100

    # Lane Markers (Yellow, marked --)
    elif s[0] == 3 and s[1] in [4, 5, 6]:
        return -10

    # Goal state (Green, marked *)
    elif s[0] == GOAL[0] and s[1] == GOAL[1]:
        return 1

    # Every other state has reward 0
    else:
        return 0

# Problem 3(a)
# Create and populate a matrix/array that stores the action a = pi0(s) prescribed by the initial policy pi0 when indexed by state s.

# Initial a policy
def policy_init(S):
    # State Space S = {s}
    # Return policy: a dictionary of all the policies

    policy = {}
    
    for s in S:
        # Get the vector from the state to the goal
        dir_vector = [GOAL[0] - s[0], GOAL[1] - s[1]]
        
        # Already reach goal
        if dir_vector == [0, 0]:
            policy[s] = (STILL, NOT_TURN)
            continue

        # Compute the moving direction 
        # Heading +x, which means h in [2, 3, 4]
        if s[2] in [2, 3, 4]:
            # If target is in the right of robot, go forwards, else go backwards 
            if dir_vector[0] >= 0:
                motion = FORWARDS
                dir_vector[0] -= 1 
            else:
                motion = BACKWARDS
                dir_vector[0] += 1

        # Heading -x, which means h in [8, 9, 10]
        if s[2] in [8, 9, 10]:
            # If target is in the left of robot, go forwards, else go backwards
            if dir_vector[0] <= 0:
                motion = FORWARDS
                dir_vector[0] += 1
            else:
                motion = BACKWARDS
                dir_vector[0] -= 1

        # Heading +y,which means h in [11, 0, 1]
        if s[2] in [11, 0, 1]:
            # If target is in front of robot, go forwards, else go backwards
            if dir_vector[1] >= 0:
                motion = FORWARDS
                dir_vector[1] -= 1
            else:
                motion = BACKWARDS
                dir_vector[1] += 1

        # Heading -y,which means h in [5,6,7]
        if s[2] in [5, 6, 7]:
            # If target is in front of robot, go backwards, else go forwards
            if dir_vector[1] <= 0:
                motion = FORWARDS
                dir_vector[1] += 1
            else:
                motion = BACKWARDS
                dir_vector[1] -= 1

        # Compute the turn direction
        # get the vector angle theta
        theta = np.arctan2(dir_vector[1], dir_vector[0]) * 180 / np.pi
        angle_diff = s[2] * 30 - (90 - theta)
        angle_diff=angle_diff % 180
        threshold = 0
        #if target is in the left front of robot or right back, turn left, if they are in one line, not turn, else turn right 
        if (angle_diff > threshold) and (angle_diff < 90):
            turn = TURN_LEFT
        elif angle_diff <= threshold or angle_diff == 90:
            turn = NOT_TURN
        else:
            turn = TURN_RIGHT
        
        policy[s] = (motion, turn)
    
    return policy


# Problem 3(b)
#Write a function to generate and plot a trajectory of a robot given policy matrix/array , initial state s0, and error probability pe.

def generate_trajectory(policy, s0, pe, show=True):
    """
    Generate a trajectory using policy from initial state s0 to the goal and
    visualize the trajectory on the grid world.
    
    Args:
        policy: a directory of all policy
        s0 = (x, y, h) : initial state
        pe: the error probability pe to pre-rotate when choosing to move.
    
    Return:
        Trajectory including passing states and actions on each state
    """
    # Confirm the feasibility of pe
    if (pe < 0 or pe >0.5):
        return ValueError('Invalid error probability Pe. Pe should be between 0 and 0.5.')
        
    # Generate the trajectory
    trajectory = []
    s_now = s0
    #the robot keep moving until it reaches target
    while (s_now[0] !=GOAL[0] or s_now[1] !=GOAL[1]):
        #add state and action now in to trajectory
        traj.append([s_now, policy[s_now]])
        #get probability of all possible next states
        P_state = next_state(s_now, policy[s_now], pe)
        #choose the max possible state to be next
        state_index = max(P_state.keys())
        s_next = P_state[state_index]
        s_now = s_next

