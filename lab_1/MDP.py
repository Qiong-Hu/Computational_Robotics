import numpy as np
import matplotlib.pyplot as plt
import random

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
        return ValueError('Invalid error probability pe. pe should be between 0 and 0.5.')

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

        # Heading -y, which means h in [5,6,7]
        if s[2] in [5, 6, 7]:
            # If target is in front of robot, go backwards, else go forwards
            if dir_vector[1] <= 0:
                motion = FORWARDS
                dir_vector[1] += 1
            else:
                motion = BACKWARDS
                dir_vector[1] -= 1

        # Compute the turn direction
        # Get the vector angle theta
        theta = np.arctan2(dir_vector[1], dir_vector[0]) * 180 / np.pi
        angle_diff = s[2] * 30 - (90 - theta)
        angle_diff = angle_diff % 180
        threshold = 0

        # if target is in the left front of robot or right back, turn left
        # if they are in one line, not turn
        # else turn right
        if (angle_diff > threshold) and (angle_diff < 90):
            turn = TURN_LEFT
        elif angle_diff <= threshold or angle_diff == 90:
            turn = NOT_TURN
        else:
            turn = TURN_RIGHT

        policy[s] = (motion, turn)

    return policy


# Problem 3(b)
# Write a function to generate and plot a trajectory of a robot given policy matrix/array, initial state s0, and error probability pe.
def generate_trajectory(policy, s0, pe = 0, show = True):
    # policy: a directory of all policy
    # s0 = (x, y, h) : initial state
    # pe: the error probability pe to pre-rotate when choosing to move.
    # Return: Trajectory including passing states and actions on each state

    # Confirm the feasibility of pe
    if (pe < 0 or pe > 0.5):
        return ValueError('Invalid error probability pe. pe should be between 0 and 0.5.')

    # Generate the trajectory
    trajectory = []
    s_now = s0

    trajectory.append([s_now, policy[s_now]])

    # The robot keeps moving until it reaches target
    while (s_now[0] != GOAL[0] or s_now[1] != GOAL[1]):

        # Get probability of all possible next states
        P_states = next_state(s_now, policy[s_now], pe)

        states = list(P_states.keys())
        probs = list(P_states.values())
        # Choose next states according to probs
        if len(probs) == 1:
            s_next = states[0]
            s_now = s_next
        else:
            # Generate a random float between (0,1)
            x = random.random()
            if x <= probs[0]:
                s_next = states[0]
                s_now = s_next
            elif probs[0] < x <= (probs[0] + probs[1]):
                s_next = states[1]
                s_now = s_next
            else:
                s_next = states[2]
                s_now = s_next
        # Add state and action now in to trajectory
        trajectory.append([s_now, policy[s_now]])

    # Grid world initialization
    fig = plt.figure(figsize = (L - 2, W - 2))
    map = fig.add_subplot(1,1,1)
    plt.xlim((0, L))
    plt.ylim((0, W))
    x_locator = plt.MultipleLocator(1)
    y_locator = plt.MultipleLocator(1)
    map.xaxis.set_minor_locator(x_locator)
    map.yaxis.set_minor_locator(y_locator)
    plt.grid(color = 'k', which = 'minor')
    plt.tick_params(width = 0)
    plt.tick_params(which = 'minor', width = 0)
    x_labels = np.arange(0, L, 1)
    y_labels = np.arange(0, W, 1)
    plt.xticks(x_labels + 0.5, x_labels)
    plt.yticks(y_labels + 0.5, y_labels)

    # Place red markers
    edge1 = plt.Rectangle((0,0), 1, L, color = 'r')
    edge2 = plt.Rectangle((0,0), W, 1, color = 'r')
    edge3 = plt.Rectangle((L - 1,0), 1, L, color = 'r')
    edge4 = plt.Rectangle((0,W - 1), W, 1, color = 'r')
    map.add_patch(edge1)
    map.add_patch(edge2)
    map.add_patch(edge3)
    map.add_patch(edge4)

    # Place yellow markers
    yellow = plt.Rectangle((3, 4), 1, 3, color = 'yellow', alpha = 1)
    map.add_patch(yellow)

    # Place green goal
    goal = plt.Rectangle((5, 6), 1, 1, color = 'greenyellow', alpha = 1)
    map.add_patch(goal)

    # Plot the start state
    plt.plot(s0[0] + 0.5, s0[1] + 0.5, 'o', markersize = '10')
    map.arrow(s0[0] + 0.5, s0[1] + 0.5, 0.4 * np.sin(30 * s0[2] * np.pi/180),0.4 * np.cos(30 * s0[2] * np.pi/180), head_width = 0.1, head_length = 0.2, fc = 'k', ec = 'k')

    # Plot all passing states
    for i in range(0, len(trajectory) - 1):
        x1 = trajectory[i][0][0]
        y1 = trajectory[i][0][1]
        x2 = trajectory[i + 1][0][0]
        y2 = trajectory[i + 1][0][1]
        h = trajectory[i + 1][0][2]
        plt.plot([x1 + 0.5, x2 + 0.5], [y1 + 0.5, y2 + 0.5], 'k--')
        plt.plot(x2 + 0.5, y2 + 0.5, 'o', markersize = '10')
        map.arrow(x2 + 0.5, y2 + 0.5, 0.4*np.sin(30 * h * np.pi/180), 0.4 * np.cos(30 * h * np.pi/180),
                 head_width = 0.1, head_length = 0.2, fc = 'k', ec = 'k')

    # Plot the grid world
    if show:
        plt.show()

    return trajectory


# Problem 3(c)
# Generate and plot a trajectory of a robot using policy 0 starting in state x = 1; y = 6; h = 6 (i.e. # top left corner, pointing down). Assume pe = 0.
# generate_trajectory(policy_init(S), (1, 6, 6), 0)


# Problem 3(d)
# Write a function to compute the policy evaluation of a policy. That is, this function should return a matrix/array of values v = V (s) when indexed by state s. The inputs will be a matrix/array storing as above, along with discount factor.
def policy_eval(S, V, policy, reward, pe = 0, discount_factor = 1, threshold = 0.001):
    # State space S
    # Value function V: for iteration, empty as initialization
    # policy {S: A}: shaped dictionary representing the policy.
    # reward: reward function to be used for policy evaluation.
    # pe: the error probability pe to pre-rotate when chosing to move. 0 <= pe <= 0.5
    # discount_factor: Lambda discount factor.
    # threshold: We stop evaluation once our value function change is less than theta for all states.
    # Returns: Dictionary of shape {S: V} representing the value function.

    # Initialize V if not claimed
    if V == {}:
        for s in S:
            V[s] = 0

    while True:
        diff = 0
        # For each state, perform the evaluation
        for s in S:
            v = 0
            # Look at the possible next states:
            P_states = next_state(s, policy[s], pe)
            # Calculate the value:
            for state in P_states.keys():
                # Calculate the expected value
                v = v + P_states[state] * (reward(state) + discount_factor * V[state])
            # Calculate how much the value function changed:
            diff = max(diff, np.abs(v - V[s]))
            # Update the value function
            V[s] = v
        # Stop evaluating once the value function change is below a threshold
        if diff < threshold:
            break
    return V


# Problem 3(e)
# The value of the trajectory in 3(c). (lambda = 0.9)

### V = policy_eval(S, {}, policy_init(S), reward, 0, 0.9, 0.001)
s0 = (1, 6, 6)
### print("The value V(s0) is:", V[s0])

# Running result: threshold = 0.1, value = -2.1827644307687253, running time = 135.9s


# Problem 3(f)
# Write a function that returns a matrix/array giving the optimal policy given a one-step lookahead on value V .
def policy_one_step_lookahead(S, V, reward, pe = 0, discount_factor = 1):
    # V: evaluated value using the current policy
    # reward: reward function to be used for policy evaluation.
    # pe: the error probability pe to pre-rotate when chosing to move. 0 <= pe <= 0.5.
    # discount_factor: Lambda
    # Return policy as dict: {next state, optimal policy}
    
    policy = {}
    
    for s in S: 
        # Initialize all action value as 0.
        action = np.zeros(NA)
        for i in range(NA):
            # Check the possible next state for each action.
            P_states = next_state(s, A[i], pe)
            # Calculate value of each action.
            for state in P_states.keys():
                action[i] += P_states[state] * (reward(state) + discount_factor * V[state])

        # Get the best action 
        best_action_index = np.argmax(action)
        policy[s] = A[best_action_index]

    return policy


# Problem 3(g)
# Combine your functions above in a new function that computes policy iteration on the system, returning optimal policy pi* with optimal value V*.
# Policy Iteration converges when pi(i+1) == pi(i)
def policy_iteration(S, V, policy, reward, pe = 0, discount_factor = 1):
    # policy: policy to be iterated
    # reward: reward function to be used for policy evaluation.
    # discount_factor: lambda
    # Returns (policy, value).

    # Keep updating policy until optimal value is found.
    while True:
        # Calculate value of each state
        V = policy_eval(S, V, policy, reward, pe, discount_factor, 1)
        # Update policy by one step lookahead
        policy_new = policy_one_step_lookahead(S, V, reward, pe, discount_factor)
        # If policy doesn't change, the iteration is done
        if policy_new == policy:
            return policy, V
        else:
            policy = policy_new
        # For debug
        print('value = '+str(V[(1,6,6)]))


# Problem 3(h) and 3(i)
# Run this function to recompute and plot the trajectory and value of the robot described in 3(c) under the optimal policy pi*. 
# Keep track of the compute time.
import time
start = time.time()
# Optimal_policy calculation
optimal_policy, optimal_V = policy_iteration(S, {}, policy_init(S), reward, 0, 0.9)

# Recompute and plot the trajectory and value of the robot under optimal policy
s0 = (1, 6, 6)
trajectory = generate_trajectory(optimal_policy, s0, pe = 0, show = True)
end = time.time()
print("Trajectory from %s to the goal is: " % str(s0), trajectory)
print("Run Time is %s sec." % str(end - start))
# Running reult: value = 3.403192020275376, Trajectory from (1, 6, 6) to the goal is:  [[(1, 6, 6), (1, 0)], [(1, 5, 6), (1, -1)], [(1, 4, 5), (1, -1)], [(1, 3, 4), (1, 0)], [(2, 3, 4), (1, 0)], [(3, 3, 4), (1, 0)], [(4, 3, 4), (1, 1)], [(5, 3, 5), (-1, 0)], [(5, 4, 5), (-1, 0)], [(5, 5, 5), (-1, 0)], [(5, 6, 5), (0, 0)]], Run Time is 429.80976271629333 sec.


# Value Iteration
# Problem 4(a)
def value_iteration(S, policy, reward, pe = 0, discount_factor = 0.9, threshold = 0.01):
# Return optimal value, optimal value
# Initial Condition:
    V = {}
    for s in S:
        V[s] = 0

    diff = 10000
    while diff >= threshold:
        diff = 0
        for s in S: 
        # Initialize all action value as 0.
            action = np.zeros(NA)
            for i in range(NA):
            # Check the possible next state for each action.
                P_states = next_state(s, A[i], pe)
                states=list(P_states.keys())
                probs=list(P_states.values()) 
            # Calculate value of each action.
                for j in range(len(states)):
                    action[i] += probs[j] * (reward(states[j]) + discount_factor * V[states[j]])
            # Get best action index and update policy
            best_action_i = np.argmax(action)
            policy[s] = A[best_action_i]
            # Get the value of the best action
            value_best_action = np.max(action)
            # V(H+1) is the best_action value
            diff = max(diff, np.abs(value_best_action - V[s]))  
            # Update value function
            V[s] = value_best_action
    return policy, V 

# Problem 4(b) and 4(c)
# Compute and plot the trajectory, value of the robot described in 3(c) under the optimal policy 
# pi*. Compare with the results from policy iteration in 3(h).
# Runtime analysis.
import time
start = time.time()
# Optimal_value calculation
optimal_policy, optimal_V = value_iteration(S, policy_init(S), reward, pe = 0, discount_factor = 0.9)

#Recompute and plot the trajectory and value of the robot under optimal policy
trajectory = generate_trajectory(optimal_policy, s0 = (1, 6, 6), pe = 0, show = True)
end = time.time()
print("Trajectory from %s to the goal is: " % str(s0), trajectory)
print("Run Time is %s sec" % str(end - start))

