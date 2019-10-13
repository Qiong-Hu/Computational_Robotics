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
