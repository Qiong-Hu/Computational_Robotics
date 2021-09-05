Main function: [MDP.py](MDP.py)

Instruction: [instruction.pdf](lab_1_instruction.pdf)

Full report: [report.pdf](lab_1_report.pdf)

<br>

### Objectives

The goal of this lab is to explore Markov Decision Processes (MDPs) to control a simple discretized robot. You will develop and implement a model of the robot behavior and use it to accomplish a prescribed task.

### Instructions

1. MDP system
    - Create state space $S$
    - Create action space $A$
    - Write function of possible new states $s'$ with probability $p$

2. Planning problem
    - Write reward function $R(s)$ given input $s$

3. Policy iteration
    - Create a matrix that stores the action $a=\pi_0(s)$
    - Write trajectory generation function of a robot given policy matrix $\pi$, initial state $s_0$, and error probability $p_e$
    - Write a function that returns a matrix $\pi$ giving the optimal policy given a one-step lookahead on value $V$
    - Compute policy iteration on the system, returning optimal policy $\pi^*$ with optimal value $V^*$

4. Value iteration
    - Write value iteration function to compute and plot the trajectory and value of the robot under the optimal policy $\pi^*$

### Results from different scenarios

<img src="result/img-022723%20policy%20reward%20pe=0.jpg">
Policy iteration with $p_e=0$

<img src="result/img-024719%20value%20reward%20pe=0.jpg">
Value iteration with $p_e=0$

<img src="result/img-032313%20policy%20reward%20pe=0.25.jpg">
Policy iteration with $p_e=0.25$

<img src="result/img-034420%20value%20reward%20pe=0.25.jpg">
Value iteration with $p_e=0.25$

<br>

Full report: [report.pdf](lab_1_report.pdf)
