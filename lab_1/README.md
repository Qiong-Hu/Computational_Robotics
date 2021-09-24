# Project 1: Markov Decision Process control of robot in a 2D grid world

Main function: [MDP.py](MDP.py)

Instruction: [instruction.pdf](lab_1_instruction.pdf)

Full report: [report.pdf](lab_1_report.pdf)

Python package used in implementation: numpy, matplotlib

<br>

### Objectives

The goal of this lab is to explore Markov Decision Processes (MDPs) to control a simple discretized robot. You will develop and implement a model of the robot behavior and use it to accomplish a prescribed task.

### Instructions

1. MDP system
    - Create state space ![S](https://render.githubusercontent.com/render/math?math=S)
    - Create action space ![A](https://render.githubusercontent.com/render/math?math=A)
    - Write function of possible new states ![s'](https://render.githubusercontent.com/render/math?math=s') with probability ![p](https://render.githubusercontent.com/render/math?math=p)

2. Planning problem
    - Write reward function <img src="https://render.githubusercontent.com/render/math?math=R(s)"> given input ![s](https://render.githubusercontent.com/render/math?math=s)

3. Policy iteration
    - Create a matrix that stores the action <img src="https://render.githubusercontent.com/render/math?math=a=\pi_0(s)">
    - Write trajectory generation function of a robot given policy matrix ![\pi](https://render.githubusercontent.com/render/math?math=\pi), initial state ![s_0](https://render.githubusercontent.com/render/math?math=s_0), and error probability ![p_e](https://render.githubusercontent.com/render/math?math=p_e)
    - Write a function that returns a matrix ![\pi](https://render.githubusercontent.com/render/math?math=\pi) giving the optimal policy given a one-step lookahead on value ![V](https://render.githubusercontent.com/render/math?math=V)
    - Compute policy iteration on the system, returning optimal policy ![\pi\*](https://render.githubusercontent.com/render/math?math=\pi*) with optimal value ![V\*](https://render.githubusercontent.com/render/math?math=V*)

4. Value iteration
    - Write value iteration function to compute and plot the trajectory and value of the robot under the optimal policy ![\pi\*](https://render.githubusercontent.com/render/math?math=\pi*)

### Results from different scenarios

(Left column: policy iteration; right column: value iteration)

<img src="result/img-022723%20policy%20reward%20pe=0.jpg" width="40%"> <img src="result/img-024719%20value%20reward%20pe=0.jpg" width="40%">

<img src="https://render.githubusercontent.com/render/math?math=p_e=0">

<img src="result/img-032313%20policy%20reward%20pe=0.25.jpg" width="40%"> <img src="result/img-034420%20value%20reward%20pe=0.25.jpg" width="40%">

<img src="https://render.githubusercontent.com/render/math?math=p_e=0.25">

<br>

Full report see: [report.pdf](lab_1_report.pdf)

