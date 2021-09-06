# Project 3: Kalman Filter (KF) for robot state estimation

Main function: [EKF.py](EKF.py)

Instruction: [instruction.pdf](lab_3_instruction.pdf)

Full report: [report.pdf](lab_3_report.pdf)

<br>

### Objectives

The goal of this lab is to explore Kalman Filtering (KF) to estimate the state of a simple two-wheeled non-holonomic robot with realistic hardware. You will develop and implement a mathematical model of the robot sensor and actuator behavior and use it to evaluate a state estimation algorithm.

### Instructions

1. Robot model
    - Set up system state: distinguish between ideal state dynamics and real state dynamics
    - Set up sensing model as observation input

2. Mathematical setup
    - Define environment, motion inputs, sensor outputs, robot state space ![S](https://render.githubusercontent.com/render/math?math=S)
    - Define system dynamics mathematically
    - Define process noise <img src="https://render.githubusercontent.com/render/math?math=w_{1e},%20w_{2e}">, measurement noise <img src="https://render.githubusercontent.com/render/math?math=d_{1e},%20d_{2e},%20\theta_e,%20\omega_e">
    - Create a Kalman Filter (KF) based state estimator to take the motor commands and sensor measurements and generate a state estimate. Compare Extended Kalman Filter (EKF) and Unscented Kalman Filter (UKF).
    - Define linear approximation equations:
    
    <img src="https://render.githubusercontent.com/render/math?math=s_{t%2b1}%20=%20f(s_t,%20u_t,%20w_t)%20\approx%20F_ts_t%2bW_tw_t">,
    
    <img src="https://render.githubusercontent.com/render/math?math=y_t%20=%20h(s_t,%20v_t)%20\approx%20H_ts_t%2bV_tv_t">
    
    - Implement EKF time update:
    
    <img src="https://render.githubusercontent.com/render/math?math=s_{t%2b1}^{-}%20=%20f(s_t,%20u_t,%200)">,
    
    <img src="https://render.githubusercontent.com/render/math?math=P_{t%2b1}^{-}%20=%20F_tP_tF_t^T%20%2b%20W_tQ_tW_t^T">
    
    - Implement EKF measurement update:
    
    <img src="https://render.githubusercontent.com/render/math?math=K_t%20=%20H_tP_{t%2b1}^{-}H_t^T%20%2b%20R_t">,
    
    <img src="https://render.githubusercontent.com/render/math?math=s_{t%2b1}%20=%20s_{t%2b1}^{-}%20+%20P_{t%2b1}^{-}H_t^TK_t^{-1}(y_t-h(s_{t%2b1}^{-},%200))">,
    
    <img src="https://render.githubusercontent.com/render/math?math=P_{t%2b1}%20=%20P_{t%2b1}^{-}%20-%20P_{t%2b1}^{-}H_t^TK_t^{-1}H_tP_{t%2b1}^{-}">

### Evaluation

<img src="img/original_traj.png" width="50%">
Eight reference trajectories for evaluation and visualization

To characterize the performance of the state estimator, we use mean error and std error between both real trajectory and estimated trajectory.

| trajectory index | with/without knowledge | mean error | std error |
|:----------------:|:----------------------:|:----------:|:--------------:|
|1|perfect knowledge|1.60|1.89|
|1|no knowledge|2.5|3.04|
|2|perfect knowledge|1.1|1.3|
|2|no knowledge|4.48|5.0|
|3|perfect knowledge|1.2|1.3|
|3|no knowledge|3.6|3.8|
|4|perfect knowledge|2.2|2.3|
|4|no knowledge|3.6|4.27|
|5|perfect knowledge|0.8|1.0|
|5|no knowledge|2.7|3.0|
|6|perfect knowledge|1.0|1.3|
|6|no knowledge|5.5|8.3|
|7|perfect knowledge|1.4|1.5|
|7|no knowledge|2.6|3.6|
|8|perfect knowledge|1.6|2.1|
|8|no knowledge|2.4|2.8|

### Visualization

(Left column: perfect knowledge of the initial state; right column: no knowledge of the initial state)

<img src="img/[100,100,0,0],all.png" width="40%"> <img src="img/[100,100,0,0],no.png" width="40%">

Trajectory No.1: Straight forward line in the direction of +x.

<img src="img/[100,100,0.5pi,0],all.png" width="40%"> <img src="img/[100,100,0.5pi,0],no.png" width="40%">

Trajectory No.2: Straight line trajectory heading in the direction of +y.

<img src="img/[200,200,0,0],all.png" width="40%"> <img src="img/[200,200,0,0],no.png" width="40%">

Trajectory No.3: Clockwise circular trajectory.

<img src="img/[200,200,0,0],1,all.png" width="40%"> <img src="img/[200,200,0,0],1,no.png" width="40%">

Trajectory No.4: Counter-clockwise circular trajectory.

<img src="img/[400,400,pi,0],all.png" width="40%"> <img src="img/[400,400,pi,0],no.png" width="40%">

Trajectory No.5: Straight line heading in -x.

<img src="img/[400,400,1.5pi,0],all.png" width="40%"> <img src="img/[400,400,1.5pi,0],no.png" width="40%">

Trajectory No.6: Straight line heading in -y.

<img src="img/[100,100,0.25pi,0],all.png" width="40%"> <img src="img/[100,100,0.25pi,0],no.png" width="40%">

Trajectory No.7: Straight line heading in the direction 45° from +x.

<img src="img/[300,300,0,0],all.png" width="40%"> <img src="img/[300,300,0,0],no.png" width="40%">

Trajectory No.8: A complex trajectory.

<br>

Full report see: [report.pdf](lab_3_report.pdf)
