# Trajectory tracking Set Theoretic Receding Horizon Control (T-ST-RHC)  for KheperaIV differential-drive robot. 
Code developed for the work published in "C. Tiriolo, W. Lucia - A Set-Theoretic Control Approach to the Trajectory Tracking Problem for Input-Output Linearized Wheeled Mobile Robots  - CDC - LCSS 2023"
Full paper at: https://doi.org/10.1109/LCSYS.2023.3286774
The code validates the strategy developed in the above paper by means of both simulations and experiments.


# Trajectory Tracking Problem Formulation 
Consider the input-constrained differential-drive robot and a bounded and smooth trajectory $q_r(k)=\left[x_r(k),y_r(k),\theta_r(k)\right]^T$
Design a trajectory tracking control law $[\omega_{R}(k),\omega_{L}(k)]^T=\phi(k,q(k),q_r(k))\in\mathcal{U}_d$ such that the tracking error $\tilde{q}(k)=q(k)-q_r(k)$ remains bounded $\forall k\geq 0$.

# Prerequisites 
The code was tested on Matlab 2020a environment. It requires the Ellipsoidal Toolbox ET  (https://www.mathworks.com/matlabcentral/fileexchange/21936-ellipsoidal-toolbox-et). 
To reproduce the experiments discussed in the paper, see **Khepera_IV_TCP** folder, a Khepera IV robot is required. The script implements a Bluetooth client that sends velocity commands to the robot in order to make it track a desired trajectory (eight-shaped and circular tractories are currently available). For further details refer to the paper.


# File Descriptions (Simulation)
- ST_FL_Trajectory_Tracking_Differential_Drive_CDC23.m: It's the main script that simulates the Trajectory tracking Set-Theoretic strategy.
- one_step_ellipsoidal_reachable_set.m: It's a Matlab function used to implement the offline phase of the algorithm. It computes the one-step reachable set starting from a given ellipsoidal set whose shaping matrix is given (see the full paper for further details). 
- online_phase_CDC.m: It's a Matlab function that implements the online phase optimization (see Eqs (25a)-(25b) in the full paper for further details).
- online_terminal_optim.m: It implements optimization (27) (see full paper for further details)
- DiffDrive.m: It's a Matlab function that implements the unicycle kinematics. It's used by the method ode45 to solve the differential equations describing the mobile robot's motion in the plane.

# File Descriptions (Experiment)
- Khepera_IV_TCP/Khepera_iv_ST_disturbance_traj_track.m: It's the main script used to perform the experiment described in the paper.  
- Khepera_IV_TCP/Khepera4.m: It represents the core of the application. It is a Matlab class that implements the main communication functionalities between the tracking controller running on Matlab and the server running on Khepera IV
- Khepera_IV_TCP/STTT_control_parameters.m: It's a Matlab class defining the parameters needed by the proposed tracking controller.
- Khepera_IV_TCP/"eight_traj_generation.m" implements the reference eight-shaped trajectory. 


# Demo (Simulation)
- Run "ST_FL_Trajectory_Tracking_Differential_Drive_CDC23.m"

# Demo (Experiment)
- Connect KheperaIV to the host machine through Bluetooth and set the right port on the script "Khepera_IV_TCP/Khepera_iv_ST_disturbance_traj_track.m".
- Run the Bluetooth server on the KheperaIV side and then, run the script  "Khepera_IV_TCP/Khepera_iv_ST_disturbance_traj_track.m".

# Videos
- https://www.youtube.com/watch?v=A0Tlbgr08tY&ab_channel=PreCySeGroup
