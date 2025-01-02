#!/usr/bin/env python3
# -*-coding:utf-8-*
import numpy as np
import casadi as ca     # CasADi solver for nonlinear optimization problems (used for solving MPC)
import time

def MPC(self_state, goal_state):
    opti = ca.Opti()  # Create an optimization problem using CasADi's Opti class
    
    ## Parameters for optimization
    T = 0.020  # Discretization time step for MPC, must match the `replan_period` in local_planner.py
    N = 50     # Prediction horizon for MPC, must match the `N` in local_planner.py
    v_max = 0.5  # Maximum linear velocity constraint for the robot
    omega_max = 3.0  # Maximum angular velocity constraint for the robot
    Q = np.array([[2.0, 0.0, 0.0],[0.0, 2.0, 0.0],[0.0, 0.0, 5.0]])  # Weight matrix for state errors
    R = np.array([[0.5, 0.0], [0.0, 0.4]])  # Weight matrix for control efforts (velocity and angular velocity)
    
    # Define the goal as the first three components of the goal state
    goal = goal_state[:, :3]
    
    # Define optimization parameters and variables
    opt_x0 = opti.parameter(3)  # Initial state of the robot (x, y, theta)
    opt_controls = opti.variable(N, 2)  # Control inputs: linear velocity (v) and angular velocity (omega)
    v = opt_controls[:, 0]  # Linear velocity
    omega = opt_controls[:, 1]  # Angular velocity

    ## State variables: robot's state over the prediction horizon
    opt_states = opti.variable(N+1, 3)  # State variables: x, y, theta for each time step
    x = opt_states[:, 0]
    y = opt_states[:, 1]
    theta = opt_states[:, 2]

    ## Create the system dynamics function (F(x, u)) using CasADi
    # The system model: x_dot = [v*cos(theta), v*sin(theta), omega]
    f = lambda x_, u_: ca.vertcat(*[u_[0] * ca.cos(x_[2]), u_[0] * ca.sin(x_[2]), u_[1]])

    ## Initial condition: the robot starts at the given initial state
    opti.subject_to(opt_states[0, :] == opt_x0.T)

    # Control input constraints: bounded by the maximum velocities
    opti.subject_to(opti.bounded(-v_max, v, v_max))  # Linear velocity constraints
    opti.subject_to(opti.bounded(-omega_max, omega, omega_max))  # Angular velocity constraints

    # System model constraints: dynamic model for each time step
    for i in range(N):
        x_next = opt_states[i, :] + T * f(opt_states[i, :], opt_controls[i, :]).T  # Compute next state using the model
        opti.subject_to(opt_states[i+1, :] == x_next)  # Enforce system dynamics

    #### Cost function: minimizes both state error and control efforts
    obj = 0  # Initialize the cost function
    for i in range(N):
        # The objective includes:
        # 1. State error cost: weighted difference between current state and goal state
        # 2. Control effort cost: weighted sum of control inputs (velocity and angular velocity)
        obj = obj + 0.8 * ca.mtimes([(opt_states[i, :] - goal[i]), Q, (opt_states[i, :] - goal[i]).T]) + 0.5 * ca.mtimes([opt_controls[i, :], R, opt_controls[i, :].T]) 
    
    # Add extra cost for the first state (initial condition)
    obj = obj + 2 * ca.mtimes([(opt_states[0, :] - goal[0]), Q, (opt_states[0, :] - goal[0]).T])

    # Minimize the cost function
    opti.minimize(obj)

    # Set options for the solver (Ipopt settings)
    opts_setting = {
        'ipopt.max_iter': 100,  # Maximum number of iterations
        'ipopt.print_level': 0,  # Disable printing iteration details
        'print_time': 0,  # Disable printing time taken
        'ipopt.acceptable_tol': 1e-4,  # Tolerance for convergence
        'ipopt.acceptable_obj_change_tol': 1e-4  # Tolerance for objective change
    }
    
    # Set the solver to IPOPT (interior point optimizer)
    opti.solver('ipopt', opts_setting)
    
    # Set the initial condition (robot's initial state)
    opti.set_value(opt_x0, self_state[:, :3])

    try:
        # Solve the optimization problem
        sol = opti.solve()  # Solve the optimization problem
        u_res = sol.value(opt_controls)  # Extract the optimal control inputs (velocity, angular velocity)
        state_res = sol.value(opt_states)  # Extract the optimal state trajectory (x, y, theta)
    except:
        # In case of a failure in solving, return the initial state and zero control inputs
        state_res = np.repeat(self_state[:3], N+1, axis=0)  # Repeat the initial state as the predicted states
        u_res = np.zeros([N, 2])  # Return zero control inputs (no movement)

    return state_res, u_res  # Return the optimal state trajectory and control inputs
