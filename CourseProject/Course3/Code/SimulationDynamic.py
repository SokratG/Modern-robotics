import numpy as np
import modern_robotics as mr
import math as math


def SimulationDynamic():
    """ Compute the joint angles with dynamic parameters and 
        save them into csv file 

    :param None
    :return None
    """
    M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
    M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0], [0, 0, 0, 1]]
    M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197], [0, 0, 1, 0.395], [0, 0, 0, 1]]
    M34 = [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.14225], [0, 0, 0, 1]]
    M45 = [[1, 0, 0, 0], [0, 1, 0, 0.093], [0, 0, 1, 0], [0, 0, 0, 1]]
    M56 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.09465], [0, 0, 0, 1]]
    M67 = [[1, 0, 0, 0], [0, 0, 1, 0.0823], [0, -1, 0, 0], [0, 0, 0, 1]]
    G1 = np.diag([0.010267495893, 0.010267495893,  0.00666, 3.7, 3.7, 3.7])
    G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393])
    G3 = np.diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275])
    G4 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
    G5 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
    G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879])
    Glist = [G1, G2, G3, G4, G5, G6]
    Mlist = [M01, M12, M23, M34, M45, M56, M67] 
    Slist = [[0,         0,         0,         0,        0,        0],
             [0,         1,         1,         1,        0,        1],
             [1,         0,         0,         0,       -1,        0],
             [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
             [0,         0,         0,         0,  0.81725,        0],
             [0,         0,     0.425,   0.81725,        0,  0.81725]]

    N = 1200 #iteration step
    dt = 0.003 
    g = np.array([0, 0, -9.81])
    sim_theta = np.zeros(6,)
    sim_theta_dot = np.zeros(6,)
    tau = np.zeros(6,)
    F = np.zeros(6,)

    sim_theta_pos = [[0, 0, 0, 0, 0, 0]]

    i = 0
    steps = (N // 2)
    
    while(i < steps):
        sim_theta_ddot = mr.ForwardDynamics(sim_theta, sim_theta_dot, tau, g, F, Mlist, Glist, Slist)
        sim_theta, sim_theta_dot = mr.EulerStep(sim_theta, sim_theta_dot, sim_theta_ddot, dt)
        sim_theta_pos.append(sim_theta)
        i = i + 1
    
    np.savetxt('simulation1.csv', sim_theta_pos, delimiter=',', fmt='%.6f')
    

    N = 1200 #iteration step
    dt = 0.005 
    sim_theta = np.array([0, 0, -1, 0, 0, 0])
    sim_theta_dot = np.zeros(6,)
    i = 0
    steps = (N // 2)
    sim_theta_pos = [[0, 0, 0, 0, 0, 0]]
    
    while(i < steps):
        sim_theta_ddot = mr.ForwardDynamics(sim_theta, sim_theta_dot, tau, g, F, Mlist, Glist, Slist)
        sim_theta, sim_theta_dot = mr.EulerStep(sim_theta, sim_theta_dot, sim_theta_ddot, dt)
        sim_theta_pos.append(sim_theta)
        i = i + 1
    
    np.savetxt('simulation2.csv', sim_theta_pos, delimiter=',', fmt='%.6f')
    
    return

