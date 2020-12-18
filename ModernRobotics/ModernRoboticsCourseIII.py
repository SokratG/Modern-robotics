import numpy as np
import modern_robotics as mr
import math as math

def Week1Quiz():
    # Question 1:
    dens = 5600
    cyl_d, cyl_l = 0.04, 0.2
    sph_d = 0.2
    pi = math.pi
    m_cyl = dens * pi * cyl_l * (cyl_d/2)**2 # mass cylinder
    m_sph = dens * 4/3 * pi * (sph_d/2)**3 # mass sphere

    c_Ixx = (m_cyl * (3*((cyl_d/2)**2) + cyl_l**2))/12
    c_Iyy = c_Ixx
    c_Izz = (m_cyl * ((cyl_d/2)**2))/2
    cyl_I = np.diag([c_Ixx, c_Iyy, c_Izz])
    

    s_Ixx = 2/5 * m_sph * (sph_d/2)**2
    s_Iyy = s_Ixx
    s_Izz = s_Ixx
    sph_I = np.diag([s_Ixx, s_Iyy, s_Izz])

    # Theorem 8.2(abuse Stiener's theorem)
    q_sph1 = np.array([[0], [0], [(sph_d/2) + cyl_l/2]]) 
    q_sph2 = np.array([[0], [0], [-(sph_d/2) - cyl_l/2]])
    sph1_I = sph_I + m_sph * (np.dot(q_sph1.transpose(),q_sph1)*np.identity(3) - np.dot(q_sph1, q_sph1.transpose()))
    sph2_I = sph_I + m_sph * (np.dot(q_sph2.transpose(),q_sph2)*np.identity(3) - np.dot(q_sph2, q_sph2.transpose()))

    I = cyl_I + sph1_I + sph2_I
    #print(np.array2string(I, separator=','))

    # Question 5:
    M01 = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]])
    M12 = np.array([[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0], [0, 0, 0, 1]])
    M23 = np.array([[1, 0, 0, 0], [0, 1, 0, -0.1197], [0, 0, 1, 0.395], [0, 0, 0, 1]])
    M34 = np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.14225], [0, 0, 0, 1]])
    M45 = np.array([[1, 0, 0, 0], [0, 1, 0, 0.093], [0, 0, 1, 0], [0, 0, 0, 1]])
    M56 = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.09465], [0, 0, 0, 1]])
    M67 = np.array([[1, 0, 0, 0], [0, 0, 1, 0.0823], [0, -1, 0, 0], [0, 0, 0, 1]])
    G1 = np.diag([0.010267495893, 0.010267495893,  0.00666, 3.7, 3.7, 3.7])
    G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393])
    G3 = np.diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275])
    G4 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
    G5 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
    G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879])
    Glist = np.array([G1, G2, G3, G4, G5, G6])
    Mlist = np.array([M01, M12, M23, M34, M45, M56, M67])
    Slist = np.array([[0,         0,         0,         0,        0,        0],
                      [0,         1,         1,         1,        0,        1],
                      [1,         0,         0,         0,       -1,        0],
                      [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
                      [0,         0,         0,         0,  0.81725,        0],
                      [0,         0,     0.425,   0.81725,        0,  0.81725]])
    theta = np.array([0, pi/6, pi/4, pi/3, pi/2, 2*pi/3])
    theta_dot = np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2])
    theta_ddot = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
    g = np.array([0, 0, -9.81])
    Ftip = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

    tau = mr.InverseDynamics(theta, theta_dot, theta_ddot, g, Ftip, Mlist, Glist, Slist)
    #tau = np.around(tau, 3)
    np.set_printoptions(precision=3, suppress = True)

    #print(np.array2string(tau, separator=','), sep='')

    return



def Week2Quiz():
    pi = math.pi
    np.set_printoptions(precision=2, suppress = True)
    # Question 1:
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

    Theta = np.array([0, pi/6, pi/4, pi/3, pi/2, 2*pi/3])
    Theta_dot = np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2])
    Theta_ddot = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
    g = np.array([0, 0, -9.81])
    F = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
    MassMatrix = mr.MassMatrix(Theta, Mlist, Glist, Slist)
    #print(np.array2string(MassMatrix, separator=','), sep='')

    # Question 2:
    VelQuadraticForces = mr.VelQuadraticForces(Theta, Theta_dot, Mlist,Glist, Slist)
    #print(np.array2string(VelQuadraticForces, separator=','), sep='')

    # Question 3:
    GravityForce = mr.GravityForces(Theta, g, Mlist, Glist, Slist)
    #print(np.array2string(GravityForce, separator=','), sep='')

    # Question 4:
    EndEffectorForces = mr.EndEffectorForces(Theta, F, Mlist, Glist, Slist)
    #print(np.array2string(EndEffectorForces, separator=','), sep='')

    # Question 5:
    tau = np.array([0.0128, -41.1477, -3.7809, 0.0323, 0.037, 0.1034])
    Acceleration = mr.ForwardDynamics(Theta, Theta_dot, tau, g, F, Mlist, Glist, Slist)
    #print(np.array2string(Acceleration, separator=','), sep='')


    return



def Week2Project():

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

def Week3Quiz():
    pi = math.pi

    # Question 1:
    T = 5
    t = 3
    qts = mr.QuinticTimeScaling(T, t)
    
    # Question 2:
    N = 10
    Xs = np.eye(4)
    Xe = np.array([[0, 0, 1, 1],
                   [1, 0, 0, 2],
                   [0, 1, 0, 3],
                   [0, 0, 0, 1]])
    T = 10
    St = mr.ScrewTrajectory(Xs, Xe, T, N, 3)
    print(np.array2string(St[8], separator=','), sep='')

    # Question 3:
    Ct = mr.CartesianTrajectory(Xs, Xe, T, N, 5)
    print(np.array2string(Ct[8], separator=','), sep='')


    return


