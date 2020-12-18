import numpy as np
import modern_robotics as mr
import math as math


def log_IKinBodyIterates(log_file, iter, thetalist, Tb, Vb, e_omega_b, e_v_b):
    """ logging data in IKinBodyIterates() function.
        :param log_file: file for logging data
        :param iter: current iteration
        :param Tb: end-effector configuration
        :param thetalist: joint angles
        :param V_b: error twist
        :param e_omega_b: current anuglar magnitude error
        :param e_v_b: current linear magnitude error
        :return: None
    """
    np.set_printoptions(precision=3, suppress = True)
    log_file.write('Iteration {}:\n'.format(iter))
    log_file.write('joint vector: {}\n'.format(thetalist))
    log_file.write('SE3 end-effector config: \n')
    log_file.write('{}\n'.format(Tb))
    log_file.write('error twist V_b: {}\n'.format(Vb))
    log_file.write('angular error magnitude ||omega_b||: {:0.3f}\n'.format(e_omega_b))
    log_file.write('linear error magnitude ||v_b||: {:0.3f}\n'.format(e_v_b))
    log_file.write('\n')
    np.set_printoptions(precision=8, suppress = False)

    return


def IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev):
    """Computes inverse kinematics in the body frame for an open chain robot
    :param Blist: The joint screw axes in the end-effector frame when the
                  manipulator is at the home position, in the format of a
                  matrix with axes as the columns
    :param M: The home configuration of the end-effector
    :param T: The desired end-effector configuration Tsd
    :param thetalist0: An initial guess of joint angles that are close to
                       satisfying Tsd
    :param eomg: A small positive tolerance on the end-effector orientation
                 error. The returned joint angles must give an end-effector
                 orientation error less than eomg
    :param ev: A small positive tolerance on the end-effector linear position
               error. The returned joint angles must give an end-effector
               position error less than ev
    :return thetalist: Joint angles that achieve T within the specified
                       tolerances,
    :return success: A logical value where TRUE means that the function found
                     a solution and FALSE means that it ran through the set
                     number of maximum iterations without finding a solution
                     within the tolerances eomg and ev.
    Uses an iterative Newton-Raphson root-finding method.
    The maximum number of iterations before the algorithm is terminated has
    been hardcoded in as a variable called maxiterations. It is set to 20 at
    the start of the function, but can be changed if needed.
    Example Input:
        Blist = np.array([[0, 0, -1, 2, 0,   0],
                          [0, 0,  0, 0, 1,   0],
                          [0, 0,  1, 0, 0, 0.1]]).T
        M = np.array([[-1, 0,  0, 0],
                      [ 0, 1,  0, 6],
                      [ 0, 0, -1, 2],
                      [ 0, 0,  0, 1]])
        T = np.array([[0, 1,  0,     -5],
                      [1, 0,  0,      4],
                      [0, 0, -1, 1.6858],
                      [0, 0,  0,      1]])
        thetalist0 = np.array([1.5, 2.5, 3])
        eomg = 0.01
        ev = 0.001
    Returns:
        (np.array([1.57073819, 2.999667, 3.14153913]), True)
    Sample call:
        IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev)
    """
    thetalist = np.array(thetalist0).copy()
    i = 0
    maxiterations = 20
    Tb = mr.FKinBody(M, Blist, thetalist)
    Vb = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(Tb), T)))
    e_omega_b = np.linalg.norm([Vb[0], Vb[1], Vb[2]])   
    e_v_b = np.linalg.norm([Vb[3], Vb[4], Vb[5]])
    err = e_omega_b > eomg or e_v_b > ev

    theta_log_cvs = [np.array(thetalist).copy()]
    log = open('log.txt', 'w') 
    log_IKinBodyIterates(log, 0, thetalist, Tb, Vb, e_omega_b, e_v_b)
    

    while err and i < maxiterations:
        thetalist = thetalist + np.dot(np.linalg.pinv(mr.JacobianBody(Blist, thetalist)), Vb)                                                       
        i = i + 1
        Tb = mr.FKinBody(M, Blist,thetalist)
        Vb  = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(Tb), T)))                                                     
        e_omega_b = np.linalg.norm([Vb[0], Vb[1], Vb[2]])   
        e_v_b = np.linalg.norm([Vb[3], Vb[4], Vb[5]])
        err = e_omega_b > eomg or e_v_b > ev
        theta_log_cvs.append(thetalist)
        log_IKinBodyIterates(log, i, thetalist, Tb, Vb, e_omega_b, e_v_b)
    
    theta_log_cvs = np.array(theta_log_cvs)
    log.close()
    np.savetxt('iterates.csv', theta_log_cvs, delimiter=',', fmt='%.6f')
    

    return (thetalist, not err)
   



def Week3Project():
    # Example
    pi = math.pi
    L1, L2 = 0.425, 0.392
    W1, W2 = 0.109, 0.082
    H1, H2 = 0.089, 0.095
    eomg, ev = 0.001, 0.0001
    M = np.array([[-1, 0, 0, L1+L2],
                  [0, 0, 1, W1+W2],
                  [0, 1, 0, H1-H2],
                  [0, 0, 0, 1]])
    Tsd = np.array([[0, 1, 0, -0.5],
                   [0, 0, -1, 0.1],
                   [-1, 0, 0, 0.1],
                   [0, 0, 0, 1]])
    Blist = np.array([[0, 1, 0, W1+W2, 0, L1+L2],
                      [0, 0, 1, H2, -L1-L2, 0],
                      [0, 0, 1, H2, -L2, 0],
                      [0, 0, 1, H2, 0, 0],
                      [0, -1, 0, -W2, 0, 0],
                      [0, 0, 1, 0, 0, 0]])
    theta_guess = np.array([pi, 0, pi/4, pi, pi/6, pi/3])
    
    #Theta = mr.IKinBody(Blist.transpose(), M, Tsd, theta_guess, eomg, ev)
    Theta = IKinBodyIterates(Blist.transpose(), M, Tsd, theta_guess, eomg, ev)

    print(Theta[0])
    return


Week3Project()