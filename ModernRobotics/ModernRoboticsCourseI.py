import numpy as np
import modern_robotics as mr


def coordinate_to_csv():
    y = np.random.rand(3, 4) # matrix 3x4 of floats
    d = np.random.randint(-100, 100, 3) # vector 3x1 of ints
    f = open("output.csv", 'w') # overwrite file with new coordinates
    for i in range(len(d)):
        output = " %10.6f, %10.6f, %10.6f, %10.6f, %d\n" % (y[i,0], y[i,1], y[i,2], y[i,3], d[i])
        f.write(output)
    f.close()


def Week3Quiz():
    I = np.eye(3)
    Rs = np.array([[1, 0, 0], 
                   [0, 1, 0], 
                   [0, 0, 1]])
    Ra = np.array([[0, 1, 0], 
                   [0, 0, 1], 
                   [1, 0, 0]])
    Rb = np.array([[1, 0, 0], 
                   [0, 0, 1], 
                   [0, -1, 0]])
    pb = np.array([1, 2, 3])
    ws = np.array([3, 2, 1])
    wt = np.array([1, 2, 0])
    wtskew = mr.VecToso3(wt)
    wskew = np.array([1, 2, 0.5])
    wskewTheta = mr.VecToso3(np.array([-2, -1, -0.5]))
    RLog = np.array([[0, 0, 1], 
                     [-1, 0, 0], 
                     [0, -1, 0]])
    Rsa = np.dot(Rs, Ra)
    Rsb = np.dot(Rs, Rb)
    Rbs = np.transpose(Rsb)
    Ras = np.transpose(Rsa)
    Rab = np.dot(Ras, Rsb)
    ps = np.dot(Rsb, pb)
    wa = np.dot(Ras, ws)
    RexpWt = mr.MatrixExp3(wtskew)
    Rskew = mr.VecToso3(wskew)
    RMatExp = mr.MatrixExp3(wskewTheta)
    wthetaso3 = mr.MatrixLog3(RLog)
    #print(np.array2string(RexpWt, separator=','))
    return

def Week4Quiz():
    Ts = np.array([[1, 0, 0, 0], 
                   [0, 1, 0, 0],
                   [0, 0, 1, 0],
                   [0, 0, 0, 1]]) #1 question
    Ta = np.array([[0, -1, 0, 0], 
                   [0, 0, -1, 0],
                   [1, 0, 0, 1],
                   [0, 0, 0, 1]]) #1 question
    Tb = np.array([[1, 0, 0, 0], 
                   [0, 0, 1, 2],
                   [0, -1, 0, 0],
                   [0, 0, 0, 1]]) #1 question
    pb = np.array([1, 2, 3, 1]) #5 question, for homogeneous add to end of list '1'
    Vs = np.array([3, 2, 1, -1, -2, -3])  #7 question
    Stheta = np.array([0, 1, 2, 3, 0, 0]) #9 question
    Fb =  np.array([1, 0, 0, 2, 1, 0]) #10 question
    T11 = np.array([[0, -1, 0, 3], 
                    [1, 0, 0, 0],
                    [0, 0, 1, 1],
                    [0, 0, 0, 1]]) #11 question
    V12 = np.array([1, 0, 0, 0, 2, 3]) #12 question
    s13 = np.array([1, 0, 0]) #13 question
    p13 = np.array([0, 0, 2]) #13 question
    S_theta = np.array([[0, -1.5708, 0, 2.3562], 
                        [1.5708, 0, 0, -2.3562],
                        [0, 0, 0, 1],
                        [0, 0, 0, 0]])  #14 question
    T = np.array([[0, -1, 0, 3], 
                       [1, 0, 0, 0],
                       [0, 0, 1, 1],
                       [0, 0, 0, 1]]) #15 question
    Tsa = np.dot(Ts, Ta) #1 question
    Tsb = np.dot(Ts, Tb) #2 question
    Tbs = mr.TransInv(Tsb) #2 question
    Tas = mr.TransInv(Tsa) #3 question
    Tab = np.dot(Tas, Tsb) #3 question
    ps = np.dot(Tsb, pb) #5 question
    Va = np.dot(mr.Adjoint(Tas), Vs) #7 question
    Theta = mr.AxisAng6(mr.se3ToVec(mr.MatrixLog6(Tsa))) #8 question
    TMatExp = mr.MatrixExp6(mr.VecTose3(Stheta)) #9 question
    Fs = np.dot(np.transpose(mr.Adjoint(Tbs)), Fb) #10 question
    T11_TrInv = mr.TransInv(T11) #11 question
    V12_se3 = mr.VecTose3(V12) #12 question
    S13 = mr.ScrewToAxis(p13, s13, 1) #13 question, h=1
    MatExp14 = mr.MatrixExp6(S_theta) #14 question
    St_15 = mr.MatrixLog6(T) #15 question


    #print(np.array2string(Fs, separator=','))
    return




