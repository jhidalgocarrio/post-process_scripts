# --------------------------------------------------------------------
# Orientation estimation using a quaternion-based Kalman filter 
#   August 4, 2009
#   Young Soo Suh (yssuh@ulsan.ac.kr)
#   Department of Electrical Engineering, University of Ulsan
#   Namgu, Ulsan 680-749, Korea
#   http://infolab.ulsan.ac.kr/
#
# Input
#   yg, ya, ym : gyroscope, accelerometer, magnetic sensor 
#   (dimension should be 3 x N for each variable)
#   tt : time data (needs not be constant sampling time)
#   Ra, Rg, Rm : 3 x 3 sensor noise error covariance
#
# Output 
#   q4 : quaternion (dimension 4 x N)
#   eulercom4 : euler angles (dimension 3 x N)  (radian)
# --------------------------------------------------------------------
from pylab import *
import numpy as np

def vec2product(v):
    M = np.array([[ 0 , -v[2] , v[1]],
                  [v[2]  , 0 , -v[0]],
                  [-v[1] , v[0] , 0 ]])
    return M

def quaternion2dcm(q):
    q0 = q[0];
    q1 = q[1];
    q2 = q[2];
    q3 = q[3];

    M = np.array(   [[2 * q0 * q0 + 2 * q1 * q1 - 1 , 2 * q1 * q2 + 2 * q0 * q3 , 2 * q1 * q3 - 2 * q0 * q2],
                    [2 * q1 * q2 - 2 * q0 * q3 , 2 * q0 * q0 + 2 * q2 * q2 - 1 , 2 * q2 * q3 + 2 * q0 * q1],
                    [2 * q1 * q3 + 2 * q0 * q2 , 2 * q2 * q3 - 2 * q0 * q1 , 2 * q0 * q0 + 2 * q3 * q3 - 1]])

    return M

def dcm2quaternion (C):
    """ Quaternion representation is [w, x i, y j, z k]. """

    f = np.zeros(4, dtype=double)

    f[0] = 0.25 * (1 + np.trace(C))
    f[1] = 0.25 * (C[0,0] - C[1,1] - C[2,2] + 1)
    f[2] = 0.25 * (C[1,1] - C[0,0] - C[2,2] + 1)
    f[3] = 0.25 * (C[2,2] - C[1,1] - C[0,0] + 1)

    maxf = np.max(f)
    index = np.argmax(f)

    q = np.zeros(4, dtype=double)

    if index == 0:
        q[0] = sqrt(f[0])
        q[1] = (C[1,2] - C[2,1]) / (4*q[0])
        q[2] = (C[0,2] - C[2,0]) / (-4*q[0])
        q[3] = (C[0,1] - C[1,0]) / (4*q[0])
    elif index == 1:
        q[1] = sqrt(f[1])
        q[0] = (C[1,2] - C[2,1]) / (4*q[1])
        q[2] = (C[0,1] + C[1,0]) / (4*q[1])
        q[3] = (C[0,2] + C[2,0]) / (4*q[1])
    elif index == 2:
        q[2] = sqrt(f[2])
        q[0] = (C[0,2] - C[2,0]) / (-4*q[2])
        q[1] = (C[0,1] + C[1,0]) / (4*q[2])
        q[3] = (C[1,2] + C[2,1]) / (4*q[2])
    elif index == 3:
        q[3] = sqrt(f[3])
        q[0] = (C[0,1] - C[1,0]) / (4*q[3])
        q[1] = (C[0,2] + C[2,0]) / (4*q[3])
        q[2] = (C[1,2] + C[2,1]) / (4*q[3])

    return q

def quater_ikf(ya=None, yg=None, ym=None, yi=None, tt=None, Ra=None, Rg=None, Rm=None, Ri=None,
                Qba=None, Qbg=None, Qbi=None, dip_angle=None,
                acc_m1 = None, acc_m2 = None, acc_gamma = None,
                inc_m1 = None, inc_m2 = None, inc_gamma = None):

    D2R = pi/180.0

    # number of data: N
    N = ya.shape[1]

    # gravitation acceleration
    g = 9.81
    gtilde = np.array([0, 0, g])

    # dip angle depending at location
    if dip_angle is not None:
        alpha = dip_angle * D2R
        mtilde = np.array([cos(alpha), 0, -sin(alpha)])

    # --------------------------------------------------------
    # Kalman Filter
    # --------------------------------------------------------

    # q4 : quaternion
    q4 = np.zeros((4, N), dtype=double)

    # eulercom4 : euler angles
    eulercom4 = np.zeros((3, N),  dtype=double)

    Qa = np.zeros((N,3,3), dtype=double)
    Qi = np.zeros((N,3,3), dtype=double)

    # estimated bias for gyroscope (bghat) and accelerometer (bahat)
    bghat = np.zeros(3, dtype=double)
    bahat = np.zeros(3, dtype=double)
    bihat = np.zeros(3, dtype=double)

    # Initial orientation estimation using the TRIAD method
    yabar = yi[:,0] / norm(yi[:,0])
    ymbar = ym[:,0] / norm(ym[:,0])

    foo1 = np.cross(yabar,ymbar) / norm(np.cross(yabar,ymbar))
    C = np.array([-np.cross(yabar,foo1), foo1, yabar])
    q4[:,0] = dcm2quaternion(C);

    # Kalman filter state, error covariance, process noise covariance
    x = np.zeros(12, dtype=double)
    P = np.zeros((12, 12), dtype=double)
    Q = np.zeros((12, 12), dtype=double)

    if Rg is not None:
        Q[0:3, 0:3] = Rg.dot(0.25)
    else:
        raise ValueError("Rg cannot be None")

    if Qbg is not None:
        Q[3:6, 3:6] = Qbg
    else:
        raise ValueError("Qbg cannot be None")

    if Qba is not None:
        Q[6:9, 6:9] = Qba
    else:
        raise ValueError("Qba cannot be None")

    if Qbi is not None:
        Q[9:12, 9:12] = Qbi

    # Initial error covariance
    P = P0

    # A matrix
    A = np.zeros((9,9), dtype=double)
    A[0:3, 3:6] = np.eye(3, dtype=double).dot(-0.5)

    # Intialization for quaternion integration
    wx = yg[0,0]
    wy = yg[1,0]
    wz = yg[2,0]
    oldomega4 = np.array([[ 0 , -wx , -wy , -wz],
                      [wx , 0 , wz , -wy],
                      [wy , -wz , 0 , wx],
                      [wz , wy , -wx , 0]])

    # variable used in the adaptive algorithm
    r2count = 100

    # H1 and H2 for measurement update
    H1 = np.zeros((3,12), dtype=double)
    H1[0:3, 6:9] = np.eye(3, dtype=double)
    H2 =  np.zeros((3,12), dtype=double)
    H3 = np.zeros((3,12), dtype=double)
    H3[0:3, 9:12] = np.eye(3, dtype=double)


    R1 = np.zeros((3*N,3), dtype=double)
    R3 = np.zeros((3*N,3), dtype=double)

    # Kalman filter loop
    for i in range (1,N):
        T = tt[i] - tt[i-1]

        Cq = quaternion2dcm(q4[:,i-1]);
        A[0:3,0:3] = -vec2product(yg[:,i-1] - bghat);
        dA = np.eye(9, dtype=double) + A.dot(T) + A.dot(A).dot(pow(T,2)) / 2.0;

        x = dA.dot(x)
        Qd = Q.dot(T) + 0.5*T*T*A.dot(Q) + 0.5*T*T*Q.dot(A.transpose())
        Qd = 0.5*(Qd + Qd.transpose())
        P = dA.dot(P).dot(dA.transpose()) + Qd;

        yyg = yg[:,i-1] - bghat;
        wx = yyg[0];
        wy = yyg[1];
        wz = yyg[2];
        omega4 = np.array([[ 0 , -wx , -wy , -wz],
                      [wx , 0 , wz , -wy],
                      [wy , -wz , 0 , wx],
                      [wz , wy , -wx , 0]])

        q4[:,i] = (np.eye(4, dtype=double) + 0.75 * omega4.dot(T)
                - 0.25 * oldomega4.dot(T)
                - (1.0/6.0) * pow(norm(yyg), 2) * pow(T,2) * np.eye(4, dtype=double)
                - (1.0/24.0) * omega4.dot(oldomega4) * pow(T,2)
                - (1.0/48.0) * pow(norm(yyg), 2) * omega4 * pow(T,3)).dot(q4[:,i-1])

        q4[:,i] = q4[:,i] / norm(q4[:,i])
        oldomega4 = omega4

        Cq = quaternion2dcm(q4[:,i])

        # ----------------------------------------------------
        # Three step measurement update
        # ----------------------------------------------------
        if ya is not None:

            H1[0:3, 0:3] = 2 * vec2product(Cq.dot(gtilde))
            z1 = ya[:,i] - bahat - Cq*gtilde

            # adaptive algorithm
            fooR1 = (z1 - H1.dot(x)).dot((z1 - H1.dot(x)).transpose())
            R1[3*(i-1):3*i,:] = fooR1
            uk = fooR1
            for j in range(i-1, min([i-(acc_m1-1),1])):
                uk = uk + R1[3*(j-1):3*j,:]

            uk = uk / acc_m1

            fooR2 = H1.dot(P).dot(H1.transpose()) + Ra

            u,s,v = np.linalg.svd(uk, full_matrices=True)
            u1 = u[:,0]
            u2 = u[:,1]
            u3 = u[:,2]

            lamb = np.array([ s(0) , s(1) , s(2)])
            mu =  np.array([ u1.transpose().dot(fooR2).dot(u1) , u2.transpose().dot(fooR2).dot(u2) , u3.transpose().dot(fooR2).dot(u3)])

            if (max(lamb - mu) > acc_gamma):
              r2count = 0
              Qstar = max(lamb(0)-mu(0),0).dot(u1).dot(u1.transpose()) + max(lamb(1) -mu(1),0).dot(u2).dot(u2.transpose()) + max(lamb(2)-mu(2),0).dot(u3).dot(u3.transpose());
            else:
              r2count = r2count + 1
              if r2count < acc_m2:
                Qstar = max(lamb(0)-mu(0),0).dot(u1).dot(u1.transpose()) + max(lamb(1) -mu(1),0).dot(u2).dot(u2.transpose()) + max(lamb(2)-mu(2),0).dot(u3).dot(u3.transpose());
              else:
                Qstar = np.zeros((3,3),dtype=double)

            #Qstar = zeros(3,3);

            Qa[:,:,i] = Qstar

            Qstar

            P1 = P
            K1 = P1.dot(H1.transpose()).dot(np.linalg.inv(H1 * P1 * H1.transpose()) + Ra + Qstar)
            x = x + K1.dot(z1 - H1.dot(x))
            P = (np.eye(9, dtype=double) - K1.dot(H1)).dot(P).dot((np.eye(9, dtype=double) - K1*H1).transpose()) + K1.dot((Ra+Qstar)).dot(K1.transpose())
            P = 0.5 * (P + P.transpose())

            qe = np.array([1, x[0], x[1], x[2]])
            q4[:,i] = quaternionmul(q4[:,i],qe)
            q4[:,i]=  q4[:,i] / norm(q4[:,i])
            x[0:3] = np.zeros(3, dtype=double)
            Cq = quaternion2dcm(q4[:,i])

        if ym is not None:
            H2[0:3,0:3] =  2 * vec2product(Cq.dot(mtilde))
            z2 = ym[:,i] - Cq.dot(mtilde)

            P2 = zeros(9,9)
            P2[0:3, 0:3] = P[0:3, 0:3]
            r3 = Cq.dot(np.array([0, 0, 1]))
            K2 = np.array([[r3.dot(r3.transpose()), np.zeros((3, 6), dtype=double)], [np.zeros((6, 3), dtype=double), np.zeros((6, 6), dtype=double)]]).dot(P2).dot(H2.transpose()).dot(np.linalg.inv(H2.dot(P2).dot(H2.transpose()) + Rm));

            x = x + K2.dot((z2 - H2.dot(x)))
            P = P - K2.dot(H2).dot(P) - P.dot(H2.transpose()).dot(K2.transpose()) + K2.dot((H2*P*H2.transpose()+Rm)).dot(K2.transpose())
            P = 0.5 * (P + P.transpose())

            qe = np.array([1, x[0], x[1], x[2]])
            q4[:,i] = quaternionmul(q4[:,i],qe)
            q4[:,i]=  q4[:,i] / norm(q4[:,i])
            x[0:3] = np.zeros(3, dtype=double)
            Cq = quaternion2dcm(q4[:,i])

        if yi is not None:
            H3[0:3, 0:3] = 2 * vec2product(Cq.dot(gtilde))
            z3 = yi[:,i] - bihat - Cq*gtilde

            # adaptive algorithm
            fooR3 = (z3 - H3.dot(x)).dot((z3 - H3.dot(x)).transpose())
            R3[3*(i-1):3*i,:] = fooR3
            uk = fooR3
            for j in range(i-1, min([i-(inc_m1-1),1])):
                uk = uk + R3[3*(j-1):3*j,:]

            uk = uk / inc_m1

            fooR2 = H3.dot(P).dot(H3.transpose()) + Ri

            u,s,v = np.linalg.svd(uk, full_matrices=True)
            u1 = u[:,0]
            u2 = u[:,1]
            u3 = u[:,2]

            lamb = np.array([ s(0) , s(1) , s(2)])
            mu =  np.array([ u1.transpose().dot(fooR2).dot(u1) , u2.transpose().dot(fooR2).dot(u2) , u3.transpose().dot(fooR2).dot(u3)])

            if (max(lamb - mu) > inc_gamma):
              r2count = 0
              Qstar = max(lamb(0)-mu(0),0).dot(u1).dot(u1.transpose()) + max(lamb(1) -mu(1),0).dot(u2).dot(u2.transpose()) + max(lamb(2)-mu(2),0).dot(u3).dot(u3.transpose());
            else:
              r2count = r2count + 1
              if r2count < inc_m2:
                Qstar = max(lamb(0)-mu(0),0).dot(u1).dot(u1.transpose()) + max(lamb(1) -mu(1),0).dot(u2).dot(u2.transpose()) + max(lamb(2)-mu(2),0).dot(u3).dot(u3.transpose());
              else:
                Qstar = np.zeros((3,3),dtype=double)

            #Qstar = zeros(3,3);

            Qi[:,:,i] = Qstar

            Qstar

            P3 = P
            K3 = P3.dot(H3.transpose()).dot(np.linalg.inv(H3 * P3 * H3.transpose()) + Ri + Qstar)
            x = x + K3.dot(z3 - H3.dot(x))
            P = (np.eye(9, dtype=double) - K3.dot(H3)).dot(P).dot((np.eye(9, dtype=double) - K3*H3).transpose()) + K3.dot((Ri+Qstar)).dot(K3.transpose())
            P = 0.5 * (P + P.transpose())

            qe = np.array([1, x[0], x[1], x[2]])
            q4[:,i] = quaternionmul(q4[:,i],qe)
            q4[:,i]=  q4[:,i] / norm(q4[:,i])
            x[0:3] = np.zeros(3, dtype=double)
            Cq = quaternion2dcm(q4[:,i])

        bghat = bghat + x[3:6]
        x[3:6] = np.zeros(3, dtype=double)

        bahat = bahat + x[6:9]
        x[6:9] = np.zeros(3, dtype=double)

        qe = np.array([1, x(0), x(1), x(2)])
        q4[:,i] = quaternionmul(q4[:,i],qe)
        q4[:,i]=  q4[:,i] / norm(q4[:,i])
        x[0:3] = np.zeros(3,dtype=double)

        eulercom4[:,i] = quaternion2euler(q4[:,i])

    return [q4, eulercom4, bahat, bghat, Qa, Qi]
