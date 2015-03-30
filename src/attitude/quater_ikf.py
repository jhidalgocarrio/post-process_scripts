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
import numpy.matlib

def vec2product(v):
    M = np.matrix([[ 0 , -v[2] , v[1]],
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

    M = np.linalg.inv(np.asmatrix(M))

    return M

def quaternion2euler(q):

    euler = np.zeros(3, dtype=double)
    euler = np.asmatrix(euler).transpose()
    q0 = q[0]
    q1 = q[1]
    q2 = q[2]
    q3 = q[3]
    euler[2] = np.arctan2( 2 * q1 * q2 + 2 * q0 * q3 , 2 * q0 * q0 + 2 * q1 * q1 - 1)
    euler[1] = np.arcsin(-2*q1*q3 + 2*q0*q2)
    euler[0] = np.arctan2( 2 * q2 * q3 + 2 * q0 * q1 , 2 * q0 * q0 + 2 * q3 * q3 - 1)

    return euler

def quaternionmul(p,q):

    p0 = p[0]
    p1 = p[1]
    p2 = p[2]
    p3 = p[3]

    M = np.array([[p0 , -p1 , -p2 , -p3],
                  [p1 , p0 , -p3 , p2],
                  [p2 , p3 , p0, -p1],
                  [p3 , -p2, p1 , p0]])

    M = np.asmatrix(M)

    v = np.array([q[0], q[1], q[2], q[3]])
    v = np.asmatrix(v).transpose()

    r = M * v;

    return r

def dcm2quaternion (C):
    """ Quaternion representation is [w, x i, y j, z k]. """

    f = np.matlib.zeros(4, dtype=double).transpose()

    f[0] = 0.25 * (1 + np.trace(C))
    f[1] = 0.25 * (C[0,0] - C[1,1] - C[2,2] + 1)
    f[2] = 0.25 * (C[1,1] - C[0,0] - C[2,2] + 1)
    f[3] = 0.25 * (C[2,2] - C[1,1] - C[0,0] + 1)

    maxf = np.max(f)
    index = np.argmax(f)

    q = np.matlib.zeros(4, dtype=double).transpose()

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

def filter(P0 = None, ya=None, yg=None, ym=None, yi=None, tt=None,
                Ra=None, Rg=None, Rm=None, Ri=None,
                Qba=None, Qbg=None, Qbi=None, dip_angle=None,
                acc_m1 = None, acc_m2 = None, acc_gamma = None,
                inc_m1 = None, inc_m2 = None, inc_gamma = None):

    D2R = pi/180.0

    if P0 is None:
        raise ValueError("P0 cannot be None")

    # number of data: N
    N = yg.shape[1]

    # gravitation acceleration
    g = 9.81
    gtilde = np.matrix([0, 0, g]).transpose()

    # dip angle depending at location
    if dip_angle is not None:
        alpha = dip_angle * D2R
    else:
        alpha = 0.00

    mtilde = np.matrix([cos(alpha), 0, -sin(alpha)]).transpose()

    # --------------------------------------------------------
    # Kalman Filter
    # --------------------------------------------------------

    # q4 : quaternion
    q4 = np.zeros((4, N), dtype=double)
    q4 = np.asmatrix(q4)

    # eulercom4 : euler angles
    eulercom4 = np.zeros((3, N),  dtype=double)
    eulercom4 = np.asmatrix(eulercom4)

    Qa = np.zeros((N,3,3), dtype=double)
    Qi = np.zeros((N,3,3), dtype=double)

    # estimated bias for gyroscope (bghat) and accelerometer (bahat)
    bghat = np.zeros(3, dtype=double)
    bghat = np.asmatrix(bghat).transpose()
    bahat = np.zeros(3, dtype=double)
    bahat = np.asmatrix(bahat).transpose()
    bihat = np.zeros(3, dtype=double)
    bihat = np.asmatrix(bihat).transpose()

    # Initial orientation estimation using the TRIAD method
    if ym is not None:
        yabar = yi[:,0] / norm(yi[:,0])
        ymbar = ym[:,0] / norm(ym[:,0])
        foo1 = np.cross(yabar,ymbar) / norm(np.cross(yabar,ymbar))
        C = np.array([-np.cross(yabar,foo1), foo1, yabar])
        C = np.asmatrix(C)
        q4[:,0] = dcm2quaternion(C)
    else:
        q4[:,0] = np.matrix([1, 0, 0, 0]).transpose()


    # Kalman filter state, error covariance, process noise covariance
    x = np.zeros(12, dtype=double)
    x = np.asmatrix(x).transpose()
    P = np.zeros((12, 12), dtype=double)
    P = np.asmatrix(P)
    Q = np.zeros((12, 12), dtype=double)
    Q = np.asmatrix(Q)

    if Rg is not None:
        Q[0:3, 0:3] = Rg * 0.25
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
    A = np.zeros((12,12), dtype=double)
    A[0:3, 3:6] = -0.5 * np.eye(3, dtype=double)
    A = np.asmatrix(A)

    # Intialization for quaternion integration
    wx = yg[0,0]
    wy = yg[1,0]
    wz = yg[2,0]
    oldomega4 = np.matrix([[ 0 , -wx , -wy , -wz],
                      [wx , 0 , wz , -wy],
                      [wy , -wz , 0 , wx],
                      [wz , wy , -wx , 0]])

    # variable used in the adaptive algorithm
    r2count = 100

    # H1 and H2 for measurement update
    H1 = np.zeros((3,12), dtype=double)
    H1[0:3, 6:9] = np.eye(3, dtype=double)
    H1 = np.asmatrix(H1)
    H2 =  np.zeros((3,12), dtype=double)
    H2 = np.asmatrix(H2)
    H3 = np.zeros((3,12), dtype=double)
    H3[0:3, 9:12] = np.eye(3, dtype=double)
    H3 = np.asmatrix(H3)


    R1 = np.zeros((3*N,3), dtype=double)
    R1 = np.asmatrix(R1)
    R3 = np.zeros((3*N,3), dtype=double)
    R3 = np.asmatrix(R3)

    # Kalman filter loop
    for i in range (1,N):
        T = double(tt[i] - tt[i-1])
        #print 'index[{0}]: delta_t {1}.'.format(i, T)

        Cq = quaternion2dcm(q4[:,i-1]);
        A[0:3,0:3] = -vec2product(yg[:,i-1] - bghat);
        dA = np.matlib.eye(12, dtype=double) + A *T + A*A*pow(T,2) / 2.0;

        x = dA * x
        Qd = Q * T + 0.5*T*T*A*Q + 0.5*T*T*Q*A.transpose()
        Qd = 0.5*(Qd + Qd.transpose())
        P = dA * P * dA.transpose() + Qd;

        yyg = yg[:,i-1] - bghat;
        wx = yyg[0];
        wy = yyg[1];
        wz = yyg[2];
        omega4 = np.matrix([[ 0 , -wx , -wy , -wz],
                      [wx , 0 , wz , -wy],
                      [wy , -wz , 0 , wx],
                      [wz , wy , -wx , 0]])

        q4[:,i] = (np.matlib.eye(4, dtype=double) + 0.75 * omega4 * T
                - 0.25 * oldomega4 * T
                - (1.0/6.0) * pow(norm(yyg), 2) * pow(T,2) * np.eye(4, dtype=double)
                - (1.0/24.0) * omega4 * oldomega4 * pow(T,2)
                - (1.0/48.0) * pow(norm(yyg), 2) * omega4 * pow(T,3)) * q4[:,i-1]

        q4[:,i] = q4[:,i] / norm(q4[:,i])
        oldomega4 = omega4


        # ----------------------------------------------------
        # Three step measurement update
        # ----------------------------------------------------
        if ya is not None:
            Cq = quaternion2dcm(q4[:,i])

            H1[0:3, 0:3] = 2 * vec2product(Cq * gtilde)
            z1 = ya[:,i] - bahat# - Cq*gtilde

            # adaptive algorithm
            fooR1 = (z1 - H1 * x) * (z1 - H1 * x).transpose()
            R1[3*(i-1):3*i,:] = fooR1
            uk = fooR1
            for j in range(i-1, min([i-(acc_m1-1),1])):
                uk = uk + R1[3*(j-1):3*j,:]

            uk = uk / acc_m1

            fooR2 = H1 * P * H1.transpose() + Ra

            u,s,v = np.linalg.svd(uk, full_matrices=True)
            u1 = u[:,0]
            u2 = u[:,1]
            u3 = u[:,2]

            lamb = np.matrix([ s[0] , s[1] , s[2]]).transpose()
            mu =  np.array([ u1.transpose()* fooR2*u1 , u2.transpose()*fooR2*u2 , u3.transpose()*fooR2*u3])
            mu = np.asmatrix(mu).transpose()

            if (np.max(lamb - mu) > acc_gamma):
              r2count = 0
              Qstar = double(np.max(lamb[0]-mu[0],0))*(u1)*u1.transpose() + double(np.max(lamb[1]-mu[1],0))*(u2)*(u2.transpose()) + double(np.max(lamb[2]-mu[2],0))*(u3)*(u3.transpose())
            else:
              r2count = r2count + 1
              if r2count < acc_m2:
                Qstar = double(np.max(lamb[0]-mu[0],0))*(u1)*u1.transpose() + double(np.max(lamb[1]-mu[1],0))*(u2)*(u2.transpose()) + double(np.max(lamb[2]-mu[2],0))*(u3)*(u3.transpose())
              else:
                Qstar = np.zeros((3,3),dtype=double)

            #Qstar = zeros(3,3);

            Qa[i,:,:] = Qstar

            Qstar

            P1 = P
            K1 = P1*H1.transpose()*np.linalg.inv(H1 * P1 * H1.transpose() + Ra + Qstar)
            x = x + K1* (z1 - H1*x)
            P = (np.matlib.eye(12, dtype=double) - K1*H1)*P*(np.matlib.eye(12, dtype=double) - K1*H1).transpose() + K1*(Ra+Qstar)*K1.transpose()
            P = 0.5 * (P + P.transpose())

            qe = np.array([1, x[0], x[1], x[2]])
            q4[:,i] = quaternionmul(q4[:,i],qe)
            q4[:,i]=  q4[:,i] / norm(q4[:,i])
            x[0:3] = np.asmatrix(np.zeros(3, dtype=double)).transpose()

        if ym is not None:
            Cq = quaternion2dcm(q4[:,i])
            H2[0:3,0:3] =  2 * vec2product(Cq *mtilde)
            z2 = ym[:,i] - Cq*mtilde

            P2 = np.zeros((12,12), dtype=double)
            P2[0:3, 0:3] = P[0:3, 0:3]
            P2 = np.asmatrix(P2)
            r3 = Cq * np.matrix([0, 0, 1]).transpose()
            aux2 = np.zeros((12, 12), dtype=double)
            aux2[0:3, 0:3] = r3*r3.transpose()
            aux2 = np.asmatrix(aux2)
            K2 = aux2*P2*H2.transpose()*np.linalg.inv(H2*P2*H2.transpose() + Rm);

            x = x + K2*(z2 - H2*x)
            P = P - K2*H2*P - P*H2.transpose()*K2.transpose() + K2*(H2*P*H2.transpose()+Rm)*K2.transpose()
            P = 0.5 * (P + P.transpose())

            qe = np.array([1, x[0], x[1], x[2]])
            q4[:,i] = quaternionmul(q4[:,i],qe)
            q4[:,i]=  q4[:,i] / norm(q4[:,i])
            x[0:3] = np.asmatrix(np.zeros(3, dtype=double)).transpose()
            Cq = quaternion2dcm(q4[:,i])

        if yi is not None:
            H3[0:3, 0:3] = 2 * vec2product(Cq*gtilde)
            z3 = yi[:,i] - bihat - Cq*gtilde

            # adaptive algorithm
            fooR3 = (z3 - H3*x)*(z3 - H3*x).transpose()
            R3[3*(i-1):3*i,:] = fooR3
            uk = fooR3
            for j in range(i-1, min([i-(inc_m1-1),1])):
                uk = uk + R3[3*(j-1):3*j,:]

            uk = uk / inc_m1

            fooR2 = H3*P*H3.transpose() + Ri

            u,s,v = np.linalg.svd(uk, full_matrices=True)
            u1 = u[:,0]
            u2 = u[:,1]
            u3 = u[:,2]

            lamb = np.matrix([ s[0] , s[1] , s[2]]).transpose()
            mu =  np.array([ u1.transpose()* fooR2*u1 , u2.transpose()*fooR2*u2 , u3.transpose()*fooR2*u3])
            mu = np.asmatrix(mu).transpose()

            if (np.max(lamb - mu) > inc_gamma):
              r2count = 0
              Qstar = double(np.max(lamb[0]-mu[0],0))*(u1)*u1.transpose() + double(np.max(lamb[1]-mu[1],0))*(u2)*(u2.transpose()) + double(np.max(lamb[2]-mu[2],0))*(u3)*(u3.transpose())
            else:
              r2count = r2count + 1
              if r2count < inc_m2:
                Qstar = double(np.max(lamb[0]-mu[0],0))*(u1)*u1.transpose() + double(np.max(lamb[1]-mu[1],0))*(u2)*(u2.transpose()) + double(np.max(lamb[2]-mu[2],0))*(u3)*(u3.transpose())
              else:
                Qstar = np.zeros((3,3),dtype=double)

            #Qstar = zeros(3,3);

            Qi[i,:,:] = Qstar

            Qstar

            P3 = P
            K3 = P3*H3.transpose()*np.linalg.inv(H3 * P3 * H3.transpose() + Ri + Qstar)
            x = x + K3*(z3 - H3*x)
            P = (np.matlib.eye(12, dtype=double) - K3*H3)*P*(np.matlib.eye(12, dtype=double) - K3*H3).transpose() + K3*(Ra+Qstar)*K3.transpose()
            P = 0.5 * (P + P.transpose())

            qe = np.array([1, x[0], x[1], x[2]])
            q4[:,i] = quaternionmul(q4[:,i],qe)
            q4[:,i]=  q4[:,i] / norm(q4[:,i])
            x[0:3] = np.asmatrix(np.zeros(3, dtype=double)).transpose()
            Cq = quaternion2dcm(q4[:,i])

        bghat = bghat + x[3:6]
        x[3:6] = np.asmatrix(np.zeros(3, dtype=double)).transpose()

        bahat = bahat + x[6:9]
        x[6:9] = np.asmatrix(np.zeros(3, dtype=double)).transpose()

        qe = np.asmatrix(np.array([1, x[0], x[1], x[2]])).transpose()
        q4[:,i] = quaternionmul(q4[:,i],qe)
        q4[:,i]=  q4[:,i] / norm(q4[:,i])
        x[0:3] = np.asmatrix(np.zeros(3, dtype=double)).transpose()

        eulercom4[:,i] = quaternion2euler(q4[:,i])

    return [q4, eulercom4, bahat, bghat, Qa, Qi]
