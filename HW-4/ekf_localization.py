"""
COMS 4733 Fall 2021 Homework 4
Scaffolding code for localization using an extended Kalman filter
Inspired by a similar example on the PythonRobotics project
https://pythonrobotics.readthedocs.io/en/latest/
"""

import math
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as Rot


# "True" robot noise (filters do NOT know these)
WHEEL1_NOISE = 0.05
WHEEL2_NOISE = 0.1
BEARING_SENSOR_NOISE = np.deg2rad(1.0)

# Physical robot parameters (filters do know these)
RHO = 1
L = 1
MAX_RANGE = 18.0    # maximum observation range

# RFID positions [x, y]
RFID = np.array([[-5.0, -5.0],
                 [10.0, 0.0],
                 [10.0, 10.0],
                 [0.0, 15.0],
                 [-5.0, 20.0]])

# Covariances used by the estimators
Q = np.diag([0.1, 0.1, np.deg2rad(1.0)]) ** 2
R = np.diag([0.4, np.deg2rad(1.0)]) ** 2

# Other parameters
DT = 0.1            # time interval [s]
SIM_TIME = 30.0     # simulation time [s]

# Plot limits
XLIM = [-20,20]
YLIM = [-10,30]
show_animation = True


"""
Robot physics
"""


def input(time, x):
    # Control inputs to the robot at a given time for a given state
    psi1dot = 3.7
    psi2dot = 4.0
    return np.array([psi1dot, psi2dot])

def move(x, u):
    # Physical motion model of the robot: x_k = f(x_{k-1}, u_k)
    # Incorporates imperfections in the wheels
    theta = x[2]
    psi1dot = u[0] * (1 + np.random.rand() * WHEEL1_NOISE)
    psi2dot = u[1] * (1 + np.random.rand() * WHEEL2_NOISE)

    velocity = np.array([RHO/2 * np.cos(theta) * (psi1dot+psi2dot),
                         RHO/2 * np.sin(theta) * (psi1dot+psi2dot),
                         RHO/L * (psi2dot - psi1dot)])

    return x + DT * velocity

def measure(x):
    # Physical measurement model of the robot: z_k = h(x_k)
    # Incorporates imperfections in both range and bearing sensors
    z = np.zeros((0, 3))

    for i in range(len(RFID[:, 0])):
        dx = RFID[i, 0] - x[0]
        dy = RFID[i, 1] - x[1]
        r = math.sqrt(dx**2 + dy**2)
        phi = math.atan2(dy, dx) - x[2]

        if r <= MAX_RANGE:
            zi = np.array([[np.round(r),
                            phi + np.random.randn() * BEARING_SENSOR_NOISE,
                            i]])
            z = np.vstack((z, zi))

    return z


"""
Extended Kalman filter procedure
"""

def EKF(x, P, u, z):

    
    x, P,F = predict(x, P, u)

    x, P,H, verbose,K = update(x, P, z)
    
    #if verbose== False:
    #print(verbose)
    #print('P is = ',P,'\n')

    return x, P


def predict(x, P, u):
    """
    :param x: State mean (x,y,theta) [size 3 array]
    :param P: State covariance [3x3 array]
    :param u: Robot inputs (u1,u2) [size 2 array]
    :return: Predicted state mean and covariance x and P
    """
    #Mean Prediction:
    #x=[0,0,0]; u=[0,0]
    #print(x,P)
    xprev=x[0];yprev=x[1];thetaprev=x[2]
    x[0]=xprev+DT*(RHO/2)*math.cos(thetaprev)*(u[0]+u[1])
    x[1]=yprev+DT*(RHO/2)*math.sin(thetaprev)*(u[0]+u[1])
    x[2]=thetaprev+DT*(RHO/L)*(u[1]-u[0])
    
    #Variance "P" Update:
    a=(RHO/2)*DT*math.sin(thetaprev)*(u[0]+u[1])
    b=(RHO/2)*DT*math.cos(thetaprev)*(u[0]+u[1])
    F=[[1,0,-a],
       [0,1,b],
       [0,0,1]]
    F=np.array(F)
    P= F @ P @ F.T + Q
    
    
    #print(x,P)
    return x, P,F

        
def update(x, P, z):
    """
    :param x: State mean (x,y,theta) [size 3 array]
    :param P: State covariance [3x3 array]
    :param z: Sensor measurements [px3 array]. Each row contains range, bearing, and landmark's true (x,y) location.
    :return: Updated state mean and covariance x and P
    
    """
    I=np.identity(3)
    xold=x[0];yold=x[1];thetaold=x[2]
    ybar=np.zeros(2*len(z))
    H=np.zeros((2*len(z),3))
    Rnew=np.zeros((2*len(z),2*len(z)))
    
    for i in range(len(z)):   
        
        xl=RFID[int(z[i,2])][0]
        yl=RFID[int(z[i,2])][1]
        ri=math.sqrt((xl-xold)**2+(yl-yold)**2)
        phi=math.atan2((yl-yold), (xl-xold))-thetaold
        #print(z[i,0],ri)
        ybar[2*i]=z[i,0]-ri
        ybar[2*i+1]=z[i,1]-phi
        
        H[2*i,:]=np.array([((xold-xl)/ri), ((yold-yl)/ri),0])
        H[2*i+1,:]=np.array([-(yold-yl)/ri**2, (xold-xl)/ri**2,-1])
        
        Rnew[2*i:2*i+2,2*i:2*i+2]=R
    #print(np.linalg.norm(ybar),'\n')
    if np.linalg.norm(ybar)>1.0:
        verbose =True
        
    else:
        verbose=False
    
    S= H @ P @ H.transpose() + Rnew
    K=P @ H.transpose() @ np.linalg.inv(S)

    
    x=x+ K @ ybar 
    P= (I- K @ H) @ P
    
    return x, P,H, verbose,K


def plot_covariance_ellipse(xEst, PEst):
    Pxy = PEst[0:2, 0:2]
    eigval, eigvec = np.linalg.eig(Pxy)

    if eigval[0] >= eigval[1]:
        bigind = 0
        smallind = 1
    else:
        bigind = 1
        smallind = 0

    t = np.arange(0, 2 * math.pi + 0.1, 0.1)
    a = math.sqrt(eigval[bigind])
    b = math.sqrt(eigval[smallind])
    x = [a * math.cos(it) for it in t]
    y = [b * math.sin(it) for it in t]
    angle = math.atan2(eigvec[1, bigind], eigvec[0, bigind])
    rot = Rot.from_euler('z', angle).as_matrix()[0:2, 0:2]
    fx = rot @ (np.array([x, y]))
    px = np.array(fx[0, :] + xEst[0]).flatten()
    py = np.array(fx[1, :] + xEst[1]).flatten()
    plt.plot(px, py, "--g")


def main():
    time = 0.0

    # Initialize state and covariance
    x_est = np.zeros(3)
    #x_est= np.array([20,20,20])
    x_true = np.zeros(3)
    P = np.eye(3)*(10000)

    # State history
    h_x_est = x_est.T
    h_x_true = x_true.T
    

    while time <= SIM_TIME:
        time += DT
        u = input(time, x_true)
        x_true = move(x_true, u)
        z = measure(x_true)
        #print(P)
        x_est, P = EKF(x_est, P, u, z)

        # store data history
        h_x_est = np.vstack((h_x_est, x_est))
        h_x_true = np.vstack((h_x_true, x_true))

        if show_animation:
            plt.cla()

            for i in range(len(z[:,0])):
                plt.plot([x_true[0], RFID[int(z[i,2]),0]], [x_true[1], RFID[int(z[i,2]),1]], "-k")
            plt.plot(RFID[:,0], RFID[:,1], "*k")
            plt.plot(np.array(h_x_true[:,0]).flatten(),
                     np.array(h_x_true[:,1]).flatten(), "-b")
            plt.plot(np.array(h_x_est[:,0]).flatten(),
                     np.array(h_x_est[:,1]).flatten(), "-r")
            plot_covariance_ellipse(x_est, P)

            plt.axis("equal")
            plt.xlim(XLIM)
            plt.ylim(YLIM)
            plt.grid(True)
            plt.pause(0.001)

    plt.figure()
    errors = np.abs(h_x_true - h_x_est)
    plt.plot(errors)
    dth = errors[:,2] % (2*np.pi)
    errors[:,2] = np.amin(np.array([2*np.pi-dth, dth]), axis=0)
    plt.legend(['x error', 'y error', 'th error'])
    plt.xlabel('time')
    plt.ylabel('error magnitude')
    plt.ylim([0,1.5])
    plt.show()


if __name__ == '__main__':
    main()
