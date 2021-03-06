# -*- coding: utf-8 -*-
"""hw3p3.ipynb

Automatically generated by Colaboratory.
"""

import numpy as np
import matplotlib.pyplot as plt

def KF(x, P, z):
    #getting updated xhat:
  #Q = np.array([[0.0001, 0.0001], [0.00001, 0.00001]]) 
  #print(w)
    x= F @ x 
#getting updated P:
    P=(F @ P @ F.transpose()) + Q*1
    #print(P)
# getting kalman gain:
    S = (H @ P @ H.transpose())+R
    Kolmon= P @ H.transpose() @ np.linalg.inv(S)
# getting innovation error:   
    innerr= z - H @ x
  #print(Kolmon)
#updating x
    x= x+ Kolmon @ innerr
  
#updating p
    P=  (np.identity(2)- (Kolmon @ H)) @ P
    print('State Covariance = ',P,"\n",'Kalman Gain = ',Kolmon,'\n','Innnoation Covariance = ',S)
    
    return x, P

  
    

F = np.array([[1, 0.5], [0, 1]])
H = np.array([[0, 1]])
Q = np.array([[0.1, 0.01], [0.01, 0.05]])
R = 0.1

x = np.array([[0.8, 2]])
xhat = np.array([[2, 4]])
P = np.array([[1,0], [0,2]])

for i in range(100):
  w = np.random.multivariate_normal(np.zeros(2), Q)
  v = np.random.normal(0, R)
  x = np.vstack((x, F @ x[-1,:] + w))
  z = H @ x[-1,:] + v

  xnew, P = KF(xhat[-1,:], P, z)
  #if i==0:
     # print(P,kolmon,inner)
  #if i> 80:
  #print('State Covariance = ',P,"\n",'Kalman Gain = ',kolmon,'\n','Innnoation Error = ',inner)
  xhat = np.vstack((xhat, xnew))
  #print(xhat)

#plt.ylim(1,3)
plt.plot(x[:,0], x[:,1], label = "actual state")
plt.plot(xhat[:,0], xhat[:,1], label = "predicted state")
plt.legend()
plt.xlabel('position')
plt.ylabel('velocity')
plt.show()
