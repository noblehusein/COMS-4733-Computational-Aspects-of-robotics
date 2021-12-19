#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Dec 15 16:55:39 2021

@author: huseinnoble
"""
#Code for Newton Raphson Method:
from math import *
from decimal import *
import random as rd
import numpy as np

theta1=0
theta2=0
theta3=0
L1=1
L2=1
L3=1

Initialq=np.array([0,0,0])

Td=np.array([[0.078, -0.494, 0.866, 1],
             [0.135, -0.855,-0.500,2],
             [0.988, 0.156, 0, 2],
             [0,0,0,1]])
 
def CalculateFk(q):
    theta1=q[0]#*np.pi/180
    theta2=q[1]#*np.pi/180
    theta3=q[2]#*np.pi/180
    Fk=np.array([[cos(theta2+theta3)*cos(theta1), -sin(theta2+theta3)*cos(theta1), sin(theta1),cos(theta1)*(L1+L2*cos(theta2)+L3*cos(theta2+theta3))],
                [ cos(theta2+theta3)*sin(theta1), -sin(theta2+theta3)*sin(theta1),-cos(theta1),sin(theta1)*(L1+L2*cos(theta2)+L3*cos(theta2+theta3))],
                [sin(theta2+theta3), cos(theta2+theta3), 0, (L2*sin(theta2)+L3*sin(theta2+theta3))],
                [0,0,0,1]])
    return Fk
def CalculateJv(q):
    theta1=q[0]#*np.pi/180
    theta2=q[1]#*np.pi/180
    theta3=q[2]#*np.pi/180
    Jv=np.array([[-sin(theta1)*(L1+L3*cos(theta2+theta3)+L2*cos(theta2)),-(cos(theta1)*(L3*sin(theta2+theta3)+L2*sin(theta2))),-cos(theta1)*L3*sin(theta2+theta3)],
                 [cos(theta1)*(L1+L3*cos(theta2+theta3)+L2*cos(theta2)),-(sin(theta1)*(L3*sin(theta2+theta3)+L2*sin(theta2))),-sin(theta1)*L3*sin(theta2+theta3)],
                 [0, L2*cos(theta2)+L3*cos(theta2+theta3),L3*cos(theta2+theta3)],
                 [0,sin(theta1),sin(theta1)],
                 [0,-cos(theta1),-cos(theta1)],
                 [1,0,0]])
    return Jv

def CalculateDLSJv(Jv):
    #U, s, V = np.linalg.svd(Jv)
    Lambda=0
    Jinv=np.linalg.inv(np.transpose(Jv) @ Jv + Lambda**2 * np.identity(3)) @ np.transpose(Jv)
    return Jinv

def CalculateErr(Td,Fk):
    od=Fk[0:3,3]
    on=Td[0:3,3]
    errLin=on-od
    errLin=errLin.reshape(3,1)
    c=0
    for i in range(0,3):
        a=Fk[0:3,i]; b= Td[0:3,i]
        c=c+np.cross(a,b)
    errAng=0.5*c
    errAng=errAng.reshape(3,1)
    FinalErr= np.concatenate((errLin,errAng))
    return FinalErr
    

def NewtonRaphson(Initialq,Td):
    q=Initialq
    
    Err=100
    while np.linalg.norm(Err)>0.1:
        Fk= CalculateFk(q)
        Jv= CalculateJv(q)
        Jinv= CalculateDLSJv(Jv)
        Err=CalculateErr(Td,Fk)
        q=q.reshape(3,1)
        q= q+ Jinv @ Err
        q=q.reshape(3,)
        print(q)
        print(np.linalg.norm(Err))
    return q

def GradDescent(Initialq,Td):
    q=Initialq
    alpha=0.1
    Err=100
    while np.linalg.norm(Err)>0.1:
        Fk= CalculateFk(q)
        Jv= CalculateJv(q)
        #Jinv= CalculateDLSJv(Jv)
        Err=CalculateErr(Td,Fk)
        q=q.reshape(3,1)
        q= q+ alpha* np.transpose(Jv) @ Err
        q=q.reshape(3,)
        print(q)
        print(np.linalg.norm(Err))
    return q

def main():
    #q=NewtonRaphson(Initialq,Td)
    q=GradDescent(Initialq,Td)


if __name__ == '__main__': 
    main()






    
    
    
