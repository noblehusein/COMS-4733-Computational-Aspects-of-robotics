#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Sep 27 18:39:04 2021

@author: huseinnoble
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.path as pp
import matplotlib.patches as patches

Start = [0,8];
Finish = [17,0.5];
Obstacle =[[4,4],
           [2,6],
           [3,10],
           [10,10],
           [15,2],
           [0,0]]; #Ignore this 
Obby = pp.Path(Obstacle,closed=True)
#print(Obby)

def findMline(Start,Finish):
    x1=Start[0]
    y1=Start[1]
    x2=Finish[0]
    y2=Finish[1]
    m= (y2-y1)/(x2-x1);
    angle =np.arctan(m)
    return (m,angle)

def nextangle(theta,x,y,d):
    global gamma 
    theta=theta+gamma
    x=x+ d*np.cos(theta)
    y=y+ d*np.sin(theta)
    return (theta,x,y)

def prevangle(theta,x,y):
    global gamma, d
    theta=theta-gamma
    x=x+ d*np.cos(theta)
    y=y+ d*np.sin(theta)
    return (theta,x,y)

def findtheta(x,y):
    theta=np.arctan(y/x)
    return theta
    
def thresholdchecker(x,y):
    x1=Start[0]
    y1=Start[1]
    x2=Finish[0]
    y2=Finish[1]
    #print(x1,y1,x2,y2,x,y)
    T = (y-y1) -((y2-y1)/(x2-x1))*(x-x1)
    #print(T)
    return T
def FinalChecker(x,y):
    x2=Finish[0]
    y2=Finish[1]
 
    Ans= ((x2-x)**2+(y2-y)**2)**0.5

    return Ans

d=0.1
gamma=0.16
xpoints=[]
ypoints=[]
thetapoints=[]
xpoints.append(Start[0])
ypoints.append(Start[1])
m,angle=findMline(Start,Finish)
thetapoints.append((angle))

def followMline(list1,list2,list3):
    x=list1[-1]
    y=list2[-1]
    
    
    xnew=x+d*np.cos(angle)
    ynew=y+d*np.sin(angle)
    thetanew = findtheta(xnew, ynew)
    xpoints.append(xnew)
    ypoints.append(ynew)
    thetapoints.append(thetanew)
    print(FinalChecker(xnew,ynew))
    if  FinalChecker(xnew,ynew) <  0.3:
        print('hogaya')
        

def borderfollow(list1,list2,list3):
    x=xpoints[-1];y=ypoints[-1];theta=thetapoints[-1]
    thetanew,xnew,ynew=nextangle(theta, x, y,2*d)
    if Obby.contains_point([xnew,ynew]) == False:
        xpoints.append(xnew)
        ypoints.append(ynew)
        thetapoints.append(thetanew)

    else:
        
        thetapoints.append(thetanew)
        borderfollow(xpoints, ypoints, thetapoints)
    
    while (thresholdchecker(xpoints[-1],ypoints[-1])) > 0.001:
        x=xpoints[-1];y=ypoints[-1];theta=thetapoints[-1]
        thetanew,xnew,ynew=prevangle(theta, x, y)
        if Obby.contains_point([xnew,ynew]) == False:
            thetapoints.append(thetanew)
            
        
        else:
              
              x=x+ d*np.cos(thetapoints[-1])
              y=y+ d*np.sin(thetapoints[-1])
              xpoints.append(x)
              ypoints.append(y)
              if Obby.contains_point([x,y]) == True:
                  theta=theta+ 0.5*gamma
                  thetapoints.append(theta)
              print('l')
              
    followMline(xpoints, ypoints, thetapoints)
 




while (FinalChecker(xpoints[-1],ypoints[-1])) >  0.25: 

    if Obby.contains_point([xpoints[-1],ypoints[-1]]) == False:
        
        followMline(xpoints, ypoints, thetapoints)
    else: 
        print('l')
        borderfollow(xpoints, ypoints, thetapoints)
        
  
   #We Plot eeverything here 
fig, ax = plt.subplots()
patch = patches.PathPatch(Obby, facecolor='orange', lw=2)
ax.add_patch(patch)
ax.set_xlim(-10, 20)
ax.set_ylim(-10, 12)
plt.plot(xpoints,ypoints,)
plt.plot(xpoints[0], ypoints[0], marker="o", markersize=10, markeredgecolor="red", markerfacecolor="green")
plt.plot(xpoints[-1], ypoints[-1], marker="o", markersize=10, markeredgecolor="green", markerfacecolor="red")
plt.show()