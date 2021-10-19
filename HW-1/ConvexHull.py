#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Sep 27 10:10:30 2021

@author: huseinnoble
"""
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull

A = [[-1,1],[1,-1],[0,1]]; #My rrobot 
B = [[3,-1],[5,-1],[5,1],[3,1]]; #My object

#plt.plot(A[0],A[1])

#x, y = zip(*A)
#plt.scatter(x,y)


def Refpoint(List):
    #This function will find the center point of thhe robot
    x,y =  zip(*List)
    refx = np.sum(x)/np.size(x);
    refy = np.sum(y)/np.size(y);
    return [refx,refy]

def translate(List,ReferencePoint,boolean):
    #This function will translate all the points in a list, add or subtract, 
    #boolean = 1 wiill add and boolean =0 wiill subtract.
    x,y = zip(*List)
    x= np.array(x)
    y=np.array(y)
    if boolean == 1:
        
        Newx= x+ReferencePoint[0];
        Newy= y+ReferencePoint[1];
    elif boolean == 0:
        
        Newx= x-ReferencePoint[0];
        Newy= y-ReferencePoint[1];
        
    else:   
        print ("Bitch put 0 or 1 wtf")
    
    return np.vstack((Newx,Newy)).T

def itsfliptime (List):
    #Make sure the list which is coming in here is centered around origiin.
    x,y = zip(*List)
    x= np.array(x)
    y=np.array(y)
    xnew = - x;ynew = -y;
    #print("this bish has been flipped lol")
    return np.vstack((xnew,ynew)).T

def hullpointscreate(List1,List2):
    #Need  to take  every cooodinate point from the object, add  it to myy flipped rrobot
    #and then  append it to a new list/2d array to generate points for the conveex hull.
    empty_array = np.zeros((1,2))
    x,y = zip(*List2)
    x= np.array(x)
    y=np.array(y)
    points2= np.vstack((x,y)).T
    for meow in range(0,np.size(points2[:,0])):
        newpoint= points2[meow,:]+ List1;
        empty_array= np.append(empty_array, newpoint,axis =0)

    return empty_array[1:,:]

def PlotItYaa(List):
    hull =ConvexHull(G)

    #Now we just plot that babe
    plt.plot(G[:,0], G[:,1], 'o')
    for simplex in hull.simplices:
        plt.plot(G[simplex, 0], G[simplex, 1], 'k-')
    plt.plot(G[hull.vertices,0], G[hull.vertices,1], 'r--', lw=2)
    plt.plot(G[hull.vertices[0],0], G[hull.vertices[0],1], 'ro')
    plt.show()
    print("The X-cords of Hull are")
    print(G[hull.vertices,0])
    print("The Y-cords of Hull are")
    print(G[hull.vertices,1])
    
    
    
    
#C= Refpoint(A)
C=A[0];
D = translate(A, C,0)
E = itsfliptime(D)

#plt.scatter(E[:,0],E[:,1])
G=hullpointscreate(E,B)
#print(G)
PlotItYaa(G)
