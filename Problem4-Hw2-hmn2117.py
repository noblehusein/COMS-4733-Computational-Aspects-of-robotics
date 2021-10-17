#!/usr/bin/env python
# coding: utf-8

# In[29]:


L= [[60,120],[150,90],[270,300],[300,240]]; O= [[30,30],[330,330]];Meow=[180,180]


# In[30]:


import numpy as np
import matplotlib.pyplot as plt
P=np.array(L);O=np.array(O)
g=[]
def distance(a,b):
    q1 = min(abs(a[0]-b[0]),(360-abs(a[0]-b[0]))) 
    q2 = min(abs(a[1]-b[1]),(360-abs(a[1]-b[1])))
    d= q1+q2
    return d


# In[27]:


#an Array of the distances from point [180,180] to all the other points in the array.
for i in L:
    g.append(distance(i,Meow))
print(g)


# In[32]:


#d= distance(L[2],L[3])
plt.scatter(P[:,0],P[:,1]);plt.scatter(O[:,0],O[:,1]);plt.scatter(180,180);plt.xlim([0, 360]);plt.ylim([0,360])

