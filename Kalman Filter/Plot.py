#!/usr/bin/env python
# coding: utf-8

# In[7]:


import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('kalman.csv',delimiter =',')
x = data[:,:-1]
z = data[:,-1:]

actual = np.zeros(52)
u = 0

for i in range(1,52):
    
    u = u + 0.2
    actual[i] = u
    
plt.figure()
plt.plot(z,'k+',label='noisy measurements')
plt.plot(x,'b-',label='a posteri estimate')
plt.plot(actual,color='g',label='truth value')
plt.legend()
plt.title('Estimate vs. iteration step', fontweight='bold')
plt.xlabel('Iteration')
plt.ylabel('Distance')

    

    


# In[ ]:




