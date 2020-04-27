#!/usr/bin/env python
# coding: utf-8

# In[4]:


import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm

data = np.loadtxt('pen.csv',delimiter =',')
x = data[:,0]
z = data[:,1]
actual = data[:,2]

error = actual - x

plt.figure()
plt.plot(z,'k+',label='noisy measurements')
plt.plot(x,'b-',label='a posteri estimate')
plt.plot(actual,color='g',label='truth value')
plt.legend()
plt.title('Estimate vs. iteration step', fontweight='bold')
plt.xlabel('Iteration')
plt.ylabel('Distance')

    

    


# In[5]:


mu, std = norm.fit(error)
plt.hist(error, bins=25, density=True, alpha=0.6, color='g')

xmin, xmax = plt.xlim()
x = np.linspace(xmin, xmax, 100)
p = norm.pdf(x, mu, std)
plt.plot(x, p, 'k', linewidth=2)
title = "Fit results: mu = %.2f,  std = %.2f" % (mu, std)
plt.title(title)

plt.show()

