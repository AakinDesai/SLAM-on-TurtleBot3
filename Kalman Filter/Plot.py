import numpy as np
import matplotlib.pyplot as plt
from scipy.stats import norm

data = np.loadtxt('kalman.csv',delimiter =',')
x = data[:,:-1]
z = data[:,-1:]

actual = np.zeros((51,1))
u = 0

for i in range(51):
    
    actual[i] = u
    u = u + 0.2
    
plt.figure()
plt.plot(z,'k+',label='noisy measurements')
plt.plot(x,'b-',label='a posteri estimate')
plt.plot(actual,color='g',label='truth value')
plt.legend()
plt.title('Estimate vs. iteration step', fontweight='bold')
plt.xlabel('Iteration')
plt.ylabel('Distance')

error = np.zeros(52)
error = actual - x

mu, std = norm.fit(error)
plt.hist(error, bins=25, density=True, alpha=0.6, color='g')

xmin, xmax = plt.xlim()
x = np.linspace(xmin, xmax, 100)
p = norm.pdf(x, mu, std)
plt.plot(x, p, 'k', linewidth=2)
title = "Fit results: mu = %.2f,  std = %.2f" % (mu, std)
plt.title(title)

plt.show()

