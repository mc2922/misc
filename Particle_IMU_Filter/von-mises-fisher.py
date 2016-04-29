#!/usr/bin/env python

import numpy as np
import scipy as sc
import scipy.stats
import matplotlib.pyplot as plt

''' rejection sampling scheme for Von-Mises-Fisher '''
def rW(n,kappa,m):
	dim = m-1
	b = dim / (np.sqrt(4*kappa*kappa + dim*dim) + 2*kappa)
	x = (1-b) / (1+b)
	c = kappa*x + dim*np.log(1-x*x)

	y = []
	for i in range(0,n):
		done = False
		while not done:
			z = sc.stats.beta.rvs(dim/2,dim/2)
			w = (1 - (1+b)*z) / (1 - (1-b)*z)
			u = sc.stats.uniform.rvs()
			if kappa*w + dim*np.log(1-x*w) - c >= np.log(u):
				done = True
		y.append(w)
	return y

def rW(kappa,m):
	dim = m-1
	b = dim / (np.sqrt(4*kappa*kappa + dim*dim) + 2*kappa)
	x = (1-b) / (1+b)
	c = kappa*x + dim*np.log(1-x*x)

	done = False
	while not done:
		z = sc.stats.beta.rvs(dim/2,dim/2)
		w = (1 - (1+b)*z) / (1 - (1-b)*z)
		u = sc.stats.uniform.rvs()
		if kappa*w + dim*np.log(1-x*w) - c >= np.log(u):
			done = True
	return w

''' Von-Mises-Fisher sampling '''
def rvMF(n,theta):
	dim = len(theta)
	kappa = np.linalg.norm(theta)
	mu = theta / kappa

	result = []
	for sample in range(0,n):
		w = rW(kappa,dim)
		v = np.random.randn(dim)
		v = v / np.linalg.norm(v)

		result.append(np.sqrt(1-w**2)*v + w*mu)

	return result

#sampling - single vector, direction is mean, magnitude is inverse to variance
n = 100
kappa = 100
direction = np.array([1,0,0])
direction = direction / np.linalg.norm(direction)

res_sampling = rvMF(n, kappa * direction)
xy = np.zeros([len(res_sampling),2])
for i in range(0,len(res_sampling)):
	print np.linalg.norm(res_sampling[i])
	xy[i,0] = res_sampling[i][0]
	xy[i,1] = res_sampling[i][1]

fig = plt.figure()
ax = fig.add_subplot(111)
ax.scatter(xy[:,0],xy[:,1])
plt.show()