#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import copy

import cv2

# XYZ <-> FWD-PRT-OVH
# l - local frame (gravity aligned) initialization: relative to initial yaw
# b - body frame
# self.peRb_pt(np.dot(lRb_val, self.particles_l[:,i:i+1]))	# local all the way to plane
# np.dot(bRl_val, self.bRpe_pt(az_el[:,i:i+1]))				# plane all the way to local

class particleIMU(object):
	def __init__(self, num_particles):
		self.num_particles = num_particles
		self.particles_l = np.zeros([3,self.num_particles])
		self.weights = np.ones([1,self.num_particles])/self.num_particles
		self.yaw = 0.0
		self.a_prev = np.zeros([3,1])
		self.bRi = np.eye(3)

	def Rx(self, th):
		return np.array([[1.0,0.0,0.0],[0.0,np.cos(th),-np.sin(th)],[0.0,np.sin(th),np.cos(th)]])

	# rotation about y-axis (pitch) - should the -sin be switched?!!
	def Ry(self, th):
		return np.array([[np.cos(th),0.0,-np.sin(th)],[0.0,1.0,0.0],[np.sin(th),0.0,np.cos(th)]])

	# rotation about z-axis (yaw)
	def Rz(self, th):
		return np.array([[np.cos(th),-np.sin(th),0.0],[np.sin(th),np.cos(th),0.0],[0.0,0.0,1.0]])

	# point-to-body (convert from local point to body point given azimuth and declination)
	def bRp(self, az, th):
		return np.dot(self.Rz(az), self.Ry(th))	#such that acoustic measurement x parameter (Rx) is equal to 0; azimuth, declination remain

	# psi/eta-to-body (convert from local point to body point given azimuth and elevation)
	def bRpe(self, az, el):
		return self.bRp(az, np.pi/2.0-el)

	# psi/eta-point-to-body (convert from local point to body point given azimuth and elevation as vector) - returns pt col vector of 3 (x,y,z)
	# pt is col vector of 2 (azimuth,elevation)
	def bRpe_pt(self, pt):
		# N = pt.shape[1]
		# body_pt = np.zeros([3,N])
		# for i in range(0,N):
		# 	body_pt[:,i:i+1] = np.dot(self.bRpe(pt[0,i], pt[1,i]), np.array([[1.0],[0.0],[0.0]]))
		# return body_pt
		return self.pe_TO_b(pt)

	# body-point-to-psi/eta (convert from body point to azimuth and declination given a (x,y,z) point in body-ball) - returns pt col vector of 2 (azimuth,elevation)
	# pt is col vector of 3 (x,y,z)
	def peRb_pt(self, pt):
		# return np.array([np.arctan2(pt[1,:], pt[0,:]), np.arctan2(np.linalg.norm(pt[0:2,:], ord=None, axis=0), pt[2,:])])
		return self.b_TO_pe(pt)

	def b_TO_pe(self, pt):
		N = pt.shape[1]
		bTOpe = np.zeros([2,N])
		bTOpe[0,:] = np.arctan2(pt[1,:],pt[0,:])
		bTOpe[1,:] = np.arccos(pt[2,:]/np.linalg.norm(pt,axis=0))
		return bTOpe

	def pe_TO_b(self, pt):
		N = pt.shape[1]
		peTOb = np.zeros([3,N])
		peTOb[0,:] = np.sin(pt[1,:])*np.cos(pt[0,:])
		peTOb[1,:] = np.sin(pt[1,:])*np.sin(pt[0,:])
		peTOb[2,:] = np.cos(pt[1,:])
		return peTOb

	# azimuth-to-body (convert from accel/mag to body) - actually body-to-azimuth (want lRb = lRa * aRb -> Rz * bRa)
	def bRa(self, acc=np.array([[0.0],[0.0],[9.81]]), mag=np.array([[1.0],[0.0],[0.0]])):
		g = acc/np.linalg.norm(acc)
		m = mag/np.linalg.norm(mag)
		v = np.cross(g.T,m.T)
		v = v/np.linalg.norm(v)
		w = np.cross(v,g.T)
		return np.hstack((w.T, v.T, g))

	# body-to-local given accel
	def lRb(self, accel=None):
		if accel == None:
			return np.dot(self.Rz(self.yaw), self.bRa(acc=self.a_prev).T)
		else:
			return np.dot(self.Rz(self.yaw), self.bRa(acc=accel).T)

	# propagate IMU measurements to keep track of yaw in local frame
	def propNavIMU(self, dt, w, a, yaw=0.0):
		bRa_val = self.bRa(acc=a)
		aRb = bRa_val.T
		aGyr = np.dot(aRb,w)
		dAzi = aGyr[2,0] # - gBiasEst
		yaw = yaw + dAzi*dt
		yaw = self.wrapRad(yaw)
		return yaw

	# TO-DO: use magnetometer to constrain IMU-calculated yaw in local frame
	def magUpdateYaw_mod(self, mag, mRef=np.array([[1.0],[0.0],[0.0]])):
		lRb_val = self.lRb()
		bRl_val = lRb_val.T
		lMag = np.dot(bRl_val,mag)
		return lMag

		# lMag = np.dot(self.lRb(),mag)
		# magYaw = np.atan2(lMag[1,0],lMag[0,0])
		# dyaw = magYaw - yaw
		# #some kind of filter that very slowly pushes yaw to magYaw
		# filtdyaw = yawFilter(dyaw)
		# self.yaw = self.yaw + filtdyaw

	def yawFilter(self):
		return 0.0

	# given points in azimuth/elevation, convert to points in body frame
	# def projPtsPlaneToBodyBall_mod(self, pts, bpts=np.zeros([3,0])):
	# 	cols = pts.shape[1]
	# 	if cols != bpts.shape[1]:
	# 		bpts = np.zeros([3,cols])
	# 	for i in range(0,cols):
	# 		bpts[:,i] = np.dot(p.bRpe(pts[:,i]), np.array([[1.0],[0.0],[0.0]]))

	def upStateIMU_mod(self, dt, w, a):
		self.yaw = self.propNavIMU(dt, w, a, self.yaw)
		self.a_prev[:,0:1] = a

		self.disperse_particles_l(self.deg_to_rad(0.7), self.deg_to_rad(0.7)) #0.35

	def upStateAcoustics_mod(self, bf_importance, im):
		lRb_val = self.lRb()
		bRl_val = lRb_val.T
		particles_pe = self.peRb_pt(np.dot(bRl_val, self.particles_l)) # lRb
		self.wrapRad2PiVec_mod(particles_pe[0,:])

		self.weights = self.sequential_importance_sampling(bf_importance**3, np.linspace(0,2*np.pi,bf_importance.shape[1]), np.linspace(0,np.pi,bf_importance.shape[0]), particles_pe[0,:], particles_pe[1,:], self.weights)
		self.systematic_resample(particles_pe[0,:], particles_pe[1,:], self.weights)

		# self.disperse_particles(particles_pe[0,:], particles_pe[1,:], self.deg_to_rad(3), self.deg_to_rad(3))

		self.particles_l = np.dot(lRb_val, self.bRpe_pt(particles_pe)) # bRl

		for i in range(0,self.particles_l.shape[1]):
			cv2.circle(im, (int(particles_pe[0,i]*180/np.pi),int(particles_pe[1,i]*180/np.pi)), 3, (0,0,0))


	### HELPERS ###
	# wrap between -PI and +PI
	def wrapRad(self, th):
		if th >= np.pi:
			th = th-2.0*np.pi
		if th < -np.pi:
			th = th+2.0*np.pi
		return th

	def wrapRad2PI(self, th):
		if th >= 2.0*np.pi:
			th = th-2.0*np.pi
		if th < 0:
			th = th+2.0*np.pi
		return th

	def wrapRad2PiVec_mod(self, vec):
		vec[vec >= 2.0*np.pi] = vec[vec >= 2.0*np.pi] - 2.0*np.pi
		vec[vec < 0.0] = vec[vec < 0.0] + 2.0*np.pi

	def rad_to_deg(self, rad):
		return rad*180.0/np.pi

	def deg_to_rad(self, deg):
		return deg*np.pi/180.0

	def set_bRi(self,R):
		self.bRi = R

	def init_sphere_uniform(self):
		self.weights = np.ones(self.num_particles)/self.num_particles
		u = np.random.uniform(size=self.num_particles)
		v = np.random.uniform(size=self.num_particles)
		psi = self.rad_to_deg(2*np.pi*u)
		theta = self.rad_to_deg(np.arccos(2*v - 1))
		self.particles_l[0,:] = np.sin(self.deg_to_rad(theta))*np.cos(self.deg_to_rad(psi))
		self.particles_l[1,:] = np.sin(self.deg_to_rad(theta))*np.sin(self.deg_to_rad(psi))
		self.particles_l[2,:] = np.cos(self.deg_to_rad(theta))
		# return psi, theta, weights

	def spherical_mean_particles(self, particles_phi, particles_theta):
		mean_phi = np.arctan2(np.mean(np.sin(deg_to_rad(particles_phi))), np.mean(np.cos(deg_to_rad(particles_phi))))
		mean_phi = self.rad_to_deg(mean_phi)
		if mean_phi < 0:
			mean_phi += 360
		elif mean_phi > 360:
			mean_phi -= 360
		mean_theta = np.arctan2(np.mean(np.sin(deg_to_rad(particles_theta))), np.mean(np.cos(deg_to_rad(particles_theta))))
		mean_theta = self.rad_to_deg(mean_theta)
		if mean_theta < 0:
			mean_theta = -mean_theta
		elif mean_theta > 180:
			mean_theta = 360 - mean_theta
		return mean_phi, mean_theta

	def spherical_mean_particles_weighted(self, particles_phi, particles_theta, particles_weights):
		mean_phi = np.arctan2(np.average(np.sin(deg_to_rad(particles_phi)), weights=particles_weights), np.average(np.cos(deg_to_rad(particles_phi)), weights=particles_weights))
		mean_phi = self.rad_to_deg(mean_phi)
		if mean_phi < 0:
			mean_phi += 360
		elif mean_phi > 360:
			mean_phi -= 360
		mean_theta = np.arctan2(np.average(np.sin(deg_to_rad(particles_theta)), weights=particles_weights), np.average(np.cos(deg_to_rad(particles_theta)), weights=particles_weights))
		mean_theta = self.rad_to_deg(mean_theta)
		if mean_theta < 0:
			mean_theta = -mean_theta
		elif mean_theta > 180:
			mean_theta = 360 - mean_theta
		return mean_phi, mean_theta

	def find_closest(self, A, target):
		#A must be sorted
		idx = A.searchsorted(target)
		idx = np.clip(idx, 1, len(A)-1)
		left = A[idx-1]
		right = A[idx]
		idx -= target - left < right - target
		return idx

	def sequential_importance_sampling(self, importance, importance_phis, importance_thetas, particles_phi, particles_theta, particles_weights):
		# find index closest to each sample position
		closest_phi_idxs = self.find_closest(importance_phis, particles_phi)
		closest_theta_idxs = self.find_closest(importance_thetas, particles_theta)
		# perfom importance sampling, re-weighting each particle
		particles_weights *= importance[closest_theta_idxs, closest_phi_idxs]
		particles_weights /= np.sum(particles_weights)
		return particles_weights

	def multinomal_resample(self, particles_phi, particles_theta, particles_weights):
		cumulative_sum = np.cumsum(particles_weights)
		cumulative_sum[-1] = 1.  # avoid round-off errors
		idxs = np.searchsorted(cumulative_sum, np.random.uniform(size=particles_weights.shape[0]))
		# resample according to indexes
		particles_phi[:] = particles_phi[idxs]
		particles_theta[:] = particles_theta[idxs]
		particles_weights[:] = particles_weights[idxs]
		particles_weights /= np.sum(particles_weights)

	def multinomal_resample_reweight(self, particles_phi, particles_theta, particles_weights):
		cumulative_sum = np.cumsum(particles_weights)
		cumulative_sum[-1] = 1.  # avoid round-off errors
		idxs = np.searchsorted(cumulative_sum, np.random.uniform(size=particles_weights.shape[0]))
		# resample according to indexes
		particles_phi[:] = particles_phi[idxs]
		particles_theta[:] = particles_theta[idxs]
		particles_weights[:] = np.ones(particles_weights.shape[0])/particles_weights.shape[0]

	def systematic_resample(self, particles_phi, particles_theta, particles_weights):
		N = len(particles_weights)
		# make N subdivisions, and choose positions with a consistent random offset
		positions = (np.random.random() + np.arange(N)) / N
		idxs = np.zeros(N, 'i')
		cumulative_sum = np.cumsum(particles_weights)
		i, j = 0, 0
		while i < N:
			if positions[i] < cumulative_sum[j]:
				idxs[i] = j
				i += 1
			else:
				j += 1
		particles_phi[:] = particles_phi[idxs]
		particles_theta[:] = particles_theta[idxs]
		particles_weights[:] = particles_weights[idxs]
		particles_weights /= np.sum(particles_weights)

	def systematic_resample_reweight(self, particles_phi, particles_theta, particles_weights):
		N = len(particles_weights)
		# make N subdivisions, and choose positions with a consistent random offset
		positions = (np.random.random() + np.arange(N)) / N
		idxs = np.zeros(N, 'i')
		cumulative_sum = np.cumsum(particles_weights)
		i, j = 0, 0
		while i < N:
			if positions[i] < cumulative_sum[j]:
				idxs[i] = j
				i += 1
			else:
				j += 1
		particles_phi[:] = particles_phi[idxs]
		particles_theta[:] = particles_theta[idxs]
		particles_weights[:] = np.ones(particles_weights.shape[0])/particles_weights.shape[0]

	def disperse_particles(self, particles_phi, particles_theta, phi_std, theta_std):
		rand_phi = np.random.normal(0, phi_std, particles_phi.shape[0])
		rand_theta = np.random.normal(0, theta_std, particles_theta.shape[0])
		particles_phi += rand_phi
		particles_theta += rand_theta
		particles_phi[particles_phi<0] += 2*np.pi
		particles_phi[particles_phi>=2*np.pi] -= 2*np.pi
		particles_theta[particles_theta<0] = -particles_theta[particles_theta<0]
		particles_theta[particles_theta>=np.pi] = 2*np.pi - particles_theta[particles_theta>=np.pi]

	def disperse_particles_l(self, phi_std, theta_std):
		lRb_val = self.lRb()
		bRl_val = lRb_val.T
		particles_pe = self.peRb_pt(np.dot(bRl_val, self.particles_l)) #lRb_val
		self.wrapRad2PiVec_mod(particles_pe[0,:])
		self.disperse_particles(particles_pe[0,:], particles_pe[1,:], phi_std, theta_std)
		self.particles_l = np.dot(lRb_val, self.bRpe_pt(particles_pe)) #bRl_val

p = particleIMU(10)
print np.dot(p.bRp(0.0,0.0), np.array([[1.0],[0.0],[0.0]]))
print np.dot(p.bRp(np.pi/2.0,0.0), np.array([[1.0],[0.0],[0.0]]))
print np.dot(p.bRp(np.pi/2.0,np.pi/4.0), np.array([[1.0],[0.0],[0.0]]))
print np.dot(p.bRp(-np.pi/2.0,np.pi/4.0), np.array([[1.0],[0.0],[0.0]]))
print np.dot(p.bRp(np.pi/2.0,-np.pi/4.0), np.array([[1.0],[0.0],[0.0]]))
print np.dot(p.bRp(np.pi,-np.pi/4.0), np.array([[1.0],[0.0],[0.0]]))
print p.bRa()
