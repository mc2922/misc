#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import copy

class particleIMU(object):
	def __init__(self, num_particles):
		self.num_particles = num_particles
		self.particles_l = np.zeros([3,self.num_particles])
		self.weights = np.ones([1,self.num_particles])/self.num_particles
		self.yaw = 0.0
		self.a_prev = np.zeros([3,1])

	# rotation about y-axis (pitch)
	def Ry(self, th):
		return np.array([[np.cos(th),0.0,-np.sin(th)],[0.0,1.0,0.0],[np.sin(th),0.0,np.cos(th)]])

	# rotation about z-axis (yaw)
	def Rz(self, th):
		return np.array([[np.cos(th),-np.sin(th),0.0],[np.sin(th),np.cos(th),0.0],[0.0,0.0,1.0]])

	# point-to-body (convert from local point to body point given azimuth and declination)
	def bRp(self, az, th):
		return np.dot(self.Rz(az), self.Ry(th))

	# psi/eta-to-body (convert from local point to body point given azimuth and elevation)
	def bRpe(self, az, el):
		return self.bRp(az, np.pi/2.0-el)

	# psi/eta-point-to-body (convert from local point to body point given azimuth and elevation as vector)
	def bRpe_pt(self, pt):
		return self.bRpe(pt[0], pt[1])

	# body-point-to-psi/eta (convert from body point to azimuth and elevation given a (x,y,z) body point)
	def peRb_pt(self, pt):
		return np.arctan2(pt[0,1], pt[0,0]), np.arctan2(np.linalg.norm(pt[0,0:2]), pt[0,2])

	# accel-to-body (convert from accel/mag to body)
	def bRa(self, acc=np.array([[0.0],[0.0],[9.81]]), mag=np.array([[1.0],[0.0],[0.0]])):
		g = acc/np.linalg.norm(acc)
		m = mag/np.linalg.norm(mag)
		v = np.cross(g.T,m.T)
		v = v/np.linalg.norm(v)
		w = np.cross(v,g.T)
		return np.hstack((w.T, v.T, g))

	# wrap between -PI and +PI
	def wrapRad(self, th):
		if th >= np.pi:
			th = th-2.0*np.pi
		if th < -np.pi:
			th = th+2.0*np.pi
		return th

	# body-to-local given accel
	def lRb(self, accel=None):
		if accel == None:
			return np.dot(self.Rz(self.yaw), self.bRa(self.a_prev))
		else:
			return np.dot(self.Rz(self.yaw), self.bRa(acc=accel))

	# propagate IMU measurements to keep track of yaw in local frame
	def propNavIMU(self, dt, w, a, yaw=0.0):
		bRa_val = self.bRa(a)
		aRb = bRa_val.T
		aGyr = np.dot(aRb,w)
		dAzi = aGyr[2,0]
		yaw = yaw + dAzi*dt
		yaw = self.wrapRad(yaw)
		lRb = np.dot(self.Rz(yaw),aRb)
		return lRb, yaw

	# TO-DO: use magnetometer to constrain IMU-calculated yaw in local frame
	def magUpdateYaw_mod(self, mag, mRef=np.array([[1.0],[0.0],[0.0]])):
		lMag = np.dot(self.lRb(),mag)
		magYaw = np.atan2(lMag[1,0],lMag[0,0])
		dyaw = magYaw - yaw
		#some kind of filter that very slowly pushes yaw to magYaw
		filtdyaw = yawFilter(dyaw)
		self.yaw = self.yaw + filtdyaw

	def yawFilter(self):
		return 0.0

	# given points in azimuth/elevation, convert to points in body frame
	def projPtsPlaneToBodyBall_mod(self, pts, bpts=np.zeros([3,0])):
		cols = pts.shape[1]
		if cols != bpts.shape[1]:
			bpts = np.zeros([3,cols])
		for i in range(0,cols):
			#bpts[:,i] = self.bRpe(pts[:,i])
			bpts[:,i] = np.dot(p.bRpe(pts[:,i]), np.array([[1.0],[0.0],[0.0]]))

	def upStateIMU_mod(self, dt, w, a):
		lRb, yaw = self.propNavIMU(dt, w, a, self.yaw)
		self.a_prev = copy.deepcopy(a)

	def upStateAcoustics_mod(self, heatmap):
		lRb_val = self.lRb()
		bRl_val = lRb_val.T
		N = self.particles_l.shape[1]
		az_el = np.zeros([2,N])
		for i in range(0,N):
			az_el[:,i] = self.peRb_pt(np.dot(bRl_val, self.particles_l[:,i]))
		print az_el*180/np.pi
		print 'THIS IS NOT DONE!!! STOP NOWW'

	#
	# function upStateAcoustics!(x::State, wGrid::Array{Float64,2})
	#   #importance sampling step to incorporate beam forming measurement information
	#   lRb = getlRb(x)
	#   # all particles in body ball
	#   bRl = lRb'
	#   N = size(bpts,2)
	#   azel = zeros(2,N)
	#   for i in 1:N
	#     azel[:,i] = peRb(vec(bRl*x.lParticles[:,i]))
	#   end
	#   # sample importance
	#
	#   #resample (equal weight) -- decrease in inter-particle correlation
	#
	#   # go back to local ball
	#   for i in 1:N
	#     x.lParticles[:,i] = lRb*bRpe(vec(azel[:,i]))
	#   end
	#   nothing
	# end

	### HELPERS ###
	def rad_to_deg(self, rad):
		return rad*180.0/np.pi

	def deg_to_rad(self, deg):
		return deg*np.pi/180.0

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
		closest_phi_idxs = self.find_closest(rad_to_deg(importance_phis), particles_phi)
		closest_theta_idxs = self.find_closest(rad_to_deg(importance_thetas), particles_theta)
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

	def disperse_particles(self, particles_phi, particles_theta, phi_std, theta_std):
		rand_phi = np.random.normal(0, phi_std, particles_phi.shape[0])
		rand_theta = np.random.normal(0, theta_std, particles_theta.shape[0])
		particles_phi += rand_phi
		particles_theta += rand_theta
		particles_phi[particles_phi<0] += 360.0
		particles_phi[particles_phi>360] -= 360.0
		particles_theta[particles_theta<0] = -particles_theta[particles_theta<0]
		particles_theta[particles_theta>180] = 360.0 - particles_theta[particles_theta>180]

p = particleIMU(10)
print np.dot(p.bRp(0.0,0.0), np.array([[1.0],[0.0],[0.0]]))
print np.dot(p.bRp(np.pi/2.0,0.0), np.array([[1.0],[0.0],[0.0]]))
print np.dot(p.bRp(np.pi/2.0,np.pi/4.0), np.array([[1.0],[0.0],[0.0]]))
print np.dot(p.bRp(-np.pi/2.0,np.pi/4.0), np.array([[1.0],[0.0],[0.0]]))
print np.dot(p.bRp(np.pi/2.0,-np.pi/4.0), np.array([[1.0],[0.0],[0.0]]))
print np.dot(p.bRp(np.pi,-np.pi/4.0), np.array([[1.0],[0.0],[0.0]]))
print p.bRa()
