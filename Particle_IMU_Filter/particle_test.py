#!/usr/bin/env python

import lcm
from microstrain import ins_t
from vicon import body_t
from bot_core import image_t

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import matplotlib.cm as cm

imu_gyro = None
imu_mag = None
imu_accel = None
imu_quat = None
vicon_pos = None
vicon_orient = None
start_time = None
curr_time = None
prev_seconds = 0
curr_seconds = None

def imu_handler(channel, data):
	global imu_gyro, imu_mag, imu_accel, imu_quat, start_time, curr_time
	msg = ins_t.decode(data)
	imu_gyro = msg.gyro
	imu_mag = msg.mag
	imu_accel = msg.accel
	imu_quat = msg.quat
	if start_time == None:
		start_time = msg.utime
	curr_time = msg.utime

def vicon_handler(channel, data):
	global vicon_pos, vicon_orient
	msg = body_t.decode(data)
	vicon_pos = msg.pos
	vicon_orient = msg.orientation

plt.ion()
ax = plt.gca()

lc = lcm.LCM()
subscription = lc.subscribe("MICROSTRAIN_INS", imu_handler)
subscription = lc.subscribe("VICON_cam_imu", vicon_handler)

vicon_x = []		# vicon position
vicon_y = []
vicon_z = []
x_axes = []			# body frame axes in vicon space
y_axes = []
z_axes = []
source_xvec = []	# fake source position
source_yvec = []
azimuths = np.linspace(0,360,360)*np.pi/180.0
elevations = np.linspace(0,180,180)*np.pi/180.0
# parameters for spherical gaussian
lobe_sharpness = 1
lobe_amplitude = 2
lobe_values = np.zeros([elevations.shape[0],azimuths.shape[0]])
lobe_values_x = np.zeros([elevations.shape[0],azimuths.shape[0]])
lobe_values_y = np.zeros([elevations.shape[0],azimuths.shape[0]])
lobe_values_z = np.zeros([elevations.shape[0],azimuths.shape[0]])
for i, elev in enumerate(elevations):
	for j, azim in enumerate(azimuths):
		lobe_values_x[i,j] = np.sin(elev)*np.cos(azim)
		lobe_values_y[i,j] = np.sin(elev)*np.sin(azim)
		lobe_values_z[i,j] = np.cos(elev)

try:
	while True:
		lc.handle()

		if start_time != None:
			curr_seconds = (curr_time-start_time)*1e-6
		
		''' transform accelerometer data into intertial/local frame, using quaternion defined by vicon '''
		if vicon_orient != None:
			w = vicon_orient[0]
			x = vicon_orient[1]
			y = vicon_orient[2]
			z = vicon_orient[3]

			# quaternion to rotation matrix
			vRw = np.zeros([3,3],dtype=np.double)
			vRw[0,0] = 1 - 2*y**2 - 2*z**2
			vRw[0,1] = 2*x*y - 2*z*w
			vRw[0,2] = 2*x*z + 2*y*w
			vRw[1,0] = 2*x*y + 2*z*w
			vRw[1,1] = 1 - 2*x**2 - 2*z**2
			vRw[1,2] = 2*y*z - 2*x*w
			vRw[2,0] = 2*x*z - 2*y*w
			vRw[2,1] = 2*y*z + 2*x*w
			vRw[2,2] = 1 - 2*x**2 - 2*y**2

			# rotational transformation between microstrain and vicon (collocated)
			vRm = np.zeros([3,3],dtype=np.double)
			vRm[0,0] = 0.747293477674224
			vRm[0,1] = 0.663765523047521
			vRm[0,2] = 0.031109301487093
			vRm[1,0] = 0.663949387400485
			vRm[1,1] = -0.747757418253684
			vRm[1,2] = 0.005482190903960
			vRm[2,0] = 0.026901100276478
			vRm[2,1] = 0.016558196158918
			vRm[2,2] = -0.999500953948458

			if imu_accel != None:
				bf = np.array([[imu_accel[0],imu_accel[1],imu_accel[2]]])
				bf = np.transpose(bf)
				# microstrain - body to vicon -> vicon to world (inertial/local)
				wf = np.dot(np.transpose(vRm), bf)
				wf = np.dot(vRw, wf)

			# standard axes - world (inertial/local) to vicon -> vicon to body
			x_axis = np.array([[1],[0],[0]])
			y_axis = np.array([[0],[1],[0]])
			z_axis = np.array([[0],[0],[1]])
			x_bf = np.dot(np.transpose(vRw), x_axis)
			x_bf = np.dot(vRm, x_bf)
			y_bf = np.dot(np.transpose(vRw), y_axis)
			y_bf = np.dot(vRm, y_bf)
			z_bf = np.dot(np.transpose(vRw), z_axis)
			z_bf = np.dot(vRm, z_bf)

			vicon_x.append(vicon_pos[0])
			vicon_y.append(vicon_pos[1])
			vicon_z.append(vicon_pos[2])

			x_axes.append(x_bf)
			y_axes.append(y_bf)
			z_axes.append(z_bf)

			# lets place a 'source' at (30,30,4) in the world frame - transform this location to the body frame, 
			# then calculate elevation/azimuth to source in this frame; this info can then be used to create synthetic acoustic 'heatmaps'
			# heatmap is calculated every n seconds
			source = np.array([[30-vicon_pos[0]],[30-vicon_pos[1]],[4-vicon_pos[2]]])
			source_bf = np.dot(np.transpose(vRw), source)
			source_bf = np.dot(vRm, source_bf)
			source_xvec.append(source_bf[0,0])
			source_yvec.append(source_bf[1,0])
			if curr_seconds != None:
				if (curr_seconds-prev_seconds > 2):
					prev_seconds = curr_seconds
					r = np.sqrt(source_bf[0,0]**2 + source_bf[1,0]**2 + source_bf[2,0]**2)	#range
					theta = np.arccos(source_bf[2,0]/r)										#elevation
					phi = np.arctan2(source_bf[1,0], source_bf[0,0])						#azimuth
					if phi < 0:
						phi = -phi
					else:
						phi = 2*np.pi - phi
					# create the heatmap as a spherical gaussian centered at this azimuth and elevation, and publish in image_t format
					lobe_values = lobe_values_x*np.sin(theta)*np.cos(phi) + lobe_values_y*np.sin(theta)*np.sin(phi) + lobe_values_z*np.cos(theta)
					lobe_values = lobe_amplitude*np.exp(lobe_sharpness*(lobe_values-1))
					min_lobe = np.min(lobe_values)
					max_lobe = np.max(lobe_values)
					lobe_values = (lobe_values-min_lobe)/(max_lobe-min_lobe)*255.0
					h = lobe_values.astype(int)
					h = np.ndarray.tolist(h.flatten())
					h = [chr(n) for n in h]
					h = ''.join(h)
					msg = image_t()
					msg.utime = curr_time
					msg.width = 360
					msg.height = 180
					msg.row_stride = 360
					msg.pixelformat = 1497715271
					msg.size = len(h)
					msg.data = h
					msg.nmetadata = 0
					lc.publish("BF_HEATMAP", msg.encode())
					print 'elevation:', theta*180/np.pi, 'azimuth:', phi*180/np.pi

except KeyboardInterrupt:
	pass

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

plt.ioff()
ax.plot(vicon_x,vicon_y)
for i in range(0,len(x_axes),200):
	x_axis = x_axes[i]
	u_x = x_axis[0,0]
	v_x = x_axis[1,0]
	y_axis = y_axes[i]
	u_y = y_axis[0,0]
	v_y = y_axis[1,0]
	ax.quiver(vicon_x[i],vicon_y[i],u_x,v_x,color='r')
	ax.quiver(vicon_x[i],vicon_y[i],u_y,v_y,color='g')
plt.axes().set_aspect('equal', 'datalim')

fig = plt.figure()
ax = fig.add_subplot(111)
x = np.array([source_xvec])
x = np.transpose(x)
y = np.array([source_yvec])
y = np.transpose(y)
nth_elem = 50
ax.scatter(x[::nth_elem], y[::nth_elem], color=cm.rainbow(np.linspace(0,1,x[::nth_elem].shape[0])))
plt.axes().set_aspect('equal', 'datalim')
plt.show()