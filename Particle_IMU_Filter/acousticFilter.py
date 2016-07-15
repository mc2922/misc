
from pinger_in_body import particleIMU
import numpy as np
import time
import copy

import lcm

import transformations as tf
from microstrain import ins_t
from vicon import body_t
from bot_core import image_t
from bot_geometry.rigid_transform import RigidTransform, Pose, tf_construct
from bot_externals.draw_utils import publish_pose_list, publish_sensor_frame, publish_cloud, publish_line_segments

import cv2

 # The acoustic filter file, with visualization in collections rendered/viewer
azimuths = np.linspace(0,360,360)*np.pi/180.0
elevations = np.linspace(0,180,180)*np.pi/180.0
# parameters for spherical gaussian
lobe_sharpness = 1.5#1
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

# lets see what happen
class theFilter(object):
    def __init__(self):
        self.N = 500
        self.prtObj = particleIMU(self.N)
        self.prtObj.init_sphere_uniform()
        self.viconmsg = None
        self.imumsg = None
        self.counter = 0
        self.imu_max = np.zeros([3,1])
        self.vicon_max = np.zeros([3,1])
        self.vicon_lRb = np.eye(3)

        # all points at (1,0,0)
        # self.prtObj.particles_l[0,:] = 1.0
        # self.prtObj.particles_l[1,:] = 0.0
        # self.prtObj.particles_l[2,:] = 0.0

        # transformation tests (5 points)
        # self.prtObj.particles_l[:,0:1] = np.array([[np.sin(0*np.pi/180.0)],[0.0],[np.cos(0*np.pi/180.0)]])
        # self.prtObj.particles_l[:,1:2] = np.array([[np.sin(30*np.pi/180.0)],[0.0],[np.cos(30*np.pi/180.0)]])
        # self.prtObj.particles_l[:,2:3] = np.array([[np.sin(45*np.pi/180.0)],[0.0],[np.cos(45*np.pi/180.0)]])
        # self.prtObj.particles_l[:,3:4] = np.array([[np.sin(60*np.pi/180.0)],[0.0],[np.cos(60*np.pi/180.0)]])
        # self.prtObj.particles_l[:,4:5] = np.array([[np.sin(90*np.pi/180.0)],[0.0],[np.cos(90*np.pi/180.0)]])

    def drawParticles(self, offset=[0,0,0]):
        X = copy.copy(self.prtObj.particles_l.T)
        for i in range(3):
            X[:,i] += offset[i]
        rl = np.matrix(self.prtObj.weights)*self.prtObj.num_particles
        C = np.hstack(    ( rl.T , np.zeros((self.N,2)) )   )
        publish_cloud('particles_l', X, c=C, frame_id='origin')
        publish_line_segments('pingerrayGT', offset, [30,30,4], c='r', frame_id='origin')

    def imu_handler(self, channel, data):
    	self.imumsg = ins_t.decode(data)
        #lRb, self.prtObj.yaw =  self.prtObj.propNavIMU(0.01, np.array(self.imumsg.gyro)[:,np.newaxis], np.array(self.imumsg.accel)[:,np.newaxis], yaw=self.prtObj.yaw)
        self.prtObj.upStateIMU_mod(0.01, np.array(self.imumsg.gyro)[:,np.newaxis], np.array(self.imumsg.accel)[:,np.newaxis])
        lRb = self.prtObj.lRb()
        #self.prtObj.a_prev[:,0] = self.imumsg.accel
        if self.viconmsg:
            p = Pose.from_rigid_transform(0, RigidTransform(tvec=self.viconmsg.pos))
            # print 'acc', self.imumsg.accel
            # bRa = self.prtObj.bRa(acc=np.matrix(self.imumsg.accel).T)
            # lRb = self.prtObj.lRb(np.matrix(self.imumsg.accel).T)
            bRl = lRb.T
            rt = RigidTransform.from_Rt(lRb , self.viconmsg.pos) # bRl
            #print( "R, P, Y, calc_Y: %.3f, %.3f, %.3f, %.3f" % (rt.to_roll_pitch_yaw_x_y_z()[0]*180/np.pi, rt.to_roll_pitch_yaw_x_y_z()[1]*180/np.pi, rt.to_roll_pitch_yaw_x_y_z()[2]*180/np.pi, self.prtObj.yaw*180/np.pi) )
            o = Pose.from_rigid_transform(1, rt)
            # plot accelerometer estimated orientation of IMU
            publish_pose_list('Localpose', [p], frame_id='origin')
            publish_pose_list('IMUpose', [o], frame_id='origin')
            #publish_cloud('particle_imu_max', self.imu_max.T+self.viconmsg.pos, c='b', frame_id='origin')

            #test magnetometer
            lMag = self.prtObj.magUpdateYaw_mod(np.array(self.imumsg.mag)[:,np.newaxis])
            #lMag = self.prtObj.magUpdateYaw_mod(np.array([[1.0],[0.0],[0.0]]))
            #lMag /= np.linalg.norm(lMag)
            #lMag = np.dot(bRl, np.array([[1.0],[0.0],[0.0]]))
            publish_cloud('mag', np.array(self.imumsg.mag)+self.viconmsg.pos, c='b', frame_id='origin')

    def gen_heatmap(self, phi, theta):
        if self.counter%120 == 0:
            lobe_values = lobe_values_x*np.sin(theta)*np.cos(phi) + lobe_values_y*np.sin(theta)*np.sin(phi) + lobe_values_z*np.cos(theta)
            lobe_values = 0.5*lobe_amplitude*np.exp(lobe_sharpness*(lobe_values-1))

            phi0 = np.pi
            theta0 = np.pi/2.0
            lobe_values0 = lobe_values_x*np.sin(theta0)*np.cos(phi0) + lobe_values_y*np.sin(theta0)*np.sin(phi0) + lobe_values_z*np.cos(theta0)
            lobe_values += 0.75*lobe_amplitude*np.exp(lobe_sharpness*(lobe_values0-1))

            min_lobe = np.min(lobe_values)
            max_lobe = np.max(lobe_values)
            lobe_values = (lobe_values-min_lobe)/(max_lobe-min_lobe)*255.0
            h = lobe_values.astype(int)
            h = np.ndarray.tolist(h.flatten())
            h = [chr(n) for n in h]
            h = ''.join(h)
            msg = image_t()
            msg.utime = self.counter*1e5/12.0
            msg.width = 360
            msg.height = 180
            msg.row_stride = 360
            msg.pixelformat = 1497715271
            msg.size = len(h)
            msg.data = h
            msg.nmetadata = 0
            #lc.publish("BF_HEATMAP_ONLINE", msg.encode())
            #print 'elevation:', theta*180/np.pi, 'azimuth:', phi*180/np.pi
            lc.publish("BF_HEATMAP", msg.encode())
        self.counter += 1

    def vicon_handler(self,channel, data):
    	self.viconmsg = body_t.decode(data)
        self.drawParticles(offset=self.viconmsg.pos)

        # plot true orientation of IMU via vicon
        vicon_orient = self.viconmsg.orientation
        w = vicon_orient[0]
        x = vicon_orient[1]
        y = vicon_orient[2]
        z = vicon_orient[3]
        # quaternion to rotation matrix
        wRv = np.zeros([3,3],dtype=np.double)
        wRv[0,0] = 1 - 2*y**2 - 2*z**2
        wRv[0,1] = 2*x*y - 2*z*w
        wRv[0,2] = 2*x*z + 2*y*w
        wRv[1,0] = 2*x*y + 2*z*w
        wRv[1,1] = 1 - 2*x**2 - 2*z**2
        wRv[1,2] = 2*y*z - 2*x*w
        wRv[2,0] = 2*x*z - 2*y*w
        wRv[2,1] = 2*y*z + 2*x*w
        wRv[2,2] = 1 - 2*x**2 - 2*y**2
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
        wRm = np.dot(wRv, vRm)
        self.vicon_lRb = wRm
        p = Pose.from_rigid_transform(2, RigidTransform.from_Rt(wRm,self.viconmsg.pos))
        publish_pose_list('VICONpose', [p], frame_id='origin')

        #drawing true max as point in new image and in local frame
        if self.imumsg != None:
            im2 = np.zeros([180,360])
            PV = np.array([[30],[30],[4]]) - np.array(self.viconmsg.pos)[:,np.newaxis]
            self.vicon_max = PV/(np.linalg.norm(PV))
            publish_cloud('particle_vicon_max', self.vicon_max.T+self.viconmsg.pos, c='g', frame_id='origin')
            # lRb_val = self.prtObj.lRb()
            # bRl_val = lRb_val.T
            # dot_prod = np.dot(bRl_val, self.vicon_max)        # use IMU local-to-body transform
            dot_prod = np.dot(self.vicon_lRb.T, self.vicon_max)   # use vicon local-to-body transform
            az_el = self.prtObj.peRb_pt(dot_prod)
            if az_el[0] < 0:
                az_el[0] = az_el[0]+2*np.pi
            # print az_el*180/np.pi
            self.gen_heatmap(az_el[0], az_el[1])
            cv2.circle(im2, (int(az_el[0]*180/np.pi),int(az_el[1]*180/np.pi)), 5, (255,0,0))
            cv2.imshow('img2', im2)
            cv2.waitKey(1)

            #self.prtObj.particles_l[:,0:1] = self.vicon_max

    def image_handler(self, channel, data):
        msg = image_t.decode(data)
        im = np.asarray(bytearray(msg.data), dtype=np.uint8).reshape(msg.height, msg.width)
        bf_heatmap = im/255.0;

        # get maximum value in heatmap, push to variable for plotting in local frame
        el,az = np.unravel_index(im.argmax(), im.shape)
        az_el = np.array([[az*np.pi/180.0],[el*np.pi/180.0]])
        cv2.circle(im, (az,el), 5, (0,0,0))
        lRb_val = self.prtObj.lRb()
        self.imu_max = np.dot(lRb_val.T, np.matrix(self.prtObj.bRpe_pt(az_el)))

        self.prtObj.upStateAcoustics_mod(bf_heatmap, im)

        # print 'VICON azimuth elevation transforms:'
        for i in range(0,self.N):
            t = self.prtObj.peRb_pt(np.dot(self.vicon_lRb.T, self.prtObj.particles_l[:,i:i+1]))   # local all the way to plane
            if t[0,0] < 0:
                t[0,0] = t[0,0]+2*np.pi
            # print t[0,0]*180/np.pi, t[1,0]*180/np.pi
            # cv2.circle(im, (int(t[0,0]*180/np.pi),int(t[1,0]*180/np.pi)), 3, (0,0,0))

        cv2.imshow('img', im)
        cv2.waitKey(10)



filt = theFilter()

lc = lcm.LCM()
subscription = lc.subscribe("MICROSTRAIN_INS", filt.imu_handler)
subscription = lc.subscribe("VICON_cam_imu", filt.vicon_handler)
subscription = lc.subscribe("BF_HEATMAP", filt.image_handler)


try:
	while True:
		lc.handle()
except KeyboardInterrupt:
    cv2.destroyAllWindows()

print 'done'
