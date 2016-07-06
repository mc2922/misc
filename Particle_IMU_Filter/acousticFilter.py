
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


# lets see what happen
class theFilter(object):
    def __init__(self):
        #self.N = 500
        self.N = 1
        self.prtObj = particleIMU(self.N)
        self.prtObj.init_sphere_uniform()
        self.viconmsg = None
        self.imumsg = None

        self.prtObj.particles_l[0,0] = 1;
        self.prtObj.particles_l[1,0] = 0;
        self.prtObj.particles_l[2,0] = 0;

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
        lRb, self.prtObj.yaw =  self.prtObj.propNavIMU(0.01, np.matrix(self.imumsg.gyro).T, np.matrix(self.imumsg.accel).T, yaw=self.prtObj.yaw)
        self.prtObj.a_prev = copy.deepcopy(np.matrix(self.imumsg.accel).T)
        if self.viconmsg:
            p = Pose.from_rigid_transform(0, RigidTransform(tvec=self.viconmsg.pos))
            # print 'acc', self.imumsg.accel
            # bRa = self.prtObj.bRa(acc=np.matrix(self.imumsg.accel).T)
            # lRb = self.prtObj.lRb(np.matrix(self.imumsg.accel).T)
            bRl = lRb.T
            rt = RigidTransform.from_Rt( bRl , self.viconmsg.pos)
            print( "R, P, Y, calc_Y: %.3f, %.3f, %.3f, %.3f" % (rt.to_roll_pitch_yaw_x_y_z()[0]*180/np.pi, rt.to_roll_pitch_yaw_x_y_z()[1]*180/np.pi, rt.to_roll_pitch_yaw_x_y_z()[2]*180/np.pi, self.prtObj.yaw*180/np.pi) )
            o = Pose.from_rigid_transform(1, rt)
            # plot accelerometer estimated orientation of IMU
            publish_pose_list('IMUpose', [o], frame_id='origin')

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
        x_axis = np.array([[1],[0],[0]])
        y_axis = np.array([[0],[1],[0]])
        z_axis = np.array([[0],[0],[1]])
        x_bf = np.dot(np.transpose(vRw), x_axis)
        x_bf = np.dot(vRm, x_bf)
        y_bf = np.dot(np.transpose(vRw), y_axis)
        y_bf = np.dot(vRm, y_bf)
        z_bf = np.dot(np.transpose(vRw), z_axis)
        z_bf = np.dot(vRm, z_bf)
        R = tf_construct(x_bf.T, y_bf.T)
        p = Pose.from_rigid_transform(2, RigidTransform.from_Rt(R,self.viconmsg.pos))
        publish_pose_list('VICONpose', [p], frame_id='origin')

    def image_handler(self, channel, data):
        msg = image_t.decode(data)
        # print 'data type', type(msg.data)
        im = np.asarray(bytearray(msg.data), dtype=np.uint8).reshape(msg.height, msg.width)
        # print 'type is', type(im)
        cv2.imshow('img', im)
        cv2.waitKey(1)
        # msg.utime = curr_time
        # msg.width = 360
        # msg.height = 180
        # msg.row_stride = 360
        # msg.pixelformat = 1497715271
        # msg.size = len(h)
        # msg.data = h
        # msg.nmetadata = 0
        # lc.publish("BF_HEATMAP", msg.encode())
        self.prtObj.upStateAcoustics_mod(None)




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
