
from pinger_in_body import particleIMU
import numpy as np
import time
import copy

import lcm

import transformations as tf
from microstrain import ins_t
from vicon import body_t
from bot_core import image_t
from bot_geometry.rigid_transform import RigidTransform, Pose
from bot_externals.draw_utils import publish_pose_list, publish_sensor_frame, publish_cloud, publish_line_segments

import cv2

 # The acoustic filter file, with visualization in collections rendered/viewer


# lets see what happen
class theFilter(object):
    def __init__(self):
        self.N = 500
        self.prtObj = particleIMU(self.N)
        self.prtObj.init_sphere_uniform()
        self.viconmsg = None
        self.imumsg = None

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
        if self.viconmsg:
            p = Pose.from_rigid_transform(0, RigidTransform(tvec=self.viconmsg.pos))
            # print 'acc', self.imumsg.accel
            # bRa = self.prtObj.bRa(acc=np.matrix(self.imumsg.accel).T)
            # lRb = self.prtObj.lRb(np.matrix(self.imumsg.accel).T)
            bRl = lRb.T
            rt = RigidTransform.from_Rt( bRl , self.viconmsg.pos)
            print( "R, P, Yy: %.3f, %.3f, %.3f" % (rt.to_roll_pitch_yaw_x_y_z()[0], rt.to_roll_pitch_yaw_x_y_z()[1], self.prtObj.yaw) )
            o = Pose.from_rigid_transform(1, rt)
            publish_pose_list('IMUpose', [p,o], frame_id='origin')

    def vicon_handler(self,channel, data):
    	self.viconmsg = body_t.decode(data)
        self.drawParticles(offset=self.viconmsg.pos)

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
