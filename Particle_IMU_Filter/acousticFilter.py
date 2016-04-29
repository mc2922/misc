
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

 # The acoustic filter file, with visualization in collections rendered/viewer


# lets see what happen
class theFilter(object):
    def __init__(self):
        self.N = 500
        self.prtObj = particleIMU(self.N)
        self.prtObj.init_sphere_uniform()

    def drawParticles(self, offset=[0,0,0]):
        print 'drawing'
        X = copy.copy(self.prtObj.particles_l.T)
        for i in range(3):
            X[:,i] += offset[i]
        publish_cloud('particles_l', X, c='b', frame_id='origin')
        publish_line_segments('pingerrayGT', offset, [30,30,4], c='r', frame_id='origin')

    def vicon_handler(self,channel, data):
    	global vicon_pos, vicon_orient
    	self.viconmsg = body_t.decode(data)
        self.drawParticles(offset=self.viconmsg.pos)




filt = theFilter()

lc = lcm.LCM()
# subscription = lc.subscribe("MICROSTRAIN_INS", imu_handler)
subscription = lc.subscribe("VICON_cam_imu", filt.vicon_handler)


try:
	while True:
		lc.handle()
except KeyboardInterrupt:
	pass

print 'done'
