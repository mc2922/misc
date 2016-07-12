#!/usr/bin/env python

import thread
import time

import lcm
from microstrain import ins_t

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


lc = lcm.LCM()
uts = 0

class IMUUnitTest:
    def __init__(self):
        self.prtObj = particleIMU(1)
        R = np.eye(3)
        self.prtObj.set_bRi(self.prtObj.Rx(np.pi))

    def my_handler(self, channel, data):
        global uts
        firstpass = False
        if uts < 1:
            firstpass = True
        msg = ins_t.decode(data)
        dt = (msg.utime - uts)*1e-6
        uts = msg.utime
        if firstpass:
            return
        bRi = self.prtObj.bRi
        acc = np.array(msg.accel)[:,np.newaxis]
        gyr = np.array(msg.gyro)[:,np.newaxis]
        acc = np.dot(bRi, acc)
        gyr = np.dot(bRi, gyr)
        self.prtObj.yaw = self.prtObj.propNavIMU(dt, gyr, acc, self.prtObj.yaw)
        self.prtObj.a_prev[:,0:1] = acc
        aRb = self.prtObj.bRa(acc=np.array(msg.accel)[:,np.newaxis]).T
        rt = RigidTransform.from_Rt( aRb , [0,0,0])
        o = Pose.from_rigid_transform(1, rt)
        publish_pose_list('aRb', [o], frame_id='origin')
        lRb = self.prtObj.lRb()
        rt2 = RigidTransform.from_Rt( lRb , [0,0,0])
        o2 = Pose.from_rigid_transform(2, rt2)
        publish_pose_list('lRb', [o2], frame_id='origin')


def IMU_loop(threadname):
  global lc
  ut01 = IMUUnitTest()
  subscription = lc.subscribe("MICROSTRAIN_INS", ut01.my_handler)
  while True:
		lc.handle()


# Create thread
try:
  thread.start_new_thread( IMU_loop, ("Thread-1", ) )
except:
  print "Error: unable to start thread"

try:
  while 1:
    raw_input("")
    #print uts
except KeyboardInterrupt:
	print "Finishing"

print "times"
print times
