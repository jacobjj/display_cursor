#!/usr/bin/env python

import rospy
import time
import numpy as np
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray

class haptic_pos:
    def __init__(self):
        self.hd_vel = np.zeros((3,1))
        self.hd_ang_vel = np.zeros((3,1))
        self.hd_transform = np.zeros((4,4))
        self.hd_position = np.zeros((3,1))
        self.baxter_transform = np.asarray([
                                [0,0,1,0],
                                [1,0,0,0],
                                [0,1,0,0],
                                [0,0,0,1]
                                ])
        rospy.Subscriber('pose_msg',Float64MultiArray,self.callback)
    def callback(self,data_stream):
        self.hd_transform = np.reshape(data_stream.data[0:16],(4,4),order='F')
        self.hd_transform = np.matmul(self.baxter_transform,self.hd_transform)
        self.hd_vel = np.asarray(data_stream.data[16:19])
        self.hd_vel = np.matmul(self.baxter_transform[0:3,0:3],self.hd_vel)
        self.hd_ang_vel = np.asarray(data_stream.data[19:22])
        self.hd_ang_vel = np.matmul(self.baxter_transform[0:3,0:3],self.hd_ang_vel)
        self.hd_position = np.asarray(data_stream.data[22:25])


def listener():
    rospy.init_node("haptic_rec")
    phantom = haptic_pos()
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        print("Transform Matrix: {}".format(phantom.hd_transform))
        print("HD Velocity: {}".format(phantom.hd_vel))
        print("HD angular velocity: {}".format(phantom.hd_ang_vel))
        rate.sleep()

if __name__ == '__main__':
    listener()
