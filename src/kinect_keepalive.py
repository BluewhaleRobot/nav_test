#!/usr/bin/env python
#coding:utf-8
'''
this script is designed for calculating the rotation matrix between kinect_depth_optical_frame and kinect_link_new.
kinect_link_new is the kenect mounted frame_id under base_link tf tree
'''
#已知理论y,根据地板点云 计算R
#R: camera_link_new ---> camera_depth_optical_frame
#R=[x y z].transport()
#x=y×z
#z=[A B C D]
# floor plane equation f=A*x+B*y+C*z+D,under camera_depth_optical_frame coordinate
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from roslib import message
import sensor_msgs.point_cloud2 as pc2
import os
import threading
import time,psutil,subprocess,signal

live_flag=False
mStatusLock = threading.Lock()
pointsSub=None

def callback_kinect(cloud_in):
    global live_flag,pointsSub
    mStatusLock.acquire()
    if cloud_in != None:
        live_flag = True
    else:
        live_flag = False
    mStatusLock.release()


def init():
    rospy.init_node("kinect_keeplive", anonymous=True)
    # pointsSub=rospy.Subscriber("kinect/depth/points", PointCloud2, callback_kinect)


if __name__ == "__main__":
    init()
    rate = rospy.Rate(0.05)
    cmd="roslaunch freenect_launch kinect-xyz.launch"
    i=0
    while not rospy.is_shutdown():
        i+=1
        if not live_flag:
            subprocess.Popen(cmd,shell=True)
            if pointsSub==None:
                pointsSub=rospy.Subscriber("kinect/depth/points", PointCloud2, callback_kinect)
                i=0
        else:
            if pointsSub !=None:
                pointsSub.unregister()
                pointsSub=None
        if i >= 3:
            i=0
            if live_flag:
                mStatusLock.acquire()
                live_flag = False
                pointsSub=rospy.Subscriber("kinect/depth/points", PointCloud2, callback_kinect)
                mStatusLock.release()
        rate.sleep()
