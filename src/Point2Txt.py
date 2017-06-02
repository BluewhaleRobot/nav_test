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
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import numpy as  np
import scipy.linalg
from scipy.optimize import leastsq
from geometry_msgs.msg import TransformStamped
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import os

nums_frame=0
fp4=None

def callback_kinect(cloud_in):
        global nums_frame
        global fp4
        #caculate the floor plane equation and average its normal vector
        nums_frame +=1
        # nums=cloud_in.row_step/8
        print "current frame num : %d  \n" %(nums_frame)
        # for field1 in cloud_in.fields:
        #     print "field:"+field1.name+"\n"
        data_out = pc2.read_points(cloud_in, field_names=None, skip_nans=True, uvs=[])

        fp4.write('     x          y            z        instensity   ring\n')
        nums=0
        for p in data_out:
            # nums.append(p)
            fp4.write("%-12.6f %-12.6f %-12.6f %-12.6f %-3.0f\n" %(p[0],p[1],p[2],p[3],p[4]))
            nums+=1
        fp4.write('# New slice ,Array shape: {0}\n'.format(str(nums)+" 5"))
        if nums_frame%5==0:
            fp4.flush()


def init():
    global fp4
    file_path=os.path.split(os.path.realpath(__file__))[0]
    fp4=open("%s/points.txt"%(file_path),'a+')

    rospy.init_node("sensor", anonymous=True)
    rospy.Subscriber("/Sensor/points", PointCloud2, callback_kinect)

if __name__ == "__main__":
    init()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()
    # fp4.close
