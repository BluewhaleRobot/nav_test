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

transform_ready=False
param0=np.array([0.0,0.10])
vector_z=np.array([0.0,-1.0,0.0])
vector_y=np.array([-1.0,0.0,0.0])
vector_x=np.array([0.0,0.0,1.0])
tf_rot=np.array([[0.,0.06435858,0.99792684],[-1.,0.,0.],[0.,-0.99792684, 0.06435858]])
kinect_tf=None
vector_temp=np.array([0.0,0.0,0.0,0.0])

def func(param,point):
    global num_calc2
    #print "point shape"+str(num_calc2)+": "+str(point.shape)+"\n"
    """
    平面拟合误差函数，其实就是平面方程,A固定为零，不参与计算,B固定为-1
    """
    C, D = param
    B=-1
    yy= point[1]
    zz= point[2]
    return B*yy+C*zz+D

def pub_tf():
    global transform_ready, transform_body, param0,vector_temp
    global vector_x, vector_y, vector_z, tf_rot
    global kinect_tf
    if transform_ready :
    #     #publish the transformed clouds
    #    transform_body.header.seq = transform_body.header.seq+1
    #    transform_body.header.stamp = rospy.Time.now()
    #    cloud_out = do_transform_cloud(cloud_in, transform_body)
    #    pub_cloud.publish(cloud_out)
       if kinect_tf != None:
           now=rospy.Time.now();
           T=np.array([0.0,0.0,0.0])
           M=np.identity(4)
           M[:3,:3]=tf_rot.T
           q=tf.transformations.quaternion_from_matrix(M)
           kinect_tf.sendTransform(T,q,
               now,
               "/camera_link_new",
               "/ORB_SLAM/World")


def quaternionCross(q1,q2):
    #return q3=q1*q2
    v1=q1[0:3]
    w1=q1[3]
    v2=q2[0:3]
    w2=q2[3]
    v3=np.cross(v1,v2)+w2*v1+w1*v2
    w3=w1*w2-v1.dot(v2)
    return np.hstack((v3,w3))

def calculat_cam():
    global transform_ready, transform_body, param0,vector_temp
    global vector_x, vector_y, vector_z, tf_rot
    global kinect_tf
    num,nums=0,[]
    if not transform_ready :
        with open("/home/xiaoqiang/.ros/KeyFrameTrajectory.txt",'r') as f:
            data = f.readlines()
            for line in data:
                odom = line.split()
                numbers_float = map(float,odom)
                p=numbers_float[1:4]
                nums.append(p)
                num += 1
    else:
        return

    #             print p
    # return
    f.close
    points=np.array(nums)
    points=np.transpose(points)
    print "points1 shape:"+str(points.shape)+"\n"
    vector_tempA1=np.array([0.,0.,0.])
    vector_tempA2=np.array([0.,0.,0.])
    vector_tempA3=np.array([0.,0.,0.])
    num2=num/3
    for i in range(num2):
        vector_tempA1=vector_tempA1+points[0:3,i]
        vector_tempA2=vector_tempA2+points[0:3,i+num2]
        vector_tempA3=vector_tempA3+points[0:3,i+num2*2]
    vector_tempA1=vector_tempA1/num2
    vector_tempA2=vector_tempA2/num2
    vector_tempA3=vector_tempA3/num2

    vector_tempB1=vector_tempA2-vector_tempA1
    vector_tempB2=vector_tempA3-vector_tempA1
    vector_tempB3=np.cross(vector_tempB1,vector_tempB2)
    # vector_tempB3[0]=0.0
    lenn=np.linalg.norm(vector_tempB3)
    vector_tempB3=vector_tempB3/lenn
    print vector_tempB3

    num,nums=0,[]
    q=np.array([0,0,0,1])
    qinv=np.array([0,0,0,1])
    with open("/home/xiaoqiang/.ros/KeyFrameTrajectory.txt",'r') as f:
        data = f.readlines()
        for line in data:
            odom = line.split()
            numbers_float = map(float,odom)
            p=numbers_float[1:4]
            if num == 3:
                qinv=np.array(numbers_float[4:8])
                q=np.array([-numbers_float[4],-numbers_float[5],-numbers_float[6],numbers_float[7]])
            if nums<4:
                continue
            p_q=np.array([numbers_float[1],numbers_float[2],numbers_float[3],0])
            q3=quaternionCross(p_q,qinv)
            q4=quaternionCross(q,q3)
            q5=q4.tolist()
            nums.append(q5[0:3])
            num += 1
    f.close
    points=np.array(nums)
    points=np.transpose(points)
    print "points1 shape:"+str(points.shape)+"\n"
    plsq = leastsq(func, param0, args=(points))
    C, D= plsq[0]
    A=0
    B=-1
    # print "the"+str(num_calc)+": "+str([A,B,C,D])+"\n"
    vector_temp2=np.array([A,B,C])
    lenn=np.linalg.norm(vector_temp2)
    D=D/lenn
    num_calc=1
    if num_calc==1:
        vector_temp[0]=A
        vector_temp[1]=B
        vector_temp[2]=C
        vector_temp[3]=D
        print "the"+str(num_calc)+" z1: "+str(vector_temp)+"\n"
    else:
        vector_temp =vector_temp + np.array([A,B,C,D])
        print "the"+str(num_calc)+" z2: "+str(vector_temp)+"\n"
    if num_calc>0:
        lenn = np.linalg.norm(vector_temp[0:3])
        vector_z=vector_temp[0:3]/lenn
        height=vector_temp[3]/num_calc
        print "FINAL z: "+str(vector_z)+"\n"
        vector_x=np.cross(vector_y,vector_z)
        tf_rot=np.array([vector_x,vector_y,vector_z])
        print "FINAL R: "+str(tf_rot)+"\n"
        file_path=os.path.split(os.path.realpath(__file__))[0]
        fp4=open("%s/cam2base.data"%(file_path),'a+')
        fp4.write(str(tf_rot)+" ")
        fp4.write(str(height)+" ")
        fp4.write("\n")
        fp4.close
        transform_ready=True


def init():
    global pub_cloud, kinect_tf
    rospy.init_node("cam", anonymous=True)
    kinect_tf=tf.TransformBroadcaster()
    calculat_cam()

if __name__ == "__main__":
    global t
    init()
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        pub_tf()
        rate.sleep()
