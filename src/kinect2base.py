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
transform_body=""
pub_cloud=""
num_calc=0
param0=np.array([0.0,0.45])
vector_z=np.array([0.0,-1.0,0.0])
vector_y=np.array([-1.0,0.0,0.0])
vector_x=np.array([0.0,0.0,1.0])
tf_rot=np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
num_calc2=0
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


def callback_kinect(cloud_in):
    global transform_ready, transform_body, pub_cloud, num_calc, param0,vector_temp
    global vector_x, vector_y, vector_z, tf_rot
    global num_calc2
    global kinect_tf
    num_calc2 +=1
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
               "/kinect_link_new",
               "/kinect_depth_optical_frame")
    else:
        #caculate the floor plane equation and average its normal vector
        data_out = pc2.read_points(cloud_in, field_names=None, skip_nans=True, uvs=[])

        num,nums=0,[]
        for p in data_out:
            if p[2]<1.5:
                if num==0:
                    nums.append(p)
                    num += 1
                    # print " x : %f  y: %f  z: %f" %(p[0],p[1],p[2])
                else:
                    yy=nums[num-1][1]
                    zz=nums[num-1][2]
                    if (abs(p[1]-yy)+abs(p[2]-zz))>0.05:
                        nums.append(p)
                        num += 1
                        # print " x : %f  y: %f  z: %f" %(p[0],p[1],p[2])
                if num>600 :
                    print " num : %d " %(num)
                    break
        points=np.array(nums)
        points=np.transpose(points)
        print "points shape:"+str(points.shape)+"\n"
        plsq = leastsq(func, param0, args=(points))
        C, D= plsq[0]
        A=0
        B=-1
        # print "the"+str(num_calc)+": "+str([A,B,C,D])+"\n"
        num_calc +=1
        vector_temp2=np.array([A,B,C])
        lenn=np.linalg.norm(vector_temp2)
        D=D/lenn
        if num_calc==1:
            vector_temp[0]=A
            vector_temp[1]=B
            vector_temp[2]=C
            vector_temp[3]=D
            print "the"+str(num_calc)+" z1: "+str(vector_temp)+"\n"
        else:
            vector_temp =vector_temp + np.array([A,B,C,D])
            print "the"+str(num_calc)+" z2: "+str(vector_temp)+"\n"
        if num_calc>200:
            lenn = np.linalg.norm(vector_temp[0:3])
            vector_z=vector_temp[0:3]/lenn
            height=vector_temp[3]/num_calc
            print "FINAL z: "+str(vector_z)+"\n"
            vector_x=np.cross(vector_y,vector_z)
            tf_rot=np.array([vector_x,vector_y,vector_z])
            print "FINAL R: "+str(tf_rot)+"\n"
            transform_body=TransformStamped()
            transform_body.header.stamp = rospy.Time.now()
            transform_body.header.frame_id = "/kinect_link_new"
            transform_body.child_frame_id = "/kinect_depth_optical_frame"
            M=np.identity(4)
            M[:3,:3]=tf_rot
            q=tf.transformations.quaternion_from_matrix(M)
            transform_body.transform.rotation.x=q[0]
            transform_body.transform.rotation.y=q[1]
            transform_body.transform.rotation.z=q[2]
            transform_body.transform.rotation.w=q[3]
            transform_ready=True
            file_path=os.path.split(os.path.realpath(__file__))[0]
            fp4=open("%s/kinect2base.data"%(file_path),'a+')
            fp4.write(str(tf_rot)+" ")
            fp4.write(str(height)+" ")
            fp4.write("\n")
            fp4.close



def init():
    global pub_cloud, kinect_tf
    rospy.init_node("kinect", anonymous=True)
    rospy.Subscriber("kinect/depth/points", PointCloud2, callback_kinect)
    pub_cloud = rospy.Publisher("kinect/points_rect", PointCloud2,queue_size=5)
    kinect_tf=tf.TransformBroadcaster()

if __name__ == "__main__":
    global t
    init()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()
