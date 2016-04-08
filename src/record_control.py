#!/usr/bin/env python
# coding: UTF-8

'''
是否录制小车的当前路线，录制后的文件会被保存在 slamdb/path里面。
采用导航模式后小车就会按照给定的路线移动。
'''

import rospy
from geometry_msgs.msg import Pose
import signal
import json

recorded_path = []
is_recording = False

def quit():
    global recorded_path
    print("Saving path record to /home/randoms/slamdb/path.json")
    path_file = open("/home/randoms/slamdb/path.json", "w+")
    path_file.write(json.dumps(recorded_path, indent=4))
    path_file.close()
    print("Bye.")


rospy.init_node('nav_record_control', anonymous=True)

def add_record(pose):
    if not is_recording:
        return
    global recorded_path
    recorded_path.append({
        "x": pose.position.x,
        "y": pose.position.y,
        "z": pose.position.z,
    })


rospy.Subscriber("/ORB_SLAM/Camera", Pose , add_record)

print("press ENTER to start recording, q to quit.")
while not rospy.is_shutdown():
    cmd = raw_input()
    is_recording = not is_recording
    if is_recording:
        print("start recording")
    else:
        print("recording paused")
    if cmd == "q":
        quit()

