#!/usr/bin/env python
# coding: UTF-8

'''
利用键盘远程遥控机器人
'''
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import thread
import termios, fcntl, sys, os
import signal
def signal_handler(signal, frame):
    os.system('reset')
    print("Bye.")
signal.signal(signal.SIGINT, signal_handler)

fd = sys.stdin.fileno()

oldterm = termios.tcgetattr(fd)
newattr = termios.tcgetattr(fd)
newattr[3] = newattr[3] & ~termios.ICANON & ~termios.ECHO
termios.tcsetattr(fd, termios.TCSANOW, newattr)

oldflags = fcntl.fcntl(fd, fcntl.F_GETFL)
fcntl.fcntl(fd, fcntl.F_SETFL, oldflags | os.O_NONBLOCK)

rospy.init_node('nav_control', anonymous=True)
cmd_pub = rospy.Publisher('/cmd_vel', Twist , queue_size=0)

current_key = ""
change_unit = 0.1
current_Odom = Odometry()
keyCache = []
upKey = [27, 91, 65]
downKey = [27, 91, 66]
leftKey = [27, 91, 68]
rightKey = [27, 91, 67]
deleteKey = [27, 91, 51, 126]
pagedownKey = [27, 91, 54, 126]

def update_speed(odom):
    global current_Odom
    current_Odom = odom

rospy.Subscriber("/xqserial_server/Odom", Odometry , update_speed)

def sendCmd(key):
    cmd = Twist()
    cmd.linear.x = current_Odom.twist.twist.linear.x
    cmd.angular.z = current_Odom.twist.twist.angular.z
    cmd.linear.x = 0
    cmd.linear.y = 0
    cmd.angular.z = 0
    if key == "up":
        cmd.linear.x = 0.3
    elif key == "down":
        cmd.linear.x = -0.3
    if key == "delete":
        cmd.linear.y = 0.3
    elif key == "pagedown":
        cmd.linear.y = -0.3
    elif key == "left":
        cmd.angular.z = 0.30
    elif key == "right":
        cmd.angular.z = -0.30
    cmd_pub.publish(cmd)

def get_current_key():
    global current_key
    while True:
        try:
            c = sys.stdin.read(1)
            if ord(c) == 27:
                # somestrange pressed
                keyCache = [27]
                continue
            elif len(keyCache) != 0:
                keyCache.append(ord(c))
                if keyCache == leftKey:
                    current_key = "left"
                    keyCache = []
                if keyCache == rightKey:
                    current_key = "right"
                    keyCache = []
                if keyCache == upKey:
                    current_key = "up"
                    keyCache = []
                if keyCache == downKey:
                    current_key = "down"
                    keyCache = []
                if keyCache == deleteKey:
                    current_key = "delete"
                if keyCache == pagedownKey:
                    current_key = "pagedown"
                if len(keyCache) >= 4:
                    keyCache = []
            else:
                current_key = c
        except:
            pass

thread.start_new_thread(get_current_key, ())

rate = rospy.Rate(5)
while not rospy.is_shutdown():
    sendCmd(current_key)
    rate.sleep()
