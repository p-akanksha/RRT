#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point,Twist
import math
import time
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry
import numpy as np
import actionlib
from tf.transformations import euler_from_quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

x = 0
y = 0
yaw = 0

def newOdom(msg):
    global x
    global y
    global yaw
 
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
 
    rot_q = msg.pose.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])                       

def findAngle(xi,yi,xf,yf):
    denom = yf-yi
    num = xf-xi
    theta = float(denom/num)
    return math.atan2(denom,num)

def getParams(fname):
    f = open(fname,"r+")
    l = f.readlines()
    l_data = [line.rstrip() for line in l]
    location = []
    for pt in l_data:
        location.append((float(pt.split(' ')[0]),float(pt.split(' ')[1])))

    loc = np.asarray(location)
    #loc = loc/10
    temp = []
    for i in range(1,len(loc)):
        T = findAngle(loc[i-1][0],loc[i-1][1],loc[i][0],loc[i][1])
        temp.append((loc[i][0],loc[i][1],T))
    temp = np.asarray(temp).astype('float32')
    return temp


if __name__ == '__main__':
    try:
        sub = rospy.Subscriber("/changu/odom", Odometry, newOdom)
        pub = rospy.Publisher('/changu/cmd_vel_mux/input/navi', Twist, queue_size=10)
        rospy.init_node('turtle_move', anonymous=True)

        rate = rospy.Rate(10)
        vel_command = Twist()
        #fname = '/home/abhishek/Documents/nodes_optimal_part4.txt'
        fname = '/home/abhishek/Documents/RRT/final_path1.txt'
        femp = getParams(fname)
        
        for p in femp:
            goal_x = p[0]
            goal_y = p[1]
            print(goal_x,goal_y)
            while(True):
                inc_x = goal_x -(x+3.0)
                inc_y = goal_y -(y+3.0)
                angle_to_goal = math.atan2(inc_y, inc_x)
                angle = p[2]
                if abs(angle - yaw)> 0.05:
                    print(angle,'angle_computed')
                    print(yaw,'angle_odom')
                    if angle<yaw:
                        vel_command.angular.z = -0.2
                        vel_command.linear.x = 0.0
                        pub.publish(vel_command)
                    else:
                        vel_command.angular.z = 0.2
                        vel_command.linear.x = 0.0
                        pub.publish(vel_command)
                    #print 'x=', (x+3.5), 'y=',(y+3.0)
                    #rospy.loginfo(vel_command)
                    rate.sleep()
                elif abs(math.sqrt(((goal_x-(x+3.0)) ** 2) + ((goal_y-(y+3.0)) ** 2)))>0.106:
                    print(abs(math.sqrt(((goal_x-(x+3.0)) ** 2) + ((goal_y-(y+3.0)) ** 2))),'distance','changu')
                    vel_command.linear.x = 0.1
                    vel_command.angular.z = 0.0
                    pub.publish(vel_command)
                    #print(abs(math.sqrt(((goal_x-(x+3.5)) ** 2) + ((goal_y-(y+3.0)) ** 2))))
                    print 'x=', (x+3.0), 'y=',(y+3.0), 'moving'
                    #rospy.loginfo(vel_command)
                    rate.sleep()
                else:
                    vel_command.angular.z = 0
                    vel_command.linear.x = 0
                    rospy.loginfo(vel_command)
                    rate.sleep()
                    print('Broken')
                    break

        
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
