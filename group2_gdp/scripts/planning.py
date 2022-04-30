#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge() 
import StereoCalibrate
import torch
import numpy as np
import triangulation as tri
from time import time
import math
from ros_yolo.msg import custom
from nav_msgs.msg import Odometry
from matplotlib import pyplot as plt
import catmullron
alpha = 56.7  # Camera field of view in the horisontal plane [degrees]
width = 5

def pub_traj(msgs): #publish trajectory
    global traj_publisher
    #rospy.init_node("demo_node", anonymous=True)
    rate = rospy.Rate(100)
    for msg in msgs:
        Odometry.pose.pose.position.x = msg[0]
        Odometry.pose.pose.position.y = msg[1]
        traj_publisher.publish(msg)
    #print('Published')
    rate.sleep()
def euler_from_quaternion(x, y, z, w): #conversion from quaternion
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        return math.degrees(yaw_z)

def gps_callback(msg):
    global x_pos,y_pos,yaw
    x_pos = msg.pose.pose.position.x
    y_pos = msg.pose.pose.position.y
    orientation_z = msg.pose.pose.orientation.z
    orientation_w = msg.pose.pose.orientation.w
    orientation_x = msg.pose.pose.orientation.x
    orientation_y = msg.pose.pose.orientation.y
    yaw = euler_from_quaternion(orientation_x,orientation_y,orientation_z,orientation_w)

def generate_path(yellow_c, blue_c): #path generator
    n = len(yellow_c)
    m = len(blue_c)
    points = []

    if n >= m:
        for i in range(0, n - 1):

            if yellow_c[i + 1][0] - yellow_c[i][0] == 0:
                angle_right = math.pi / 2
            else:
                angle_right = math.atan2((yellow_c[i + 1][1] - yellow_c[i][1]),
                                        (yellow_c[i + 1][0] - yellow_c[i][0]))
                angle_right = (angle_right + math.pi * 2) % (math.pi * 2)
            angle_right_path = angle_right + math.pi / 2

            if i == 0:
                point_from_right_x = yellow_c[0][0] + math.cos(
                    angle_right_path) * width / 2
                point_from_right_y = yellow_c[0][1] + math.sin(
                    angle_right_path) * width / 2
                path_points = [point_from_right_x, point_from_right_y]
                points.append(path_points)
                point_from_right_x = yellow_c[
                    i + 1][0] + math.cos(angle_right_path) * width / 2
                point_from_right_y = yellow_c[
                    i + 1][1] + math.sin(angle_right_path) * width / 2
            else:
                point_from_right_x = yellow_c[
                    i + 1][0] + math.cos(angle_right_path) * width / 2
                point_from_right_y = yellow_c[
                    i + 1][1] + math.sin(angle_right_path) * width / 2
            path_points = [point_from_right_x, point_from_right_y]
            points.append(path_points)

    else:
        for j in range(0, m - 1):

            if blue_c[j + 1][0] - blue_c[j][0] == 0:
                angle_left = math.pi / 2
            else:
                angle_left = math.atan2((blue_c[j + 1][1] - blue_c[j][1]),
                                        (blue_c[j + 1][0] - blue_c[j][0]))
                # angle_left = (angle_left + math.pi * 2) % math.pi * 2
            angle_left_path = angle_left - math.pi / 2

            if j == 0:
                point_from_left_x = blue_c[0][0] + math.cos(
                    angle_left_path) * width / 2
                point_from_left_y = blue_c[0][1] + math.sin(
                    angle_left_path) * width / 2
                path_points = [point_from_left_x, point_from_left_y]
                points.append(path_points)
                point_from_left_x = blue_c[
                    j + 1][0] + math.cos(angle_left_path) * width / 2
                point_from_left_y = blue_c[
                    j + 1][1] + math.sin(angle_left_path) * width / 2
            else:
                point_from_left_x = blue_c[
                    j + 1][0] + math.cos(angle_left_path) * width / 2
                point_from_left_y = blue_c[
                    j + 1][1] + math.sin(angle_left_path) * width / 2
            path_points = [point_from_left_x, point_from_left_y]
            points.append(path_points)
    return points

def dist(p1,p2):
    return math.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)

def callback(msg): #input from perception node
    global x_pos,y_pos,yaw,traj_last,flag
    if flag==0:
        traj_last = [x_pos,y_pos]
        flag =1
    
    
    L_Y_cone = msg.L_Y_cones
    L_B_cone = msg.L_B_cones
    R_B_cone = msg.R_B_cones
    R_Y_cone = msg.R_Y_cones
    stereo_Y_cone,stereo_B_cone,glob_yx,glob_yy,glob_bx,glob_by = [],[],[],[],[],[]
    try:
        left_cv_image = bridge.imgmsg_to_cv2(msg.left_frame, "8UC3") 
        right_cv_image = bridge.imgmsg_to_cv2(msg.right_frame, "8UC3")

    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
    if L_Y_cone:
        print('L_Y', L_Y_cone[0].x)
    #cv.imshow('Left',left_cv_image)
    #cv.imshow('Right',right_cv_image)
    #cv.waitKey(1)
    for point1 in R_Y_cone: #conversion from local to global coordinate frames
        for point2 in L_Y_cone:
            if abs(point1.y-point2.y)<50:                
                depth = (abs(tri.find_depth(point1.x,point2.x,right_cv_image,left_cv_image)))*10
                ang = (((point1.x-1224)/1224)*(alpha/2))+yaw
                print('Y',depth,ang)
                x = x_pos+((depth*math.cos(math.radians(90-ang)))*0.01)
                y = y_pos+((depth*math.sin(math.radians(90-ang)))*0.01)
                glob_yx.append(x)
                glob_yy.append(y)
                #print([glob_yx,glob_yy])
                stereo_Y_cone.append([x,y])
    for point1 in R_B_cone:
        for point2 in L_B_cone:
            if abs(point1.y-point2.y)<50:
                depth = (abs(tri.find_depth(point1.x,point2.x,right_cv_image,left_cv_image)))*10
                ang = (((point1.x-1224)/1224)*(alpha/2))+yaw
                print('B',depth,ang)
                x = x_pos+((depth*math.cos(math.radians(90-ang)))*0.01)
                y = y_pos+((depth*math.sin(math.radians(90-ang)))*0.01)

                glob_bx.append(x)
                glob_by.append(y)
                #print([glob_bx,glob_by])
                stereo_B_cone.append([x,y])
    print('B_cone',stereo_B_cone)
    print(traj_last)
    if dist([x_pos,y_pos],traj_last)<100:
        print("entered")
        try:
            traj_new = generate_path(stereo_Y_cone,stereo_B_cone)
            traj_last = traj_new[-1]
            print('traj',traj_new)
            pub_traj(traj_new)
        except:
            print('No feasible trajectory')
        
    
    
    plt.scatter(glob_yx,glob_yy,color='red')#plot of 2d map
    plt.scatter(glob_bx,glob_by,color= 'blue')
    plt.scatter(x_pos,y_pos,color= 'green')
    plt.pause(0.04)


if __name__ == '__main__':
    global traj_last,flag, traj_publisher
    flag =0
    
    rospy.init_node('Planner', anonymous=True) 
    
    sub = rospy.Subscriber("/Perception_main", custom, callback)
    sub_gps = rospy.Subscriber("/gps/odom", Odometry, gps_callback)
    traj_publisher = rospy.Publisher("/Trajectory", Odometry, queue_size=10)
    plt.axis([700000, 800000, 4400000, 4700000])
    while not rospy.is_shutdown():
        #r = rospy.Rate(1)
        #r.sleep()
        rospy.spin()






