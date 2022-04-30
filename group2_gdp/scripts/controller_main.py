#!/usr/bin/python3


import rospy
import pandas as pd
import xlrd
import matplotlib.pyplot as plt
import math
import numpy as np
import pure_pursuit as ctrl
from scipy.interpolate import interp1d
import math
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Odometry

stAngleFreqHz = 0.1
ackermannMsg = AckermannDrive()
show_animation = True

def ack_str(msg): #publish steering command
    global stAnglePublisher
    #rospy.init_node("demo_node", anonymous=True)
    rate = rospy.Rate(200)
    startTime = rospy.Time.now()
    
    currentTime = rospy.Time.now()
    t = float(currentTime.secs) + float(currentTime.nsecs)/(1e9)
    ackermannMsg.steering_angle =msg#+60 to -60
    stAnglePublisher.publish(ackermannMsg)
    #print('Published')
    rate.sleep()

def load_data(msg): #load traj in real tym
    global cx, cy
    cx,cy = [],[]
    cx = msg.pose.pose.position.x
    cy = msg.pose.pose.position.y


def euler_from_quaternion(x, y, z, w):
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        return yaw_z # in radians

def gps_callback(msg):
    
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    orientation_z = msg.pose.pose.orientation.z
    orientation_w = msg.pose.pose.orientation.w
    orientation_x = msg.pose.pose.orientation.x
    orientation_y = msg.pose.pose.orientation.y
    v_x = msg.twist.twist.linear.x
    v_y = msg.twist.twist.linear.y
    v = math.sqrt(v_x**2 + v_y**2)
    yaw = euler_from_quaternion(orientation_x,orientation_y,orientation_z,orientation_w)
    global flag,cx,cy,lastIndex,target_ind, target_speed,state,target_course,time,states
    lastIndex = len(cx) - 1
    if flag ==0:
        # initial state
        #print("Entered")
        time = 0.0
        states = ctrl.States()
        state = ctrl.State(x=x, y=y, yaw=yaw,v=v)
        #print(state.x,state.y,state.yaw,state.v)
        states.append(time, state)
        
        target_course = ctrl.TargetCourse(cx, cy)
        target_ind, _ = target_course.search_target_index(state)
        #T = 100.0  # max simulation time
        #lastIndex = len(cx) - 1

        flag = 1

    
    else:#Main loop
        if lastIndex > target_ind:

            # Calc control input
            print("entered controller")
            ai = ctrl.proportional_control(target_speed, state.v)# v = velocity
            di, target_ind = ctrl.pure_pursuit_steer_control(
                state, target_course, target_ind)
            #state.update(x,y,yaw,v, di) 
            #ros send steer radian
            # ros send steer radian
            
            thrtl_cntrl = interp1d([-5,5],[-1,1])
            thrtl_pos = float(thrtl_cntrl(ai))
            
            ster_deg = math.degrees(di)*-21
            ack_str(ster_deg)
            #ros pos
            #ster_deg = di*1200
            print('ster_deg',ster_deg)
            print('target_ind',target_ind)
            state.update(x,y,yaw,v, di)  # Control vehicle
            #print(state.x,state.y,state.yaw,state.v)
            time += 0.1
            states.append(time, state)

            if show_animation:  # pragma: no cover
                plt.cla()
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect(
                    'key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
                ctrl.plot_arrow(state.x, state.y, state.yaw)
                plt.plot(cx, cy, "-r", label="course")
                plt.plot(states.x, states.y, "-b", label="trajectory")
                plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
                plt.axis("equal")
                plt.grid(True)
                #plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
                plt.pause(0.001)

        # Test
        assert lastIndex >= target_ind, "Cannot goal"

        if show_animation:  # pragma: no cover
            plt.cla()
            plt.plot(cx, cy, ".r", label="course")
            plt.plot(states.x, states.y, "-b", label="trajectory")
            plt.legend()
            plt.xlabel("x[m]")
            plt.ylabel("y[m]")
            plt.axis("equal")
            plt.grid(True)
'''
            plt.subplots(1)
            plt.plot(states.t, [iv * 3.6 for iv in states.v], "-r")
            plt.xlabel("Time[s]")
            plt.ylabel("Speed[km/h]")
            plt.grid(True)
            plt.show()
'''

    


def main():
    global flag,cx,cy,lastIndex,target_speed,stAnglePublisher
    flag = 0
    rospy.init_node('Path_follower', anonymous=True)
    rospy.Subscriber("/gps/odom", Odometry, gps_callback)
    rospy.Subscriber("/Trajectory", Odometry, load_data)
    stAnglePublisher = rospy.Publisher("/CAN/steering_angle_topic", AckermannDrive, queue_size=10)
    #df = pd.read_excel('GPS_coord.xls')
    #cx = np.array(df[0].tolist())
    #cy = np.array(df[1].tolist())
    #plt.plot(cx, cy, "-r", label="course")
    target_speed = 10.0 / 3.6  # [m/s]

    #T = 100.0  # max simulation time
    lastIndex = len(cx) - 1
    print(lastIndex)
    rospy.spin()
    
    



# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    main()

    
    

