#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
import pandas as pd
from std_msgs.msg import Float32

def fax_handler(msg):
    global x,y,x_past
    x.append(msg.pose.pose.position.x)   
    y.append(msg.pose.pose.position.y)
    
    
def str_msgs(data):

    global steer
    steer.append(data.data)


def pub():
    rospy.init_node('Path_generator', anonymous=True)
    rospy.Subscriber("/gps/odom", Odometry, fax_handler)
    rospy.Subscriber("steering_angle_degree", Float32, str_msgs)
    rospy.spin()


if __name__ == '__main__':
    try:
        global x,y,steer,x_past
        x, y,steer = [], [],[]
        x_past = 0
        pub()
        array = [x,y,steer]
        array = pd.DataFrame(array).T
        array.rename(columns={0:0, 1:1,2:2})
        array.to_excel(excel_writer=("GPS_coord.xls"))
        #print(x,y)
        
        
    except rospy.ROSInterruptException:

        pass
