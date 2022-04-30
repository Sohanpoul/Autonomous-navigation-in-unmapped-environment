#!/usr/bin/python3


import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

import StereoCalibrate
import torch
import numpy as np
import triangulation as tri
from time import time
from ros_yolo.msg import custom



class msgStruct:
    def __init__(self,x,y,z=0):
        self.x = x
        self.y = y
        self.z = z

class Detection:

    def __init__(self, capture_index, model_name):
        self.bridge = CvBridge()
        self.capture_index = capture_index
        self.model = self.load_model(model_name)
        self.classes = self.model.names
        #print(self.classes)
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        #print("Using Device: ", self.device)
        self.Y_lis = []
        self.B_lis = []
        self.pointY = []
        self.pointB = []

    def load_model(self, model_name):

        if model_name:
            # model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_name, force_reload=True)
            model = torch.hub.load('group2_weight', 'custom', path='best_w3.pt', source='local')
        else:
            model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        return model

    def score_frame(self, frame):

        self.model.to(self.device)
        frame = [frame]
        results = self.model(frame)
        labels, cord = results.xyxyn[0][:, -1], results.xyxyn[0][:, :-1]
        return labels, cord

    def class_to_label(self, x):

        return self.classes[int(x)]

    def plot_boxes(self, results, frame, cameras):

        
        
        labels, cord = results
        print(labels)
        n = len(labels)
        self.pointY,self.pointB = [],[]
        x_shape, y_shape = frame.shape[1], frame.shape[0]
        for i in range(n):
            row = cord[i]
            if row[4] >= 0.65:
                x1, y1, x2, y2 = int(row[0] * x_shape), int(row[1] * y_shape), int(row[2] * x_shape), int(
                    row[3] * y_shape)
                bgr = (0, 255, 0)
                cv2.rectangle(frame, (x1, y1), (x2, y2), bgr, 2)
                cv2.putText(frame, self.class_to_label(labels[i]), (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.9, bgr, 2)

                # x1 = int(xyxy[0].item())
                # y1 = int(xyxy[1].item())
                # x2 = int(xyxy[2].item())
                # y2 = int(xyxy[3].item())
                x_central = (x1 + x2) / 2
                y_central = (y1 + y2) / 2
                # conf = row[4]
                d = (x_central, y_central)  # ,conf)
                cone_class = self.class_to_label(labels[i])
                if cone_class == 'Y_Cone':
                    self.Y_lis.append(d)
                    self.pointY.append(msgStruct(x_central,y_central))
                elif cone_class == 'B_Cone':
                    self.B_lis.append(d)
                    self.pointB.append(msgStruct(x_central,y_central))
                
        '''        
        if cameras == 'L':
            print("Left Camera_Yellow Cone: ", self.Y_lis)
            print("Left Camera_Blue Cone: ", self.B_lis)
        elif cameras == 'R':
            print("Right Camera_Yellow Cone: ", self.Y_lis)
            print("Right Camera_Blue Cone: ", self.B_lis)'''
        return frame

    def image_callback_l(self, img_msg):
        # log some info about the image topic
        # print("printing_LLL")
        # rospy.loginfo(img_msg.header)
        
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")  # "rgb8") # "bgr8")

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        # cv2.imshow('org_image',cv_image)
        # cv2.waitKey(1)
        self.cv_image = StereoCalibrate.CalibrateImage(self.cv_image,'L')
        results = self.score_frame(self.cv_image)
        frame = self.plot_boxes(results, self.cv_image, 'L')
        pubData(self,detector_r)
        #cv2.namedWindow("YOLOv5 Detection_L", 0);
        #cv2.imshow('YOLOv5 Detection_L', frame)
        #cv2.waitKey(1)

    def image_callback_r(self, img_msg):
        # log some info about the image topic
        # print("printing_RRR")
        # rospy.loginfo(img_msg.header)
        
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(img_msg, "rgb8")  # "rgb8") # "bgr8")

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
        # show_image(cv_image)
        self.cv_image = StereoCalibrate.CalibrateImage(self.cv_image,'R')
        results = self.score_frame(self.cv_image)
        frame = self.plot_boxes(results, self.cv_image, 'R')
        
        #cv2.namedWindow("YOLOv5 Detection_R", 0);
        #cv2.imshow("YOLOv5 Detection_R", frame)
        #cv2.waitKey(1)


'''
def despthEstimator(right,left):
    stereo_Y_cone = []
    stereo_B_cone = []
    R_Y_cone = right.Y_lis.sort(key = lambda x:x[1])
    R_B_cone = right.B_lis.sort(key = lambda x:x[1])
    L_Y_cone = left.Y_lis.sort(key = lambda x:x[1])
    L_B_cone = left.B_lis.sort(key = lambda x:x[1])
    print(R_Y_cone)
    for x,y in R_Y_cone:
        for x1,y1 in L_Y_cone:
            if abs(y-y1)<10:
                
                depth = tri.find_depth(x,x1,right.cv_image,left.cv_image)
                stereo_Y_cone.append([x,y,x1,y1,depth])
    for x,y in R_B_cone:
        for x1,y1 in L_B_cone:
            if abs(y-y1)<20:
                depth = tri.find_depth(x,x1,right.cv_image,left.cv_image)
                stereo_B_cone.append([x,y,x1,y1,depth])
               
    print(stereo_Y_cone,stereo_B_cone)'''
    
def pubData(obj1,obj2):
    global pub
    msg = custom()
    msg.L_Y_cones = obj1.pointY
    msg.L_B_cones = obj1.pointB
    msg.R_Y_cones = obj2.pointY
    msg.R_B_cones = obj2.pointB
    msg_lframe = obj1.bridge.cv2_to_imgmsg(obj1.cv_image)
    msg.left_frame = msg_lframe
    msg_rframe = obj2.bridge.cv2_to_imgmsg(obj2.cv_image)
    msg.right_frame = msg_rframe
    pub.publish(msg)

if __name__ == '__main__':
    global pub
    rospy.init_node('Perception', anonymous=True)    
    detector_l = Detection(capture_index=0, model_name='best_w3.pt')
    detector_r = Detection(capture_index=0, model_name='best_w3.pt')
    sub_image_l = rospy.Subscriber("/cameras/left/camera/image_raw", Image, detector_l.image_callback_l)
    sub_image_r = rospy.Subscriber("/cameras/right/camera/image_raw", Image, detector_r.image_callback_r)
    pub = rospy.Publisher('Perception_main', custom,queue_size=10)
    # cv2.namedWindow("Image Window", 1)
    rospy.loginfo("Hello ROS!")

    while not rospy.is_shutdown():
        r = rospy.Rate(100)
        r.sleep()


