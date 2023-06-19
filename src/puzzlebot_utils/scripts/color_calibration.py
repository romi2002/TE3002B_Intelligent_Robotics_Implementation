#!/usr/bin/env python3
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

class CameraRectifier():

    def __init__(self) -> None:
        
        self.image_sub = rospy.Subscriber('/video_source/raw', Image, self.image_cb)
        self.red_mask_ṕub = rospy.Publisher('/traffic_light/red', Image)
        self.yellow_mask_ṕub = rospy.Publisher('/traffic_light/yellow', Image)
        self.green_mask_ṕub = rospy.Publisher('/traffic_light/green', Image)

        self.bridge = CvBridge()

# Creating a window for later use

    def image_cb(self, img_msg):
        rospy.loginfo(img_msg.header)
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, "rgb8")
        
    
        #converting to HSV
        hsv = cv2.cvtColor(cv_image,cv2.COLOR_RGB2HSV)

        # get info from track bar and appy to result

        submask_red_1 = cv2.inRange(hsv,np.array([0,88,179]), np.array([12,255,255]))
        submask_red_2 = cv2.inRange(hsv,np.array([170,88,179]),np.array([180,255,255]))
        red_mask = cv2.bitwise_or(submask_red_1,submask_red_2)

        yellow_mask = cv2.inRange(hsv,np.array([20,50,150]),np.array([33,255,255]))

        green_mask = cv2.inRange(hsv,np.array([49,39,130]),np.array([98,255,255]))

        result_red = cv2.bitwise_and(cv_image,cv_image,mask = red_mask)
        result_yellow = cv2.bitwise_and(cv_image,cv_image,mask = yellow_mask)
        result_green = cv2.bitwise_and(cv_image,cv_image,mask = green_mask)


        self.red_mask_ṕub.publish(self.bridge.cv2_to_imgmsg(result_red, "rgb8"))
        self.yellow_mask_ṕub.publish(self.bridge.cv2_to_imgmsg(result_yellow, "rgb8"))
        self.green_mask_ṕub.publish(self.bridge.cv2_to_imgmsg(result_green, "rgb8"))

if __name__ == '__main__':
    rospy.init_node('cam_rectifier')
    node = CameraRectifier()
    rospy.spin()
    
