#!/usr/bin/env python3
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2

class CameraRectifier():

    def __init__(self) -> None:
        
        self.image_sub = rospy.Subscriber('/video_source/raw', Image, self.image_cb)
        self.img_rect_pub = rospy.Publisher('/video_source/rect', Image, queue_size=10)

        self.bridge = CvBridge()

        self.dist = np.array([-0.312486, 0.095021, 0.003731, 0.001468, 0.000000])
        self.mtx = np.array([[800.981014, 0.000000, 629.098491], [0.000000, 806.539594, 364.049324], [0.0, 0.0, 1.0]])   
        self.w = 1280
        self.h = 720
        self.image_size = (self.w, self.h)
        self.newcameramtx, self.roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, self.image_size, 1, self.image_size) 

    def image_cb(self, img_msg):
        rospy.loginfo(img_msg.header)
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
        
        frame = cv2.resize(cv_image, (1280, 720))
        
        frame = cv2.undistort(cv_image, self.mtx, self.dist, 1, self.newcameramtx)

        self.img_rect_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))



if __name__ == '__main__':
    rospy.init_node('cam_rectifier')
    node = CameraRectifier()
    rospy.spin()
    