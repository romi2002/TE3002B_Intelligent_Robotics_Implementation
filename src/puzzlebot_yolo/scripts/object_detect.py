#!/usr/bin/env python3
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import requests

class ObjectDetect():

    def __init__(self) -> None:
        
        self.image_source_sub = rospy.Subscriber('/video_source/raw', Image, self.image_source_cb)
        self.annotaded_image_pub = rospy.Publisher('yolo/annotated_image', Image)

        self.bridge = CvBridge()

    def parse_image(self, data):
        buff = np.fromstring(data, np.uint8).reshape(1, -1)
        img = cv2.imdecode(buff, cv2.IMREAD_COLOR)
        return img
    
    def image_source_cb(self, img_msg):
        rospy.loginfo(img_msg.header)
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
        
        frame = cv2.resize(cv_image, (1280, 720))
        enc_img = cv2.imencode('.jpg', frame)[1].tobytes()

        ret = requests.get('http://localhost:8000/process_image', data=enc_img)
        annotated_img = self.parse_image(ret.content)


        self.img_rect_pub.publish(self.bridge.cv2_to_imgmsg(annotated_img, "bgr8"))




if __name__ == '__main__':
    rospy.init_node('object_detection_node')
    node = ObjectDetect()
    rospy.spin()
    
