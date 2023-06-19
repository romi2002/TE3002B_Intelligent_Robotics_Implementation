#!/usr/bin/env python3
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
import cv2
import requests
import json
import base64
import datetime

class ObjectDetect():

    def __init__(self) -> None:
        # Declare Publishers
        self.image_source_sub = rospy.Subscriber('/video_source/raw', Image, self.image_source_cb)
        self.mask_pub = rospy.Publisher('yolo/mask', Image, queue_size=5)
        self.annotaded_image_pub = rospy.Publisher('yolo/annotated_image', Image, queue_size=5)
        self.classname_pub = rospy.Publisher('yolo/prediction/class_name',String, queue_size=5)
        self.pred_confidence_pub = rospy.Publisher('yolo/prediction/confidence', Float32, queue_size=5)
        self.bridge = CvBridge()

    def parse_image(self, data):
        buff = np.fromstring(data, np.uint8).reshape(1, -1)
        img = cv2.imdecode(buff, cv2.IMREAD_COLOR)
        return np.array(img)
    
    #def decode_image(self, prediction, shape):
        

    def image_source_cb(self, img_msg):
        rospy.loginfo(img_msg.header)

        # Get image from camera
        start = datetime.datetime.now()
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
        #cv_image = cv2.cvtColor(cv_image,cv2.COLOR_RGB2BGR)
        frame = cv2.resize(cv_image, (640, 640))

        # Encode frame and call YOLO API
        enc_img = cv2.imencode('.jpg', frame)[1].tobytes()
        end = datetime.datetime.now()
        rospy.loginfo(f"Prepare time: {(end-start).total_seconds() * 1000}")
        start = datetime.datetime.now()
        ret = requests.get('http://localhost:8000/process_image', data=enc_img)
        #ret = requests.get('http://10.42.0.14:8000/process_image', data=enc_img)
        end = datetime.datetime.now()
        rospy.loginfo(f"Server time: {(end-start).total_seconds() * 1000}")
        #rospy.loginfo(f"{ret.headers.get('data')}")

        # Annotate Image
        # rospy.loginfo("ANNOTATED")
    
        # Decode and Format 
        start = datetime.datetime.now()
        results = json.loads(ret.headers.get('data'))
        dataArray = np.frombuffer(base64.decodestring(bytes(results['prediction'], 'utf-8')),results['prediction-dtype'])
        dataArray = dataArray.reshape(results['prediction-shape'])
        maskArray = np.zeros((640, 640))
        
        classes = results['names']
        if(len(dataArray)>0):
        # Publish predicted classname and confidence
            for prediction in dataArray:
                top_left, bottom_right = (prediction[0], prediction[1]), (prediction[2], prediction[3])
                cv2.rectangle(maskArray, top_left, bottom_right, 255, -1)
                
                classname = classes[str(int(prediction[-1]))]
                confidence = prediction[-2]
                rospy.loginfo(f'{classname}: {confidence}')

                self.classname_pub.publish(classname)
                self.pred_confidence_pub.publish(confidence)
        else:
            self.classname_pub.publish("None")
        # Annotate Image and publish
        annotated_img = self.parse_image(ret.content)
        annotated_img = cv2.cvtColor(annotated_img, cv2.COLOR_BGR2RGB)
        self.annotaded_image_pub.publish(self.bridge.cv2_to_imgmsg(annotated_img, "bgr8"))
        self.mask_pub.publish(self.bridge.cv2_to_imgmsg(cv2.cvtColor(maskArray.astype(np.uint8), cv2.COLOR_GRAY2BGR), "bgr8"))
        end = datetime.datetime.now()
        rospy.loginfo(f"Publish time: {(end-start).total_seconds() * 1000}")



if __name__ == '__main__':
    rospy.init_node('object_detection_node')
    node = ObjectDetect()
    rospy.spin()
    
