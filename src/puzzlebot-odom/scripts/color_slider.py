#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def passfunction(x):
    pass

cv2.namedWindow('tracking')
cv2.createTrackbar('LH', 'tracking', 0,255, passfunction)
cv2.createTrackbar('UH', 'tracking', 255,255, passfunction)
cv2.createTrackbar('LS', 'tracking', 0,255, passfunction)
cv2.createTrackbar('US', 'tracking', 255,255, passfunction)
cv2.createTrackbar('LV', 'tracking', 0,255, passfunction)
cv2.createTrackbar('UV', 'tracking', 255,255, passfunction)

bridge = CvBridge()

def image_callback(msg):
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    hsv=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lh = cv2.getTrackbarPos('LH', 'tracking')
    uh = cv2.getTrackbarPos('UH', 'tracking')
    ls = cv2.getTrackbarPos('LS', 'tracking')
    us = cv2.getTrackbarPos('US', 'tracking')
    lv = cv2.getTrackbarPos('LV', 'tracking')
    uv = cv2.getTrackbarPos('UV', 'tracking')
    lower_region= np.array([lh,ls,lv])
    upper_region = np.array([uh,us,uv])
    mask = cv2.inRange(hsv, lower_region, upper_region)
    res = cv2.bitwise_and(frame, frame, mask=mask)
    cv2.imshow('frame', frame)
    cv2.imshow('mask', mask)
    cv2.imshow('res', res)

def main ():
    rospy.init_node('nodo_semaforo2')
    image_sub = rospy.Subscriber('/video_source', Image, image_callback)
    rospy.set_param('~image_transport', 'raw')
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

#Corta el video y cierra la ventana de "Colores"
# cv2.destroyAllWindows()

