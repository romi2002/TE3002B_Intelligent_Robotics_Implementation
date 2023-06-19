#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from scipy.signal import find_peaks
from geometry_msgs.msg import Twist
import scipy.ndimage as ndi
import datetime

class LineFollower():
    def __init__(self):
        self.image_sub = rospy.Subscriber('/video_source/raw', Image, self.image_cb)
        self.annotated_image_pub = rospy.Publisher('annotated_image', Image, queue_size=10)
        self.annotated_lane_image_pub = rospy.Publisher('annotated_image_intersection', Image, queue_size=10)
        self.vel_pub = rospy.Publisher('lane_follower/cmd_vel', Twist, queue_size=10)
        self.bridge = CvBridge()

        self.dist = np.array([-0.3455606014866875, 0.12255254707345672, 0.006423288264006455, 0.0018066873838785656, 0.0])
        self.mtx = np.array([[766.8269974373959, 0.0, 590.3166384277098], [0.0, 773.2729649552803, 313.14230133393687], [0.0, 0.0, 1.0]])   
        self.w = 1280
        self.h = 720
        self.image_size = (self.w, self.h)
        self.newcameramtx, self.roi = cv2.getOptimalNewCameraMatrix(self.mtx, self.dist, self.image_size, 1, self.image_size) 

        self.kP = 0.25
        self.kD = 0.0
        self.last_error = 0
        self.last_lane_det = 0
        self.i = 0

    def image_cb(self, img_msg):
        #rospy.loginfo(img_msg.header)
        start = datetime.datetime.now()
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
        #cv_image = cv2.resize(cv_image, (640, 480))
        peak, annotated_img = self.process_image_second_approach(cv_image)
        peak = peak[0]
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated_img, "bgr8")
        self.annotated_image_pub.publish(annotated_msg)

        has_lane, lane_img = self.lane_process(cv_image)
        annotated_msg = self.bridge.cv2_to_imgmsg(lane_img, "bgr8")
        self.annotated_lane_image_pub.publish(annotated_msg)

        twist_msg = Twist()
        twist_msg.linear.x = 0.07
        error = (0.5 - peak/250)
        u = error * self.kP + (error - self.last_error) * self.kD
        u = u

        if has_lane:
            u = 0
            twist_msg.linear.x = 0

        twist_msg.angular.z = u
        self.last_error = error
        self.vel_pub.publish(twist_msg)
        rospy.loginfo(f"U: {u} COI: {peak} error: {error}")
        end = datetime.datetime.now()
        delta = end - start
        rospy.loginfo(f"TIME: {delta.total_seconds() * 1000}")


        # Warps points from normalized cords src into normalized coords in dst
    def perspective_warp(self, img,
                        src=np.float32([(0.1,0.5),(0.9,0.5),(0.1,0.8),(0.9,0.8)]),
                        dst=np.float32([(0,0), (1, 0), (0,1), (1,1)])):
        img_size = np.float32([(img.shape[1],img.shape[0])])
        src = src * img_size
        dst = dst * img_size
        M = cv2.getPerspectiveTransform(src, dst)
        warped = cv2.warpPerspective(img, M, (img.shape[1],img.shape[0]))
        return warped
    
    def lane_process(self, frame, annotate_image=True):
        _,  w = frame.shape[:2]
        hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
        hls = hls[100:,100:300]
        #hls = hls[100:,200:500]
        l_channel = hls[:,:,1]
        s_channel = hls[:,:,2]
        
        # Define roi
        _, l_mask = cv2.threshold(l_channel, 120, 255, cv2.THRESH_BINARY_INV)
        _, s_mask = cv2.threshold(s_channel, 30, 255, cv2.THRESH_BINARY)
        mask = cv2.bitwise_and(l_mask, s_mask)

        edges = np.abs(cv2.Sobel(l_channel, ddepth=cv2.CV_64F, dx=0, dy=1, ksize=3)).astype(np.uint8)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, ksize=(3,3))
        _, edges = cv2.threshold(edges, 60, 255, cv2.THRESH_BINARY)
        edges = cv2.dilate(edges, kernel, iterations=1)
        #edges = cv2.GaussianBlur(edges, (5,5),0)
        edges = cv2.bitwise_and(edges, edges, mask=mask)

        # Compute coi from, x_hist, this can be changed to just use edges, to get two axis
        y_hist = np.sum(edges, axis=1).astype(float)
        lane_det = np.max(y_hist)
        is_intersection = lane_det > 10000
        rospy.loginfo(f"{lane_det}")
        center = ndi.center_of_mass(y_hist)
        if center is None:
            # set center to middle
            center = (w/2)

        if not annotate_image:
            return center, frame
        
        annotated_image = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        if is_intersection:
            cv2.line(annotated_image, (0, int(center[0])), (1200, int(center[0])), (255,0,0), 5)
        self.i += 1
        if self.i % 100:
            cv2.imwrite('/tmp/test.png', annotated_image)
        return is_intersection, annotated_image


    def process_image_second_approach(self, frame, annotate_image=True):
        _,  w = frame.shape[:2]
        hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
        l_channel = hls[:,:,1]
        
        # Define roi
        #l_channel = l_channel[300:400,200:450]
        l_channel = l_channel[300:500,200:450]
        _, l_mask = cv2.threshold(l_channel, 120, 255, cv2.THRESH_BINARY_INV)

        edges = np.abs(cv2.Sobel(l_channel, ddepth=cv2.CV_64F, dx=1, dy=0, ksize=3)).astype(np.uint8)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, ksize=(3,3))
        _, edges = cv2.threshold(edges, 50, 255, cv2.THRESH_BINARY)
        #edges = cv2.dilate(edges, kernel, iterations=10)
        #edges = cv2.GaussianBlur(edges, (5,5),0)
        edges *= l_mask.astype(np.uint8)

        # Compute coi from, x_hist, this can be changed to just use edges, to get two axis
        x_hist = np.sum(edges, axis=0).astype(float)
        center = ndi.center_of_mass(x_hist)
        if center is None:
            # set center to middle
            center = (w/2)

        if not annotate_image:
            return center, frame
        
        annotated_image = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        cv2.line(annotated_image, (int(center[0]), 0), (int(center[0]),100), (255,0,0), 5)
        return center, annotated_image

    def process_image(self, image):
        frame = cv2.resize(image, (1280, 720))
        h,  w = frame.shape[:2]
        
        '''frame = cv2.undistort(frame, self.mtx, self.dist, None, self.newcameramtx)'''
        # crop the image
        x, y, w, h = self.roi
        frame = frame[y:y+h, x:x+w]
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
        
        
        perspective_frame = self.perspective_warp(frame)
        #perspective_frame = frame
        l_channel = perspective_frame[:,:,1]

        # TODO Parameter this lol
        mask = cv2.inRange(l_channel, 0, 100)
        
        # TODO parameter this
        x_hist = np.sum(mask[300:,:], axis=0).astype(float)
        x_hist /= np.max(x_hist)
        x_hist[x_hist > 0.5] = 1
        x_hist[x_hist < 0.5] = 0

        lane_start = np.min(np.nonzero(x_hist))
        lane_end = np.max(np.nonzero(x_hist))

        #x_hist = np.sum(mask, axis=0)
        peaks, _ = find_peaks(x_hist)
        
        annotated_img = cv2.cvtColor(perspective_frame.copy(), cv2.COLOR_HLS2BGR)
        first_peak = peaks[0]
        for p in peaks:
            cv2.putText(annotated_img, f"Peak: {p/1280}", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0),2, cv2.LINE_AA)
            cv2.line(annotated_img, (p.astype(int).item(), perspective_frame.shape[0]), (p.astype(int).item(),0), (255,0,0), 5)

        cv2.line(annotated_img, (lane_start, perspective_frame.shape[0]), (lane_start, 0), (0,255,0), 5)
        cv2.line(annotated_img, (lane_end, perspective_frame.shape[0]), (lane_end, 0), (0,0,255), 5)
        annotated_img = cv2.resize(annotated_img, (1280, 720))
        return first_peak, annotated_img


if __name__ == "__main__":
    rospy.init_node('line_follower')
    lineFollower = LineFollower()
    rospy.spin()
