#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

#escalas de colores
# red1_lower = np.array([0, 0, 200])
# red1_upper = np.array([40, 10, 255])
red1_lower = np.array([0, 20, 120])
red1_upper = np.array([15, 255, 255])
# red2_lower = np.array([120, 0, 200])
# red2_upper = np.array([180, 10, 255])
red2_lower = np.array([140, 0, 110])
red2_upper = np.array([180, 255, 255])
yellow_lower = np.array([18, 50, 90])
yellow_upper = np.array([35, 255, 255])
# yellow_lower = np.array([15, 0, 170])
# yellow_upper = np.array([167, 45, 250])
# green_lower = np.array([66, 63, 115])
# green_upper = np.array([91, 255, 255])
green_lower = np.array([50, 59, 60])
green_upper = np.array([80, 255, 255])

#nombres de los colores elegidos, en sus respectivos colores
color_names = {
    "red": (0, 0, 255), 
    "yellow": (0, 255, 255),
    "green": (0, 255, 0)
}

#inizializacion de la variable circle
circle = False

result_image_pub = rospy.Publisher('traffic_light/image', Image, queue_size=10)
red_pub = rospy.Publisher('traffic_light/red', Image, queue_size=10)
yellow_pub = rospy.Publisher('traffic_light/yellow', Image, queue_size=10)
green_pub = rospy.Publisher('traffic_light/green', Image, queue_size=10)
color_detected_pub = rospy.Publisher('traffic_light/color', String, queue_size=10 )
bridge = CvBridge()
mask_frame = np.zeros((1280, 720), np.uint8)

def mask_callback(msg):
    global mask_frame
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY)
    mask_frame = cv2.resize(frame, (1280, 720))

def image_callback(msg):

    original_frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    frame = cv2.GaussianBlur(original_frame, (5,5), 0)

    #se cambia a al espectro de color hsv
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    #mascaras de colores para reconocer el color correcto
    red1_mask = cv2.inRange(hsv, red1_lower, red1_upper)
    red2_mask = cv2.inRange(hsv, red2_lower, red2_upper)
    red_mask = cv2.bitwise_or(red1_mask, red2_mask)
    yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
    green_mask = cv2.inRange(hsv, green_lower, green_upper)

    #inizializacion de variables necesarias
    colors = []
    masks = [red_mask, yellow_mask, green_mask]
    color_names_list = list(color_names.keys())
    contourMax, areaMax, colorMax = None, 0, None
    #for loop que busca circulos del los colores especificados
    for i, mask in enumerate(masks):
        mask = cv2.bitwise_not
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
        erosion = cv2.erode(mask, kernel, 1)

        if(np.sum(erosion)>13000):
            rospy.loginfo(f'{color_names_list[i]} {np.sum(erosion)}')
            color_detected_pub.publish(color_names_list[i])

                
    # if areaMax > 1000:
    #     center = (int(x), int(y))
    #     radius = int(radius)
    #     cv2.circle(frame, center, radius, color_names[color_names_list[i]], 2)
    #     cv2.putText(frame, color_names_list[i], (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color_names[color_names_list[i]], 2)
    #     rospy.loginfo(color_names_list[i])
                
    #Ventana en la que se muestra el video de la camara y los circulos detectados en tiempo real            
    # cv2.imshow("Colores", frame)
    yellow_masked = cv2.cvtColor(yellow_mask.astype(np.uint8), cv2.COLOR_GRAY2BGR)
    green_masked = cv2.cvtColor(green_mask.astype(np.uint8), cv2.COLOR_GRAY2BGR)
    red_masked = cv2.cvtColor(red_mask.astype(np.uint8), cv2.COLOR_GRAY2BGR)
    red_pub.publish(bridge.cv2_to_imgmsg(red_masked,"bgr8"))
    green_pub.publish(bridge.cv2_to_imgmsg(green_masked,"bgr8"))
    yellow_pub.publish(bridge.cv2_to_imgmsg(yellow_masked,"bgr8"))
    result_image_pub.publish(bridge.cv2_to_imgmsg(frame,"bgr8"))
    # cv2.waitKey(1)
    

def main ():
    rospy.init_node('nodo_semaforo')
    rospy.loginfo("Starting Traffic Light Node")
    image_sub = rospy.Subscriber('/video_source/raw', Image, image_callback)
    rospy.Subscriber('/yolo/mask', Image, mask_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

#Corta el video y cierra la ventana de "Colores"
# cv2.destroyAllWindows()

