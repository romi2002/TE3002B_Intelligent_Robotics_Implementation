#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

#escalas de colores
red1_lower = np.array([0, 100, 20])
red1_upper = np.array([10, 255, 255])
red2_lower = np.array([160, 100, 20])
red2_upper = np.array([179, 255, 255])
yellow_lower = np.array([20, 100, 100])
yellow_upper = np.array([30, 255, 255])
green_lower = np.array([25, 90, 90])
green_upper = np.array([95, 255, 255])

#nombres de los colores elegidos, en sus respectivos colores
color_names = {
    "ROJO": (0, 0, 255), 
    "AMARILLO": (0, 255, 255),
    "VERDE": (0, 255, 0)
}

#inizializacion de la variable circle
circle = False

result_image_pub = rospy.Publisher('traffic_light/image', Image)
bridge = CvBridge()



def image_callback(msg):

    frame = bridge.imgmsg_to_cv2(msg, "bgr8")

    #se cambia a al espectro de color hsv
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    #mascaras de colores para reconocer el color correcto
    red1_mask = cv2.inRange(hsv, red1_lower, red1_upper)
    red2_mask = cv2.inRange(hsv, red2_lower, red2_upper)
    red_mask = cv2.bitwise_or(red1_mask, red2_mask)
    yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
    green_mask = cv2.inRange(hsv, green_lower, green_upper)

    #inizializacion de variables necesarias
    contours = []
    colors = []
    masks = [red_mask, yellow_mask, green_mask]
    color_names_list = ["ROJO", "AMARILLO", "VERDE"]

    #for loop que busca circulos del los colores especificados
    for i, mask in enumerate(masks):
        color_contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in color_contours:
            area = cv2.contourArea(contour)
            if area > 100:
                (x, y), radius = cv2.minEnclosingCircle(contour)
                if radius > 10:
                    center = (int(x), int(y))
                    radius = int(radius)
                    contours.append(contour)
                    colors.append(color_names_list[i])
                    cv2.circle(frame, center, radius, color_names[color_names_list[i]], 2)
                    cv2.putText(frame, color_names_list[i], (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color_names[color_names_list[i]], 2)
                    rospy.loginfo(color_names_list[i])
                    rospy.loginfo("----------------")
                    break
                
    #Ventana en la que se muestra el video de la camara y los circulos detectados en tiempo real            
    # cv2.imshow("Colores", frame)
    result_image_pub.publish(bridge.cv2_to_imgmsg(frame,"bgr8"))
    # cv2.waitKey(1)
    

def main ():
    rospy.init_node('nodo_semaforo')
    rospy.loginfo("Starting Traffic Light Node")
    image_sub = rospy.Subscriber('/video_source/raw', Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

#Corta el video y cierra la ventana de "Colores"
# cv2.destroyAllWindows()