#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
import numpy as np
import math
from skimage import morphology

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        self.publisher_ = self.create_publisher(Int32MultiArray, '/camera/bounding_box', 10)
        self.bridge = CvBridge()
        self.boxes = np.zeros((7,4)) #matriz para promediar
        self.aux_m = 0 #auxiliar para controlar el promedio 

    def publish_bounding(self):
            msg = Int32MultiArray() 
            msg.data = self.array_data
            self.publisher_.publish(msg) #Publica la bounding box
            self.get_logger().info('Publishing array: {}'.format(msg.data))

    def image_callback(self, msg):
        filtered_contours = [] #Arreglo de contornos filtrados

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') 
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        lower_red_dark = np.array([98, 30, 30])
        upper_red_dark = np.array([237, 71, 150])
        lower_red_light = np.array([0, 0, 0])
        upper_red_light = np.array([0, 0, 0])

        # Threshold the HSV image to get only red colors
        mask_dark = cv2.inRange(hsv, lower_red_dark, upper_red_dark)
        mask_light = cv2.inRange(hsv, lower_red_light, upper_red_light)
        # Morphological operations

        mask = cv2.bitwise_or(mask_light,mask_dark)
        res = cv2.bitwise_and(hsv,hsv,mask=mask)
        _,binarized_image = cv2.threshold(cv2.cvtColor(res,cv2.COLOR_BGR2GRAY),1,255,cv2.THRESH_OTSU)
        binarized_image = morphology.remove_small_objects(binarized_image,min_size=200)
        
        # Find contours in the mask
        contours, _ = cv2.findContours(binarized_image.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


        # If contours are found
         #Condicional si encuentra contornos ---->
        if contours:

#---------------FILTRO DE SIMETRÍA DE LA BOUNDING BOX ---------------
            for contours in contours:
                x, y, w, h = cv2.boundingRect(contours)
                if(h/w>0.85 and h/w<1.1): #Rango de los contornos aceptados
                    filtered_contours.append(contours)

        # Get the bounding rectangle of the contour
#Si hay contornos filtrados -----------------------------------
            if filtered_contours:
                max_contour = max(filtered_contours, key=cv2.contourArea) #Encuentra el contorno con mayor área
                
                area = cv2.contourArea(max_contour) 
                if(area > 100): 
                    x, y, w, h = cv2.boundingRect(max_contour)
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2) #Dibujo la bounding box en el frmae
                    
                    #AJUSTE PARA QUE LA SALIDA DE LAS BOUNDING BOX SEAN IGUAL A YOLO
                    w = x+w
                    h = y+h
                    #----------------------------------------------------------
                    self.boxes[self.aux_m, :] = [x, y, w, h]                  
                    boxes = np.mean(self.boxes, axis=0)
                    self.aux_m = (self.aux_m + 1)*(self.aux_m<6)
                    self.array_data = [int(boxes[0]), int(boxes[1]), int(boxes[2]), int(boxes[3])]
                    self.publish_bounding() #Publica la bounding box


        cv2.imshow('Frame', frame) 
        cv2.waitKey(1)  # Esperar un breve período de tiempo para que se muestre la ventana

def nothing(x):
    pass

def main(args=None):
    rclpy.init(args=None)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
