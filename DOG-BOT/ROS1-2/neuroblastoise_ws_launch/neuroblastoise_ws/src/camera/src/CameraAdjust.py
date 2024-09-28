#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import Int32MultiArray
import numpy as np

class CameraPublisher:
    def __init__(self):
        rospy.init_node('camera_publisher')
        self.cameraDeviceNumber=0 #Setup del puerto de la cámara
        self.camera = cv2.VideoCapture(self.cameraDeviceNumber) #Uso cv2 para capturar el video de la cámara
        
        self.publisher_ = rospy.Publisher('/camera/bounding_box',Int32MultiArray, queue_size=10) #Publisher de la boundung box

        # Publisher de imagen ---------------------------------- Borrar si no es necesario
        self.bridgeObject = CvBridge() 
        self.topicNameFrames='topic_camera_image' #Topico del frame
        self.queueSize = 20
        self.publisher = rospy.Publisher(self.topicNameFrames, Image, queue_size=self.queueSize)
        
        self.periodComunication = 0.03

        self.timer = rospy.Timer(rospy.Duration(self.periodComunication), self.image_callback)
        # ------------------------------------------------------------------------------ 

        self.i = 0 #contador de publicaciones


    def remove_small_objects(self, image, min_size):
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(image, connectivity=8)
        for label in range(1, num_labels):
            if stats[label, cv2.CC_STAT_AREA] < min_size:
                labels[labels == label] = 0
        return np.uint8(labels > 0) * 255

    def image_callback(self, event):
        
        
        succes, frame = self.camera.read() #Lee el frame de la cámara
        frame = cv2.resize(frame, (640,480), interpolation=cv2.INTER_CUBIC) #Tamaño requerido del lider
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_h_dark = cv2.getTrackbarPos('LowerD H', 'Trackbars')
        lower_s_dark = cv2.getTrackbarPos('LowerD S', 'Trackbars')
        lower_v_dark = cv2.getTrackbarPos('LowerD V', 'Trackbars')
        upper_h_dark = cv2.getTrackbarPos('UpperD H', 'Trackbars')
        upper_s_dark = cv2.getTrackbarPos('UpperD S', 'Trackbars')
        upper_v_dark = cv2.getTrackbarPos('UpperD V', 'Trackbars')

        lower_h_light = cv2.getTrackbarPos('LowerL H', 'Trackbars')
        lower_s_light = cv2.getTrackbarPos('LowerL S', 'Trackbars')
        lower_v_light = cv2.getTrackbarPos('LowerL V', 'Trackbars')
        upper_h_light = cv2.getTrackbarPos('UpperL H', 'Trackbars')
        upper_s_light = cv2.getTrackbarPos('UpperL S', 'Trackbars')
        upper_v_light = cv2.getTrackbarPos('UpperL V', 'Trackbars')


        lower_red_dark = np.array([lower_h_dark, lower_s_dark, lower_v_dark])
        upper_red_dark = np.array([upper_h_dark, upper_s_dark, upper_v_dark])

        lower_red_light = np.array([lower_h_light, lower_s_light, lower_v_light])
        upper_red_light = np.array([upper_h_light, upper_s_light, upper_v_light])

        # Threshold the HSV image to get only red colors
        mask_dark = cv2.inRange(hsv, lower_red_dark, upper_red_dark)
        mask_light = cv2.inRange(hsv, lower_red_light, upper_red_light)
        # Morphological operations

        mask = cv2.bitwise_or(mask_light,mask_dark)

        res = cv2.bitwise_and(hsv,hsv,mask=mask)

        

        _,binarized_image = cv2.threshold(cv2.cvtColor(res,cv2.COLOR_BGR2GRAY),1,255,cv2.THRESH_OTSU)
        binarized_image = self.remove_small_objects(binarized_image, min_size=100)        

        # Find contours in the mask
        contours, _ = cv2.findContours(binarized_image.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)




        # If contours are found
        if contours:
            # Find the largest contour
            max_contour = max(contours, key=cv2.contourArea)
                    # Find the largest contour
            
            
            # Get the bounding rectangle of the contour
            x, y, w, h = cv2.boundingRect(max_contour)
            
            # Draw the bounding box on the original image
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            # Calculate the centroid using the moments of the contour
            M = cv2.moments(max_contour)
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            
            # Draw the centroid on the original frame
            cv2.circle(frame, (cx, cy), 10, (0, 255, 0), -1)

            bounding_box_array = [x, y, w, h, cx, cy]

            # Publish the bounding box array
            msg = Int32MultiArray(data=bounding_box_array)
            self.publisher_.publish(msg)

            #self.get_logger().info(f"Bounding box published: {bounding_box_array}")  # Add logging statement, uncomment when the red ball is used

        cv2.imshow('Frame', binarized_image) 
        cv2.waitKey(1)  # Esperar un breve período de tiempo para que se muestre la ventana

        #resized_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        #self.publisher_.publish(resized_msg)

        if succes == True:
            ROS2ImageMessage=self.bridgeObject.cv2_to_imgmsg(frame) 
            self.publisher.publish(ROS2ImageMessage)
        
        rospy.loginfo("Received camera frame.")
        
def nothing(x):
    pass


if __name__ == '__main__':

    try:
        cv2.namedWindow('Frame')
        cv2.namedWindow('Trackbars')


        # Create trackbars for adjusting lower and upper bounds of red color
        cv2.createTrackbar('LowerD H', 'Trackbars', 0, 255, nothing)
        cv2.createTrackbar('LowerD S', 'Trackbars', 0, 255, nothing)
        cv2.createTrackbar('LowerD V', 'Trackbars', 0, 255, nothing)
        cv2.createTrackbar('UpperD H', 'Trackbars', 0, 255, nothing)
        cv2.createTrackbar('UpperD S', 'Trackbars', 0, 255, nothing)
        cv2.createTrackbar('UpperD V', 'Trackbars', 0, 255, nothing)

        cv2.createTrackbar('LowerL H', 'Trackbars', 0, 255, nothing)
        cv2.createTrackbar('LowerL S', 'Trackbars', 0, 255, nothing)
        cv2.createTrackbar('LowerL V', 'Trackbars', 0, 255, nothing)
        cv2.createTrackbar('UpperL H', 'Trackbars', 0, 255, nothing)
        cv2.createTrackbar('UpperL S', 'Trackbars', 0, 255, nothing)
        cv2.createTrackbar('UpperL V', 'Trackbars', 0, 255, nothing)
        camera_publisher = CameraPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
