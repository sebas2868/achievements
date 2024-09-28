#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import math
from sensor_msgs.msg import LaserScan

class CV2Subscriber(Node):
    def __init__(self):
        super().__init__('cv2_subscriber')

        
        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/camera/bounding_box', 
            self.network_callback,
            10)
        
        self.odom_subscription = self.create_subscription(Odometry,'/demo/odom',self.odom_callback,20)
        self.servo_publisher = self.create_publisher(Float32MultiArray, '/move/gripper', 10)
        self.velocity_publisher = self.create_publisher(Twist, '/demo/cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan,'/scan',self.scan_callback, 10)
        self.subscription = self.create_subscription(LaserScan,'/scan2',self.scan_callback2, 10)
        self.timer_ = self.create_timer(0.05,self.timer_callback)  # Llama a timer_callback cada 2 segundos

        self.subscription 
        self.yaw = 0
        
        self.neuron = np.zeros((26,2), dtype=np.float32) #arreglo de neuronas

        self.boxes = np.zeros((7,4)) #matriz para promediar
        self.aux_m = 0 #auxiliar para controlar el promedio 
        
        self.diff_theta = 0 #posicion angular
        self.h = 0
        self.ini_current_x = 0
        self.ini_current_y = 0
        self.current_x = 0
        self.current_y = 0
        self.a = 0
        self.b = 0
        self.presion = 0
    
        self.diff_home = 0
        self.pos_error = 0
        self.obstaculo_R = 0
        self.obstaculo_L = 0 #entrada para esquiar obstaculo a la izquierda
        self.ini_yaw = 0
    def euler_from_quaternion(self, orientation):
    
        t3 = +2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        t4 = +1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        yaw_z = math.atan2(t3, t4)
        if yaw_z<0:
            yaw_z += 2*math.pi
     
        return yaw_z # in radians

    def odom_callback(self, msg):
            self.current_x = msg.pose.pose.position.x
            self.current_y = msg.pose.pose.position.y
            yaw = self.euler_from_quaternion(msg.pose.pose.orientation)
            self.yaw = yaw 

    def convert_to_Float32MultiArray(self, data):
        msg = Float32MultiArray()
        if isinstance(data, (int, float)):
            msg.data.append(float(data))
        elif isinstance(data, list):
            msg.data.extend([float(x) for x in data])
        else:
           pass

        return msg

    def scan_callback(self, msg):
        aux = 0
        for i in range(len(msg.ranges)):
            if msg.ranges[i] < 1:
                self.obstaculo_L = 10  #entrada para esquiar obstaculo a la izquierda
               # print(self.obstaculo_L)
                aux +=1
            if aux ==0:
                self.obstaculo_L = 0     

    def scan_callback2(self, msg):
        aux = 0
        for i in range(len(msg.ranges)):
            if msg.ranges[i] < 1 :
                self.obstaculo_R = 10  #entrada para esquiar obstaculo a la izquierda
                aux +=1
            if aux ==0:
                self.obstaculo_R = 0   

  
       # self.get_logger().info("Datos de rango: %s" % str(msg.ranges))

    def network_callback(self, msg):
        self.a = 1
        x1, y1, x2, y2 = msg.data
        self.boxes[self.aux_m, :] = [x1, y1, x2, y2]

        boxes = np.mean(self.boxes, axis=0)
        self.aux_m = (self.aux_m + 1)*(self.aux_m<6)
        x1, y1, x2, y2  = boxes[0], boxes[1], boxes[2], boxes[3]
        center_x = int((x1+x2)/2)
        horizontal_fov = 0.95993
        angle_ball = ((center_x-320) / 640) * horizontal_fov
        self.diff_theta = (angle_ball+self.yaw)-self.yaw
        self.h = round(y2-y1)
             #Condicional de la red
        # print("h: ", self.h)
        # print("theta: ",self.diff_theta)

    
        
        # if self.diff_home<0.1 and self.diff_home>-0.1:
        #         self.diff_home = 0

  

   
    def timer_callback(self):
        if self.a == 0:
            self.diff_theta = (self.yaw - np.pi/4)
            self.diff_theta = self.diff_theta*(self.diff_theta<1)*(self.diff_theta>-1) + 1*(self.diff_theta>1) - 1*(self.diff_theta<-1)

        diff_1=math.atan2(-self.current_y, -self.current_x)
        if diff_1<0:
                diff_1 += 2*math.pi
        self.diff_home = (diff_1-self.yaw)*(diff_1-self.yaw<0.35) + 0.35*(diff_1-self.yaw>0.35)
        #print(self.diff_theta)

        self.pos_error = math.sqrt((self.current_x)**2 + (self.current_y)**2)

        cmd_vel_msg = Twist()
        tau1 = 30 
        umbral = 0.05
        instinto_caza = 0.8

        ac = 5
        Ac = 2
        Bc = 150

        self.neuron[0,1] = max((self.neuron[0,0] + (1/tau1)*(-self.neuron[0,0] + self.neuron[5,0] + self.neuron[14,0]*0.8- self.neuron[1, 0]*4- self.neuron[2, 0]*4 - 100*self.neuron[3, 0]  - self.neuron[19,0] - self.neuron[13,0]*2 - self.neuron[12,0]*2 )), 0) #neurona central

        self.neuron[1,1] = max(self.neuron[1,0] + (1/15)*(-self.neuron[1,0] + self.neuron[8,0] + self.neuron[13,0] - umbral - self.neuron[2, 0]*3 - self.neuron[19,0] ), 0) #giro a la izquierda
        self.neuron[2,1] = max(self.neuron[2,0] + (1/15)*(-self.neuron[2,0] + self.neuron[9,0] + self.neuron[12,0] - umbral - self.neuron[1, 0]*3  - self.neuron[19,0] - 10*self.neuron[21,0]), 0) #giro a la derecha

        self.neuron[3,1] = max(( self.neuron[3,0] + (1/tau1)*(-self.neuron[3,0] - 140 + self.h - 100*self.neuron[7,0])), 0) #neurona paro
        
        self.neuron[4,1] = max(( self.neuron[4,0] + (1/tau1)*(-self.neuron[4,0] - 170 + self.h - 100*self.neuron[7,0] - self.neuron[19,0])), 0)*0.2 #neurona paro

        self.neuron[5,1] = max(( self.neuron[5,0] + (1/tau1)*(-self.neuron[5,0] + instinto_caza - 100*self.neuron[7,0])), 0) #neurona instinto de caza

        self.neuron[6,1] = max(self.neuron[6,0] + (1/tau1)*(-self.neuron[6,0] + self.neuron[3,0] + 6*self.neuron[7,0]- self.neuron[4,0]*20 - 2 - self.neuron[19,0]*500), 0) #neurona recoger
                    
        self.neuron[7,1] = max(self.neuron[7,0] + (1/15)*(-self.neuron[7,0] + (6*(self.neuron[6,0])**2)/((2)**2+(self.neuron[6,0])**2)), 0) #neurona condi_recoger
 
        self.neuron[8,1] = max(self.neuron[8,0] + (1/2)*(-self.neuron[8,0] + self.diff_theta - 100*self.neuron[7,0]),0) #neurona DIFF ANGULAR (pelota-robot)
        self.neuron[9,1] = max(self.neuron[9,0] + (1/2)*(-self.neuron[9,0] + (-1)*self.diff_theta - 100*self.neuron[7,0]),0) #neurona DIFF ANGULAR (robo-pelota)
        
        self.neuron[10,1] = max(self.neuron[10,0] + (1/100)*(-self.neuron[10,0] + self.neuron[17,0]  - 45),0) #neurona DIFF ANGULAR (robo-pelota)

        self.neuron[11,1] = max(self.neuron[11,0] + (1/100)*(-self.neuron[11,0] - 200*self.neuron[10,0] + 4*instinto_caza),0) #neruona condicion retorno
        
        self.neuron[12,1] = max((self.neuron[12,0] + (1/2)*(-self.neuron[12,0] + self.diff_home -self.neuron[13, 0]*20 -200*self.neuron[11,0])),0) #neurona DIFF ANGULAR (home-robot)
        self.neuron[13,1] = max((self.neuron[13,0] + (1/2)*(-self.neuron[13,0] + (-1)*self.diff_home -self.neuron[12, 0]*20 - 200*self.neuron[11,0])),0) #neurona DIFF ANGULAR (home-pelota)

        self.neuron[14,1] = max((self.neuron[14,0] + (1/tau1)*(-self.neuron[14,0] - self.neuron[11,0]*100 + self.pos_error*0.3 - 20*self.neuron[15, 0])),0) #retorno


        self.neuron[15,1] = max((self.neuron[15,0] + (1/tau1)*(-self.neuron[15,0] +1/(self.pos_error+0.001)-2.5)),0) #deterner el regreso

        a = (65*self.neuron[6,0]**2/((10+self.neuron[10,0]*50)**2 + self.neuron[6,0]**2))

        self.neuron[16,1] = max(self.neuron[16,0] + (1/15)*(-self.neuron[16,0] + a),0)#motor vertical
        
        b = (65*self.neuron[6,0]**2/((10+self.neuron[18,0]*50 + self.neuron[19,0]*50)**2 + self.neuron[6,0]**2))
        
        self.neuron[17,1] = max(self.neuron[17,0] + (1/60)*(-self.neuron[17,0]+ b),0)#gripper

        self.neuron[18,1] = max((self.neuron[18,0] + (1/tau1)*(-self.neuron[18,0] + self.neuron[17,0]  - 55)),0) #Simulacion de presion

        self.neuron[19,1] = max((self.neuron[19,0]+(1/tau1)*(-self.neuron[19,0]-self.neuron[14,0]*10-self.neuron[5,0]*2-self.neuron[11,0]*2+ 1+ self.neuron[20,0]*4)),0) #Shutdown
        
        c = (30*round(self.neuron[19,0]**2)/(4+round(self.neuron[19,0])**2))

        self.neuron[20,1] = max((self.neuron[20,0]+(1/tau1)*(-self.neuron[20,0]+c)),0) #Shutdown

        wta = (1*((-ac*self.neuron[22,0]) + self.obstaculo_L**2))/(Bc + ((-ac*(self.neuron[22,0]) + self.obstaculo_L**2)))

        self.neuron[21,1] = max((self.neuron[21,0]+(1/20)*(-self.neuron[21,0] + wta - 10*self.neuron[25,0])),0) #obstaculos_1

        wta2 = (Ac*((-ac*self.neuron[21,0]) + self.obstaculo_R**2))/(Bc + ((-ac*(self.neuron[21,0]) + self.obstaculo_R**2)))

        self.neuron[22,1] = max((self.neuron[22,0]+(1/20)*(-self.neuron[22,0] + wta2 - 10*self.neuron[25,0])),0)  #obstaculos_2


        self.neuron[23,1] = max((self.neuron[23,0]+(1/40)*(-self.neuron[23,0] + self.neuron[23,0]*0.5  + self.neuron[22,0] )),0)  #obstaculos_2

        self.neuron[24,1] = max((self.neuron[24,0]+(1/40)*(-self.neuron[24,0] + self.neuron[24,0]*0.5 + self.neuron[21,0] )),0)  #obstaculos_1

        self.neuron[25,1] = max((self.neuron[25,0]+(1/20)*(-self.neuron[25,0] + self.h -80 - 5*self.neuron[17,1])),0)  #altura

        self.neuron[:,0] = self.neuron[:,1] 

        # cmd_vel_msg.linear.x = float(self.neuron[0,0] - self.neuron[4,0] + self.neuron[23,0]*0.3 + self.neuron[24,0]*0.3)*0.5

        # cmd_vel_msg.angular.z = float(self.neuron[2,0]-self.neuron[1,0]-self.neuron[21,0]*1.2 + self.neuron[22,0]*1.2)*0.5

        # cmd_vel_msg.angular.z = cmd_vel_msg.angular.z*( cmd_vel_msg.angular.z<0.7) + 0.7*(cmd_vel_msg.angular.z>1) 

        # cmd_vel_msg.linear.x = cmd_vel_msg.linear.x*(cmd_vel_msg.linear.x<0.7) + 0.7*(cmd_vel_msg.linear.x>1)

        # print("h", self.h)
        # print("home", self.diff_home)
        print("WTA: ", self.neuron[21,0])

        print("WTA2: ", self.neuron[22,0])

        print("Adelante: ", self.neuron[23,0])
        print("condi: ", self.neuron[25,0])
        #print("vertical", self.neuron[17,1])
        #print("angulo del robot", self.yaw)
        #print("x:", cmd_vel_msg.linear.x)
        #print("z:", cmd_vel_msg.angular.z)

        #print("lado_1", self.neuron[21,0])
        #print("lado_2", self.neuron[22,0])

        #print("acumulador", self.neuron[23,0])

        self.velocity_publisher.publish(cmd_vel_msg)
        neuron_data = [self.neuron[17, 0], self.neuron[16, 0]]
        msg2 = self.convert_to_Float32MultiArray(neuron_data)
        self.servo_publisher.publish(msg2)

def main(args=None):
    rclpy.init(args=None)
    cv2_subscriber = CV2Subscriber()
    rclpy.spin(cv2_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
