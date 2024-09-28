#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import serial
import time
import struct
import numpy as np
from serial.tools.list_ports import comports
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
class ArduinoSubcriber(Node):
    def __init__(self):
        super().__init__('arduino_subscriber')


        self.subscription_velocity = self.create_subscription(Twist, '/demo/cmd_vel', self.velocity_callback, 10)

        self.publisher_ = self.create_publisher(Float32MultiArray, '/odometry', 5)
        timer_period = 0.1  # Frecuencia de publicación en segundos (10 Hz)
        self.timer = self.create_timer(timer_period, self.publish_serial_data)
        self.V_r = 0 #velocidad llanta derecha
        self.V_L = 0 #velocidad llanta izquierda
        self. arduinoSerial = serial.Serial(port='/dev/ttyUSB0',
                                    baudrate=115200,
                                    timeout=0.1)

    def publish_serial_data(self):
         linea = self.arduinoSerial.readline().decode('utf-8').strip()
         if linea.startswith('a,'):
           datos = linea.split(',')
           if len(datos) == 4 and datos[0] == 'a':
            try:
                dis_x = float(datos[1])
                dis_y = float(datos[2])
                phi = float(datos[3])
                msg = Float32MultiArray()
                msg.data = [dis_x, dis_y, phi]
                self.publisher_.publish(msg)
                print("Datos válidos recibidos:", dis_x, dis_y, phi)
                # Realiza las acciones que necesites con los datos recibidos
            except ValueError:
                pass
       
        #time.sleep(0.01)

    def velocity_callback(self, msg):
        V = msg.linear.x #velocidad lineal
        w = msg.angular.z  #velocidad angular 
        L = 0.14 #longitud del carro en metros
        r = 0.65*0.5 #radio de la rueda

        self.V_r = V + (w*L)*0.5 #velocidad llanta derecha
        self.V_L = V - (w*L)*0.5 #velocidad llanta izquierda

        rpm_r = (self.V_r*60)/(np.pi*2*r)*(self.V_r>0)
        rpm_l = (self.V_L*60)/(np.pi*2*r)*(self.V_L>0)
        
        if V<0:
            rpm_r = (self.V_r*60)/(np.pi*2*r)
            rpm_l = (self.V_L*60)/(np.pi*2*r)

        motor_left = int((rpm_l/ 114) * 1023) #velocidad del motor de la izquierda
        motor_right = int((rpm_r/114) * 1023) #velocidad del motor de la izquierda

        array_motor = np.array([motor_left, motor_right]).astype(np.float32)
        String_c = struct.pack('2f', *array_motor)
        self.arduinoSerial.write(String_c) 
        print('recibido', str(array_motor))
        #print(x)





if __name__ == '__main__':
    rclpy.init(args=None)
    yolo_subscriber = ArduinoSubcriber()
    rclpy.spin(yolo_subscriber)
    rclpy.shutdown()
