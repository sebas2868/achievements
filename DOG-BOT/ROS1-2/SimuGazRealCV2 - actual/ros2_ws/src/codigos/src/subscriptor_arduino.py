#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from std_msgs.msg import Int32MultiArray
import serial
import time
import struct
from serial.tools.list_ports import comports

class ArduinoSubcriber(Node):
    def __init__(self):
        super().__init__('arduino_subscriber')

        self.subscription = self.create_subscription(Float32MultiArray, '/move/gripper', self.arduino_callback, 10)

        self.subscription
        self. arduinoSerial = serial.Serial(port='/dev/ttyUSB0',
                                    baudrate=115200,
                                    timeout=0.1)
    def arduino_callback(self, msg):
        numberToSend=msg.data
        x=2*numberToSend[0]+130
        y=0.5*numberToSend[1]+150
        numberToSend[0]=x
        numberToSend[1]=y
        String_c = struct.pack('2f', *numberToSend)
        self.arduinoSerial.write(String_c) 
        print('recibido', str(msg.data))
        time.sleep(0.01)

    #def publish_serial_data(self):
        # Lee el mensaje recibido a través del puerto serial
       # try:
          #  serial_data = self.arduinoSerial.readline().strip().decode('utf-8')
            # Publica el mensaje en el topic 'serial_data'
        #    msg = String()
         #   msg.data = serial_data
          #  self.publisher_.publish(msg)
           # print(msg.data)
       # except:
            #pass



if __name__ == '__main__':
    rclpy.init(args=None)
    yolo_subscriber = ArduinoSubcriber()
    rclpy.spin(yolo_subscriber)
    rclpy.shutdown()
