#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class ValuePublisherNode(Node):
    def __init__(self):
        super().__init__('value_publisher_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/move/gripper', 10)
        self.subscription = self.create_subscription(Float32MultiArray, '/Presion', self.presion, 10)
        self.timer_period = 0.3  # Tiempo en segundos para publicar el valor
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.inix = 150.0
        self.iniy = 150.0
        self.presion = 0
    def timer_callback(self):
        valor1 =  float(input("Ingrese un valor [x]: "))
        valor2 = float(input("Ingrese un valor [y]: "))
        msg = Float32MultiArray()
        msg.data = [valor1, valor2]
        self.publisher_.publish(msg)
        self.get_logger().info('Array publicado: [%f, %f]' % (valor1, valor2))
        self.inix += 1.
        self.iniy += 1.5
    
    def presion(self, msg):
        self.presion = msg.data

    

def main(args=None):
    rclpy.init(args=args)
    node = ValuePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
