#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

class ValuePublisherNode(Node):
    def __init__(self):
        super().__init__('vel_publisher_node')
        
        self.velocity_publisher = self.create_publisher(Twist, '/demo/cmd_vel', 10)
        self.timer_period = 0.3  # Tiempo en segundos para publicar el valor
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
   
        
    def timer_callback(self):
        
        cmd_vel_msg = Twist()
        
        valor1 =  float(input("Lineal X: "))
        valor2 = float(input("Angular Z: "))
        
        cmd_vel_msg.linear.x = valor1
        cmd_vel_msg.angular.z = valor2

        #print("velocidad angular: ",cmd_vel_msg.angular.z)
        #print("velocidad angular: ",cmd_vel_msg.linear.x)

        self.velocity_publisher.publish(cmd_vel_msg)
        
        self.get_logger().info('Array publicado: [%f, %f]' % (valor1, valor2))
    
    # def presion(self, msg):
    #     cmd_vel_msg = Twist()
    #     self.presion = msg.data

    

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
