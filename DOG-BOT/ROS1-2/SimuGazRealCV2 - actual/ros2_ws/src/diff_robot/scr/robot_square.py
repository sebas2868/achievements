#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist
import time
a=1
ini=time.time()
def move_square():
    rclpy.init()
    node = rclpy.create_node('move_square')
    velocity_publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    while a==1:
        vel_msg = Twist()
        vel_msg.linear.x = 0.2
        vel_msg.angular.z = 0.0
        velocity_publisher.publish(vel_msg)



if __name__ == '__main__':
    
    try:
        move_square()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
