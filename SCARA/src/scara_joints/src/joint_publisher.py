#!/usr/bin/env python3

# Este codigo se encarga de leer los valores sensados por el arduino y publicados por el nodo de comunicacion, y de comunicarlas a la herramienta RVIZ2
# Esto se hace mediante el tipo de mensaje JointState o estado de articulacion.

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import numpy as np
import threading

class StatePublisher(Node):
    def __init__(self):
        super().__init__('state_publisher')
        self.joint_pub = self.create_publisher(JointState, 'joint_states', QoSProfile(depth=10))
        self.subscription = self.create_subscription(Float32MultiArray, 'salida_arduino', self.entrada_callback, QoSProfile(depth=10))
        self.timer = self.create_timer(1/60, self.actualizar_joints)
        self.joint_state = JointState()
        self.joint_positions = [0.0] * 6

    def entrada_callback(self, msg):
        data_array = np.array(msg.data,dtype=float)
        data_array *= np.pi/180
        data_array[1] *= 0.180/np.pi
        data_array[4] *= (0.180/np.pi * -0.022/0.090) + 0.022
        print(data_array)
        self.joint_positions = list(data_array) + [data_array[-1]]
        self.get_logger().info(f'Recibido nuevo mensaje de entrada_arduino: {self.joint_positions}')
        self.actualizar_joints()

    def actualizar_joints(self):
        now = self.get_clock().now()
        self.joint_state.header.stamp = now.to_msg()
        self.joint_state.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        self.joint_state.position = self.joint_positions
        self.joint_pub.publish(self.joint_state)

def ros_spin_thread(): 
    global node
    rclpy.spin(node)

def main(args=None):
    global node
    rclpy.init(args=args)
    node = StatePublisher()
    spin_thread = threading.Thread(target=ros_spin_thread)
    spin_thread.start()
    
    try:
        spin_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
