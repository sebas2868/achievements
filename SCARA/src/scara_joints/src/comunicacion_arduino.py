#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float32MultiArray
import serial
import threading
import numpy as np
import struct
class ComunicacionArduino(Node):

    def __init__(self):
        super().__init__('comunicacion_arduino')
        self.publisher = self.create_publisher(Float32MultiArray, 'salida_arduino', QoSProfile(depth=10))
        self.subscription = self.create_subscription(Float32MultiArray, 'entrada_arduino', self.entrada_callback, QoSProfile(depth=10))

        SERIAL_PORT = '/dev/ttyACM0'  # Cambia esto al puerto correcto para tu sistema
        BAUD_RATE = 115200

        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            self.get_logger().info('Puerto serial abierto con éxito.')
            self.read_thread = threading.Thread(target=self.read_from_serial)
            self.read_thread.start()
        except serial.SerialException as e:
            self.get_logger().error(f'Error al abrir el puerto serial: {e}')

    def entrada_callback(self, msg):
        data_list = msg.data

        if len(data_list) != 5:
            self.get_logger().error('Número de valores incorrecto en el mensaje recibido.')
            return

        try:
            # Empaquetar los 5 floats en una sola cadena de bytes
            packed_data = struct.pack('fffff', *data_list)
            # Enviar todos los datos de una vez
            self.ser.write(packed_data)
            self.get_logger().info(f"Enviando comando: {data_list}")
        except serial.SerialException as e:
            self.get_logger().error(f'Error al enviar datos: {e}')

    def read_from_serial(self):
        while rclpy.ok():
            try:
                if self.ser.in_waiting > 0:
                    incoming_data = self.ser.readline().decode('ascii', errors='ignore').strip()
                    
                    # Manejo de mensajes de error enviados desde Arduino
                    if "Error en el comando" in incoming_data or "Ángulo del servo fuera de rango" in incoming_data:
                        self.get_logger().error(f'Error recibido del Arduino: {incoming_data}')
                        continue  # Saltar procesamiento si es un mensaje de error

                    self.get_logger().info(f'Datos recibidos del Arduino: {incoming_data}')
                    try:
                        # Procesar los datos recibidos si es necesario
                        data = np.fromstring(incoming_data.split(": ")[1], sep=',')
                        data[0] *= -1
                        msg = Float32MultiArray(data=data.tolist())
                        self.publisher.publish(msg)
                    except ValueError:
                        self.get_logger().error('Error al procesar los datos recibidos.')
            except serial.SerialException as e:
                self.get_logger().error(f'Error al leer datos: {e}')
                break

def ros_spin_thread(): 
    global node
    rclpy.spin(node)

def main(args=None):
    global node
    rclpy.init(args=args)
    node = ComunicacionArduino()
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
