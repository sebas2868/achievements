#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32
import serial

def serial_publisher():
    # Inicializar el nodo ROS
    rospy.init_node('serial_publisher_node', anonymous=True)

    # Crear un publicador para el tópico "/presion"
    pub = rospy.Publisher('/presion', Int32, queue_size=10)

    # Configurar el puerto serial
    serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

    rate = rospy.Rate(10)  # Frecuencia de publicación de 10 Hz

    while not rospy.is_shutdown():
        # Leer una línea desde el puerto serial
        line = serial_port.readline().strip()

        try:
            # Convertir la línea a un entero
            presion_value = int(line)

            # Publicar el valor en el tópico "/presion"
            pub.publish(presion_value)

            # Imprimir el valor en la terminal de ROS
            rospy.loginfo("Valor de presion: %d" % presion_value)

        except ValueError:

            # Si no se puede convertir la línea a un entero, imprimir un mensaje de advertencia
            #rospy.logwarn("Cadena no válida recibida desde el puerto serie: %s" % line)
            pass

        rate.sleep()

if __name__ == '__main__':
    try:
        serial_publisher()
    except rospy.ROSInterruptException:
        pass