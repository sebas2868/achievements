#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <fstream>
#include <vector>
#include <cstring>

void topic_callback(const std_msgs::Float32MultiArray::ConstPtr& msg) {


    std::ofstream arduino_serial("/dev/ttyUSB0", std::ios::binary);
    
    // Verifica si el tamaño del mensaje es correcto
    if (msg->data.size() != 2) {
        ROS_ERROR("El tamaño del mensaje no es 2");
        return;
    }
    
    // Guarda los datos del mensaje en una variable llamada numberToSend
    float numberToSend[2];
    numberToSend[0] = msg->data[0];
    numberToSend[1] = msg->data[1];

    // Asigna las posiciones 0 y 1 del arreglo a x y y respectivamente
    float x = numberToSend[0];
    float y = numberToSend[1];
    
    // Realizamo manipulación de los datos con la ecuacion de la recta
    x = 2 * x + 130;
    y = 0.5 * y + 150;

    // Actualiza los valores de x e y en el arreglo numberToSend
    numberToSend[0] = x;
    numberToSend[1] = y;
    
    // Por ahora, simplemente imprime los valores de x e y
    //ROS_INFO("x = %f, y = %f", x, y);
    
    
    // Copiar los datos del arreglo numberToSend en un buffer de bytes
    char buffer[sizeof(float) * 2];
    std::memcpy(buffer, numberToSend, sizeof(float) * 2);
    
    arduino_serial.write(buffer, sizeof(buffer));
    arduino_serial.close();

    // Imprime un mensaje para confirmar la recepción de datos
    //ROS_INFO("Datos recibidos y enviados por el puerto serial");

}



int main(int argc, char **argv) {
    ros::init(argc, argv, "esp32_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber subscriber = nh.subscribe("/move/gripper", 10, topic_callback);

    ros::spin();

    return 0;
}