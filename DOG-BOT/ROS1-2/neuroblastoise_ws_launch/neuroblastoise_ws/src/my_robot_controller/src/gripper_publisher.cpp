#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>

int main(int argc, char **argv) {
    ros::init(argc, argv, "gripper_publisher");
    ros::NodeHandle nh;

    ros::Publisher publisher = nh.advertise<std_msgs::Float32MultiArray>("/move/gripper", 10);
    ros::Rate loop_rate(1);  // 1 Hz

    while (ros::ok()) {
        std_msgs::Float32MultiArray message;
        float num1, num2;

        std::cout << "Ingrese el primer número: ";
        std::cin >> num1;
        std::cout << "Ingrese el segundo número: ";
        std::cin >> num2;
        
        float neuron_data[] = {num1, num2}; // Corrección 1: Utilizar llaves para inicializar el arreglo

        message.data.clear(); // Corrección 2: Mover esta línea antes de agregar elementos al arreglo
        for (int i = 0; i < 2; ++i) {
            message.data.push_back(neuron_data[i]);
        } // Corrección 3: Agregar un punto y coma al final de esta línea

        publisher.publish(message);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}