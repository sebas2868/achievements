#include"ros/ros.h"
#include"std_msgs/String.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#include <iostream>
#include <cmath>
#include <vector>
#include <chrono>
#include <functional>
#include <memory>

//variables globales
double current_x;
double current_y;
double yaw;
double diff_theta;
double diff_home;
double regreso_stop;
int h;
int a; //variable de control
double neuron_[21][2] = {};
int gripper[2];
int presion;

float euler_from_quaternion(const nav_msgs::Odometry::ConstPtr& msg) {
    float t3 = +2.0 * (msg->pose.pose.orientation.w *msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
    float t4 = +1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
    float yaw_z = std::atan2(t3, t4);
    if (yaw_z < 0) {
        yaw_z += 2.0 * M_PI;
    }
    return yaw_z; // en radianes
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Aquí puedes procesar los datos de odometría recibidos
    current_x =  msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;

    yaw = euler_from_quaternion(msg);
    double diff_1 = std::atan2(-current_y, -current_x);
    if (diff_1 < 0) {
        diff_1 += 2.0 * M_PI;
    }
    diff_home = (diff_1 - yaw) * (diff_1 - yaw < 0.35)*(diff_1 - yaw >-0.35) + 0.35 * (diff_1 - yaw > 0.35) - 0.35 * (diff_1 - yaw <-0.35);
    //ROS_INFO("Odometrya: yaw=%.2f", yaw);
}

void cameracallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    a=1;
    const auto& data = msg->data;
    int x1 = data[0];
    int y1 = data[1];
    int x2 = data[2];
    int y2 = data[3];

    int center_x = static_cast<int>((x1 + x2) / 2);
    float horizontal_fov = 0.95993;
    float angle_ball = ((center_x - 320) / 640.0) * horizontal_fov;
    diff_theta = (angle_ball + yaw) - yaw;
    h = int(y2 - y1);
   
}

void presioncallback(const std_msgs::Int32::ConstPtr& msg) {
    presion = msg->data;
}


int main(int argc, char**argv){
    ros::init(argc, argv, "Publisher");
    ros::init(argc, argv, "odom_subscriber");
    ros::init(argc, argv, "camera_subscriber");
    ros::init(argc, argv, "timer_node");
    ros::init(argc, argv, "gripper_publisher");
    ros::init(argc, argv, "presion");

    ros::NodeHandle nh;

    ros::Publisher topic_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::Subscriber sub = nh.subscribe("/odom", 10, odomCallback);
    ros::Subscriber sub_cam = nh.subscribe("/camera/bounding_box", 10, cameracallback);
    ros::Publisher pub_gripper = nh.advertise<std_msgs::Float32MultiArray>("/move/gripper", 10);
    ros::Subscriber sub_presion = nh.subscribe("/presion", 10, presioncallback);

    ros::Rate loop_rate(1000);

    while(ros::ok()){
    // if (a==0){
    //     diff_theta = yaw - (M_PI/4);
    //     diff_theta = diff_theta*(diff_theta<1)*(diff_theta>-1) + 1*(diff_theta>1) - 1*(diff_theta<-1);
    // }
    ROS_INFO("atura pelota: %i", h);
    float pos_error = std::sqrt(std::pow(current_x, 2) + std::pow(current_y, 2));
    regreso_stop = 1*(0.8>pos_error);
    pos_error = 0*(1.1<pos_error) + 3*(3>pos_error)*(1.1<pos_error);
    //  RCLCPP_INFO(this->get_logger(), "pos_error: %f",  pos_error);
    double tau1 = 20.0;
    double umbral = 0.06;
    double instinto_caza = 0.4;

    neuron_[0][1] = std::max((neuron_[0][0] + (1 / tau1) * (-neuron_[0][0] + neuron_[5][0] + neuron_[14][0] - neuron_[1][0] * 4 - neuron_[2][0] * 4 - 100 * neuron_[3][0] - neuron_[19][0])), 0.0); //neurona central

    neuron_[1][1] = std::max(neuron_[1][0] + (1.0 / 15.0) * (-neuron_[1][0] + neuron_[8][0] + neuron_[13][0] - umbral - neuron_[2][0] * 3 - neuron_[19][0]), 0.0); //giro a la izquierda
    neuron_[2][1] = std::max(neuron_[2][0] + (1.0 / 15.0) * (-neuron_[2][0] + neuron_[9][0] + neuron_[12][0] - umbral - neuron_[1][0] * 3 - neuron_[19][0]), 0.0); //giro a la derecha

    neuron_[3][1] = std::max((neuron_[3][0] + (1.0 / tau1) * (-neuron_[3][0] - 295 + h - 100 * neuron_[7][0])), (double)0.0); //neurona paro

    neuron_[4][1] = std::max((neuron_[4][0] + (1.0 / tau1) * (-neuron_[4][0] - 345 + h - 100 * neuron_[7][0] - neuron_[19][0])), (double)0.0) * 0.015; //neurona paro

    neuron_[5][1] = std::max((neuron_[5][0] + (1.0 / tau1) * (-neuron_[5][0] + instinto_caza - 100 * neuron_[7][0])), (double)0.0); //neurona instinto de caza

    //neuron_[6][1] = std::max(neuron_[6][0] + (1.0 / tau1) * (-neuron_[6][0] + neuron_[3][0] + 6 * neuron_[7][0] - neuron_[4][0] * 20 - 2 - neuron_[19][0] * 500), (double)0.0); //neurona recoger

    //neuron_[7][1] = std::max(neuron_[7][0] + (1.0 / 15.0) * (-neuron_[7][0] + (6 * std::pow(neuron_[6][0], 2)) / (std::pow(2, 2) + std::pow(neuron_[6][0], 2))), (double)0.0); //neurona condi_recoger

    neuron_[8][1] = std::max(neuron_[8][0] + (1.0 / 2.0) * (-neuron_[8][0] + diff_theta - (100*neuron_[7][0])), 0.0); //neurona DIFF ANGULAR (pelota-robot)
    neuron_[9][1] = std::max(neuron_[9][0] + (1.0 / 2.0) * (-neuron_[9][0] + ((-1) * diff_theta) - (100*neuron_[7][0])), 0.0); //neurona DIFF ANGULAR (robo-pelota)

    // neuron_[10][1] = std::max(neuron_[10][0] + (1.0 / 100.0) * (-neuron_[10][0] + neuron_[17][0] - 45), (double)0.0); //neurona DIFF ANGULAR (robo-pelota)

    // neuron_[11][1] = std::max(neuron_[11][0] + (1.0 / 100.0) * (-neuron_[11][0] - 200 * neuron_[10][0] + 4 * instinto_caza), (double)0.0); //neruona condicion retorno

    // neuron_[12][1] = std::max((neuron_[12][0] + (1.0 / 2.0) * (-neuron_[12][0] + diff_home - neuron_[13][0] * 20 - 200 * neuron_[11][0])), (double)0.0); //neurona DIFF ANGULAR (home-robot)
    // neuron_[13][1] = std::max((neuron_[13][0] + (1.0 / 2.0) * (-neuron_[13][0] + (-1) * diff_home - neuron_[12][0] * 20 - 200 * neuron_[11][0])), (double)0.0); //neurona DIFF ANGULAR (home-pelota)

    // neuron_[14][1] = std::max((neuron_[14][0] + (1.0 / 15) * (-neuron_[14][0] - neuron_[11][0] * 100 + pos_error * 0.2 -  2*neuron_[15][0])), (double)0.0); //retorno

    // neuron_[15][1] = std::max((neuron_[15][0] + (1.0 / 15) * (-neuron_[15][0] +  regreso_stop)), (double)0.0); //deterner el regreso

    float a = (65.0 * std::pow(neuron_[6][0], 2) / ((10.0 + neuron_[10][0] * 50.0) * (10.0 + neuron_[10][0] * 50.0) + std::pow(neuron_[6][0], 2)));

    neuron_[16][1] = std::max(neuron_[16][0] + (1.0/ 15.0) * (-neuron_[16][0] + a), (double)0.0); //motor vertical

    float b = (65.0 * std::pow(neuron_[6][0], 2) / ((10.0 + neuron_[18][0] * 50.0 + neuron_[19][0] * 50.0) * (10 + neuron_[18][0] * 50.0 + neuron_[19][0] * 50.0) + std::pow(neuron_[6][0], 2)));

    neuron_[17][1] = std::max(neuron_[17][0] + (1.0 / 60.0) * (-neuron_[17][0] + b), (double)0.0); //gripper

    // neuron_[18][1] = std::max((neuron_[18][0] + (1.0 / tau1) * (-neuron_[18][0] + neuron_[17][0] - 55.0)), (double)0.0); //Simulacion de presion

    // neuron_[19][1] = std::max((neuron_[19][0] + (1 / tau1) * (-neuron_[19][0] - neuron_[14][0] * 10.0 - neuron_[5][0] * 2.0 - neuron_[11][0] * 2.0 + 1.0 + neuron_[20][0] * 4.0)), (double)0.0); //Shutdown

    // float c = (30.0 * round(std::pow(neuron_[19][0], 2)) / (4.0 + round(neuron_[19][0]) * round(neuron_[19][0])));
    // neuron_[20][1] = std::max((neuron_[20][0] + (1.0 / tau1) * (-neuron_[20][0] + c)), (double)0.0); //Shutdown

    for (int i = 0; i < 21; ++i) {
        neuron_[i][0] = neuron_[i][1];
    }
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = (neuron_[0][0] - neuron_[4][0]*0.6) *0.6;
    vel_msg.angular.z = (neuron_[2][0] - neuron_[1][0]) * 0.105;
    topic_pub.publish(vel_msg);
    std_msgs::Float32MultiArray msg2;
    msg2.data.push_back(neuron_[17][0]);
    msg2.data.push_back(neuron_[16][0]);
    pub_gripper.publish(msg2);

    //ROS_INFO("Odometry: x=%.2f, z=%.2f", vel_msg.linear.x,  vel_msg.angular.z);

        ros::spinOnce();
        loop_rate.sleep();
    }
    // Mantenerse a la escucha de los mensajes
    ros::spin();
    return 0;
}
