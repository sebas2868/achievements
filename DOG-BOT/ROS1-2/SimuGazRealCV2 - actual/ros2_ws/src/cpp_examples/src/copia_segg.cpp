#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>
#include <cmath>
#include <vector>
#include <chrono>
#include <functional>
#include <memory>


using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
    public:
        MinimalSubscriber()
            : Node("minimal_subscriber")
        {
            subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/camera/bounding_box", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));

            odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/demo/odom", 10, std::bind(&MinimalSubscriber::odom_callback, this, _1));

            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/demo/cmd_vel", 10);

            publisher_gripper_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/move/gripper", 10);
                
            // DEF: rclcpp::WallTimer< CallbackT >::SharedPtr create_wall_timer(	std::chrono::duration< int64_t, DurationT > period, CallbackT callback)
            timer_ = this->create_wall_timer(std::chrono::milliseconds(80), std::bind(&MinimalSubscriber::timer_callback, this));

            
        }

    private:
        double current_x;
        double current_y;
        double yaw;
        double diff_theta;
        double diff_home;
        double regreso_stop;
        int h;
        double neuron_[21][2] = {};
        int gripper[2];
        // Definir el tipo de dato para la matriz
        
        void topic_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) 
        {
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
        //std::cout << "h: " << h << std::endl;
        //std::cout << "theta: " << diff_theta << std::endl;

       
        }

           void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) 
        {
            current_x =  msg->pose.pose.position.x;
            current_y = msg->pose.pose.position.y;
            yaw = euler_from_quaternion(msg);

            double diff_1 = std::atan2(-current_y, -current_x);
            if (diff_1 < 0) {
                diff_1 += 2.0 * M_PI;
            }
            diff_home = (diff_1 - yaw) * (diff_1 - yaw < 0.35)*(diff_1 - yaw >-0.35) + 0.35 * (diff_1 - yaw > 0.35) - 0.35 * (diff_1 - yaw <-0.35);
            // RCLCPP_INFO(this->get_logger(), "diff_theta: %f",  diff_1);
            // RCLCPP_INFO(this->get_logger(), "yaw: %f",  yaw);
            // RCLCPP_INFO(this->get_logger(), "diff_home: %f",  diff_home);
            //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", ss.str().c_str());
        }

        float euler_from_quaternion(const nav_msgs::msg::Odometry::SharedPtr msg) {
            float t3 = +2.0 * (msg->pose.pose.orientation.w *msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
            float t4 = +1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
            float yaw_z = std::atan2(t3, t4);
            if (yaw_z < 0) {
                yaw_z += 2.0 * M_PI;
            }
            return yaw_z; // en radianes
           
        }

        void timer_callback()
        {
            float pos_error = std::sqrt(std::pow(current_x, 2) + std::pow(current_y, 2));
            regreso_stop = 1*(0.8>pos_error);
            pos_error = 0*(1.1<pos_error) + 3*(3>pos_error)*(1.1<pos_error);
            RCLCPP_INFO(this->get_logger(), "pos_error: %f",  pos_error);
            geometry_msgs::msg::Twist cmd_vel_msg;
            double tau1 = 30.0;
            double umbral = 0.05;
            double instinto_caza = 0.4;

            neuron_[0][1] = std::max((neuron_[0][0] + (1 / tau1) * (-neuron_[0][0] + neuron_[5][0] + neuron_[14][0] - neuron_[1][0] * 4 - neuron_[2][0] * 4 - 100 * neuron_[3][0] - neuron_[19][0])), 0.0); //neurona central

            neuron_[1][1] = std::max(neuron_[1][0] + (1.0 / 15.0) * (-neuron_[1][0] + neuron_[8][0] + neuron_[13][0] - umbral - neuron_[2][0] * 3 - neuron_[19][0]), 0.0); //giro a la izquierda
            neuron_[2][1] = std::max(neuron_[2][0] + (1.0 / 15.0) * (-neuron_[2][0] + neuron_[9][0] + neuron_[12][0] - umbral - neuron_[1][0] * 3 - neuron_[19][0]), 0.0); //giro a la derecha

            neuron_[3][1] = std::max((neuron_[3][0] + (1.0 / tau1) * (-neuron_[3][0] - 220 + h - 100 * neuron_[7][0])), (double)0.0); //neurona paro

            neuron_[4][1] = std::max((neuron_[4][0] + (1.0 / tau1) * (-neuron_[4][0] - 240 + h - 100 * neuron_[7][0] - neuron_[19][0])), (double)0.0) * 0.2; //neurona paro

            neuron_[5][1] = std::max((neuron_[5][0] + (1.0 / tau1) * (-neuron_[5][0] + instinto_caza - 100 * neuron_[7][0])), (double)0.0); //neurona instinto de caza

            neuron_[6][1] = std::max(neuron_[6][0] + (1.0 / tau1) * (-neuron_[6][0] + neuron_[3][0] + 6 * neuron_[7][0] - neuron_[4][0] * 20 - 2 - neuron_[19][0] * 500), (double)0.0); //neurona recoger

            neuron_[7][1] = std::max(neuron_[7][0] + (1.0 / 15.0) * (-neuron_[7][0] + (6 * std::pow(neuron_[6][0], 2)) / (std::pow(2, 2) + std::pow(neuron_[6][0], 2))), (double)0.0); //neurona condi_recoger

            neuron_[8][1] = std::max(neuron_[8][0] + (1.0 / 2.0) * (-neuron_[8][0] + diff_theta - (100*neuron_[7][0])), 0.0); //neurona DIFF ANGULAR (pelota-robot)
            neuron_[9][1] = std::max(neuron_[9][0] + (1.0 / 2.0) * (-neuron_[9][0] + ((-1) * diff_theta) - (100*neuron_[7][0])), 0.0); //neurona DIFF ANGULAR (robo-pelota)

            neuron_[10][1] = std::max(neuron_[10][0] + (1.0 / 100.0) * (-neuron_[10][0] + neuron_[17][0] - 45), (double)0.0); //neurona DIFF ANGULAR (robo-pelota)

            neuron_[11][1] = std::max(neuron_[11][0] + (1.0 / 100.0) * (-neuron_[11][0] - 200 * neuron_[10][0] + 4 * instinto_caza), (double)0.0); //neruona condicion retorno

            neuron_[12][1] = std::max((neuron_[12][0] + (1.0 / 2.0) * (-neuron_[12][0] + diff_home - neuron_[13][0] * 20 - 200 * neuron_[11][0])), (double)0.0); //neurona DIFF ANGULAR (home-robot)
            neuron_[13][1] = std::max((neuron_[13][0] + (1.0 / 2.0) * (-neuron_[13][0] + (-1) * diff_home - neuron_[12][0] * 20 - 200 * neuron_[11][0])), (double)0.0); //neurona DIFF ANGULAR (home-pelota)

            neuron_[14][1] = std::max((neuron_[14][0] + (1.0 / tau1) * (-neuron_[14][0] - neuron_[11][0] * 100 + pos_error * 0.2 -  neuron_[15][0])), (double)0.0); //retorno

            neuron_[15][1] = std::max((neuron_[15][0] + (1.0 / tau1) * (-neuron_[15][0] +  regreso_stop)), (double)0.0); //deterner el regreso

            float a = (65.0 * std::pow(neuron_[6][0], 2) / ((10.0 + neuron_[10][0] * 50.0) * (10.0 + neuron_[10][0] * 50.0) + std::pow(neuron_[6][0], 2)));

            neuron_[16][1] = std::max(neuron_[16][0] + (1.0/ 15.0) * (-neuron_[16][0] + a), (double)0.0); //motor vertical

            float b = (65.0 * std::pow(neuron_[6][0], 2) / ((10.0 + neuron_[18][0] * 50.0 + neuron_[19][0] * 50.0) * (10 + neuron_[18][0] * 50.0 + neuron_[19][0] * 50.0) + std::pow(neuron_[6][0], 2)));

            neuron_[17][1] = std::max(neuron_[17][0] + (1.0 / 60.0) * (-neuron_[17][0] + b), (double)0.0); //gripper

            neuron_[18][1] = std::max((neuron_[18][0] + (1.0 / tau1) * (-neuron_[18][0] + neuron_[17][0] - 55.0)), (double)0.0); //Simulacion de presion

            neuron_[19][1] = std::max((neuron_[19][0] + (1 / tau1) * (-neuron_[19][0] - neuron_[14][0] * 10.0 - neuron_[5][0] * 2.0 - neuron_[11][0] * 2.0 + 1.0 + neuron_[20][0] * 4.0)), (double)0.0); //Shutdown

            float c = (30.0 * round(std::pow(neuron_[19][0], 2)) / (4.0 + round(neuron_[19][0]) * round(neuron_[19][0])));
            neuron_[20][1] = std::max((neuron_[20][0] + (1.0 / tau1) * (-neuron_[20][0] + c)), (double)0.0); //Shutdown

            for (int i = 0; i < 21; ++i) {
                neuron_[i][0] = neuron_[i][1];
            }

            cmd_vel_msg.linear.x = (neuron_[0][0] - neuron_[4][0]) * 0.5;
            cmd_vel_msg.angular.z = (neuron_[2][0] - neuron_[1][0]) * 0.5;
            //Imprimir los valores deseados
            // Crear un mensaje Int32MultiArray
            auto message = std_msgs::msg::Float32MultiArray();
            std::vector<_Float32> gripper_data;
            gripper_data.push_back((_Float32)neuron_[17][0]);
            gripper_data.push_back((_Float32)neuron_[16][0]);
            message.data = gripper_data;
            publisher_gripper_->publish(message);
            RCLCPP_INFO(this->get_logger(), "regreso: %f", neuron_[14][0]);
            RCLCPP_INFO(this->get_logger(), "agarre: %f",  neuron_[17][0]);
            RCLCPP_INFO(this->get_logger(), "stop_regreso: %f",  neuron_[15][0]);
            //RCLCPP_INFO(this->get_logger(), "h: %d",  h);

            publisher_->publish(cmd_vel_msg);
        }
        
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr  publisher_gripper_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
        rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}