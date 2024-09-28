#include"ros/ros.h"
#include"std_msgs/String.h"

void writeMsgTolong(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("Message we recived was: %s", msg->data.c_str());
}
int main(int argc, char**argv){
    ros::init(argc, argv, "Subscriber");
    ros::NodeHandle nh;

    ros::Subscriber topic_pub = nh.subscribe("tutorial", 1000, writeMsgTolong);

    ros::spin();
    return 0;
}