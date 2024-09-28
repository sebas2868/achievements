#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>

class SimplePublisher : public rclcpp::Node
{
public:
    SimplePublisher() : Node("simple_publisher"), counter_(0)
    {
        pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);
        timer_ = create_wall_timer(1s, std::bind(&&SimplePublisher::timer_callback, this));

        RCLCPP_INFO(get_logger(), "publishing at 1 Hz")

    }

private:
    unsigned int conter_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void timerCallback()
    {
        auto message = std_msgs::msg::String();
        message.data = "hello ros2: " + std::to_string(counter_++);

        pub_->publish(message);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimplePublisher>();
    rclcpp::spin(node);
    return 0;
}


