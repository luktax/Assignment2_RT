#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class user_twist_publisher : public rclcpp::Node
{
public:
    user_twist_publisher() : Node("user_twist_publisher"){
        publisher_= this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("/user_cmd_vel", 10, std::bind(&user_twist_publisher::topic_callback, this, _1));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&user_twist_publisher::timer_callback, this));
        message_.linear.x = 0;
        message_.angular.z = 0;
    }
private:
    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        message_ = *msg;
    }
    void timer_callback()
    {
        publisher_->publish(message_);
    }
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist message_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<user_twist_publisher>());
    rclcpp::shutdown();
    return 0;
}