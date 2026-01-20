#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "std_msgs/msg/bool.hpp"

using std::placeholders::_1;

class user_twist_publisher : public rclcpp::Node
{
public:
    user_twist_publisher() : Node("user_twist_publisher"), safety_active_(false)
    {
        publisher_= this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("/user_cmd_vel", 10, std::bind(&user_twist_publisher::topic_callback, this, _1));
        
        safety_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/safety_alert", 10, std::bind(&user_twist_publisher::safety_callback, this, _1));
            
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&user_twist_publisher::timer_callback, this));
        message_.linear.x = 0;
        message_.angular.z = 0;
        last_msg_time_ = std::chrono::steady_clock::now() - std::chrono::seconds(10);
    }
private:
    void safety_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        // Update safety state based on current sensor data
        safety_active_ = msg->data;
        if (safety_active_) {
             RCLCPP_WARN(this->get_logger(), "Safety Alert Active!");
        }
    }

    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Update user command
        message_ = *msg;
        last_msg_time_ = std::chrono::steady_clock::now();
    }

    void timer_callback()
    {
        if (safety_active_) {
            // Safety Maneuver: Moving strictly opposite to last command
            geometry_msgs::msg::Twist safety_cmd;
            safety_cmd.angular.z = 0.0; // Keep straight for safety
            safety_cmd.linear.x = -message_.linear.x; // Was going FWD -> Go BACK
            publisher_->publish(safety_cmd);
            return;
        }
        
        // Normal operation (User command with timeout)
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_msg_time_).count();

        if (elapsed < 5) {
            publisher_->publish(message_);
        } else {
            geometry_msgs::msg::Twist zero_msg;
            zero_msg.linear.x = 0.0;
            zero_msg.angular.z = 0.0;
            publisher_->publish(zero_msg);
        }
    }
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    geometry_msgs::msg::Twist message_;
    std::chrono::steady_clock::time_point last_msg_time_;
    
    // Safety state
    bool safety_active_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<user_twist_publisher>());
    rclcpp::shutdown();
    return 0;
}