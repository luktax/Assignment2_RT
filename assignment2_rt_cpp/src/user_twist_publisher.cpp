#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "std_msgs/msg/bool.hpp"

#include <deque>
#include "assignment2_rt_cpp/srv/get_average.hpp"

using GetAverage = assignment2_rt_cpp::srv::GetAverage;
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

        //service average inputs
        service_ = this->create_service<GetAverage>("get_average",std::bind(&user_twist_publisher::handle_service, this, std::placeholders::_1, std::placeholders::_2));

        message_.linear.x = 0;
        message_.angular.z = 0;
        last_msg_time_ = std::chrono::steady_clock::now() - std::chrono::seconds(10);
        RCLCPP_INFO(this->get_logger(), "User_twist_publisher ready");
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

        last_linear_.push_back(message_.linear.x);
        last_angular_.push_back(message_.angular.z);

        if (last_linear_.size() > 5){
            last_linear_.pop_front();
            last_angular_.pop_front();
        }
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
    void handle_service(const std::shared_ptr<GetAverage::Request>, std::shared_ptr<GetAverage::Response> response){
        if (last_linear_.empty()) {
            response->success = false;
            response->message = "No inputs received yet";
            response->linear_average = 0.0;
            response->angular_average = 0.0;
            return;
        }
        float sum_lin = 0.0, sum_ang = 0.0;
        for (size_t i = 0; i < last_linear_.size(); ++i) {
            sum_lin += last_linear_[i];
            sum_ang += last_angular_[i];
        }

        response->linear_average = sum_lin / last_linear_.size();
        response->angular_average = sum_ang / last_angular_.size();
        response->success = true;
        response->message = "Average computed successfully";
    }
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<GetAverage>::SharedPtr service_;

    std::deque<float> last_linear_;
    std::deque<float> last_angular_;
    
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