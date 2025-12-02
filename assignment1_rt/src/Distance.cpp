#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class Distance: public rclcpp::Node
{
    public:
    Distance(): Node("Distance")
    {
        subscription1_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, std::bind(&Distance::topic_callback_turtle1, this, _1));
        subscription2_ = this->create_subscription<turtlesim::msg::Pose>("/turtle2/pose", 10, std::bind(&Distance::topic_callback_turtle2, this, _1));
        publisher_distance = this->create_publisher<std_msgs::msg::Float32>("/distance", 10);
        publisher1_vel = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        publisher2_vel = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&Distance::timer_callback, this));
        x_1 = 0;
        x_2 = 0;
        y_1 = 0;
        y_2 = 0;
    }
    private:
    void timer_callback()
    {
        float threshold = 1.0;
        float distance = std::sqrt((x_1-x_2)*(x_1-x_2) + (y_1-y_2)*(y_1-y_2));

        msg.data = distance;
        publisher_distance->publish(msg);

        geometry_msgs::msg::Twist stop_msg;
        stop_msg.linear.x = 0.0;
        stop_msg.angular.z = 0.0;

        if (distance < threshold)
        {
            publisher1_vel->publish(stop_msg);
            publisher2_vel->publish(stop_msg);
        }

        if (x_1 > 10 || x_1 < 1 || y_1 > 10 || y_1 < 1){
            publisher1_vel->publish(stop_msg);
        }
        if (x_2 > 10 || x_2 < 1 || y_2 > 10 || y_2 < 1){
            publisher2_vel->publish(stop_msg);
        }
    }

    void topic_callback_turtle1(const turtlesim::msg::Pose::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "The position of the turtle1 is (x, y): '%f, %f'", msg->x, msg->y);
        x_1 = msg->x;
        y_1 = msg->y;
    }

    void topic_callback_turtle2(const turtlesim::msg::Pose::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "The position of the turtle2 is (x, y): '%f, %f'", msg->x, msg->y);
        x_2 = msg->x;
        y_2 = msg->y;
    }

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription1_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription2_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_distance;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher1_vel;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher2_vel;
    rclcpp::TimerBase::SharedPtr timer_;
    std_msgs::msg::Float32 msg;
    float x_1;
    float y_1;
    float x_2;
    float y_2;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Distance>());
    rclcpp::shutdown();
    return 0;
}