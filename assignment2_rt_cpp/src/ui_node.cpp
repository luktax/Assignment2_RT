#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>

class UI : public rclcpp::Node{
    public:
    UI() : Node("ui_node")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/user_cmd_vel", 10);
        RCLCPP_INFO(this->get_logger(), "UI Node ready. Insert velocities:");
    }

    void run(){
        while (rclcpp::ok()){
            double linear_v, angular_v;
            std::cout << "Insert linear velocity: ";
            std::cin >> linear_v;
            if(std::cin.fail()){ 
                std::cin.clear(); 
                std::cin.ignore(1000, '\n'); 
                continue;
            }
            std::cout << "Insert angular velocity: ";
            std::cin >> angular_v;
            if(std::cin.fail()){ 
                std::cin.clear(); 
                std::cin.ignore(1000, '\n'); 
                continue;
            }  
            geometry_msgs::msg::Twist msg;
            msg.linear.x = linear_v;
            msg.angular.z = angular_v;

            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Published linear=%.2f angular=%.2f", linear_v, angular_v);

        }
    }
    private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UI>();
    node->run();
    rclcpp::shutdown();
    return 0;
}