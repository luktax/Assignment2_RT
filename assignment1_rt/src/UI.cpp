#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class UI: public rclcpp::Node{
    public:
    UI(): Node("UI")
    {
        publisher1_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        publisher2_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&UI::timer_callback, this));
        flag = 0;
        counter = 20;
    }
    
    private:
    void timer_callback()
    {
        int turtle = 0;
        double linear_v, angular_v;

        if (flag == 1 && counter >= 20){
            message_.linear.x = 0;
            message_.angular.z = 0;

            if (turtle == 1)
                publisher1_->publish(message_);
            else if (turtle == 2)
                publisher2_->publish(message_);
            flag = 0;
        }

        if (flag == 0 && counter>=20){
            if (!rclcpp::ok()) return;

            while(turtle != 1 && turtle != 2){
                std::cout << "Press 1 or 2 to select a turtle";
                std::cin >> turtle;
                if (turtle != 1 && turtle != 2){
                    std::cout << "Not a valid turtle! insert 1 or 2.\n";
                }
            }

            while(true){
                std::cout << "Linear velocity: ";
                std::cin >> linear_v;     
                
                if (std::cin.fail()){
                    std::cin.clear();
                    std::cin.ignore(1000, '\n');
                    std::cout <<"Not a valid input! Insert a number.\n";
                } else break;
            }

            while(true){
                std::cout << "Angular velocity: ";
                std::cin >> angular_v;     
                
                if (std::cin.fail()){
                    std::cin.clear();
                    std::cin.ignore(1000, '\n');
                    std::cout <<"Not a valid input! Insert a number.\n";
                } else break;
            }

            message_.linear.x = linear_v;
            message_.angular.z = angular_v;

            if (turtle == 1)
                publisher1_->publish(message_);
            else if (turtle == 2)
                publisher2_->publish(message_);
            
            flag = 1;
            counter = 0;
        }
        if (flag == 1)
            counter++;

    }
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher1_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher2_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist message_;
    int flag, counter;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UI>());
    rclcpp::shutdown();
    return 0;
}