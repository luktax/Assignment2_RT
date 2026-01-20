#include "rclcpp/rclcpp.hpp"
#include "assignment2_rt_cpp/srv/set_threshold.hpp"

using SetThreshold = assignment2_rt_cpp::srv::SetThreshold;

class ThresholdServer : public rclcpp::Node
{
public:
    ThresholdServer() : Node("threshold_server")
    {
        this->declare_parameter("threshold", 0.5);
        threshold_ = this->get_parameter("threshold").as_double();

        service_ = this->create_service<SetThreshold>(
            "set_threshold",
            std::bind(&ThresholdServer::setThresholdCallback, this, std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(this->get_logger(), "Service /set_threshold ready. Initial threshold: %.2f", threshold_);
    }

    double getThreshold() const
    {
        return threshold_;
    }

private:
    void setThresholdCallback(
        const std::shared_ptr<SetThreshold::Request> request,
        std::shared_ptr<SetThreshold::Response> response)
    {
        threshold_ = request->threshold;
        this->set_parameter(rclcpp::Parameter("threshold", threshold_));
        RCLCPP_INFO(this->get_logger(), "Threshold updated to: %.2f", threshold_);
        response->success = true;
        response->message = "Threshold updated successfully.";
    }

    rclcpp::Service<SetThreshold>::SharedPtr service_;
    double threshold_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ThresholdServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
