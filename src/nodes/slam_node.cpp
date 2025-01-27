#include "rclcpp/rclcpp.hpp"

class MinimalNode : public rclcpp::Node {
public:
    MinimalNode() : Node("minimal_test") {
        RCLCPP_INFO(this->get_logger(), "Success! Node is working!");
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalNode>());
    rclcpp::shutdown();
    return 0;
}