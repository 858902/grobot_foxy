#include "rclcpp/rclcpp.hpp"
#include "SensorFusionNode.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorFusionNode>());
    rclcpp::shutdown();
    return 0;
}