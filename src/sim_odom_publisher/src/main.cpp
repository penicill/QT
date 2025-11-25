#include <rclcpp/rclcpp.hpp>
#include "sim_odom_publisher/sim_odom_publisher.hpp"

int main(int argc, char * argv[])
{
    // 初始化ROS2
    rclcpp::init(argc, argv);

    // 创建节点并自旋
    rclcpp::spin(std::make_shared<SimOdomPublisher>());

    // 关闭ROS2
    rclcpp::shutdown();
    return 0;
}
