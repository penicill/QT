#include <QApplication>
#include "qt_odom_viewer/odom_viewer_window_ui.h"

// ROS2包含
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    // 初始化 QT 应用
    QApplication a(argc, argv);

    // 初始化 ROS2 - 正确包含rclcpp
    try {
        rclcpp::init(argc, argv);
    } catch (const std::exception& e) {
        // 如果ROS2初始化失败，仍然可以继续运行UI，但会显示未连接状态
        // 这对于在ROS2环境外进行基础的UI测试很有用
    }

    // 创建并显示主窗口
    OdomViewerWindow w;
    w.show();

    // 运行 QT 事件循环
    int ret = a.exec();

    // 关闭 ROS2（如果已初始化）
    try {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    } catch (const std::exception& e) {
        // 忽略可能的关闭异常
    }

    return ret;
}