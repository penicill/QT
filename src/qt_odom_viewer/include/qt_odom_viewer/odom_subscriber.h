#ifndef ODOM_SUBSCRIBER_H
#define ODOM_SUBSCRIBER_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <functional>
#include <QDateTime>
#include <QMetaType>

// 里程计数据结构体
struct OdometryData {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    double linear_x = 0.0;
    double angular_z = 0.0;
    QDateTime timestamp;
    bool is_valid = false;
};

// 注册自定义数据类型到 QT 元系统
Q_DECLARE_METATYPE(OdometryData)

// 纯 ROS2 里程计订阅器节点
class OdomSubscriber : public rclcpp::Node
{
public:
    using OdomDataCallback = std::function<void(const OdometryData&)>;

    explicit OdomSubscriber(const OdomDataCallback& callback)
        : Node("odom_subscriber_node"), callback_(callback)
    {
        // 创建 /odom 话题订阅者
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            10,  // QoS 深度
            std::bind(&OdomSubscriber::odomCallback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Odom subscriber started, listening to /odom");
    }

    ~OdomSubscriber() = default;

private:
    // 里程计数据回调函数
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!callback_) return;

        OdometryData data;
        // 解析位置信息
        data.x = msg->pose.pose.position.x;
        data.y = msg->pose.pose.position.y;
        data.z = msg->pose.pose.position.z;

        // 四元数转欧拉角（RPY）
        quaternionToEuler(msg->pose.pose.orientation, data.roll, data.pitch, data.yaw);

        // 解析速度信息
        data.linear_x = msg->twist.twist.linear.x;
        data.angular_z = msg->twist.twist.angular.z;

        // 解析时间戳
        data.timestamp = QDateTime::fromSecsSinceEpoch(msg->header.stamp.sec)
                         .addMSecs(msg->header.stamp.nanosec / 1000000);
        data.is_valid = true;

        // 回调传递数据到 QT 界面
        callback_(data);
    }

    // 四元数转欧拉角（Z-Y-X 旋转顺序，右手坐标系）
    void quaternionToEuler(const geometry_msgs::msg::Quaternion& q, double& roll, double& pitch, double& yaw)
    {
        double qx = q.x;
        double qy = q.y;
        double qz = q.z;
        double qw = q.w;

        // 滚转角 roll (x轴旋转)
        roll = atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy));
        // 俯仰角 pitch (y轴旋转)
        pitch = asin(2 * (qw * qy - qz * qx));
        // 偏航角 yaw (z轴旋转)
        yaw = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    OdomDataCallback callback_;  // 数据回调函数
};

#endif // ODOM_SUBSCRIBER_H
