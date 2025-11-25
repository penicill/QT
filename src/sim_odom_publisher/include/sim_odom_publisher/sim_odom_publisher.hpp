#ifndef SIM_ODOM_PUBLISHER_HPP
#define SIM_ODOM_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <random>
#include <cmath>

enum class MotionMode {
    LINEAR = 0,    // 匀速直线运动
    CIRCULAR = 1,  // 圆周运动
    RANDOM = 2     // 随机漫步
};

class SimOdomPublisher : public rclcpp::Node
{
public:
    SimOdomPublisher(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~SimOdomPublisher() = default;

private:
    // 初始化参数
    void initParameters();
    // 定时器回调：发布里程计数据
    void publishOdomCallback();
    // 更新机器人状态（位置、姿态、速度）
    void updateRobotState(double dt);
    // 生成高斯噪声
    double generateGaussianNoise(double mean, double std_dev);
    // 四元数生成（从偏航角yaw）
    geometry_msgs::msg::Quaternion getQuaternionFromYaw(double yaw);

    // ROS2发布者
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    // 定时器
    rclcpp::TimerBase::SharedPtr publish_timer_;

    // 模拟参数
    double publish_freq_;        // 发布频率（Hz）
    MotionMode motion_mode_;     // 运动模式
    double linear_speed_;        // 线速度（m/s）
    double angular_speed_;       // 角速度（rad/s）
    double pos_noise_std_;       // 位置噪声标准差（m）
    double vel_noise_std_;       // 速度噪声标准差（m/s）
    std::string odom_frame_id_;  // 里程计参考系
    std::string base_link_id_;   // 机器人基坐标系

    // 机器人状态
    double x_;       // X轴位置（m）
    double y_;       // Y轴位置（m）
    double yaw_;     // 偏航角（rad）
    double vx_;      // X轴线速度（m/s）
    double vy_;      // Y轴线速度（m/s）
    double wz_;      // Z轴角速度（rad/s）

    // 随机数生成器
    std::random_device rd_;
    std::mt19937 gen_;
    std::normal_distribution<double> gaussian_dist_;
};

#endif // SIM_ODOM_PUBLISHER_HPP
