#include "sim_odom_publisher/sim_odom_publisher.hpp"

SimOdomPublisher::SimOdomPublisher(const rclcpp::NodeOptions& options)
    : Node("sim_odom_publisher", options)
    , gen_(rd_())
    , gaussian_dist_(0.0, 1.0)  // 标准正态分布（均值0，方差1）
{
    // 初始化参数
    initParameters();

    // 创建发布者（队列大小10）
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/odom", 10
    );

    // 创建定时器（按发布频率触发）
    double publish_period = 1.0 / publish_freq_;
    publish_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(publish_period),
        std::bind(&SimOdomPublisher::publishOdomCallback, this)
    );

    RCLCPP_INFO(this->get_logger(), "模拟里程计发布器启动成功！");
    RCLCPP_INFO(this->get_logger(), "运动模式：%d | 发布频率：%.1fHz | 线速度：%.2fm/s | 角速度：%.2frad/s",
                static_cast<int>(motion_mode_), publish_freq_, linear_speed_, angular_speed_);
}

void SimOdomPublisher::initParameters()
{
    // 声明并获取参数（支持启动时通过命令行重写）
    this->declare_parameter<double>("publish_freq", 10.0);  // 默认10Hz
    this->declare_parameter<int>("motion_mode", 0);         // 默认匀速直线
    this->declare_parameter<double>("linear_speed", 0.5);   // 默认0.5m/s
    this->declare_parameter<double>("angular_speed", 0.2);  // 默认0.2rad/s
    this->declare_parameter<double>("pos_noise_std", 0.005); // 位置噪声0.5cm
    this->declare_parameter<double>("vel_noise_std", 0.01);  // 速度噪声0.01m/s
    this->declare_parameter<std::string>("odom_frame_id", "odom");
    this->declare_parameter<std::string>("base_link_id", "base_link");

    // 读取参数
    this->get_parameter("publish_freq", publish_freq_);
    int mode = 0;
    this->get_parameter("motion_mode", mode);
    motion_mode_ = static_cast<MotionMode>(mode);
    this->get_parameter("linear_speed", linear_speed_);
    this->get_parameter("angular_speed", angular_speed_);
    this->get_parameter("pos_noise_std", pos_noise_std_);
    this->get_parameter("vel_noise_std", vel_noise_std_);
    this->get_parameter("odom_frame_id", odom_frame_id_);
    this->get_parameter("base_link_id", base_link_id_);

    // 初始化机器人状态
    x_ = 0.0;
    y_ = 0.0;
    yaw_ = 0.0;
    vx_ = linear_speed_;
    vy_ = 0.0;
    wz_ = angular_speed_;
}

void SimOdomPublisher::publishOdomCallback()
{
    // 计算时间步长（两次发布的时间间隔）
    static auto last_time = this->get_clock()->now();
    auto current_time = this->get_clock()->now();
    double dt = (current_time - last_time).seconds();
    last_time = current_time;

    // 更新机器人状态（位置、姿态、速度）
    updateRobotState(dt);

    // 构建Odometry消息
    auto odom_msg = nav_msgs::msg::Odometry();

    // 1. 消息头（时间戳+参考系）
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = odom_frame_id_;
    odom_msg.child_frame_id = base_link_id_;

    // 2. 位置（添加噪声）
    odom_msg.pose.pose.position.x = x_ + generateGaussianNoise(0.0, pos_noise_std_);
    odom_msg.pose.pose.position.y = y_ + generateGaussianNoise(0.0, pos_noise_std_);
    odom_msg.pose.pose.position.z = 0.0;  // 2D机器人，Z轴位置为0

    // 3. 姿态（四元数，从偏航角转换）
    odom_msg.pose.pose.orientation = getQuaternionFromYaw(yaw_);

    // 4. 位置协方差矩阵（对角线为位置噪声方差，非对角线为0）
    odom_msg.pose.covariance.fill(0.0);
    odom_msg.pose.covariance[0] = pos_noise_std_ * pos_noise_std_;    // x方差
    odom_msg.pose.covariance[7] = pos_noise_std_ * pos_noise_std_;    // y方差
    odom_msg.pose.covariance[14] = 1e-6;                              // z方差（固定小值）
    odom_msg.pose.covariance[21] = 1e-3;                              // roll方差
    odom_msg.pose.covariance[28] = 1e-3;                              // pitch方差
    odom_msg.pose.covariance[35] = 1e-2;                              // yaw方差

    // 5. 速度（添加噪声）
    odom_msg.twist.twist.linear.x = vx_ + generateGaussianNoise(0.0, vel_noise_std_);
    odom_msg.twist.twist.linear.y = vy_ + generateGaussianNoise(0.0, vel_noise_std_);
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = wz_ + generateGaussianNoise(0.0, vel_noise_std_/10);

    // 6. 速度协方差矩阵
    odom_msg.twist.covariance.fill(0.0);
    odom_msg.twist.covariance[0] = vel_noise_std_ * vel_noise_std_;    // vx方差
    odom_msg.twist.covariance[7] = vel_noise_std_ * vel_noise_std_;    // vy方差
    odom_msg.twist.covariance[35] = (vel_noise_std_/10) * (vel_noise_std_/10);  // wz方差

    // 发布消息
    odom_publisher_->publish(odom_msg);
}

void SimOdomPublisher::updateRobotState(double dt)
{
    switch (motion_mode_) {
        case MotionMode::LINEAR:
            // 匀速直线运动：x递增，y不变，yaw不变
            x_ += vx_ * dt;
            y_ += vy_ * dt;
            break;

        case MotionMode::CIRCULAR:
            // 圆周运动：线速度沿切线方向，yaw随角速度增加
            wz_ = angular_speed_;
            vx_ = linear_speed_ * cos(yaw_);
            vy_ = linear_speed_ * sin(yaw_);
            x_ += vx_ * dt;
            y_ += vy_ * dt;
            yaw_ += wz_ * dt;
            break;

        case MotionMode::RANDOM:
            // 随机漫步：角速度随机变化，线速度恒定
            wz_ = angular_speed_ * (generateGaussianNoise(0.0, 0.5) * 2.0 - 1.0);  // [-angular_speed_, angular_speed_]
            vx_ = linear_speed_;
            vy_ = 0.0;
            x_ += vx_ * cos(yaw_) * dt;
            y_ += vx_ * sin(yaw_) * dt;
            yaw_ += wz_ * dt;
            break;
    }

    // 限制偏航角在[-π, π]范围内
    yaw_ = std::fmod(yaw_ + M_PI, 2 * M_PI) - M_PI;
}

double SimOdomPublisher::generateGaussianNoise(double mean, double std_dev)
{
    // 生成高斯噪声（标准正态分布 → 缩放至目标均值和标准差）
    return mean + std_dev * gaussian_dist_(gen_);
}

geometry_msgs::msg::Quaternion SimOdomPublisher::getQuaternionFromYaw(double yaw)
{
    // 从偏航角生成四元数（roll=0, pitch=0）
    geometry_msgs::msg::Quaternion q;
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    q.w = cy;
    q.x = 0.0;
    q.y = 0.0;
    q.z = sy;
    return q;
}

// 注册节点（ROS2要求）
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(SimOdomPublisher)
