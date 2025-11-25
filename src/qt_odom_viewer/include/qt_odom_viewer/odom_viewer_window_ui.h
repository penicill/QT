#ifndef ODOM_VIEWER_WINDOW_UI_H
#define ODOM_VIEWER_WINDOW_UI_H

#include <QMainWindow>
#include <QFuture>
#include <memory>
#include <QProcess>
// 注意：ui_odom_viewer_window.h 将由 CMake AUTOUIC 自动生成

// 前向声明
struct OdometryData;
class OdomSubscriber;
class TrajectoryFrame;
class DashboardFrame;

namespace Ui {
class OdomViewerWindow;
}

/**
 * @brief ROS2里程计可视化主窗口类
 *
 * 功能：
 * - 订阅ROS2的/odom话题，获取里程计数据
 * - 显示位置、姿态、速度的数值信息
 * - 通过仪表盘显示偏航角
 * - 通过轨迹图显示机器人的移动轨迹
 * - 支持鼠标滚轮缩放轨迹图
 * - 支持鼠标左键拖动轨迹图
 * - 支持暂停/继续数据更新
 * - 支持清除轨迹并重置视图
 * - 支持度/弧度单位切换
 */
class OdomViewerWindow : public QMainWindow
{
    Q_OBJECT

public:
    /**
     * @brief 构造函数
     * @param parent 父窗口指针（默认为nullptr）
     */
    explicit OdomViewerWindow(QWidget *parent = nullptr);

    /**
     * @brief 析构函数
     */
    ~OdomViewerWindow();

private slots:

    /**
     * @brief 清除轨迹按钮点击槽函数
     */
    void on_clearBtn_clicked();

    /**
     * @brief 重置缩放按钮点击槽函数
     */
    void on_resetScaleBtn_clicked();

    void on_pushButton_follow_clicked(bool checked);
    void on_pushButton_radar_clicked(bool checked);
    void on_pushButton_rtk_clicked(bool checked);
    void readProcessOutput();

    /**
     * @brief 处理里程计数据的槽函数
     * @param data 里程计数据
     */
    void handleOdomData(const OdometryData& data);

private:
    /**
     * @brief 初始化UI
     */
    void initUI();

    /**
     * @brief 初始化ROS2订阅器
     */
    void initROS2Subscriber();

    /**
     * @brief ROS2数据回调函数
     * @param data 里程计数据
     */
    void rosOdomCallback(const OdometryData& data);

    /**
     * @brief 更新UI控件的显示值
     * @param data 里程计数据
     */
    void updateUI(const OdometryData& data);

    /**
     * @brief 更新轨迹显示
     * @param data 里程计数据
     */
    void updateTrajectory(const OdometryData& data);

    /**
     * @brief 重置所有UI控件的样式
     */
    void resetUIStyles();

private:
    // UI相关
    std::unique_ptr<Ui::OdomViewerWindow> ui;  /**< UI指针 */

    QProcess *process_follow;
    QProcess *process_radar;
    QProcess *process_rtk;
    QString scriptPath_follow = "/home/user/qtproject/APP1/Sh_File/launch_person_follow.sh";
    QString scriptPath_radar = "/home/mxt/qtproject/APP/Sh_File/launch_multi_robot.sh";
    QString scriptPath_rtk = "/home/user/qtproject/APP1/Sh_File/rtk_launch.sh";

    // 自定义绘图控件
    TrajectoryFrame* trajectory_frame_;    /**< 轨迹图控件 */
    DashboardFrame* dashboard_frame_;      /**< 姿态仪表盘控件 */

    // 数据相关
    std::unique_ptr<OdometryData> current_odom_;  /**< 当前里程计数据（指针避免不完整类型） */
    bool is_updating_;                     /**< 是否正在更新数据 */
    bool use_degree_;                      /**< 是否使用角度单位 */
    QVector<QPointF> trajectory_points_;   /**< 轨迹点列表 */
    static constexpr int trajectory_max_points_ = 500; /**< 最大轨迹点数量 */
    bool is_first_frame_ = true;  // 是否是第一帧数据（避免首帧计算错误）
    std::unique_ptr<OdometryData> last_odom_data_; // 上一帧里程计数据（记录位置、时间戳）
    double total_distance_ = 0.0; // 总路程累计值（单位：m）
    bool use_position_based_calc_ = true; // 计算方式：true=基于位置，false=基于速度

    // ROS2相关
    std::shared_ptr<OdomSubscriber> odom_subscriber_; /**< ROS2里程计订阅器 */
    QFuture<void> ros_spin_future_;      /**< ROS2自旋线程 */
    bool ros_running_;                     /**< ROS2是否在运行 */
};

#endif // ODOM_VIEWER_WINDOW_UI_H