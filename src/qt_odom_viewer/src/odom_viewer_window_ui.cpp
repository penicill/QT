// UI文件由CMake AUTOUIC自动生成
#include "ui_odom_viewer_window.h"

#include "qt_odom_viewer/odom_viewer_window_ui.h"
#include "qt_odom_viewer/odom_subscriber.h"
#include "qt_odom_viewer/trajectory_frame.h"
#include "qt_odom_viewer/dashboard_frame.h"

#include <QtConcurrent/QtConcurrent>
#include <QMetaObject>
#include <chrono>
#include <thread>
#include <QDebug>
#include <QMessageBox>
#include <QTextCursor>
#include <cmath>  // 添加数学函数库
#include <rclcpp/rclcpp.hpp>

OdomViewerWindow::OdomViewerWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(std::make_unique<Ui::OdomViewerWindow>())
    , is_updating_(true)
    , use_degree_(true)
    , ros_running_(false)
{
    // 注册Qt元类型
    qRegisterMetaType<OdometryData>("OdometryData");

    // 初始化当前里程计数据
    current_odom_ = std::make_unique<OdometryData>();

    // 初始化UI
    ui->setupUi(this);

     // 初始化进程对象
    process_follow = new QProcess(this);
    process_radar = new QProcess(this);
    process_rtk = new QProcess(this);
    
    
    initUI();

    // 初始化ROS2
    initROS2Subscriber();

    // 重置UI样式
    resetUIStyles();

    // 关联进程输出信号
    connect(process_follow, &QProcess::readyReadStandardOutput, this, &OdomViewerWindow::readProcessOutput);
    connect(process_follow, &QProcess::readyReadStandardError, this, &OdomViewerWindow::readProcessOutput);
    connect(process_radar, &QProcess::readyReadStandardOutput, this, &OdomViewerWindow::readProcessOutput);
    connect(process_radar, &QProcess::readyReadStandardError, this, &OdomViewerWindow::readProcessOutput);
    connect(process_rtk, &QProcess::readyReadStandardOutput, this, &OdomViewerWindow::readProcessOutput);
    connect(process_rtk, &QProcess::readyReadStandardError, this, &OdomViewerWindow::readProcessOutput);
}

OdomViewerWindow::~OdomViewerWindow()
{
    // 停止ROS2线程
    ros_running_ = false;
    try {
        rclcpp::shutdown();
    } catch (const std::exception& e) {
        // 忽略可能的异常
    }
    ros_spin_future_.waitForFinished();
}

void OdomViewerWindow::initUI()
{
    // 创建自定义绘图控件
    trajectory_frame_ = new TrajectoryFrame();
    dashboard_frame_ = new DashboardFrame();

    // 将自定义控件替换占位符
    ui->verticalLayout->replaceWidget(ui->trajectoryPlaceholder, trajectory_frame_);
    ui->horizontalLayout->replaceWidget(ui->dashboardPlaceholder, dashboard_frame_);

    // 删除占位符
    delete ui->trajectoryPlaceholder;
    delete ui->dashboardPlaceholder;
    ui->trajectoryPlaceholder = nullptr;
    ui->dashboardPlaceholder = nullptr;

    // 连接信号槽 - 使用Qt5兼容语法
    connect(ui->clearBtn, SIGNAL(clicked()), this, SLOT(on_clearBtn_clicked()));
    connect(ui->resetScaleBtn, SIGNAL(clicked()), this, SLOT(on_resetScaleBtn_clicked()));
}

void OdomViewerWindow::initROS2Subscriber()
{
    // 创建ROS2里程计订阅器
    odom_subscriber_ = std::make_shared<OdomSubscriber>(
        [this](const OdometryData& data) { this->rosOdomCallback(data); }
    );

    // 启动ROS2自旋线程（非阻塞）
    ros_running_ = true;
    ros_spin_future_ = QtConcurrent::run([this]() {
        while (ros_running_ && rclcpp::ok()) {
            rclcpp::spin_some(odom_subscriber_);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });

    // 更新ROS2连接状态
    ui->rosStatusLabel->setText("ROS2 状态：已连接");
    ui->rosStatusLabel->setStyleSheet("color: green; font-size: 14px;");
}

void OdomViewerWindow::rosOdomCallback(const OdometryData& data)
{
    if (!is_updating_) return;

    // 跨线程传递数据到UI线程 - Qt5兼容语法
    QMetaObject::invokeMethod(
        this,
        "handleOdomData",
        Qt::QueuedConnection,
        Q_ARG(OdometryData, data)
    );
}

void OdomViewerWindow::handleOdomData(const OdometryData& data)
{
    // 复制数据到当前存储
    *current_odom_ = data;

    // 更新UI显示
    updateUI(data);

    // 更新轨迹
    updateTrajectory(data);

    // 更新仪表盘的偏航角显示
    dashboard_frame_->setYawData(data.yaw, use_degree_);
}

void OdomViewerWindow::updateUI(const OdometryData& data)
{
    // --------------------------
    // 总路程计算（智能指针操作）
    // --------------------------
    if (!is_first_frame_ && last_odom_data_)  // unique_ptr 可直接判断是否非空（替代 != nullptr）
    {
        double distance_step = 0.0;

        if (use_position_based_calc_)
        {
            // 智能指针访问成员用 ->（和裸指针一致）
            double dx = data.x - last_odom_data_->x;
            double dy = data.y - last_odom_data_->y;
            double dz = data.z - last_odom_data_->z;
            distance_step = sqrt(dx*dx + dy*dy + dz*dz);
        }
        else
        {
            // 基于速度积分
            qint64 time_diff_ms = last_odom_data_->timestamp.msecsTo(data.timestamp);
            double time_diff_s = time_diff_ms / 1000.0;
            if (time_diff_s > 0 && time_diff_s < 1.0)
            {
                double avg_linear_x = (data.linear_x + last_odom_data_->linear_x) / 2.0;
                distance_step = avg_linear_x * time_diff_s;
            }
        }

        if (distance_step >= 0.0)
        {
            total_distance_ += distance_step;
        }
    }
    else
    {
        // 第一帧：用 reset() 创建新对象（替代 new + 赋值）
        // reset() 会自动释放原有内存（首次调用时无内存可释放）
        last_odom_data_.reset(new OdometryData(data)); 
        // 或 C++14+ 更简洁的方式：last_odom_data_ = std::make_unique<OdometryData>(data);
        is_first_frame_ = false;
    }

    // 非第一帧：更新智能指针指向的对象内容（用 operator* 解引用）
    *last_odom_data_ = data;

    // 更新位置数据
    ui->xValue->setText(QString::number(data.x, 'f', 3));
    ui->yValue->setText(QString::number(data.y, 'f', 3));
    ui->zValue->setText(QString::number(data.z, 'f', 3));

    // 更新姿态数据（支持角度/弧度切换）
    double roll = use_degree_ ? data.roll * 180 / M_PI : data.roll;
    double pitch = use_degree_ ? data.pitch * 180 / M_PI : data.pitch;
    double yaw = use_degree_ ? data.yaw * 180 / M_PI : data.yaw;

    ui->rollValue->setText(QString::number(roll, 'f', 3) + (use_degree_ ? "°" : "rad"));
    ui->pitchValue->setText(QString::number(pitch, 'f', 3) + (use_degree_ ? "°" : "rad"));
    ui->yawValue->setText(QString::number(yaw, 'f', 3) + (use_degree_ ? "°" : "rad"));

    // 更新速度数据
    ui->linearXValue->setText(QString::number(data.linear_x, 'f', 3));
    ui->angularZValue->setText(QString::number(data.angular_z, 'f', 3));

    // 显示总路程（保留3位小数，单位m）
    ui->odomValue->setText(QString::number(total_distance_, 'f', 3) + " m");
    // 更新时间戳
    ui->timestampValue->setText(data.timestamp.toString("HH:mm:ss.zzz"));
}

void OdomViewerWindow::updateTrajectory(const OdometryData& data)
{
    if (!data.is_valid) return;

    int traj_width = trajectory_frame_->width();
    int traj_height = trajectory_frame_->height();

    // 计算轨迹点坐标（将机器人坐标转换为像素坐标）
    QPointF point(
        data.x * 10 + traj_width / 2,
        -data.y * 10 + traj_height / 2
    );  // Y轴取反，让机器人前进方向向上

    trajectory_points_.append(point);

    // 限制轨迹点数量，防止内存溢出
    if (trajectory_points_.size() > trajectory_max_points_) {
        trajectory_points_.removeFirst();
    }

    // 更新轨迹图
    trajectory_frame_->setTrajectoryData(trajectory_points_);
}

void OdomViewerWindow::resetUIStyles()
{
    // 统一所有显示值的样式
    QList<QLabel*> valueLabels = {
        ui->xValue, ui->yValue, ui->zValue,
        ui->rollValue, ui->pitchValue, ui->yawValue,
        ui->linearXValue, ui->angularZValue, ui->odomValue, ui->timestampValue
    };

    for (QLabel* label : valueLabels) {
        label->setStyleSheet("border: 1px solid #cccccc; padding: 4px; font-size: 14px;");
        label->setAlignment(Qt::AlignCenter);
        label->setMinimumWidth(100);
    }
}

void OdomViewerWindow::on_clearBtn_clicked()
{
    trajectory_points_.clear();
    trajectory_frame_->setTrajectoryData(trajectory_points_);
    trajectory_frame_->resetScaleAndOffset();
}

void OdomViewerWindow::on_resetScaleBtn_clicked()
{
    trajectory_frame_->resetScaleAndOffset();
}

// 读取进程输出并显示
void OdomViewerWindow::readProcessOutput()
{
    QProcess *senderProcess = qobject_cast<QProcess*>(sender());
    if (!senderProcess) return;

    // 读取标准输出
    QByteArray output = senderProcess->readAllStandardOutput();
    if (!output.isEmpty()) {
        ui->logEdit->append("[INFO] " + output);
    }

    // 读取错误输出
    QByteArray error = senderProcess->readAllStandardError();
    if (!error.isEmpty()) {
        ui->logEdit->append("<font color='red'>[ERROR] " + error + "</font>");
    }

    // 滚动到最新内容
    ui->logEdit->moveCursor(QTextCursor::End);
}

void OdomViewerWindow::on_pushButton_follow_clicked(bool checked)
{
    if (checked) {
        if (process_follow->state() == QProcess::Running) {
            QMessageBox::information(this, "提示", "脚本已在运行中");
            return;
        }
        process_follow->start("bash", QStringList() << scriptPath_follow);

        if (!process_follow->waitForStarted(5000)) {
            QMessageBox::critical(this, "错误", "启动失败：" + process_follow->errorString());
            ui->pushButton_follow->setChecked(false);
            return;
        }
        ui->pushButton_follow->setText("结束跟随");
        qDebug() << "脚本启动成功";
    } else {
        if (process_follow->state() != QProcess::Running) {
            qDebug() << "没有运行中的脚本";
            ui->pushButton_follow->setText("开启跟随");
            return;
        }

        process_follow->terminate();
        if (!process_follow->waitForFinished(5000)) {
            qDebug() << "脚本超时，已强制终止";
        } else {
            qDebug() << "脚本已优雅终止";
        }

        ui->pushButton_follow->setText("开启跟随");
    }
}

void OdomViewerWindow::on_pushButton_radar_clicked(bool checked)
{
    if (checked) {
        if (process_radar->state() == QProcess::Running) {
            QMessageBox::information(this, "提示", "脚本2已在运行中");
            return;
        }

        process_radar->start("bash", QStringList() << scriptPath_radar);

        if (!process_radar->waitForStarted(5000)) {
            QMessageBox::critical(this, "错误", "启动失败：" + process_radar->errorString());
            ui->pushButton_radar->setChecked(false);
            return;
        }
        ui->pushButton_radar->setText("结束导航");
        qDebug() << "脚本2启动成功";
    } else {
        if (process_radar->state() != QProcess::Running) {
            qDebug() << "没有运行中的脚本";
            ui->pushButton_radar->setText("雷达导航");
            return;
        }

        process_radar->terminate();
        process_radar->start("bash", QStringList() << scriptPath_radar << "clean");
        if (!process_radar->waitForFinished(5000)) {
            qDebug() << "脚本2超时，已强制终止";
        } else {
            qDebug() << "脚本2已优雅终止";
        }

        ui->pushButton_radar->setText("雷达导航");
    }
}

void OdomViewerWindow::on_pushButton_rtk_clicked(bool checked)
{
    if (checked) {
        if (process_rtk->state() == QProcess::Running) {
            QMessageBox::information(this, "提示", "脚本3已在运行中");
            return;
        }

        process_rtk->start("bash", QStringList() << scriptPath_rtk);

        if (!process_rtk->waitForStarted(5000)) {
            QMessageBox::critical(this, "错误", "启动失败：" + process_rtk->errorString());
            ui->pushButton_rtk->setChecked(false);
            return;
        }
        ui->pushButton_rtk->setText("结束导航");
        qDebug() << "脚本3启动成功";
    } else {
        if (process_rtk->state() != QProcess::Running) {
            qDebug() << "没有运行中的脚本";
            ui->pushButton_rtk->setText("rtk导航");
            return;
        }

        process_rtk->terminate();
        if (!process_rtk->waitForFinished(5000)) {
            qDebug() << "脚本3超时，已强制终止";
        } else {
            qDebug() << "脚本3已优雅终止";
        }
        ui->pushButton_rtk->setText("rtk导航");
    }
}
