// 修复：添加完整的头文件路径（相对于 src 目录，指向 include/qt_odom_viewer/ 下的文件）
#include "qt_odom_viewer/odom_viewer_window.h"

OdomViewerWindow::OdomViewerWindow(QWidget *parent)
    : QMainWindow(parent)
{
    // 初始化 QT 元类型（自定义结构体跨线程传递）
    qRegisterMetaType<OdometryData>();

    initUI();
    initROS2Subscriber();

    setWindowTitle("ROS2 里程计可视化工具");
    resize(1000, 700);
}

OdomViewerWindow::~OdomViewerWindow()
{
    // 停止 ROS2 线程
    ros_running_ = false;
    rclcpp::shutdown();
    ros_spin_future_.waitForFinished();
}

// 初始化 UI
void OdomViewerWindow::initUI()
{
    createWidgets();
    setupLayouts();
}

// 创建所有 UI 控件
void OdomViewerWindow::createWidgets()
{
    central_widget_ = new QWidget(this);
    setCentralWidget(central_widget_);

    // 顶部状态和控制按钮
    ros_status_label_ = new QLabel("ROS2 状态：未连接");
    ros_status_label_->setStyleSheet("color: red; font-size: 14px;");

    start_btn_ = new QPushButton("暂停更新");
    start_btn_->setStyleSheet("background-color: #ff4444; color: white; font-size: 14px;");
    start_btn_->setFixedSize(100, 30);

    clear_btn_ = new QPushButton("清除轨迹");
    clear_btn_->setStyleSheet("font-size: 14px;");
    clear_btn_->setFixedSize(100, 30);

    unit_btn_ = new QPushButton("切换角度单位");
    unit_btn_->setStyleSheet("font-size: 14px;");
    unit_btn_->setFixedSize(120, 30);

    reset_scale_btn_ = new QPushButton("重置缩放");
    reset_scale_btn_->setStyleSheet("font-size: 14px;");
    reset_scale_btn_->setFixedSize(100, 30);

    angle_unit_label_ = new QLabel("单位：度");
    angle_unit_label_->setStyleSheet("font-size: 14px;");

    // 数据显示标签（位置、姿态、速度）
    x_label_ = new QLabel("X 位置 (m)：");
    x_value_ = new QLabel("0.000");
    y_label_ = new QLabel("Y 位置 (m)：");
    y_value_ = new QLabel("0.000");
    z_label_ = new QLabel("Z 位置 (m)：");
    z_value_ = new QLabel("0.000");

    roll_label_ = new QLabel("滚转角：");
    roll_value_ = new QLabel("0.000");
    pitch_label_ = new QLabel("俯仰角：");
    pitch_value_ = new QLabel("0.000");
    yaw_label_ = new QLabel("偏航角：");
    yaw_value_ = new QLabel("0.000");

    linear_x_label_ = new QLabel("线速度 (m/s)：");
    linear_x_value_ = new QLabel("0.000");
    angular_z_label_ = new QLabel("角速度 (rad/s)：");
    angular_z_value_ = new QLabel("0.000");

    timestamp_label_ = new QLabel("更新时间：");
    timestamp_value_ = new QLabel("---");

    // 设置数值标签样式
    QList<QLabel*> value_labels = {x_value_, y_value_, z_value_, roll_value_, pitch_value_, yaw_value_, linear_x_value_, angular_z_value_, timestamp_value_};
    for (QLabel *label : value_labels) {
        label->setStyleSheet("border: 1px solid #cccccc; padding: 4px; font-size: 14px;");
        label->setAlignment(Qt::AlignCenter);
        label->setMinimumWidth(100);
    }

    // 自定义绘图容器
    trajectory_frame_ = new TrajectoryFrame();
    dashboard_frame_ = new DashboardFrame();

    // 绑定按钮信号槽
    connect(start_btn_, SIGNAL(clicked()), this, SLOT(on_startBtn_clicked()));
    connect(clear_btn_, SIGNAL(clicked()), this, SLOT(on_clearBtn_clicked()));
    connect(unit_btn_, SIGNAL(clicked()), this, SLOT(on_unitBtn_clicked()));
    connect(reset_scale_btn_, SIGNAL(clicked()), this, SLOT(on_resetScaleBtn_clicked()));
}

// 布局 UI 控件
void OdomViewerWindow::setupLayouts()
{
    main_layout_ = new QGridLayout(central_widget_);
    main_layout_->setSpacing(15);
    main_layout_->setContentsMargins(20, 20, 20, 20);

    // 顶部布局（状态+按钮）
    top_layout_ = new QHBoxLayout();
    top_layout_->setSpacing(20);
    top_layout_->addWidget(ros_status_label_);
    top_layout_->addWidget(start_btn_);
    top_layout_->addWidget(clear_btn_);
    top_layout_->addWidget(unit_btn_);
    top_layout_->addWidget(reset_scale_btn_);
    top_layout_->addWidget(angle_unit_label_);
    top_layout_->addStretch();  // 右对齐填充

    // 数据显示布局（位置、姿态、速度）
    data_layout_ = new QGridLayout();
    data_layout_->setSpacing(10);
    data_layout_->setColumnStretch(1, 1);
    data_layout_->setColumnStretch(3, 1);

    // 第一行：X、Y 位置
    data_layout_->addWidget(x_label_, 0, 0, Qt::AlignRight);
    data_layout_->addWidget(x_value_, 0, 1);
    data_layout_->addWidget(y_label_, 0, 2, Qt::AlignRight);
    data_layout_->addWidget(y_value_, 0, 3);

    // 第二行：Z 位置、滚转角
    data_layout_->addWidget(z_label_, 1, 0, Qt::AlignRight);
    data_layout_->addWidget(z_value_, 1, 1);
    data_layout_->addWidget(roll_label_, 1, 2, Qt::AlignRight);
    data_layout_->addWidget(roll_value_, 1, 3);

    // 第三行：俯仰角、偏航角
    data_layout_->addWidget(pitch_label_, 2, 0, Qt::AlignRight);
    data_layout_->addWidget(pitch_value_, 2, 1);
    data_layout_->addWidget(yaw_label_, 2, 2, Qt::AlignRight);
    data_layout_->addWidget(yaw_value_, 2, 3);

    // 第四行：线速度、角速度
    data_layout_->addWidget(linear_x_label_, 3, 0, Qt::AlignRight);
    data_layout_->addWidget(linear_x_value_, 3, 1);
    data_layout_->addWidget(angular_z_label_, 3, 2, Qt::AlignRight);
    data_layout_->addWidget(angular_z_value_, 3, 3);

    // 第五行：更新时间（跨4列）
    data_layout_->addWidget(timestamp_label_, 4, 0, Qt::AlignRight);
    data_layout_->addWidget(timestamp_value_, 4, 1, 1, 3);

    // 主布局装配
    main_layout_->addLayout(top_layout_, 0, 0, 1, 2);  // 顶部布局占1行2列
    main_layout_->addLayout(data_layout_, 1, 0);       // 数据布局占第2行第1列
    main_layout_->addWidget(dashboard_frame_, 1, 1);   // 仪表盘占第2行第2列
    main_layout_->addWidget(trajectory_frame_, 2, 0, 1, 2);  // 轨迹图占第3行2列

    // 拉伸配置（轨迹图占满剩余空间）
    main_layout_->setRowStretch(2, 1);
    main_layout_->setColumnStretch(0, 1);
}

// 初始化 ROS2 订阅器
void OdomViewerWindow::initROS2Subscriber()
{
    // 创建 ROS2 里程计订阅器
    odom_subscriber_ = std::make_shared<OdomSubscriber>(
        std::bind(&OdomViewerWindow::rosOdomCallback, this, std::placeholders::_1)
    );

    // 启动 ROS2 自旋线程（非阻塞）
    ros_running_ = true;
    ros_spin_future_ = QtConcurrent::run([this]() {
        while (ros_running_ && rclcpp::ok()) {
            rclcpp::spin_some(odom_subscriber_);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));  // 降低CPU占用
        }
    });

    // 更新 ROS2 连接状态
    ros_status_label_->setText("ROS2 状态：已连接");
    ros_status_label_->setStyleSheet("color: green; font-size: 14px;");
}

// ROS2 数据回调（运行在 ROS2 线程）
void OdomViewerWindow::rosOdomCallback(const OdometryData& data)
{
    if (!is_updating_) return;

    // 跨线程传递数据到 UI 线程
    QMetaObject::invokeMethod(
        this,
        "handleOdomData",
        Qt::QueuedConnection,
        Q_ARG(OdometryData, data)
    );
}

// 处理里程计数据（运行在 UI 线程）
void OdomViewerWindow::handleOdomData(const OdometryData& data)
{
    current_odom_ = data;

    // 更新位置数据
    x_value_->setText(QString::number(data.x, 'f', 3));
    y_value_->setText(QString::number(data.y, 'f', 3));
    z_value_->setText(QString::number(data.z, 'f', 3));

    // 更新姿态数据（支持角度/弧度切换）
    double roll = use_degree_ ? data.roll * 180 / M_PI : data.roll;
    double pitch = use_degree_ ? data.pitch * 180 / M_PI : data.pitch;
    double yaw = use_degree_ ? data.yaw * 180 / M_PI : data.yaw;
    roll_value_->setText(QString::number(roll, 'f', 3) + (use_degree_ ? "°" : "rad"));
    pitch_value_->setText(QString::number(pitch, 'f', 3) + (use_degree_ ? "°" : "rad"));
    yaw_value_->setText(QString::number(yaw, 'f', 3) + (use_degree_ ? "°" : "rad"));

    // 更新速度数据
    linear_x_value_->setText(QString::number(data.linear_x, 'f', 3));
    angular_z_value_->setText(QString::number(data.angular_z, 'f', 3));

    // 更新时间戳
    timestamp_value_->setText(data.timestamp.toString("HH:mm:ss.zzz"));

    // 更新轨迹数据
    if (data.is_valid) {
        int traj_width = trajectory_frame_->width();
        int traj_height = trajectory_frame_->height();
        // 计算原始轨迹点（基于轨迹图中心）
        QPointF point(
            data.x * 10 + traj_width / 2,
            -data.y * 10 + traj_height / 2
        );
        trajectory_points_.append(point);

        // 限制轨迹点数量（防止内存溢出）
        if (trajectory_points_.size() > trajectory_max_points_) {
            trajectory_points_.removeFirst();
        }

        // 传递轨迹数据到绘图容器
        trajectory_frame_->setTrajectoryData(trajectory_points_);
        
    }

    // 传递偏航角数据到仪表盘
    //dashboard_frame_->setYawData(data.yaw);
    // 传递偏航角数据到仪表盘（新增 use_degree_ 参数，同步角度单位）
    dashboard_frame_->setYawData(data.yaw, use_degree_);
}

// 暂停/继续更新按钮
void OdomViewerWindow::on_startBtn_clicked()
{
    is_updating_ = !is_updating_;
    if (is_updating_) {
        start_btn_->setText("暂停更新");
        start_btn_->setStyleSheet("background-color: #ff4444; color: white; font-size: 14px;");
    } else {
        start_btn_->setText("继续更新");
        start_btn_->setStyleSheet("background-color: #44dd44; color: white; font-size: 14px;");
    }
}

// 切换角度单位按钮（度/弧度）
void OdomViewerWindow::on_unitBtn_clicked()
{
    use_degree_ = !use_degree_;
    angle_unit_label_->setText(use_degree_ ? "单位：度" : "单位：弧度");
    handleOdomData(current_odom_);  // 重新更新显示
}

// 清除轨迹按钮（调用重置缩放+偏移的函数）
void OdomViewerWindow::on_clearBtn_clicked()
{
    trajectory_points_.clear();
    trajectory_frame_->setTrajectoryData(trajectory_points_);
    trajectory_frame_->resetScaleAndOffset();  // 重置缩放和拖动偏移
}

// 重置缩放按钮（重置缩放+偏移）
void OdomViewerWindow::on_resetScaleBtn_clicked()
{
    trajectory_frame_->resetScaleAndOffset();  // 同时重置缩放和拖动偏移
}
