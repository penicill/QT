#ifndef ODOM_VIEWER_WINDOW_H
#define ODOM_VIEWER_WINDOW_H

#include <QMainWindow>
#include <QColor>
#include <QPainter>
#include <QPen>
#include <QVector>
#include <QPointF>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QFrame>
#include <QMetaObject>
#include <QtConcurrent/QtConcurrent>
#include <QPaintEvent>
#include <QWheelEvent>
#include <QResizeEvent>
#include <cmath>
#include <memory>
#include "qt_odom_viewer/odom_subscriber.h"

// 自定义轨迹绘图容器（支持鼠标滚轮缩放 + 鼠标拖动）
class TrajectoryFrame : public QFrame
{
    Q_OBJECT
public:
    // const 成员在初始化列表中初始化
    explicit TrajectoryFrame(QWidget *parent = nullptr) 
        : QFrame(parent),
          scale_factor_(1.0),
          scale_step_(0.1),
          min_scale_(0.1),
          max_scale_(10.0),
          is_dragging_(false)  // 初始未拖动
    {
        setStyleSheet("background-color: white; border: 1px solid #dddddd;");
        setMinimumSize(800, 300);
        setMouseTracking(true);
        // 设置鼠标样式（拖动时显示抓手）
        setCursor(Qt::OpenHandCursor);
    }

    // 设置轨迹数据
    void setTrajectoryData(const QVector<QPointF>& points)
    {
        raw_trajectory_points_ = points;
        updateScaledPoints();
        update();
    }

    // 重置缩放和拖动偏移（恢复默认中心点）
    void resetScaleAndOffset()
    {
        scale_factor_ = 1.0;
        drag_offset_x_ = 0.0;
        drag_offset_y_ = 0.0;
        updateScaledPoints();
        update();
    }

protected:
    // 鼠标按下事件：开始拖动
    void mousePressEvent(QMouseEvent *event) override
    {
        if (event->button() == Qt::LeftButton) {
            is_dragging_ = true;
            last_mouse_pos_ = event->pos();  // 记录鼠标初始位置
            setCursor(Qt::ClosedHandCursor);  // 鼠标样式改为闭合抓手
        }
        QFrame::mousePressEvent(event);
    }

    // 鼠标移动事件：处理拖动
    void mouseMoveEvent(QMouseEvent *event) override
    {
        if (is_dragging_) {
            // 计算鼠标移动的偏移量（像素）
            double delta_x = event->x() - last_mouse_pos_.x();
            double delta_y = event->y() - last_mouse_pos_.y();

            // 更新拖动总偏移量（基于当前缩放比例，避免缩放后拖动速度异常）
            drag_offset_x_ += delta_x / scale_factor_;
            drag_offset_y_ += delta_y / scale_factor_;

            // 记录当前鼠标位置
            last_mouse_pos_ = event->pos();

            // 重新计算轨迹点并刷新
            updateScaledPoints();
            update();
        }
        QFrame::mouseMoveEvent(event);
    }

    // 鼠标释放事件：结束拖动
    void mouseReleaseEvent(QMouseEvent *event) override
    {
        if (event->button() == Qt::LeftButton) {
            is_dragging_ = false;
            setCursor(Qt::OpenHandCursor);  // 鼠标样式恢复为开放抓手
        }
        QFrame::mouseReleaseEvent(event);
    }

    // 鼠标滚轮事件：控制缩放（保持原有逻辑）
    void wheelEvent(QWheelEvent *event) override
    {
        if (event->angleDelta().y() > 0) {
            scale_factor_ += scale_step_;
        } else {
            scale_factor_ -= scale_step_;
        }
        scale_factor_ = qBound(min_scale_, scale_factor_, max_scale_);
        updateScaledPoints();
        update();
    }

    // 绘图事件（保持原有逻辑，新增拖动提示）
    void paintEvent(QPaintEvent *event) override
    {
        QFrame::paintEvent(event);
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing, true);

        // 绘制轨迹线
        if (scaled_trajectory_points_.size() > 1) {
            QPen traj_pen(QColor(0, 128, 255), 2);
            painter.setPen(traj_pen);
            for (int i = 1; i < scaled_trajectory_points_.size(); ++i) {
                painter.drawLine(scaled_trajectory_points_[i-1], scaled_trajectory_points_[i]);
            }
        }

        // 绘制当前位置
        if (!scaled_trajectory_points_.isEmpty()) {
            QPen point_pen(Qt::red, 4);
            painter.setPen(point_pen);
            painter.drawPoint(scaled_trajectory_points_.last());
        }

        // 绘制坐标轴（中心点受拖动偏移影响）
        int center_x = width() / 2 + drag_offset_x_ * scale_factor_;
        int center_y = height() / 2 + drag_offset_y_ * scale_factor_;
        QPen axis_pen(Qt::gray, 1);
        painter.setPen(axis_pen);
        painter.drawLine(center_x, 0, center_x, height());  // Y轴
        painter.drawLine(0, center_y, width(), center_y);    // X轴

        // 绘制操作提示文字
        // painter.setPen(Qt::black);
        // painter.drawText(center_x + 5, 20, "X");
        // painter.drawText(width() - 20, center_y - 5, "Y");
        // painter.drawText(10, 20, QString("缩放：鼠标滚轮 | 拖动：左键按住拖动 | 缩放比例: %.1f倍").arg(scale_factor_));
    }

    // 窗口大小变化事件（保持原有逻辑）
    void resizeEvent(QResizeEvent *event) override
    {
        QFrame::resizeEvent(event);
        updateScaledPoints();
    }

private:
    // 计算缩放+拖动后的轨迹点（核心修改：加入拖动偏移量）
    void updateScaledPoints()
    {
        scaled_trajectory_points_.clear();
        int original_center_x = width() / 2;
        int original_center_y = height() / 2;

        for (const QPointF& raw_point : raw_trajectory_points_) {
            // 1. 原始点相对于窗口原始中心的偏移
            double offset_x = raw_point.x() - original_center_x;
            double offset_y = raw_point.y() - original_center_y;

            // 2. 应用缩放（基于原始中心）
            double scaled_offset_x = offset_x * scale_factor_;
            double scaled_offset_y = offset_y * scale_factor_;

            // 3. 应用拖动偏移（调整中心点位置）
            double final_offset_x = scaled_offset_x + drag_offset_x_ * scale_factor_;
            double final_offset_y = scaled_offset_y + drag_offset_y_ * scale_factor_;

            // 4. 计算最终坐标
            QPointF scaled_point(
                original_center_x + final_offset_x,
                original_center_y + final_offset_y
            );
            scaled_trajectory_points_.append(scaled_point);
        }
    }

private:
    QVector<QPointF> raw_trajectory_points_;     // 原始轨迹点
    QVector<QPointF> scaled_trajectory_points_;  // 缩放+拖动后的轨迹点
    double scale_factor_;                        // 缩放因子
    const double scale_step_;                    // 缩放步长
    const double min_scale_;                     // 最小缩放比例
    const double max_scale_;                     // 最大缩放比例

    // 拖动相关成员
    bool is_dragging_;          // 是否正在拖动
    QPoint last_mouse_pos_;     // 上一帧鼠标位置
    double drag_offset_x_ = 0.0; // X轴拖动总偏移量（像素，基于原始中心）
    double drag_offset_y_ = 0.0; // Y轴拖动总偏移量（像素，基于原始中心）
};


// 自定义姿态仪表盘容器
// class DashboardFrame : public QFrame
// {
//     Q_OBJECT
// public:
//     explicit DashboardFrame(QWidget *parent = nullptr) : QFrame(parent)
//     {
//         setStyleSheet("background-color: white; border: 1px solid #dddddd;");
//         setMinimumSize(200, 200);
//         dashboard_radius_ = qMin(width(), height()) / 2 - 10;
//     }

//     // 设置偏航角数据
//     void setYawData(double yaw)
//     {
//         current_yaw_ = yaw;
//         update();  // 触发重绘
//     }

// protected:
//     // 窗口大小变化事件
//     void resizeEvent(QResizeEvent *event) override
//     {
//         QFrame::resizeEvent(event);
//         dashboard_radius_ = qMin(width(), height()) / 2 - 10;
//     }

//     // 绘图事件
//     void paintEvent(QPaintEvent *event) override
//     {
//         QFrame::paintEvent(event);
//         QPainter painter(this);
//         painter.setRenderHint(QPainter::Antialiasing, true);

//         QPointF center(width() / 2, height() / 2);

//         // 绘制仪表盘外圈
//         QPen circle_pen(Qt::lightGray, 3);
//         painter.setPen(circle_pen);
//         painter.drawEllipse(center, dashboard_radius_, dashboard_radius_);

//         // 绘制方向刻度
//         QPen tick_pen(Qt::gray, 2);
//         painter.setPen(tick_pen);
//         QStringList ticks = {"0°", "90°", "180°", "270°"};
//         for (int i = 0; i < 4; ++i) {
//             double angle = i * M_PI / 2;
//             QPointF start(
//                 center.x() + (dashboard_radius_ - 10) * cos(angle),
//                 center.y() - (dashboard_radius_ - 10) * sin(angle)
//             );
//             QPointF end(
//                 center.x() + dashboard_radius_ * cos(angle),
//                 center.y() - dashboard_radius_ * sin(angle)
//             );
//             painter.drawLine(start, end);

//             // 绘制刻度文字
//             QPointF text_pos(
//                 center.x() + (dashboard_radius_ - 20) * cos(angle),
//                 center.y() - (dashboard_radius_ - 20) * sin(angle)
//             );
//             painter.drawText(text_pos, ticks[i]);
//         }

//         // 绘制偏航角指针（红色）
//         QPen pointer_pen(Qt::red, 3);
//         painter.setPen(pointer_pen);
//         QPointF pointer_end(
//             center.x() + (dashboard_radius_ - 15) * cos(current_yaw_),
//             center.y() - (dashboard_radius_ - 15) * sin(current_yaw_)
//         );
//         painter.drawLine(center, pointer_end);

//         // 绘制中心圆点
//         QPen center_pen(Qt::black, 5);
//         painter.setPen(center_pen);
//         painter.drawPoint(center);
//     }

// private:
//     double current_yaw_ = 0.0;  // 当前偏航角
//     int dashboard_radius_;      // 仪表盘半径
// };

// 自定义姿态仪表盘容器（美化版）
class DashboardFrame : public QFrame
{
    Q_OBJECT
public:
    explicit DashboardFrame(QWidget *parent = nullptr) : QFrame(parent)
    {
        // 优化背景和边框样式（圆角+阴影）
        setStyleSheet(R"(
            background-color: #f8f9fa;
            border: 2px solid #e0e0e0;
            border-radius: 12px;
            box-shadow: 0 2px 8px rgba(0, 0, 0, 0.08);
        )");
        setMinimumSize(220, 220);
        dashboard_radius_ = qMin(width(), height()) / 2 - 20; // 预留更多边距
    }

    // 设置偏航角数据（支持角度/弧度显示）
    void setYawData(double yaw, bool use_degree = true)
    {
        current_yaw_ = yaw;
        use_degree_ = use_degree;
        update();  // 触发重绘
    }

protected:
    // 窗口大小变化事件
    void resizeEvent(QResizeEvent *event) override
    {
        QFrame::resizeEvent(event);
        dashboard_radius_ = qMin(width(), height()) / 2 - 20;
    }

    // 绘图事件（核心美化逻辑）
    void paintEvent(QPaintEvent *event) override
    {
        QFrame::paintEvent(event);
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing, true); // 抗锯齿，让线条更平滑
        painter.setRenderHint(QPainter::TextAntialiasing, true); // 文字抗锯齿

        QPointF center(width() / 2, height() / 2); // 仪表盘中心

        // 1. 绘制渐变背景（从浅蓝到白色，提升层次感）
        QRadialGradient radial_grad(center, dashboard_radius_ + 10);
        radial_grad.setColorAt(0.0, QColor(230, 245, 255)); // 中心浅蓝色
        radial_grad.setColorAt(0.8, QColor(245, 250, 255)); // 中间过渡色
        radial_grad.setColorAt(1.0, QColor(255, 255, 255)); // 边缘白色
        painter.setBrush(radial_grad);
        painter.setPen(Qt::NoPen);
        painter.drawEllipse(center, dashboard_radius_ + 10, dashboard_radius_ + 10);

        // 2. 绘制仪表盘外圈（双层圆环，更精致）
        // 外层粗圆环（深灰色）
        QPen outer_circle_pen(QColor(100, 100, 100), 4);
        painter.setPen(outer_circle_pen);
        painter.drawEllipse(center, dashboard_radius_, dashboard_radius_);
        // 内层细圆环（浅蓝色）
        QPen inner_circle_pen(QColor(150, 200, 255), 2);
        painter.setPen(inner_circle_pen);
        painter.drawEllipse(center, dashboard_radius_ - 5, dashboard_radius_ - 5);

        // 3. 绘制细化刻度（每30°一个主刻度，每10°一个副刻度）
        drawTicks(painter, center);

        // 4. 绘制方向标识文字（优化位置和样式）
        drawDirectionLabels(painter, center);

        // 5. 绘制偏航角指针（箭头样式+渐变，更醒目）
        drawYawPointer(painter, center);

        // 6. 绘制中心区域（圆点+背景圆）
        drawCenterArea(painter, center);

        // 7. 绘制实时角度显示框（显示当前偏航角数值）
        drawAngleDisplay(painter, center);
    }

private:
    // 绘制刻度（主刻度+副刻度）
    void drawTicks(QPainter &painter, const QPointF &center)
    {
        // 主刻度（30°，深色粗线）
        QPen main_tick_pen(QColor(80, 80, 80), 2);
        // 副刻度（10°，浅色细线）
        QPen sub_tick_pen(QColor(180, 180, 180), 1);

        for (int i = 0; i < 36; ++i) { // 36个副刻度（每10°一个）
            double angle = i * M_PI / 18; // 10° = π/18 弧度
            int tick_length = (i % 3 == 0) ? 15 : 8; // 主刻度长15px，副刻度长8px
            painter.setPen((i % 3 == 0) ? main_tick_pen : sub_tick_pen);

            QPointF start(
                center.x() + (dashboard_radius_ - tick_length) * cos(angle),
                center.y() - (dashboard_radius_ - tick_length) * sin(angle)
            );
            QPointF end(
                center.x() + dashboard_radius_ * cos(angle),
                center.y() - dashboard_radius_ * sin(angle)
            );
            painter.drawLine(start, end);
        }
    }

    // 绘制方向标识（N/E/S/W 替代原角度文字，更直观）
    void drawDirectionLabels(QPainter &painter, const QPointF &center)
    {
        painter.setPen(QColor(60, 60, 60));
        QFont label_font("Arial", 12, QFont::Bold);
        painter.setFont(label_font);

        // 方向映射：0°=北(N)，90°=东(E)，180°=南(S)，270°=西(W)
        QMap<double, QString> direction_map = {
            {0.0, "N"},
            {M_PI/2, "E"},
            {M_PI, "S"},
            {3*M_PI/2, "W"}
        };

        for (auto it = direction_map.constBegin(); it != direction_map.constEnd(); ++it) {
            double angle = it.key();
            QString label = it.value();

            // 文字位置向外偏移，避免遮挡刻度
            QPointF text_pos(
                center.x() + (dashboard_radius_ - 30) * cos(angle) - label.size() * 4,
                center.y() - (dashboard_radius_ - 30) * sin(angle) + 6
            );
            painter.drawText(text_pos, label);
        }
    }

    // 绘制偏航角指针（箭头样式+渐变填充）
    void drawYawPointer(QPainter &painter, const QPointF &center)
    {
        // 指针路径（箭头形状）
        QPolygonF pointer_shape;
        double pointer_length = dashboard_radius_ - 25; // 指针长度
        // 箭头头部三点
        pointer_shape.append(center);
        pointer_shape.append(QPointF(
            center.x() + pointer_length * cos(current_yaw_),
            center.y() - pointer_length * sin(current_yaw_)
        ));
        pointer_shape.append(QPointF(
            center.x() + 10 * cos(current_yaw_ + M_PI/6),
            center.y() - 10 * sin(current_yaw_ + M_PI/6)
        ));
        pointer_shape.append(QPointF(
            center.x() + 10 * cos(current_yaw_ - M_PI/6),
            center.y() - 10 * sin(current_yaw_ - M_PI/6)
        ));
        pointer_shape.append(center);

        // 指针渐变填充（从红色到橙色，更有立体感）
        QLinearGradient pointer_grad(center, pointer_shape.at(1));
        pointer_grad.setColorAt(0.0, QColor(255, 60, 60));
        pointer_grad.setColorAt(1.0, QColor(255, 120, 60));
        painter.setBrush(pointer_grad);
        painter.setPen(QColor(200, 40, 40)); // 指针边框
        painter.drawPolygon(pointer_shape);
    }

    // 绘制中心区域（圆点+浅色背景圆）
    void drawCenterArea(QPainter &painter, const QPointF &center)
    {
        // 中心背景圆（浅蓝色）
        painter.setBrush(QColor(200, 230, 255));
        painter.setPen(Qt::NoPen);
        painter.drawEllipse(center, 12, 12);

        // 中心圆点（深灰色）
        painter.setBrush(QColor(80, 80, 80));
        painter.drawEllipse(center, 6, 6);
    }

    // 绘制实时角度显示框（半透明背景+清晰文字）
    void drawAngleDisplay(QPainter &painter, const QPointF &center)
    {
        // 计算显示的角度值（度/弧度转换）
        double display_angle = use_degree_ ? current_yaw_ * 180 / M_PI : current_yaw_;
        QString angle_text = QString::number(display_angle, 'f', 1) + (use_degree_ ? "°" : "rad");

        // 显示框背景（半透明白色，圆角）
        QRectF text_rect(center.x() - 57, center.y() + 15, 120, 30);
        // painter.setBrush(QColor(255, 255, 255, 200)); // 半透明白色
        // painter.setPen(QColor(200, 200, 200));
        // painter.drawRoundedRect(text_rect, 15, 15); // 圆角矩形

        // 角度文字（加粗，居中）
        //QRectF text_draw_rect = text_rect.translated(0, -14);
        painter.setPen(QColor(60, 60, 60));
        QFont angle_font("Arial", 14, QFont::Bold);
        painter.setFont(angle_font);
        painter.drawText(text_rect, Qt::AlignCenter, angle_text);

        // 标签文字（小字体，显示"偏航角"）
        // painter.setPen(QColor(120, 120, 120));
        // QFont label_font("Arial", 10);
        // painter.setFont(label_font);
        //painter.drawText(center.x() - 25, center.y() + 35, "偏航角");
    }

private:
    double current_yaw_ = 0.0;  // 当前偏航角
    int dashboard_radius_;      // 仪表盘半径
    bool use_degree_ = true;    // 是否显示角度（默认度，支持切换）
};


// 主窗口类
class OdomViewerWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit OdomViewerWindow(QWidget *parent = nullptr);
    ~OdomViewerWindow();

private slots:
    // 按钮槽函数
    void on_startBtn_clicked();
    void on_clearBtn_clicked();
    void on_unitBtn_clicked();
    void on_resetScaleBtn_clicked();

    // 里程计数据处理槽函数
    void handleOdomData(const OdometryData& data);

private:
    // 初始化函数
    void initUI();
    void createWidgets();
    void setupLayouts();
    void initROS2Subscriber();

    // ROS2 数据回调（线程安全）
    void rosOdomCallback(const OdometryData& data);

    // 数据存储
    OdometryData current_odom_;
    bool is_updating_ = true;
    bool use_degree_ = true;
    QVector<QPointF> trajectory_points_;
    const int trajectory_max_points_ = 500;  // 最大轨迹点数量

    // ROS2 相关
    std::shared_ptr<OdomSubscriber> odom_subscriber_;
    QFuture<void> ros_spin_future_;
    bool ros_running_ = false;

    // UI 控件
    QWidget *central_widget_;
    QLabel *ros_status_label_;
    QPushButton *start_btn_;
    QPushButton *clear_btn_;
    QPushButton *unit_btn_;
    QPushButton *reset_scale_btn_;
    QLabel *angle_unit_label_;

    // 数据显示标签
    QLabel *x_label_, *x_value_;
    QLabel *y_label_, *y_value_;
    QLabel *z_label_, *z_value_;
    QLabel *roll_label_, *roll_value_;
    QLabel *pitch_label_, *pitch_value_;
    QLabel *yaw_label_, *yaw_value_;
    QLabel *linear_x_label_, *linear_x_value_;
    QLabel *angular_z_label_, *angular_z_value_;
    QLabel *timestamp_label_, *timestamp_value_;

    // 自定义绘图容器
    TrajectoryFrame *trajectory_frame_;
    DashboardFrame *dashboard_frame_;

    // 布局管理器
    QGridLayout *main_layout_;
    QHBoxLayout *top_layout_;
    QGridLayout *data_layout_;
};

#endif // ODOM_VIEWER_WINDOW_H
