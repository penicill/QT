#include "qt_odom_viewer/trajectory_frame.h"
#include "qt_odom_viewer/odom_subscriber.h"

#include <QPainter>
#include <QMouseEvent>
#include <QWheelEvent>
#include <cmath>  // 添加数学函数库

TrajectoryFrame::TrajectoryFrame(QWidget *parent)
    : QFrame(parent)
    , scale_factor_(1.0)
    , scale_step_(0.1)
    , min_scale_(0.1)
    , max_scale_(10.0)
    , is_dragging_(false)
    , drag_offset_x_(0.0)
    , drag_offset_y_(0.0)
{
    // 设置样式
    setStyleSheet("background-color: white; border: 1px solid #dddddd;");
    setMinimumSize(800, 300);
    setMouseTracking(true);

    // 设置鼠标样式
    setCursor(Qt::OpenHandCursor);
}

void TrajectoryFrame::setTrajectoryData(const QVector<QPointF>& points)
{
    raw_trajectory_points_ = points;
    updateScaledPoints();
    update();  // 触发重绘
}

void TrajectoryFrame::resetScaleAndOffset()
{
    scale_factor_ = 1.0;
    drag_offset_x_ = 0.0;
    drag_offset_y_ = 0.0;
    updateScaledPoints();
    update();
}

void TrajectoryFrame::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        is_dragging_ = true;
        last_mouse_pos_ = event->pos();
        setCursor(Qt::ClosedHandCursor);  // 改为抓取样式
    }
    QFrame::mousePressEvent(event);
}

void TrajectoryFrame::mouseMoveEvent(QMouseEvent *event)
{
    if (is_dragging_) {
        // 计算鼠标移动偏移量
        double delta_x = event->x() - last_mouse_pos_.x();
        double delta_y = event->y() - last_mouse_pos_.y();

        // 更新拖动偏移量
        drag_offset_x_ += delta_x / scale_factor_;
        drag_offset_y_ += delta_y / scale_factor_;

        // 记录当前鼠标位置
        last_mouse_pos_ = event->pos();

        // 重新计算轨迹点
        updateScaledPoints();
        update();  // 刷新
    }
    QFrame::mouseMoveEvent(event);
}

void TrajectoryFrame::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        is_dragging_ = false;
        setCursor(Qt::OpenHandCursor);  // 恢复开放抓手样式
    }
    QFrame::mouseReleaseEvent(event);
}

void TrajectoryFrame::wheelEvent(QWheelEvent *event)
{
    // 滚轮向前：放大，滚轮向后：缩小
    if (event->angleDelta().y() > 0) {
        scale_factor_ += scale_step_;
    } else {
        scale_factor_ -= scale_step_;
    }

    // 限制缩放范围
    scale_factor_ = qBound(min_scale_, scale_factor_, max_scale_);

    // 更新轨迹点
    updateScaledPoints();
    update();
}

void TrajectoryFrame::paintEvent(QPaintEvent *event)
{
    QFrame::paintEvent(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);  // 抗锯齿

    // 绘制轨迹线
    if (scaled_trajectory_points_.size() > 1) {
        QPen traj_pen(QColor(0, 128, 255), 2);  // 蓝色，2像素宽
        painter.setPen(traj_pen);

        // 绘制连续的线段
        for (int i = 1; i < scaled_trajectory_points_.size(); ++i) {
            painter.drawLine(scaled_trajectory_points_[i-1], scaled_trajectory_points_[i]);
        }
    }

    // 绘制当前位置点
    if (!scaled_trajectory_points_.isEmpty()) {
        QPen point_pen(Qt::red, 4);
        painter.setPen(point_pen);
        painter.drawPoint(scaled_trajectory_points_.last());
    }

    // 绘制坐标轴
    int center_x = width() / 2 + drag_offset_x_ * scale_factor_;
    int center_y = height() / 2 + drag_offset_y_ * scale_factor_;

    QPen axis_pen(Qt::gray, 1);
    painter.setPen(axis_pen);
    painter.drawLine(center_x, 0, center_x, height());     // Y轴
    painter.drawLine(0, center_y, width(), center_y);      // X轴

    // 可选：绘制操作提示
    // painter.setPen(Qt::black);
    // painter.drawText(10, 20, QString("缩放：鼠标滚轮 | 拖动：左键按住拖动 |
    //                                      缩放比例: %.1f倍").arg(scale_factor_));
}

void TrajectoryFrame::resizeEvent(QResizeEvent *event)
{
    QFrame::resizeEvent(event);
    updateScaledPoints();
}

void TrajectoryFrame::updateScaledPoints()
{
    scaled_trajectory_points_.clear();

    int original_center_x = width() / 2;
    int original_center_y = height() / 2;

    for (const QPointF& raw_point : raw_trajectory_points_) {
        // 1. 计算相对于原始中心的偏移
        double offset_x = raw_point.x() - original_center_x;
        double offset_y = raw_point.y() - original_center_y;

        // 2. 应用缩放
        double scaled_offset_x = offset_x * scale_factor_;
        double scaled_offset_y = offset_y * scale_factor_;

        // 3. 应用拖动偏移
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