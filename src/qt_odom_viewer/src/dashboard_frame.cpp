#include "qt_odom_viewer/dashboard_frame.h"
#include "qt_odom_viewer/odom_subscriber.h"

#include <QPainter>
#include <QPen>
#include <QBrush>
#include <QRadialGradient>
#include <QLinearGradient>
#include <QFont>
#include <QPolygonF>
#include <QRectF>
#include <cmath>  // 使用C++标准数学函数库

// 初始化静态成员（方向映射）
const QMap<double, QString> DashboardFrame::direction_map_ = {
    {0.0, "N"},
    {M_PI/2, "E"},
    {M_PI, "S"},
    {3*M_PI/2, "W"}
};

DashboardFrame::DashboardFrame(QWidget *parent)
    : QFrame(parent)
    , current_yaw_(0.0)
    , dashboard_radius_(0)
    , use_degree_(true)
{
    setMinimumSize(220, 220);

    // 设置样式（圆角边框，稍微的阴影效果）
    setStyleSheet(R"(
        background-color: #f8f9fa;
        border: 2px solid #e0e0e0;
        border-radius: 12px;
        box-shadow: 0 2px 8px rgba(0, 0, 0, 0.08);
    )");

    // 初始化仪表盘半径
    dashboard_radius_ = qMin(width(), height()) / 2 - 20;
}

void DashboardFrame::setYawData(double yaw, bool use_degree)
{
    current_yaw_ = yaw;
    use_degree_ = use_degree;
    update();  // 触发重绘
}

void DashboardFrame::resizeEvent(QResizeEvent *event)
{
    QFrame::resizeEvent(event);
    // 更新仪表盘半径
    dashboard_radius_ = qMin(width(), height()) / 2 - 20;
}

void DashboardFrame::paintEvent(QPaintEvent *event)
{
    QFrame::paintEvent(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setRenderHint(QPainter::TextAntialiasing, true);

    QPointF center(width() / 2, height() / 2);  // 仪表盘中心

    // 1. 绘制渐进式背景（从浅蓝到白色）
    QRadialGradient radial_grad(center, dashboard_radius_ + 10);
    radial_grad.setColorAt(0.0, QColor(230, 245, 255)); // 中心浅蓝色
    radial_grad.setColorAt(0.8, QColor(245, 250, 255)); // 中间过渡色
    radial_grad.setColorAt(1.0, QColor(255, 255, 255)); // 边缘白色

    painter.setBrush(radial_grad);
    painter.setPen(Qt::NoPen);
    painter.drawEllipse(center, dashboard_radius_ + 10, dashboard_radius_ + 10);

    // 2. 绘制双层圆环（外圈+内圈）
    QColor outer_circle_color(100, 100, 100);  // 深灰色
    QColor inner_circle_color(150, 200, 255);  // 浅蓝色

    // 外层粗圆环
    painter.setPen(QPen(outer_circle_color, 4));
    painter.setBrush(Qt::NoBrush);
    painter.drawEllipse(center, dashboard_radius_, dashboard_radius_);

    // 内层细圆环
    painter.setPen(QPen(inner_circle_color, 2));
    painter.drawEllipse(center, dashboard_radius_ - 5, dashboard_radius_ - 5);

    // 3. 绘制细分刻度（主刻度+副刻度）
    drawTicks(painter, center);

    // 4. 绘制方向标识（N/E/S/W）
    drawDirectionLabels(painter, center);

    // 5. 绘制偏航角指针（箭头样式+渐变填充）
    drawYawPointer(painter, center);

    // 6. 绘制中心区域
    drawCenterArea(painter, center);

    // 7. 绘制实时角度显示
    drawAngleDisplay(painter, center);
}

void DashboardFrame::drawTicks(QPainter &painter, const QPointF &center)
{
    // 主刻度（30°，深色粗线）
    QPen main_tick_pen(QColor(80, 80, 80), 2);
    // 副刻度（10°，浅色细线）
    QPen sub_tick_pen(QColor(180, 180, 180), 1);

    // 36个刻度，每10°一个
    for (int i = 0; i < 36; ++i) {
        double angle = i * M_PI / 18;  // 10° = π/18 弧度
        int tick_length = (i % 3 == 0) ? 15 : 8;  // 主刻度长，副刻度短

        painter.setPen((i % 3 == 0) ? main_tick_pen : sub_tick_pen);

        // 刻度起点和终点
        QPointF start(
            center.x() + (dashboard_radius_ - tick_length) * std::cos(angle),
            center.y() - (dashboard_radius_ - tick_length) * std::sin(angle)
        );
        QPointF end(
            center.x() + dashboard_radius_ * std::cos(angle),
            center.y() - dashboard_radius_ * std::sin(angle)
        );

        painter.drawLine(start, end);
    }
}

void DashboardFrame::drawDirectionLabels(QPainter &painter, const QPointF &center)
{
    painter.setPen(QColor(60, 60, 60));
    QFont label_font("Arial", 12, QFont::Bold);
    painter.setFont(label_font);

    for (auto it = direction_map_.constBegin(); it != direction_map_.constEnd(); ++it) {
        double angle = it.key();
        QString label = it.value();

        // 计算文字位置（向外偏移，避免遮挡刻度）
        QPointF text_pos(
            center.x() + (dashboard_radius_ - 30) * std::cos(angle) - label.size() * 4,
            center.y() - (dashboard_radius_ - 30) * std::sin(angle) + 6
        );

        painter.drawText(text_pos, label);
    }
}

void DashboardFrame::drawYawPointer(QPainter &painter, const QPointF &center)
{
    // 定义箭头形状（多边形）
    QPolygonF pointer_shape;
    double pointer_length = dashboard_radius_ - 25;  // 指针长度

    // 箭头头部三点
    pointer_shape.append(center);  // 起点（中心点）
    pointer_shape.append(QPointF(
        center.x() + pointer_length * std::cos(current_yaw_),
        center.y() - pointer_length * std::sin(current_yaw_)
    ));

    double arrow_angle = M_PI / 6;  // 箭头角度（30°）
    double arrow_length = 10;       // 箭头长度

    pointer_shape.append(QPointF(
        center.x() + arrow_length * std::cos(current_yaw_ + arrow_angle),
        center.y() - arrow_length * std::sin(current_yaw_ + arrow_angle)
    ));

    pointer_shape.append(QPointF(
        center.x() + arrow_length * std::cos(current_yaw_ - arrow_angle),
        center.y() - arrow_length * std::sin(current_yaw_ - arrow_angle)
    ));

    pointer_shape.append(center);  // 回到起点

    // 渐变色填充
    QLinearGradient pointer_grad(center, pointer_shape.at(1));
    pointer_grad.setColorAt(0.0, QColor(255, 60, 60));   // 红色起点
    pointer_grad.setColorAt(1.0, QColor(255, 120, 60));  // 橙色终点

    painter.setBrush(pointer_grad);
    painter.setPen(QColor(200, 40, 40));  // 深红色边框
    painter.drawPolygon(pointer_shape);
}

void DashboardFrame::drawCenterArea(QPainter &painter, const QPointF &center)
{
    // 中心背景圆（浅蓝色）
    painter.setBrush(QColor(200, 230, 255));
    painter.setPen(Qt::NoPen);
    painter.drawEllipse(center, 12, 12);

    // 中心圆点（深灰色，小号）
    painter.setBrush(QColor(80, 80, 80));
    painter.drawEllipse(center, 6, 6);
}

void DashboardFrame::drawAngleDisplay(QPainter &painter, const QPointF &center)
{
    // 计算显示角度（度/弧度转换）
    double display_angle = use_degree_ ? current_yaw_ * 180 / M_PI : current_yaw_;
    QString angle_text = QString::number(display_angle, 'f', 1) + (use_degree_ ? "°" : "rad");

    // 角度显示区域
    QRectF text_rect(center.x() - 57, center.y() + 15, 120, 30);

    // 角度文字
    painter.setPen(QColor(60, 60, 60));
    QFont angle_font("Arial", 14, QFont::Bold);
    painter.setFont(angle_font);
    painter.drawText(text_rect, Qt::AlignCenter, angle_text);
}