#ifndef DASHBOARD_FRAME_H
#define DASHBOARD_FRAME_H

#include <QFrame>
#include <QColor>
#include <QMap>
#include <cmath>

/**
 * @brief 姿态仪表盘控件
 *
 * 功能：
 * - 显示偏航角（航向角）
 * - 支持度/弧度单位切换
 * - 美观的双层圆环设计
 * - 每10°一个刻度（主刻度30°）
 * - 箭头指针显示当前偏航角
 * - 实时角度数值显示
 * - 渐变背景和阴影效果
 */
class DashboardFrame : public QFrame
{
    Q_OBJECT

public:
    /**
     * @brief 构造函数
     * @param parent 父窗口指针
     */
    explicit DashboardFrame(QWidget *parent = nullptr);

    /**
     * @brief 设置偏航角数据
     * @param yaw 偏航角（弧度）
     * @param use_degree 是否使用角度（默认使用角度）
     */
    void setYawData(double yaw, bool use_degree = true);

protected:
    /**
     * @brief 窗口大小变化事件
     * @param event 大小事件
     */
    void resizeEvent(QResizeEvent *event) override;

    /**
     * @brief 重绘事件
     * @param event 重绘事件
     */
    void paintEvent(QPaintEvent *event) override;

private:
    /**
     * @brief 绘制刻度线
     * @param painter 绘图对象
     * @param center 仪表盘中心点
     */
    void drawTicks(QPainter &painter, const QPointF &center);

    /**
     * @brief 绘制方向标识（N/E/S/W）
     * @param painter 绘图对象
     * @param center 仪表盘中心点
     */
    void drawDirectionLabels(QPainter &painter, const QPointF &center);

    /**
     * @brief 绘制偏航角指针
     * @param painter 绘图对象
     * @param center 仪表盘中心点
     */
    void drawYawPointer(QPainter &painter, const QPointF &center);

    /**
     * @brief 绘制中心区域
     * @param painter 绘图对象
     * @param center 仪表盘中心点
     */
    void drawCenterArea(QPainter &painter, const QPointF &center);

    /**
     * @brief 绘制角度显示
     * @param painter 绘图对象
     * @param center 仪表盘中心点
     */
    void drawAngleDisplay(QPainter &painter, const QPointF &center);

private:
    double current_yaw_;        /**< 当前偏航角（弧度） */
    int dashboard_radius_;      /**< 仪表盘半径 */
    bool use_degree_;           /**< 是否使用角度单位 */
    static const QMap<double, QString> direction_map_;  /**< 方向映射 */
};

#endif // DASHBOARD_FRAME_H