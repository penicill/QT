#ifndef TRAJECTORY_FRAME_H
#define TRAJECTORY_FRAME_H

#include <QFrame>
#include <QVector>
#include <QPointF>
#include <QColor>

/**
 * @brief 轨迹显示控件
 *
 * 功能：
 * - 显示机器人的移动轨迹
 * - 支持鼠标滚轮缩放
 * - 支持鼠标左键拖动
 * - 滚动缩放比例：0.1x - 10.0x
 * - 最大显示500个轨迹点
 */
class TrajectoryFrame : public QFrame
{
    Q_OBJECT

public:
    /**
     * @brief 构造函数
     * @param parent 父窗口指针
     */
    explicit TrajectoryFrame(QWidget *parent = nullptr);

    /**
     * @brief 设置轨迹数据
     * @param points 轨迹点列表
     */
    void setTrajectoryData(const QVector<QPointF>& points);

    /**
     * @brief 重置缩放和拖动偏移
     */
    void resetScaleAndOffset();

protected:
    /**
     * @brief 鼠标按下事件
     * @param event 鼠标事件
     */
    void mousePressEvent(QMouseEvent *event) override;

    /**
     * @brief 鼠标移动事件
     * @param event 鼠标事件
     */
    void mouseMoveEvent(QMouseEvent *event) override;

    /**
     * @brief 鼠标释放事件
     * @param event 鼠标事件
     */
    void mouseReleaseEvent(QMouseEvent *event) override;

    /**
     * @brief 鼠标滚轮事件
     * @param event 滚轮事件
     */
    void wheelEvent(QWheelEvent *event) override;

    /**
     * @brief 重绘事件
     * @param event 重绘事件
     */
    void paintEvent(QPaintEvent *event) override;

    /**
     * @brief 窗口大小变化事件
     * @param event 大小事件
     */
    void resizeEvent(QResizeEvent *event) override;

private:
    /**
     * @brief 更新缩放后的轨迹点
     */
    void updateScaledPoints();

private:
    QVector<QPointF> raw_trajectory_points_;     /**< 原始轨迹点 */
    QVector<QPointF> scaled_trajectory_points_;  /**< 缩放+拖动后的轨迹点 */

    double scale_factor_;                        /**< 缩放因子 */
    const double scale_step_;                    /**< 缩放步长 */
    const double min_scale_;                     /**< 最小缩放比例 */
    const double max_scale_;                     /**< 最大缩放比例 */

    // 拖动相关
    bool is_dragging_;                          /**< 是否正在拖动 */
    QPoint last_mouse_pos_;                     /**< 上一帧鼠标位置 */
    double drag_offset_x_;                       /**< X轴拖动偏移量 */
    double drag_offset_y_;                       /**< Y轴拖动偏移量 */
};

#endif // TRAJECTORY_FRAME_H