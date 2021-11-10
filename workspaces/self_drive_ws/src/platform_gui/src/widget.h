#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QtNetwork>
#include <QDebug>
#include <QMessageBox>
#include <QImage>
#include <QGraphicsPixmapItem>
#include <QtNetwork>
#include <QProcess>
#include <QTcpSocket>
#include <QNetworkInterface>
#include <QTimer>
#include <QMouseEvent>
#include <QPainter>
#include <QBrush>

#include <rclcpp/rclcpp.hpp>
#include "platform_node.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();    

    std::shared_ptr<platform_node> node_ptr;
    std::shared_ptr<QGraphicsScene> scene_map;
    void callback(cv::Mat mat);
    void init_ros2(int argc, char *argv[]);

private slots:
    void on_Front_clicked();

    void on_Left_Front_clicked();

    void on_Right_Front_clicked();

    void on_Left_clicked();

    void on_Right_clicked();

    void on_Left_Back_clicked();

    void on_Back_clicked();

    void on_Right_Back_clicked();

    void on_Stop_clicked();

    void pic_auto_clicked();

    void on_btn_mapping_clicked();

    void on_btn_auto_clicked();

    void on_btn_manual_clicked();

    void on_btn_setting_clicked();

    void IP_Connect_Timer_Callback();

    void init_tab_mapping();

    void on_Zeus_Widget_currentChanged(int index);

    void on_btn_change_brush_clicked();

    void on_btn_manual_setting_clicked();

    void on_btn_mapping_pressed();

    void on_btn_mapping_released();

    void on_btn_auto_pressed();

    void on_btn_auto_released();

    void on_btn_manual_pressed();

    void on_btn_manual_released();

    void on_btn_setting_pressed();

    void on_btn_setting_released();

    void on_btn_reset_clicked();

    void on_btn_sucktion_clicked();

    void on_btn_sucktion_pressed();

    void on_btn_brush_clicked();

    void on_btn_brush_pressed();

    void on_btn_brush_released();

    void on_btn_sucktion_released();

    void on_waypoint_1_clicked();

    void on_waypoint_2_clicked();

    void on_waypoint_3_clicked();

    void on_waypoint_4_clicked();

    void on_waypoint_5_clicked();

    void on_brush_on_clicked();

    void on_Brush_OFF_clicked();

    void on_pump_on_clicked();

    void on_Pump_OFF_clicked();

    void on_lbl_map_img_clicked(QMouseEvent *);

    void on_lbl_map_img_moved(QMouseEvent *);

    void on_lbl_map_img_release(QMouseEvent *);

    void on_lbl_reset_area_clicked();

    void on_lbl_start_clean_clicked();

    void on_lbl_set_bbox_clicked();

    void on_vacumm_on_clicked();

    void on_vacumm_OFF_clicked();

private:
    Ui::Widget *ui;            
    bool target_available;
    QTimer *ip_connect_timer;

    QPoint m_selectedArea_LU, m_selectedArea_RD;

    std::unique_ptr<spin_thread> thread;

};
#endif // WIDGET_H
