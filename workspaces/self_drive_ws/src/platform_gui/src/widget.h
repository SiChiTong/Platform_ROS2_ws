#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QtSerialPort/QSerialPort>
#include <QtNetwork>

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

    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

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

private:
    Ui::Widget *ui;
    QSerialPort *target_board;
    static const quint16 target_board_vender_id = 1155; //arduino 9025, arduino_softwareserial 1659, mbed 1155
    static const quint16 target_board_product_id = 14155;  // arduino 67, arduino_softwareserial 8963, mbed 14155
    QString target_board_name;
    bool target_available;
    QTimer *ip_connect_timer;

    std::unique_ptr<spin_thread> thread;

};
#endif // WIDGET_H
