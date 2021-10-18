#ifndef PLATFORM_GUI_H
#define PLATFORM_GUI_H

#include <QMainWindow>
#include <rclcpp/rclcpp.hpp>
#include "platform_node.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class platform_gui; }
QT_END_NAMESPACE

class platform_gui : public QMainWindow
{
    Q_OBJECT

public:
    platform_gui(QWidget *parent = nullptr);
    ~platform_gui();
    std::shared_ptr<platform_node> node_ptr;
    void callback(cv::Mat mat);
    void init_ros2();    

private:
    Ui::platform_gui *ui;
    std::unique_ptr<spin_thread> thread;
};
#endif // PLATFORM_GUI_H
