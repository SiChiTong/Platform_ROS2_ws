#ifndef PLATFORM_NODE_HPP
#define PLATFORM_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <memory>
#include <QtDebug>
#include <QThread>
#include <functional>

typedef std::function<void(cv::Mat)> cvImgCallback;

class spin_thread;

class platform_node : public rclcpp::Node
{
public:
  platform_node(std::string imgname);
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image;
  void imgCallback(const sensor_msgs::msg::Image::SharedPtr msg) const;
  void set_callback(cvImgCallback callback);

private:
  cvImgCallback callback_node;
};

class spin_thread : public QThread
{
  Q_OBJECT

public:
  explicit spin_thread(QObject* parent = 0);
  void spin_node(platform_node::SharedPtr ptr);
  void stop_node();
  platform_node::SharedPtr node_ptr_ = nullptr;

private:
  void run();

};


#endif // PLATFORM_NODE_HPP