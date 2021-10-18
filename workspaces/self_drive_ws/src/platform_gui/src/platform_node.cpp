#include "platform_node.hpp"

// --------------------------------------------------- platform_node --------------------------------------------------- //

platform_node::platform_node(std::string imgname) : rclcpp::Node ("platform_gui")
{
  auto func_ = [this](const sensor_msgs::msg::Image::SharedPtr msg)
  {
    this->imgCallback(msg);
  };  
  this->sub_image = this->create_subscription<sensor_msgs::msg::Image>(imgname, rclcpp::SensorDataQoS(), func_);
  this->pub_waypoint = this->create_publisher<std_msgs::msg::Int8>("waypoint", 10);
  this->pub_cmd_W_Br = this->create_publisher<std_msgs::msg::String>("cmd_w_br", 10);
  RCLCPP_INFO(this->get_logger(),"node initalized!");
}

void platform_node::imgCallback(const sensor_msgs::msg::Image::SharedPtr msg) const
{
  cv_bridge::CvImagePtr img_ptr;
  try {
    img_ptr = cv_bridge::toCvCopy(msg, msg->encoding);    
    this->callback_node(img_ptr->image);
    RCLCPP_INFO(this->get_logger(),"get image!");
  }
  catch (cv_bridge::Exception& e) {
    RCLCPP_INFO(this->get_logger(),"%s",e.what());
  }
}

void platform_node::set_callback(cvImgCallback callback)
{
  this->callback_node = std::move(callback);
}

// --------------------------------------------------- spin_thread --------------------------------------------------- //

spin_thread::spin_thread(QObject* parent) : QThread (parent)
{

}

void spin_thread::run()
{
  rclcpp::spin(node_ptr_);
}

void spin_thread::spin_node(platform_node::SharedPtr ptr)
{
  this->node_ptr_ = ptr;
  this->start();
}

void spin_thread::stop_node()
{
  if(node_ptr_ != nullptr) rclcpp::shutdown();
}

