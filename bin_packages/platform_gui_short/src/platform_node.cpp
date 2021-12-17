#include "platform_node.hpp"

// --------------------------------------------------- platform_node --------------------------------------------------- //

platform_node::platform_node(std::string imgname) : rclcpp::Node ("platform_gui")
{
  auto func_ = [this](const sensor_msgs::msg::Image::SharedPtr msg)
  {
    this->imgCallback(msg);
  };  

  rclcpp::QoS qos_custom(rclcpp::KeepLast(5));
  qos_custom.transient_local();

  this->sub_image = this->create_subscription<sensor_msgs::msg::Image>(imgname, rclcpp::ClockQoS(), func_);

  this->pub_waypoint = this->create_publisher<geometry_msgs::msg::Point32>("waypoint", 5);
  this->pub_bbox = this->create_publisher<geometry_msgs::msg::Polygon>("selected_area", 5);
  this->pub_cmd_navigator = this->create_publisher<std_msgs::msg::String>("cmd_navigator", 5);
  this->pub_cmd_controller = this->create_publisher<std_msgs::msg::String>("cmd_controller", 5);
  this->pub_selected_btn = this->create_publisher<std_msgs::msg::String>("selected_button", 5);

  RCLCPP_INFO(this->get_logger(),"node initalized!");
}

void platform_node::imgCallback(const sensor_msgs::msg::Image::SharedPtr msg) const
{
  cv_bridge::CvImagePtr img_ptr;
  try {
    img_ptr = cv_bridge::toCvCopy(msg, msg->encoding);    
    this->callback_node(img_ptr->image);    
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

