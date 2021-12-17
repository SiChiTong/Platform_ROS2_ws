#include "widget.h"
#include "ui_widget.h"
#include "clickablelabel.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui_c.h>
#include "image_converter.hpp"

bool brush_change_flag = false;
int brush_power_level = 1;
int sucktion_power_lever = 1;

ClickableLabel::ClickableLabel(QWidget* parent, Qt::WindowFlags f)
    : QLabel(parent) {

}

ClickableLabel::~ClickableLabel() {}

void ClickableLabel::mousePressEvent(QMouseEvent* event) {
    emit clicked(event);
}

void ClickableLabel::mouseMoveEvent(QMouseEvent* event) {
    emit moved(event);
}

void ClickableLabel::mouseReleaseEvent(QMouseEvent* event) {
    emit release(event);
}


void init_setting(Ui::Widget *ui)
{
  QPixmap icon_map, icon_auto, icon_manual, icon_setting, icon_sucktion, icon_brush, icon_bottle, icon_logo, icon_cleaning, icon_comeback, icon_charge, icon_water_in, icon_water_out,
      icon_txt_map, icon_txt_auto, icon_txt_manual, icon_txt_setting, icon_charge_gage, icon_charge1, icon_charge2, icon_charge3, icon_robot_speed, icon_water_clean, icon_water_dirt;

  icon_robot_speed.load(":img/speed100.png");
  ui->lbl_robot_speed->setPixmap(icon_robot_speed);


  icon_water_clean.load(":img/water_blue.png");
  ui->water_clean->setPixmap(icon_water_clean);
  icon_water_dirt.load(":img/water_red.png");
  ui->water_dirt->setPixmap(icon_water_dirt);

  icon_charge_gage.load(":img/empty-battery-512-removebg-preview.png");
  ui->lbl_charge_gage->setPixmap(icon_charge_gage);
  icon_charge1.load(":img/battery_green.png");
  ui->lbl_charge1->setPixmap(icon_charge1);
  icon_charge2.load(":img/battery_green.png");
  ui->lbl_charge2->setPixmap(icon_charge2);
  icon_charge3.load(":img/battery_green.png");
  ui->lbl_charge3->setPixmap(icon_charge3);

  icon_map.load(":img/pic_mapping.jpg");
  ui->btn_mapping->setIcon(icon_map);
  icon_auto.load(":img/pic_auto.jpg");
  ui->btn_auto->setIcon(icon_auto);
  icon_manual.load(":img/pic_auto.jpg");
  ui->btn_manual->setIcon(icon_manual);
  icon_setting.load(":img/pic_setting.jpg");
  ui->btn_setting->setIcon(icon_setting);

  icon_txt_map.load(":img/mapping.png");
  ui->txt_mapping->setPixmap(icon_txt_map);
  icon_txt_auto.load(":img/autoclean.png");
  ui->txt_mapping_2->setPixmap(icon_txt_auto);
  icon_txt_manual.load(":img/manualclean.png");
  ui->txt_mapping_3->setPixmap(icon_txt_manual);
  icon_txt_setting.load(":img/setting.png");
  ui->txt_mapping_4->setPixmap(icon_txt_setting);


  icon_sucktion.load(":img/sucktion1_1-removebg-preview.png");
  ui->btn_sucktion->setIcon(icon_sucktion);
  icon_brush.load(":img/brush1_2-removebg-preview.png");
  ui->btn_brush->setIcon(icon_brush);

  icon_bottle.load(":img/water-removebg-preview (1).png");
  ui->bottle_clean->setPixmap(icon_bottle);
  ui->bottle_dirt->setPixmap(icon_bottle);

  icon_logo.load(":img/pic_logo_1.jpg");
  ui->lable_logo->setPixmap(icon_logo);

  icon_cleaning.load(":img/btn_clean2.PNG");

  ui->lbl_status_cleaning->setPixmap(icon_cleaning);
  icon_charge.load(":img/btn_charging2.PNG");
  ui->lbl_status_charging->setPixmap(icon_charge);
  icon_comeback.load(":img/btn_homing2.PNG");
  ui->lbl_status_comeback->setPixmap(icon_comeback);
  icon_water_in.load(":img/btn_waterin2.PNG");
  ui->lbl_status_water_in->setPixmap(icon_water_in);
  icon_water_out.load(":img/btn_waterout2.PNG");
  ui->lbl_status_water_out->setPixmap(icon_water_out);


  ui->group_manual->setVisible(false);
  ui->Zeus_Widget->setCurrentIndex(int(0));
  ui->btn_change_brush->setStyleSheet("border-image: url(:img/brush_change1.png);");
  //ui->btn_mapping->setStyleSheet("")

}

Widget::Widget(QWidget *parent)
  : QWidget(parent)
  , ui(new Ui::Widget)
{
  ui->setupUi(this);
  init_setting(ui);
  ip_connect_timer = new QTimer();
  connect(ip_connect_timer,SIGNAL(timeout()), this, SLOT(IP_Connect_Timer_Callback()));
  ip_connect_timer->start(3000);
}

Widget::~Widget()
{
  thread->stop_node();
  delete ui;
}

void Widget::IP_Connect_Timer_Callback(){
  QPixmap wifi_con, wifi_discon;
  //("border-image: url(:img/Wifi-2-icon);");
  wifi_discon.load(":img/free-icon-no-wifi-1686538");
  wifi_con.load(":img/Wifi-2-icon");
  QNetworkInterface interface;
  QString macAddress;
  QString ipAddress;
  QString Adddress;
  QList<QNetworkInterface> macList = interface.allInterfaces();
  QList<QHostAddress> ipList=interface.allAddresses();
  for (int i = 0; i < macList.size(); i++)
  {
    QString str = macList.at(i).hardwareAddress();       // MAC
    if(str != "00:00:00:00:00:00")
    {
      macAddress = str;
    }
  }
  for (int i = 0; i < ipList.size(); i++)
  {
    if (ipList.at(i) != QHostAddress::LocalHost && ipList.at(i).toIPv4Address())
    {

      ipAddress = ipList.at(i).toString();
      break;
    }
  }

  if(ipAddress!=nullptr)
  {
    Adddress = "IP : " + ipAddress;
    ui->label_2->setText(Adddress);
    ui->label_3->setPixmap(wifi_con);

  }
  else
  {
    ui->label_2->setText("No Connection");
    ui->label_3->setPixmap(wifi_discon);

  }

  ipAddress = "";


}

void Widget::pic_auto_clicked()
{
  ui->Zeus_Widget->setCurrentIndex(3);
}

void Widget::init_tab_mapping(){
  /*
    ui->lbl_mapping_init->setVisible(false);
    ui->lbl_mapping_start->setVisible(false);
    ui->lbl_mapping_complete->setVisible(false);
    ui->btn_auto_clean->setVisible(false);
    ui->btn_mapping_start->setVisible(true);*/
}

void Widget::on_btn_mapping_clicked()
{
  //ui->Zeus_Widget->setCurrentIndex(1);


}

void Widget::on_btn_auto_clicked()
{
  //ui->Zeus_Widget->setCurrentIndex(2);
  //    imageLabel.setPixmap(QPixmap(":/image/test1.png")
  ui->lbl_status_cleaning->setPixmap(QPixmap(":img/btn_clean.PNG"));
  //ui->lbl_status_cleaning->setStyleSheet("border-image: url(:img/pic_mapping.jpg);");
}

void Widget::on_btn_manual_clicked()
{
  //ui->Zeus_Widget->setCurrentIndex(3);
  ui->group_manual->setVisible(true);

}

void Widget::on_btn_setting_clicked()
{
  //ui->Zeus_Widget->setCurrentIndex(4);

}

void Widget::on_Zeus_Widget_currentChanged(int index)
{
  //init_tab_mapping();
}

void Widget::on_btn_change_brush_clicked()
{
  if(!brush_change_flag)
  {
    ui->btn_change_brush->setStyleSheet("border-image: url(:img/brush_change2.png);");
    brush_change_flag = true;
  }
  else if(brush_change_flag)
  {
    ui->btn_change_brush->setStyleSheet("border-image: url(:img/brush_change1.png);");
    brush_change_flag = false;
  }
}

void Widget::on_btn_manual_setting_clicked()
{
  ui->group_manual->setVisible(false);
}

void Widget::on_btn_mapping_pressed()
{
  ui->btn_mapping->setStyleSheet("background-color: rgb(211, 215, 207);");

}

void Widget::on_btn_mapping_released()
{
  ui->btn_mapping->setStyleSheet("background-color: rgb(255, 255, 255);");
  ui->btn_mapping->setStyleSheet("border: 1px solid white;");
}

void Widget::on_btn_auto_pressed()
{
  ui->btn_auto->setStyleSheet("background-color: rgb(211, 215, 207);");
}

void Widget::on_btn_auto_released()
{
  ui->btn_auto->setStyleSheet("background-color: rgb(255, 255, 255);");
  ui->btn_auto->setStyleSheet("border: 1px solid white;");
}

void Widget::on_btn_manual_pressed()
{
  ui->btn_manual->setStyleSheet("background-color: rgb(211, 215, 207);");
}

void Widget::on_btn_manual_released()
{
  ui->btn_manual->setStyleSheet("background-color: rgb(255, 255, 255);");
  ui->btn_manual->setStyleSheet("border: 1px solid white;");
}

void Widget::on_btn_setting_pressed()
{
  ui->btn_setting->setStyleSheet("background-color: rgb(211, 215, 207);");
}

void Widget::on_btn_setting_released()
{
  ui->btn_setting->setStyleSheet("background-color: rgb(255, 255, 255);");
  ui->btn_setting->setStyleSheet("border: 1px solid white;");
}

void Widget::on_btn_reset_clicked()
{
  ui->lbl_status_cleaning->setPixmap(QPixmap(":img/btn_clean2.PNG"));
}

void Widget::on_btn_sucktion_clicked()
{


}

void Widget::on_btn_sucktion_pressed()
{
  ui->btn_sucktion->setStyleSheet("background-color: rgb(211, 215, 207);");
}

void Widget::on_btn_brush_clicked()
{

}

void Widget::on_btn_brush_pressed()
{
  ui->btn_brush->setStyleSheet("background-color: rgb(211, 215, 207);");
}

void Widget::on_btn_brush_released()
{
  ui->btn_brush->setStyleSheet("background-color: rgb(255, 255, 255);");
  ui->btn_brush->setStyleSheet("border: 1px solid white;");

  brush_power_level++;
  if(brush_power_level == 4)
    brush_power_level = 1;
  if(brush_power_level == 1)
    ui->btn_brush->setIcon(QIcon(":img/brush1_2-removebg-preview.png"));
  else if(brush_power_level == 2)
    ui->btn_brush->setIcon(QIcon(":img/brush2_2-removebg-preview.png"));
  else if(brush_power_level == 3)
    ui->btn_brush->setIcon(QIcon(":img/brush3_2-removebg-preview.png"));
}

void Widget::on_btn_sucktion_released()
{
  ui->btn_sucktion->setStyleSheet("background-color: rgb(255, 255, 255);");
  ui->btn_sucktion->setStyleSheet("border: 1px solid white;");

  sucktion_power_lever++;
  if(sucktion_power_lever == 4)
    sucktion_power_lever = 1;
  if(sucktion_power_lever == 1)
    ui->btn_sucktion->setIcon(QIcon(":img/sucktion1_1-removebg-preview.png"));
  else if(sucktion_power_lever == 2)
    ui->btn_sucktion->setIcon(QIcon(":img/sucktion2_1-removebg-preview.png"));
  else if(sucktion_power_lever == 3)
    ui->btn_sucktion->setIcon(QIcon(":img/sucktion3_1-removebg-preview.png"));

}

// -------------------- ROS2 ------------------------- //

double ratio_map_to_pixmap = 1.0;

QRect get_image_size(int cur_w, int cur_h, int target_w, int target_h, double* ratio_transform=nullptr)
{
  QRect rect;
  double ratio_w = double(target_w) / cur_w;
  double ratio_h = double(target_h) / cur_h;  

  //target = cur * ratio

  if(ratio_w > ratio_h){
    rect.setWidth(int(cur_w*ratio_h));
    rect.setHeight(target_h);

    if(ratio_transform != nullptr)
    {
      *ratio_transform = ratio_h;
    }
  }
  else {
    rect.setWidth(target_w);
    rect.setHeight(int(cur_h*ratio_w));

    if(ratio_transform != nullptr)
    {
      *ratio_transform = ratio_w;
    }
  }

  return rect;
}

void limit_point(QPoint* p, QRect rect_size, int offset=5)
{
  int w_limit = rect_size.width();
  int h_limit = rect_size.height();

  if(p->x() > w_limit-offset)
  {
    p->setX(w_limit-offset);
  }

  if(p->y() > h_limit-offset)
  {
    p->setY(h_limit-offset);
  }

  if(p->x() < 0)
  {
    p->setX(offset);
  }

  if(p->y() < 0)
  {
    p->setY(offset);
  }
}

void Widget::init_ros2(int argc, char *argv[])
{
  rclcpp::init(argc,argv);
  node_ptr = std::make_shared<platform_node>("/processed_map");
  node_ptr->set_callback(std::bind(&Widget::callback,this,std::placeholders::_1));
  thread = std::make_unique<spin_thread>(this);
  thread->spin_node(node_ptr);  

  //ui->Zeus_Widget->setCurrentIndex(1);
}

QPoint get_point_pixmap_to_map(QPoint p, double ratio_p_to_m)
{
  return QPoint(int(p.x()/ratio_p_to_m), int(p.y()/ratio_p_to_m));
}

#pragma region Map {

void Widget::callback(cv::Mat mat)
{
  cv::cvtColor(mat, mat, cv::COLOR_RGB2BGRA); //COLOR_BGR2RGB doesn't behave so use RGBA
  QImage image_result;
  mat_to_qimage(mat, image_result);

  auto map_size = image_result.rect();
  auto map_w = map_size.width();
  auto map_h = map_size.height();

  auto target_w = ui->lbl_map_img->geometry().width();
  auto target_h = ui->lbl_map_img->geometry().height();

  auto rect_size = get_image_size(map_w, map_h, target_w, target_h, &ratio_map_to_pixmap);
  image_result = image_result.scaled(rect_size.width(),rect_size.height());

  if((!m_selectedArea_LU.isNull()) && (!m_selectedArea_RD.isNull()))
  {
    QPainter qPainter(&image_result);
    QPen pen(Qt::red);
    qPainter.setPen(pen);

    limit_point(&m_selectedArea_RD,rect_size);

    int w = m_selectedArea_RD.x() - m_selectedArea_LU.x();
    int h = m_selectedArea_RD.y() - m_selectedArea_LU.y();
    qPainter.drawRect(m_selectedArea_LU.x(),m_selectedArea_LU.y(),w,h);

    qPainter.end();
  }

  ui->lbl_map_img->setPixmap(QPixmap::fromImage(image_result));
//  ui->lbl_map_img->update();

  auto time_now = node_ptr->get_clock()->now().seconds();
  char buffer[200] = "";

//  sprintf(buffer,"time: %.2f",time_now);
//  ui->lbl_time->setText(QString(buffer));
//  ui->lbl_time->update();
}

void Widget::on_lbl_map_img_clicked(QMouseEvent *event)
{
  auto pixmap = ui->lbl_map_img->pixmap();

  if(pixmap == nullptr)
  {
    RCLCPP_INFO(node_ptr->get_logger(),"pixmap is Null.");
    return;
  }

  auto rect_size = pixmap->rect();
  bool isInner = (rect_size.width() > event->pos().x()) && (rect_size.height() > event->pos().y());

  if ( event->button() == Qt::LeftButton && isInner)
  {
    m_selectedArea_LU = event->pos();

    // auto p = get_point_pixmap_to_map(m_selectedArea_LU,ratio_map_to_pixmap);
    // RCLCPP_INFO(node_ptr->get_logger(),"x: %d, y: %d", p.x(), p.y());
  }
  else
  {
    m_selectedArea_LU = QPoint();
  }
}

void Widget::on_lbl_map_img_moved(QMouseEvent *event)
{
  if ( event->buttons() & Qt::LeftButton )
  {
    m_selectedArea_RD = event->pos();
    // RCLCPP_INFO(node_ptr->get_logger(),"mouse dragged!!");
  }
}

void Widget::on_lbl_map_img_release(QMouseEvent *event)
{
  if ( event->button() == Qt::LeftButton )
  {
    m_selectedArea_RD = event->pos();
  }

  // RCLCPP_INFO(node_ptr->get_logger(),"mouse release!!");
}

void Widget::on_lbl_reset_area_clicked()
{
  m_selectedArea_LU = QPoint();
  m_selectedArea_RD = QPoint();

  node_ptr->pub_bbox->publish(geometry_msgs::msg::Polygon());
}

void Widget::on_lbl_start_clean_clicked()
{
  std_msgs::msg::String msg;
  msg.data = "Cleaning:Start";
  this->node_ptr->pub_cmd_navigator->publish(msg);
}


void Widget::on_lbl_set_bbox_clicked()
{
  auto points = geometry_msgs::msg::Polygon();

  if(m_selectedArea_LU.isNull() || m_selectedArea_RD.isNull())
  {
    RCLCPP_INFO(node_ptr->get_logger(),"bbox is not selected!");
    return;
  }

  auto p1 = get_point_pixmap_to_map(m_selectedArea_LU,ratio_map_to_pixmap);
  auto p2 = get_point_pixmap_to_map(m_selectedArea_RD,ratio_map_to_pixmap);

  auto x1 = p1.x() < p2.x()? p1.x() : p2.x();
  auto x2 = p1.x() < p2.x()? p2.x() : p1.x();

  auto y1 = p1.y() < p2.y()? p1.y() : p2.y();
  auto y2 = p1.y() < p2.y()? p2.y() : p1.y();

  auto p1_msg = geometry_msgs::msg::Point32();
  p1_msg.x = x1;
  p1_msg.y = y1;

  auto p2_msg = geometry_msgs::msg::Point32();
  p2_msg.x = x2;
  p2_msg.y = y2;

  points.points.insert(points.points.end(),p1_msg);
  points.points.insert(points.points.end(),p2_msg);

  node_ptr->pub_bbox->publish(points);

  m_selectedArea_LU = QPoint();
  m_selectedArea_RD = QPoint();
}


#pragma endregion}

#pragma region Waypoints {

void Widget::on_waypoint_1_clicked()
{
  geometry_msgs::msg::Point32 msg;
  msg.x = ui->le_loaction_x_1->text().toFloat();
  msg.y = ui->le_loaction_y_1->text().toFloat();
  this->node_ptr->pub_waypoint->publish(msg);
}

void Widget::on_waypoint_2_clicked()
{
  geometry_msgs::msg::Point32 msg;
  msg.x = ui->le_loaction_x_2->text().toFloat();
  msg.y = ui->le_loaction_y_2->text().toFloat();
  this->node_ptr->pub_waypoint->publish(msg);
}

void Widget::on_waypoint_3_clicked()
{
  geometry_msgs::msg::Point32 msg;
  msg.x = ui->le_loaction_x_3->text().toFloat();
  msg.y = ui->le_loaction_y_3->text().toFloat();
  this->node_ptr->pub_waypoint->publish(msg);
}

void Widget::on_waypoint_4_clicked()
{
  geometry_msgs::msg::Point32 msg;
  msg.x = ui->le_loaction_x_4->text().toFloat();
  msg.y = ui->le_loaction_y_4->text().toFloat();
  this->node_ptr->pub_waypoint->publish(msg);
}

void Widget::on_waypoint_5_clicked()
{
  geometry_msgs::msg::Point32 msg;
  msg.x = ui->le_loaction_x_5->text().toFloat();
  msg.y = ui->le_loaction_y_5->text().toFloat();
  this->node_ptr->pub_waypoint->publish(msg);
}

#pragma endregion}

#pragma region Command_to_Platform {

void Widget::on_brush_on_clicked()
{
  std_msgs::msg::String msg;
  msg.data = "<Br:1;>";
  this->node_ptr->pub_cmd_controller->publish(msg);
}

void Widget::on_Brush_OFF_clicked()
{
  std_msgs::msg::String msg;
  msg.data = "<Br:0;>";
  this->node_ptr->pub_cmd_controller->publish(msg);
}

void Widget::on_pump_on_clicked()
{
  std_msgs::msg::String msg;
  msg.data = "<W:1;>";
  this->node_ptr->pub_cmd_controller->publish(msg);
}

void Widget::on_Pump_OFF_clicked()
{
  std_msgs::msg::String msg;
  msg.data = "<W:0;>";
  this->node_ptr->pub_cmd_controller->publish(msg);
}


void Widget::on_vacumm_on_clicked()
{
  std_msgs::msg::String msg;
  msg.data = "<V:1;>";
  this->node_ptr->pub_cmd_controller->publish(msg);
}

void Widget::on_vacumm_OFF_clicked()
{
  std_msgs::msg::String msg;
  msg.data = "<V:0;>";
  this->node_ptr->pub_cmd_controller->publish(msg);
}

void Widget::on_btn_send_button_cmd_clicked()
{
    std_msgs::msg::String msg;
    msg.data = ui->lbl_selected_button->text().toStdString().c_str();
    this->node_ptr->pub_selected_btn->publish(msg);
}

void Widget::on_Front_clicked()
{

}

void Widget::on_Left_clicked()
{

}

void Widget::on_Right_clicked()
{

}

void Widget::on_Back_clicked()
{

}

void Widget::on_Left_Front_clicked()
{

}

void Widget::on_Right_Front_clicked()
{

}

void Widget::on_Left_Back_clicked()
{

}

void Widget::on_Right_Back_clicked()
{

}

void Widget::on_Stop_clicked()
{

}

#pragma endregion}

// -------------------- ROS2 ------------------------- //

