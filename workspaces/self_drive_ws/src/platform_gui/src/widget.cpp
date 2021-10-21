#include "widget.h"
#include "ui_widget.h"

#include <QSerialPort>
#include <QSerialPortInfo>
#include <QDebug>
#include <QMessageBox>
#include <QImage>
#include <QGraphicsPixmapItem>
#include <QtNetwork>
#include <QProcess>
#include <QTcpSocket>
#include <QNetworkInterface>
#include <QTimer>
#include "clickablelabel.h"

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


    emit clicked();
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
    ui->Zeus_Widget->setCurrentIndex((int)0);
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

    target_available = false;
    target_board_name = "";
    target_board = new QSerialPort;
    target_board->clear();
    qDebug() << "Number of available ports: " << QSerialPortInfo::availablePorts().length();
        foreach(const QSerialPortInfo &serialPortInfo, QSerialPortInfo::availablePorts()){
            qDebug() << "Has vendor ID: " << serialPortInfo.hasVendorIdentifier();
            if(serialPortInfo.hasVendorIdentifier()){
                qDebug() << "Vendor ID: " << serialPortInfo.vendorIdentifier();
            }
            qDebug() << "Has Product ID: " << serialPortInfo.hasProductIdentifier();
            if(serialPortInfo.hasProductIdentifier()){
                qDebug() << "Product ID: " << serialPortInfo.productIdentifier();
            }
        }


        foreach(const QSerialPortInfo &serialPortInfo, QSerialPortInfo::availablePorts()){
            if(serialPortInfo.hasVendorIdentifier() && serialPortInfo.hasProductIdentifier()){
                if(serialPortInfo.vendorIdentifier() == target_board_vender_id){
                    if(serialPortInfo.productIdentifier() == target_board_product_id){
                        target_board_name = serialPortInfo.portName();
                        target_available = true;

                    }
                }
            }
        }

        if(target_available){
            // open and configure the serialport
            target_board->setPortName(target_board_name);

            //target_board->open(QSerialPort::ReadWrite);
            target_board->setBaudRate(QSerialPort::Baud9600);
            target_board->setDataBits(QSerialPort::Data8);
            target_board->setParity(QSerialPort::NoParity);
            target_board->setStopBits(QSerialPort::OneStop);
            target_board->setFlowControl(QSerialPort::NoFlowControl);
            target_board->open(QSerialPort::WriteOnly);
        }else{
            // give error message if not available
            //QMessageBox::warning(this, "Port error", "Couldn't find the target_board!");
        }




}

Widget::~Widget()
{
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

void Widget::on_Front_clicked()
{
    const char test_data[4] = {'F',':',50,','};
    if(target_board->isWritable()){
            target_board->write(test_data,4);
        }else{
            qDebug() << "Couldn't write to serial!";
        }
}

void Widget::pic_auto_clicked()
{
    ui->Zeus_Widget->setCurrentIndex(3);
}

void Widget::on_Left_Front_clicked()
{
    const char test_data[5] = {'L','F',':',50,','};
    if(target_board->isWritable()){
            target_board->write(test_data,5);
        }else{
            qDebug() << "Couldn't write to serial!";
        }
}

void Widget::on_Right_Front_clicked()
{
    const char test_data[5] = {'R','F',':',50,','};
    if(target_board->isWritable()){
            target_board->write(test_data,5);
        }else{
            qDebug() << "Couldn't write to serial!";
        }
}

void Widget::on_Left_clicked()
{
    const char test_data[4] = {'L',':',50,','};
    if(target_board->isWritable()){
            target_board->write(test_data,4);
        }else{
            qDebug() << "Couldn't write to serial!";
        }
}

void Widget::on_Right_clicked()
{
    const char test_data[4] = {'R',':',50,','};
    if(target_board->isWritable()){
            target_board->write(test_data,4);
        }else{
            qDebug() << "Couldn't write to serial!";
        }
}

void Widget::on_Left_Back_clicked()
{
    const char test_data[5] = {'L','B',':',50,','};
    if(target_board->isWritable()){
            target_board->write(test_data,5);
        }else{
            qDebug() << "Couldn't write to serial!";
        }
}


void Widget::on_Back_clicked()
{
    const char test_data[4] = {'B',':',50,','};
    if(target_board->isWritable()){
            target_board->write(test_data,4);
        }else{
            qDebug() << "Couldn't write to serial!";
        }
}


void Widget::on_Right_Back_clicked()
{
    const char test_data[5] = {'R','B',':',50,','};
    if(target_board->isWritable()){
            target_board->write(test_data,5);
        }else{
            qDebug() << "Couldn't write to serial!";
        }
}


void Widget::on_Stop_clicked()
{
    const char test_data[4] = {'S',':',0,','};
    if(target_board->isWritable()){
            target_board->write(test_data,4);
        }else{
            qDebug() << "Couldn't write to serial!";
        }
}


void Widget::on_pushButton_clicked()
{
target_board->clear();
    target_board->close();
    target_board->clear();
    if(target_board->isOpen())
    {
        qDebug() << "not closed!";
    }
    else if(!target_board->isOpen())
    {
        qDebug() << "closed!";
    }
    close();
}


void Widget::on_pushButton_2_clicked()
{

        foreach(const QSerialPortInfo &serialPortInfo, QSerialPortInfo::availablePorts()){
            if(serialPortInfo.hasVendorIdentifier() && serialPortInfo.hasProductIdentifier()){
                if(serialPortInfo.vendorIdentifier() == target_board_vender_id){
                    if(serialPortInfo.productIdentifier() == target_board_product_id){
                        target_board_name = serialPortInfo.portName();
                        target_available = true;

                    }
                }
            }
        }

        if(target_available){
            // open and configure the serialport
            target_board->setPortName(target_board_name);

            //target_board->open(QSerialPort::ReadWrite);
            target_board->setBaudRate(QSerialPort::Baud9600);
            target_board->setDataBits(QSerialPort::Data8);
            target_board->setParity(QSerialPort::NoParity);
            target_board->setStopBits(QSerialPort::OneStop);
            target_board->setFlowControl(QSerialPort::NoFlowControl);
            target_board->open(QSerialPort::WriteOnly);
            if(target_board->isOpen())
            {
                qDebug() << "open!";
            }
            else
            {
                qDebug() << "not opend!";
            }
        }else{
            // give error message if not available
            QMessageBox::warning(this, "Port error", "Couldn't find the target_board!");
        }

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

void Widget::init_ros2(int argc, char *argv[])
{
  rclcpp::init(argc,argv);
  node_ptr = std::make_shared<platform_node>("/processed_map");
  node_ptr->set_callback(std::bind(&Widget::callback,this,std::placeholders::_1));
  node_ptr->get_clock();
  thread = std::make_unique<spin_thread>(this);
  thread->spin_node(node_ptr);
}

void Widget::callback(cv::Mat mat)
{
  cv::cvtColor(mat, mat, cv::COLOR_RGB2BGRA); //COLOR_BGR2RGB doesn't behave so use RGBA
  // RCLCPP_INFO(node_ptr->get_logger(),"get image!");
  QImage image_result;
  mat_to_qimage(mat, image_result);
  QGraphicsScene* scene = new QGraphicsScene();
  QGraphicsPixmapItem* item = new QGraphicsPixmapItem(QPixmap::fromImage(image_result));
  scene->addItem(item);
  ui->graphicsView->setScene(scene);
}

void Widget::on_waypoint_1_clicked()
{
  std_msgs::msg::Int8 msg;
  msg.data = 1;
  this->node_ptr->pub_waypoint->publish(msg);
}

void Widget::on_waypoint_2_clicked()
{
  std_msgs::msg::Int8 msg;
  msg.data = 2;
  this->node_ptr->pub_waypoint->publish(msg);
}

void Widget::on_waypoint_3_clicked()
{
  std_msgs::msg::Int8 msg;
  msg.data = 3;
  this->node_ptr->pub_waypoint->publish(msg);
}

void Widget::on_waypoint_4_clicked()
{
  std_msgs::msg::Int8 msg;
  msg.data = 4;
  this->node_ptr->pub_waypoint->publish(msg);
}

void Widget::on_waypoint_5_clicked()
{
  std_msgs::msg::Int8 msg;
  msg.data = 5;
  this->node_ptr->pub_waypoint->publish(msg);
}

void Widget::on_brush_on_clicked()
{
  std_msgs::msg::String msg;
  msg.data = "<Br:1;>";
  this->node_ptr->pub_cmd_W_Br->publish(msg);
}

void Widget::on_Brush_OFF_clicked()
{
  std_msgs::msg::String msg;
  msg.data = "<Br:0;>";
  this->node_ptr->pub_cmd_W_Br->publish(msg);
}

void Widget::on_pump_on_clicked()
{
  std_msgs::msg::String msg;
  msg.data = "<W:1;>";
  this->node_ptr->pub_cmd_W_Br->publish(msg);
}

void Widget::on_Pump_OFF_clicked()
{
  std_msgs::msg::String msg;
  msg.data = "<W:0;>";
  this->node_ptr->pub_cmd_W_Br->publish(msg);
}

// -------------------- ROS2 ------------------------- //
