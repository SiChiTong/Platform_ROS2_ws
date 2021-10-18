#include "platform_gui.h"
#include "ui_platform_gui.h"
#include "platform_node.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <QImage>

#include "image_converter.hpp"
#include <QGraphicsPixmapItem>

void platform_gui::init_ros2()
{
  node_ptr = std::make_shared<platform_node>("/camera/color/image_raw");  
  node_ptr->set_callback(std::bind(&platform_gui::callback,this,std::placeholders::_1));
  node_ptr->get_clock();
  thread = std::make_unique<spin_thread>(this);
  thread->spin_node(node_ptr);
}

void platform_gui::callback(cv::Mat mat)
{
  cv::cvtColor(mat, mat, cv::COLOR_RGB2BGRA); //COLOR_BGR2RGB doesn't behave so use RGBA

  QImage image_result;
  mat_to_qimage(mat, image_result);
  QGraphicsScene* scene = new QGraphicsScene();
  QGraphicsPixmapItem* item = new QGraphicsPixmapItem(QPixmap::fromImage(image_result));
  scene->addItem(item);
  ui->graphicsView->setScene(scene);
}

platform_gui::platform_gui(QWidget *parent)
  : QMainWindow(parent),    
    ui(new Ui::platform_gui)
{
  ui->setupUi(this);
  init_ros2();
}

platform_gui::~platform_gui()
{
  thread->stop_node();
  delete ui;
}

