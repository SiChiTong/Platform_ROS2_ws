#include "platform_gui.h"

#include <QApplication>

int main(int argc, char *argv[])
{
  rclcpp::init(argc,argv);

  QApplication a(argc, argv);
  platform_gui w;
  w.show();
  return a.exec();
}
