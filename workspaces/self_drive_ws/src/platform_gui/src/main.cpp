#include "widget.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Widget w;
    w.init_ros2(argc, argv);
    w.show();
    return a.exec();
}
