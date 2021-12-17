QT       += core gui
QT       += serialport #Test Serial
QT       += network
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    main.cpp \
    widget.cpp

HEADERS += \
    clickablelabel.h \
    widget.h

FORMS += \
    widget.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

DISTFILES += \
    img/pic_auto.jpg \
    img/pic_logo_1.jpg \
    img/pic_logo_2.jpg \
    img/pic_manual.jpg \
    img/pic_mapping.jpg \
    img/pic_setting.jpg

RESOURCES += \
    zeus_resource.qrc
