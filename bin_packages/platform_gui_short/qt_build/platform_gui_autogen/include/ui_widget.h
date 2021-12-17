/********************************************************************************
** Form generated from reading UI file 'widget.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_WIDGET_H
#define UI_WIDGET_H

#include <QtCore/QVariant>
#include <QtGui/QIcon>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "src/clickablelabel.hpp"

QT_BEGIN_NAMESPACE

class Ui_Widget
{
public:
    QTabWidget *Zeus_Widget;
    QWidget *tab_setting;
    QLabel *label_10;
    QGroupBox *control_group;
    QWidget *layoutWidget;
    QGridLayout *gridLayout;
    QPushButton *Front;
    QPushButton *Right_Front;
    QPushButton *Back;
    QPushButton *Left;
    QPushButton *Stop;
    QPushButton *Left_Back;
    QPushButton *Right;
    QPushButton *Right_Back;
    QPushButton *Left_Front;
    QGroupBox *control_group_waypoint;
    QPushButton *waypoint_1;
    QPushButton *waypoint_3;
    QPushButton *waypoint_4;
    QPushButton *waypoint_2;
    QPushButton *waypoint_5;
    QLabel *lbl_map_R;
    QLabel *lbl_map_G;
    QLabel *lbl_map_B;
    ClickableLabel *lbl_map_img;
    QWidget *layoutWidget1;
    QGridLayout *gridLayout_2;
    QPushButton *pump_on;
    QPushButton *Brush_OFF;
    QPushButton *Pump_OFF;
    QPushButton *brush_on;
    QPushButton *vacumm_on;
    QPushButton *vacumm_OFF;
    QGroupBox *control_group_waypoint_location;
    QGroupBox *gb_location_1;
    QLabel *label_4;
    QLabel *label_5;
    QLineEdit *le_loaction_x_1;
    QLineEdit *le_loaction_y_1;
    QGroupBox *gb_location_2;
    QLabel *label_6;
    QLabel *label_7;
    QLineEdit *le_loaction_x_2;
    QLineEdit *le_loaction_y_2;
    QGroupBox *gb_location_3;
    QLabel *label_8;
    QLabel *label_9;
    QLineEdit *le_loaction_x_3;
    QLineEdit *le_loaction_y_3;
    QGroupBox *gb_location_4;
    QLabel *label_12;
    QLabel *label_13;
    QLineEdit *le_loaction_x_4;
    QLineEdit *le_loaction_y_4;
    QGroupBox *gb_location_5;
    QLabel *label_14;
    QLabel *label_15;
    QLineEdit *le_loaction_x_5;
    QLineEdit *le_loaction_y_5;
    QGroupBox *gb_speed;
    QLabel *label_16;
    QPushButton *btn_refresh_speed;
    QPushButton *btn_apply_speed;
    QLineEdit *le_speed;
    QWidget *layoutWidget2;
    QVBoxLayout *verticalLayout;
    QPushButton *lbl_set_bbox;
    QPushButton *lbl_start_clean;
    QPushButton *lbl_reset_area;
    QGroupBox *control_group_elevator_button;
    QLineEdit *lbl_selected_button;
    QPushButton *btn_send_button_cmd;
    QWidget *tab_main;
    QPushButton *btn_mapping;
    QPushButton *btn_auto;
    QPushButton *btn_manual;
    QPushButton *btn_setting;
    QLabel *lable_logo;
    QLabel *water_clean;
    QLabel *water_dirt;
    QLabel *lbl_robot_speed;
    QLabel *lbl_status_cleaning;
    QLabel *lbl_status_comeback;
    QLabel *lbl_status_charging;
    QLabel *lbl_status_water_out;
    QLabel *lbl_status_water_in;
    QPushButton *btn_change_brush;
    QLabel *bottle_clean;
    QLabel *bottle_dirt;
    QGroupBox *group_manual;
    QGroupBox *group_clean;
    QPushButton *btn_Dry;
    QPushButton *btn_wet;
    QPushButton *btn_both_dry_wet;
    QPushButton *btn_sterilization;
    QGroupBox *setup_group;
    QPushButton *btn_brush_up;
    QPushButton *btn_brush_down;
    QPushButton *btn_manual_setting;
    QLabel *txt_mapping;
    QLabel *txt_mapping_2;
    QLabel *txt_mapping_3;
    QLabel *txt_mapping_4;
    QPushButton *btn_sucktion;
    QPushButton *btn_brush;
    QLabel *label_11;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *lbl_charge_gage;
    QLabel *lbl_charge1;
    QLabel *lbl_charge2;
    QLabel *lbl_charge3;
    QPushButton *btn_reset;

    void setupUi(QWidget *Widget)
    {
        if (Widget->objectName().isEmpty())
            Widget->setObjectName(QString::fromUtf8("Widget"));
        Widget->resize(1399, 699);
        Zeus_Widget = new QTabWidget(Widget);
        Zeus_Widget->setObjectName(QString::fromUtf8("Zeus_Widget"));
        Zeus_Widget->setGeometry(QRect(10, 30, 1381, 661));
        Zeus_Widget->setAutoFillBackground(false);
        Zeus_Widget->setStyleSheet(QString::fromUtf8(""));
        tab_setting = new QWidget();
        tab_setting->setObjectName(QString::fromUtf8("tab_setting"));
        label_10 = new QLabel(tab_setting);
        label_10->setObjectName(QString::fromUtf8("label_10"));
        label_10->setGeometry(QRect(25, 10, 101, 31));
        QFont font;
        font.setPointSize(18);
        font.setBold(true);
        font.setWeight(75);
        label_10->setFont(font);
        control_group = new QGroupBox(tab_setting);
        control_group->setObjectName(QString::fromUtf8("control_group"));
        control_group->setGeometry(QRect(924, 460, 281, 151));
        QFont font1;
        font1.setPointSize(13);
        control_group->setFont(font1);
        control_group->setAlignment(Qt::AlignCenter);
        layoutWidget = new QWidget(control_group);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(8, 29, 268, 111));
        gridLayout = new QGridLayout(layoutWidget);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        Front = new QPushButton(layoutWidget);
        Front->setObjectName(QString::fromUtf8("Front"));
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(Front->sizePolicy().hasHeightForWidth());
        Front->setSizePolicy(sizePolicy);
        Front->setFont(font1);

        gridLayout->addWidget(Front, 0, 1, 1, 1);

        Right_Front = new QPushButton(layoutWidget);
        Right_Front->setObjectName(QString::fromUtf8("Right_Front"));
        sizePolicy.setHeightForWidth(Right_Front->sizePolicy().hasHeightForWidth());
        Right_Front->setSizePolicy(sizePolicy);
        Right_Front->setFont(font1);

        gridLayout->addWidget(Right_Front, 0, 2, 1, 1);

        Back = new QPushButton(layoutWidget);
        Back->setObjectName(QString::fromUtf8("Back"));
        sizePolicy.setHeightForWidth(Back->sizePolicy().hasHeightForWidth());
        Back->setSizePolicy(sizePolicy);
        Back->setFont(font1);

        gridLayout->addWidget(Back, 2, 1, 1, 1);

        Left = new QPushButton(layoutWidget);
        Left->setObjectName(QString::fromUtf8("Left"));
        sizePolicy.setHeightForWidth(Left->sizePolicy().hasHeightForWidth());
        Left->setSizePolicy(sizePolicy);
        Left->setFont(font1);

        gridLayout->addWidget(Left, 1, 0, 1, 1);

        Stop = new QPushButton(layoutWidget);
        Stop->setObjectName(QString::fromUtf8("Stop"));
        sizePolicy.setHeightForWidth(Stop->sizePolicy().hasHeightForWidth());
        Stop->setSizePolicy(sizePolicy);
        Stop->setFont(font1);

        gridLayout->addWidget(Stop, 1, 1, 1, 1);

        Left_Back = new QPushButton(layoutWidget);
        Left_Back->setObjectName(QString::fromUtf8("Left_Back"));
        sizePolicy.setHeightForWidth(Left_Back->sizePolicy().hasHeightForWidth());
        Left_Back->setSizePolicy(sizePolicy);
        Left_Back->setFont(font1);

        gridLayout->addWidget(Left_Back, 2, 0, 1, 1);

        Right = new QPushButton(layoutWidget);
        Right->setObjectName(QString::fromUtf8("Right"));
        sizePolicy.setHeightForWidth(Right->sizePolicy().hasHeightForWidth());
        Right->setSizePolicy(sizePolicy);
        Right->setFont(font1);

        gridLayout->addWidget(Right, 1, 2, 1, 1);

        Right_Back = new QPushButton(layoutWidget);
        Right_Back->setObjectName(QString::fromUtf8("Right_Back"));
        sizePolicy.setHeightForWidth(Right_Back->sizePolicy().hasHeightForWidth());
        Right_Back->setSizePolicy(sizePolicy);
        Right_Back->setFont(font1);

        gridLayout->addWidget(Right_Back, 2, 2, 1, 1);

        Left_Front = new QPushButton(layoutWidget);
        Left_Front->setObjectName(QString::fromUtf8("Left_Front"));
        sizePolicy.setHeightForWidth(Left_Front->sizePolicy().hasHeightForWidth());
        Left_Front->setSizePolicy(sizePolicy);
        Left_Front->setFont(font1);

        gridLayout->addWidget(Left_Front, 0, 0, 1, 1);

        control_group_waypoint = new QGroupBox(tab_setting);
        control_group_waypoint->setObjectName(QString::fromUtf8("control_group_waypoint"));
        control_group_waypoint->setGeometry(QRect(924, 20, 281, 161));
        control_group_waypoint->setFont(font1);
        control_group_waypoint->setAlignment(Qt::AlignCenter);
        waypoint_1 = new QPushButton(control_group_waypoint);
        waypoint_1->setObjectName(QString::fromUtf8("waypoint_1"));
        waypoint_1->setGeometry(QRect(10, 30, 89, 61));
        waypoint_1->setFont(font1);
        waypoint_3 = new QPushButton(control_group_waypoint);
        waypoint_3->setObjectName(QString::fromUtf8("waypoint_3"));
        waypoint_3->setGeometry(QRect(190, 30, 89, 61));
        waypoint_3->setFont(font1);
        waypoint_4 = new QPushButton(control_group_waypoint);
        waypoint_4->setObjectName(QString::fromUtf8("waypoint_4"));
        waypoint_4->setGeometry(QRect(10, 90, 89, 61));
        waypoint_4->setFont(font1);
        waypoint_2 = new QPushButton(control_group_waypoint);
        waypoint_2->setObjectName(QString::fromUtf8("waypoint_2"));
        waypoint_2->setGeometry(QRect(100, 30, 89, 61));
        waypoint_2->setFont(font1);
        waypoint_5 = new QPushButton(control_group_waypoint);
        waypoint_5->setObjectName(QString::fromUtf8("waypoint_5"));
        waypoint_5->setGeometry(QRect(100, 90, 89, 61));
        waypoint_5->setFont(font1);
        lbl_map_R = new QLabel(tab_setting);
        lbl_map_R->setObjectName(QString::fromUtf8("lbl_map_R"));
        lbl_map_R->setGeometry(QRect(807, 7, 67, 17));
        lbl_map_R->setFont(font1);
        lbl_map_R->setStyleSheet(QString::fromUtf8("color: rgb(255, 25, 0)"));
        lbl_map_G = new QLabel(tab_setting);
        lbl_map_G->setObjectName(QString::fromUtf8("lbl_map_G"));
        lbl_map_G->setGeometry(QRect(807, 25, 67, 17));
        lbl_map_G->setFont(font1);
        lbl_map_G->setStyleSheet(QString::fromUtf8("color: rgb(0, 200, 0)"));
        lbl_map_B = new QLabel(tab_setting);
        lbl_map_B->setObjectName(QString::fromUtf8("lbl_map_B"));
        lbl_map_B->setGeometry(QRect(808, 42, 101, 17));
        lbl_map_B->setFont(font1);
        lbl_map_B->setStyleSheet(QString::fromUtf8("color: rgb(25, 25, 200)"));
        lbl_map_img = new ClickableLabel(tab_setting);
        lbl_map_img->setObjectName(QString::fromUtf8("lbl_map_img"));
        lbl_map_img->setGeometry(QRect(20, 65, 891, 551));
        lbl_map_img->setStyleSheet(QString::fromUtf8("border: 1px solid black;"));
        lbl_map_img->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignTop);
        layoutWidget1 = new QWidget(tab_setting);
        layoutWidget1->setObjectName(QString::fromUtf8("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(1028, 192, 177, 158));
        gridLayout_2 = new QGridLayout(layoutWidget1);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        gridLayout_2->setSizeConstraint(QLayout::SetMinimumSize);
        gridLayout_2->setVerticalSpacing(6);
        gridLayout_2->setContentsMargins(0, 0, 0, 0);
        pump_on = new QPushButton(layoutWidget1);
        pump_on->setObjectName(QString::fromUtf8("pump_on"));
        sizePolicy.setHeightForWidth(pump_on->sizePolicy().hasHeightForWidth());
        pump_on->setSizePolicy(sizePolicy);
        pump_on->setFont(font1);

        gridLayout_2->addWidget(pump_on, 1, 0, 1, 1);

        Brush_OFF = new QPushButton(layoutWidget1);
        Brush_OFF->setObjectName(QString::fromUtf8("Brush_OFF"));
        sizePolicy.setHeightForWidth(Brush_OFF->sizePolicy().hasHeightForWidth());
        Brush_OFF->setSizePolicy(sizePolicy);
        Brush_OFF->setFont(font1);

        gridLayout_2->addWidget(Brush_OFF, 0, 1, 1, 1);

        Pump_OFF = new QPushButton(layoutWidget1);
        Pump_OFF->setObjectName(QString::fromUtf8("Pump_OFF"));
        sizePolicy.setHeightForWidth(Pump_OFF->sizePolicy().hasHeightForWidth());
        Pump_OFF->setSizePolicy(sizePolicy);
        Pump_OFF->setFont(font1);

        gridLayout_2->addWidget(Pump_OFF, 1, 1, 1, 1);

        brush_on = new QPushButton(layoutWidget1);
        brush_on->setObjectName(QString::fromUtf8("brush_on"));
        sizePolicy.setHeightForWidth(brush_on->sizePolicy().hasHeightForWidth());
        brush_on->setSizePolicy(sizePolicy);
        brush_on->setFont(font1);

        gridLayout_2->addWidget(brush_on, 0, 0, 1, 1);

        vacumm_on = new QPushButton(layoutWidget1);
        vacumm_on->setObjectName(QString::fromUtf8("vacumm_on"));
        sizePolicy.setHeightForWidth(vacumm_on->sizePolicy().hasHeightForWidth());
        vacumm_on->setSizePolicy(sizePolicy);
        vacumm_on->setFont(font1);

        gridLayout_2->addWidget(vacumm_on, 2, 0, 1, 1);

        vacumm_OFF = new QPushButton(layoutWidget1);
        vacumm_OFF->setObjectName(QString::fromUtf8("vacumm_OFF"));
        sizePolicy.setHeightForWidth(vacumm_OFF->sizePolicy().hasHeightForWidth());
        vacumm_OFF->setSizePolicy(sizePolicy);
        vacumm_OFF->setFont(font1);

        gridLayout_2->addWidget(vacumm_OFF, 2, 1, 1, 1);

        control_group_waypoint_location = new QGroupBox(tab_setting);
        control_group_waypoint_location->setObjectName(QString::fromUtf8("control_group_waypoint_location"));
        control_group_waypoint_location->setGeometry(QRect(1213, 20, 151, 471));
        control_group_waypoint_location->setFont(font1);
        control_group_waypoint_location->setAlignment(Qt::AlignCenter);
        gb_location_1 = new QGroupBox(control_group_waypoint_location);
        gb_location_1->setObjectName(QString::fromUtf8("gb_location_1"));
        gb_location_1->setGeometry(QRect(10, 28, 131, 81));
        label_4 = new QLabel(gb_location_1);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(4, 31, 20, 17));
        label_5 = new QLabel(gb_location_1);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(4, 57, 20, 17));
        le_loaction_x_1 = new QLineEdit(gb_location_1);
        le_loaction_x_1->setObjectName(QString::fromUtf8("le_loaction_x_1"));
        le_loaction_x_1->setGeometry(QRect(21, 30, 100, 20));
        le_loaction_x_1->setFont(font1);
        le_loaction_y_1 = new QLineEdit(gb_location_1);
        le_loaction_y_1->setObjectName(QString::fromUtf8("le_loaction_y_1"));
        le_loaction_y_1->setGeometry(QRect(21, 56, 100, 20));
        le_loaction_y_1->setFont(font1);
        gb_location_2 = new QGroupBox(control_group_waypoint_location);
        gb_location_2->setObjectName(QString::fromUtf8("gb_location_2"));
        gb_location_2->setGeometry(QRect(10, 113, 131, 81));
        label_6 = new QLabel(gb_location_2);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(4, 31, 20, 17));
        label_7 = new QLabel(gb_location_2);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setGeometry(QRect(4, 57, 20, 17));
        le_loaction_x_2 = new QLineEdit(gb_location_2);
        le_loaction_x_2->setObjectName(QString::fromUtf8("le_loaction_x_2"));
        le_loaction_x_2->setGeometry(QRect(21, 30, 100, 20));
        le_loaction_x_2->setFont(font1);
        le_loaction_y_2 = new QLineEdit(gb_location_2);
        le_loaction_y_2->setObjectName(QString::fromUtf8("le_loaction_y_2"));
        le_loaction_y_2->setGeometry(QRect(21, 56, 100, 20));
        le_loaction_y_2->setFont(font1);
        gb_location_3 = new QGroupBox(control_group_waypoint_location);
        gb_location_3->setObjectName(QString::fromUtf8("gb_location_3"));
        gb_location_3->setGeometry(QRect(11, 203, 131, 81));
        label_8 = new QLabel(gb_location_3);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setGeometry(QRect(4, 31, 20, 17));
        label_9 = new QLabel(gb_location_3);
        label_9->setObjectName(QString::fromUtf8("label_9"));
        label_9->setGeometry(QRect(4, 57, 20, 17));
        le_loaction_x_3 = new QLineEdit(gb_location_3);
        le_loaction_x_3->setObjectName(QString::fromUtf8("le_loaction_x_3"));
        le_loaction_x_3->setGeometry(QRect(21, 30, 100, 20));
        le_loaction_x_3->setFont(font1);
        le_loaction_y_3 = new QLineEdit(gb_location_3);
        le_loaction_y_3->setObjectName(QString::fromUtf8("le_loaction_y_3"));
        le_loaction_y_3->setGeometry(QRect(21, 56, 100, 20));
        le_loaction_y_3->setFont(font1);
        gb_location_4 = new QGroupBox(control_group_waypoint_location);
        gb_location_4->setObjectName(QString::fromUtf8("gb_location_4"));
        gb_location_4->setGeometry(QRect(12, 287, 131, 81));
        label_12 = new QLabel(gb_location_4);
        label_12->setObjectName(QString::fromUtf8("label_12"));
        label_12->setGeometry(QRect(4, 31, 20, 17));
        label_13 = new QLabel(gb_location_4);
        label_13->setObjectName(QString::fromUtf8("label_13"));
        label_13->setGeometry(QRect(4, 57, 20, 17));
        le_loaction_x_4 = new QLineEdit(gb_location_4);
        le_loaction_x_4->setObjectName(QString::fromUtf8("le_loaction_x_4"));
        le_loaction_x_4->setGeometry(QRect(21, 30, 100, 20));
        le_loaction_x_4->setFont(font1);
        le_loaction_y_4 = new QLineEdit(gb_location_4);
        le_loaction_y_4->setObjectName(QString::fromUtf8("le_loaction_y_4"));
        le_loaction_y_4->setGeometry(QRect(21, 56, 100, 20));
        le_loaction_y_4->setFont(font1);
        gb_location_5 = new QGroupBox(control_group_waypoint_location);
        gb_location_5->setObjectName(QString::fromUtf8("gb_location_5"));
        gb_location_5->setGeometry(QRect(13, 376, 131, 81));
        label_14 = new QLabel(gb_location_5);
        label_14->setObjectName(QString::fromUtf8("label_14"));
        label_14->setGeometry(QRect(4, 31, 20, 17));
        label_15 = new QLabel(gb_location_5);
        label_15->setObjectName(QString::fromUtf8("label_15"));
        label_15->setGeometry(QRect(4, 57, 20, 17));
        le_loaction_x_5 = new QLineEdit(gb_location_5);
        le_loaction_x_5->setObjectName(QString::fromUtf8("le_loaction_x_5"));
        le_loaction_x_5->setGeometry(QRect(21, 30, 100, 20));
        le_loaction_x_5->setFont(font1);
        le_loaction_y_5 = new QLineEdit(gb_location_5);
        le_loaction_y_5->setObjectName(QString::fromUtf8("le_loaction_y_5"));
        le_loaction_y_5->setGeometry(QRect(21, 56, 100, 20));
        le_loaction_y_5->setFont(font1);
        gb_speed = new QGroupBox(tab_setting);
        gb_speed->setObjectName(QString::fromUtf8("gb_speed"));
        gb_speed->setGeometry(QRect(1214, 500, 149, 91));
        label_16 = new QLabel(gb_speed);
        label_16->setObjectName(QString::fromUtf8("label_16"));
        label_16->setGeometry(QRect(8, 31, 41, 17));
        btn_refresh_speed = new QPushButton(gb_speed);
        btn_refresh_speed->setObjectName(QString::fromUtf8("btn_refresh_speed"));
        btn_refresh_speed->setGeometry(QRect(8, 58, 60, 25));
        sizePolicy.setHeightForWidth(btn_refresh_speed->sizePolicy().hasHeightForWidth());
        btn_refresh_speed->setSizePolicy(sizePolicy);
        btn_apply_speed = new QPushButton(gb_speed);
        btn_apply_speed->setObjectName(QString::fromUtf8("btn_apply_speed"));
        btn_apply_speed->setGeometry(QRect(73, 58, 64, 25));
        sizePolicy.setHeightForWidth(btn_apply_speed->sizePolicy().hasHeightForWidth());
        btn_apply_speed->setSizePolicy(sizePolicy);
        le_speed = new QLineEdit(gb_speed);
        le_speed->setObjectName(QString::fromUtf8("le_speed"));
        le_speed->setGeometry(QRect(45, 30, 92, 20));
        le_speed->setFont(font1);
        layoutWidget2 = new QWidget(tab_setting);
        layoutWidget2->setObjectName(QString::fromUtf8("layoutWidget2"));
        layoutWidget2->setGeometry(QRect(925, 189, 100, 161));
        verticalLayout = new QVBoxLayout(layoutWidget2);
        verticalLayout->setSpacing(1);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        lbl_set_bbox = new QPushButton(layoutWidget2);
        lbl_set_bbox->setObjectName(QString::fromUtf8("lbl_set_bbox"));
        sizePolicy.setHeightForWidth(lbl_set_bbox->sizePolicy().hasHeightForWidth());
        lbl_set_bbox->setSizePolicy(sizePolicy);
        QFont font2;
        font2.setPointSize(13);
        font2.setBold(true);
        font2.setWeight(75);
        lbl_set_bbox->setFont(font2);

        verticalLayout->addWidget(lbl_set_bbox);

        lbl_start_clean = new QPushButton(layoutWidget2);
        lbl_start_clean->setObjectName(QString::fromUtf8("lbl_start_clean"));
        sizePolicy.setHeightForWidth(lbl_start_clean->sizePolicy().hasHeightForWidth());
        lbl_start_clean->setSizePolicy(sizePolicy);
        lbl_start_clean->setFont(font2);

        verticalLayout->addWidget(lbl_start_clean);

        lbl_reset_area = new QPushButton(layoutWidget2);
        lbl_reset_area->setObjectName(QString::fromUtf8("lbl_reset_area"));
        sizePolicy.setHeightForWidth(lbl_reset_area->sizePolicy().hasHeightForWidth());
        lbl_reset_area->setSizePolicy(sizePolicy);
        lbl_reset_area->setFont(font1);
        lbl_reset_area->setStyleSheet(QString::fromUtf8("color: rgb(255, 25, 0)"));

        verticalLayout->addWidget(lbl_reset_area);

        control_group_elevator_button = new QGroupBox(tab_setting);
        control_group_elevator_button->setObjectName(QString::fromUtf8("control_group_elevator_button"));
        control_group_elevator_button->setGeometry(QRect(926, 356, 121, 101));
        control_group_elevator_button->setFont(font1);
        control_group_elevator_button->setAlignment(Qt::AlignCenter);
        lbl_selected_button = new QLineEdit(control_group_elevator_button);
        lbl_selected_button->setObjectName(QString::fromUtf8("lbl_selected_button"));
        lbl_selected_button->setGeometry(QRect(10, 30, 101, 20));
        lbl_selected_button->setFont(font1);
        btn_send_button_cmd = new QPushButton(control_group_elevator_button);
        btn_send_button_cmd->setObjectName(QString::fromUtf8("btn_send_button_cmd"));
        btn_send_button_cmd->setGeometry(QRect(10, 60, 101, 32));
        sizePolicy.setHeightForWidth(btn_send_button_cmd->sizePolicy().hasHeightForWidth());
        btn_send_button_cmd->setSizePolicy(sizePolicy);
        btn_send_button_cmd->setFont(font1);
        Zeus_Widget->addTab(tab_setting, QString());
        tab_main = new QWidget();
        tab_main->setObjectName(QString::fromUtf8("tab_main"));
        btn_mapping = new QPushButton(tab_main);
        btn_mapping->setObjectName(QString::fromUtf8("btn_mapping"));
        btn_mapping->setGeometry(QRect(20, 100, 191, 181));
        btn_mapping->setStyleSheet(QString::fromUtf8("border: 1px solid white;"));
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/img/pic_mapping.jpg"), QSize(), QIcon::Normal, QIcon::Off);
        btn_mapping->setIcon(icon);
        btn_mapping->setIconSize(QSize(170, 180));
        btn_auto = new QPushButton(tab_main);
        btn_auto->setObjectName(QString::fromUtf8("btn_auto"));
        btn_auto->setGeometry(QRect(340, 100, 191, 181));
        btn_auto->setStyleSheet(QString::fromUtf8("border: 1px solid white;"));
        QIcon icon1;
        icon1.addFile(QString::fromUtf8(":img/pic_auto.jpg"), QSize(), QIcon::Normal, QIcon::On);
        btn_auto->setIcon(icon1);
        btn_auto->setIconSize(QSize(170, 170));
        btn_manual = new QPushButton(tab_main);
        btn_manual->setObjectName(QString::fromUtf8("btn_manual"));
        btn_manual->setGeometry(QRect(700, 100, 161, 181));
        btn_manual->setStyleSheet(QString::fromUtf8("border: 1px solid white;"));
        QIcon icon2;
        icon2.addFile(QString::fromUtf8(":img/pic_manual.jpg"), QSize(), QIcon::Normal, QIcon::On);
        btn_manual->setIcon(icon2);
        btn_manual->setIconSize(QSize(170, 170));
        btn_setting = new QPushButton(tab_main);
        btn_setting->setObjectName(QString::fromUtf8("btn_setting"));
        btn_setting->setGeometry(QRect(980, 100, 191, 181));
        btn_setting->setStyleSheet(QString::fromUtf8("border: 1px solid white;"));
        QIcon icon3;
        icon3.addFile(QString::fromUtf8(":img/pic_setting.jpg"), QSize(), QIcon::Normal, QIcon::On);
        btn_setting->setIcon(icon3);
        btn_setting->setIconSize(QSize(170, 170));
        lable_logo = new QLabel(tab_main);
        lable_logo->setObjectName(QString::fromUtf8("lable_logo"));
        lable_logo->setGeometry(QRect(1140, 530, 71, 41));
        lable_logo->setPixmap(QPixmap(QString::fromUtf8(":img/pic_logo_1.jpg")));
        lable_logo->setScaledContents(true);
        lable_logo->setAlignment(Qt::AlignCenter);
        water_clean = new QLabel(tab_main);
        water_clean->setObjectName(QString::fromUtf8("water_clean"));
        water_clean->setGeometry(QRect(1000, 481, 61, 20));
        water_clean->setPixmap(QPixmap(QString::fromUtf8(":img/water_blue.png")));
        water_clean->setScaledContents(true);
        water_dirt = new QLabel(tab_main);
        water_dirt->setObjectName(QString::fromUtf8("water_dirt"));
        water_dirt->setGeometry(QRect(1090, 480, 61, 21));
        water_dirt->setPixmap(QPixmap(QString::fromUtf8(":img/water_red.png")));
        water_dirt->setScaledContents(true);
        lbl_robot_speed = new QLabel(tab_main);
        lbl_robot_speed->setObjectName(QString::fromUtf8("lbl_robot_speed"));
        lbl_robot_speed->setGeometry(QRect(50, 400, 131, 131));
        lbl_robot_speed->setPixmap(QPixmap(QString::fromUtf8(":img/speed100.png")));
        lbl_robot_speed->setScaledContents(true);
        lbl_status_cleaning = new QLabel(tab_main);
        lbl_status_cleaning->setObjectName(QString::fromUtf8("lbl_status_cleaning"));
        lbl_status_cleaning->setGeometry(QRect(620, 10, 101, 61));
        lbl_status_cleaning->setFrameShape(QFrame::NoFrame);
        lbl_status_cleaning->setPixmap(QPixmap(QString::fromUtf8(":img/btn_clean2.PNG")));
        lbl_status_cleaning->setScaledContents(true);
        lbl_status_cleaning->setAlignment(Qt::AlignCenter);
        lbl_status_comeback = new QLabel(tab_main);
        lbl_status_comeback->setObjectName(QString::fromUtf8("lbl_status_comeback"));
        lbl_status_comeback->setGeometry(QRect(740, 10, 101, 61));
        lbl_status_comeback->setFrameShape(QFrame::NoFrame);
        lbl_status_comeback->setPixmap(QPixmap(QString::fromUtf8(":img/btn_homing2.PNG")));
        lbl_status_comeback->setScaledContents(true);
        lbl_status_comeback->setAlignment(Qt::AlignCenter);
        lbl_status_charging = new QLabel(tab_main);
        lbl_status_charging->setObjectName(QString::fromUtf8("lbl_status_charging"));
        lbl_status_charging->setGeometry(QRect(860, 10, 101, 61));
        lbl_status_charging->setFrameShape(QFrame::NoFrame);
        lbl_status_charging->setPixmap(QPixmap(QString::fromUtf8(":img/btn_charging2.PNG")));
        lbl_status_charging->setScaledContents(true);
        lbl_status_charging->setAlignment(Qt::AlignCenter);
        lbl_status_water_out = new QLabel(tab_main);
        lbl_status_water_out->setObjectName(QString::fromUtf8("lbl_status_water_out"));
        lbl_status_water_out->setGeometry(QRect(980, 10, 101, 61));
        lbl_status_water_out->setFrameShape(QFrame::NoFrame);
        lbl_status_water_out->setPixmap(QPixmap(QString::fromUtf8(":img/btn_waterin2.PNG")));
        lbl_status_water_out->setScaledContents(true);
        lbl_status_water_out->setAlignment(Qt::AlignCenter);
        lbl_status_water_in = new QLabel(tab_main);
        lbl_status_water_in->setObjectName(QString::fromUtf8("lbl_status_water_in"));
        lbl_status_water_in->setGeometry(QRect(1100, 10, 101, 61));
        lbl_status_water_in->setFrameShape(QFrame::NoFrame);
        lbl_status_water_in->setPixmap(QPixmap(QString::fromUtf8(":img/btn_waterout2.PNG")));
        lbl_status_water_in->setScaledContents(true);
        lbl_status_water_in->setAlignment(Qt::AlignCenter);
        btn_change_brush = new QPushButton(tab_main);
        btn_change_brush->setObjectName(QString::fromUtf8("btn_change_brush"));
        btn_change_brush->setGeometry(QRect(220, 380, 191, 181));
        btn_change_brush->setAutoFillBackground(false);
        btn_change_brush->setStyleSheet(QString::fromUtf8(""));
        btn_change_brush->setIconSize(QSize(200, 250));
        bottle_clean = new QLabel(tab_main);
        bottle_clean->setObjectName(QString::fromUtf8("bottle_clean"));
        bottle_clean->setGeometry(QRect(960, 360, 141, 151));
        bottle_clean->setPixmap(QPixmap(QString::fromUtf8(":img/water-removebg-preview (1).png")));
        bottle_clean->setScaledContents(true);
        bottle_dirt = new QLabel(tab_main);
        bottle_dirt->setObjectName(QString::fromUtf8("bottle_dirt"));
        bottle_dirt->setGeometry(QRect(1050, 360, 141, 151));
        bottle_dirt->setPixmap(QPixmap(QString::fromUtf8(":img/water-removebg-preview (1).png")));
        bottle_dirt->setScaledContents(true);
        group_manual = new QGroupBox(tab_main);
        group_manual->setObjectName(QString::fromUtf8("group_manual"));
        group_manual->setGeometry(QRect(610, 290, 351, 281));
        group_manual->setStyleSheet(QString::fromUtf8("background-color: rgb(255, 255, 255);"));
        group_manual->setAlignment(Qt::AlignCenter);
        group_clean = new QGroupBox(group_manual);
        group_clean->setObjectName(QString::fromUtf8("group_clean"));
        group_clean->setGeometry(QRect(10, 40, 191, 181));
        group_clean->setAlignment(Qt::AlignCenter);
        btn_Dry = new QPushButton(group_clean);
        btn_Dry->setObjectName(QString::fromUtf8("btn_Dry"));
        btn_Dry->setGeometry(QRect(10, 40, 81, 61));
        btn_wet = new QPushButton(group_clean);
        btn_wet->setObjectName(QString::fromUtf8("btn_wet"));
        btn_wet->setGeometry(QRect(100, 40, 81, 61));
        btn_both_dry_wet = new QPushButton(group_clean);
        btn_both_dry_wet->setObjectName(QString::fromUtf8("btn_both_dry_wet"));
        btn_both_dry_wet->setGeometry(QRect(100, 110, 81, 61));
        QFont font3;
        font3.setPointSize(11);
        btn_both_dry_wet->setFont(font3);
        btn_sterilization = new QPushButton(group_clean);
        btn_sterilization->setObjectName(QString::fromUtf8("btn_sterilization"));
        btn_sterilization->setGeometry(QRect(10, 110, 81, 61));
        QFont font4;
        font4.setPointSize(10);
        btn_sterilization->setFont(font4);
        setup_group = new QGroupBox(group_manual);
        setup_group->setObjectName(QString::fromUtf8("setup_group"));
        setup_group->setGeometry(QRect(210, 40, 121, 181));
        setup_group->setAlignment(Qt::AlignCenter);
        btn_brush_up = new QPushButton(setup_group);
        btn_brush_up->setObjectName(QString::fromUtf8("btn_brush_up"));
        btn_brush_up->setGeometry(QRect(20, 40, 81, 61));
        btn_brush_down = new QPushButton(setup_group);
        btn_brush_down->setObjectName(QString::fromUtf8("btn_brush_down"));
        btn_brush_down->setGeometry(QRect(20, 110, 81, 61));
        btn_manual_setting = new QPushButton(group_manual);
        btn_manual_setting->setObjectName(QString::fromUtf8("btn_manual_setting"));
        btn_manual_setting->setGeometry(QRect(230, 244, 89, 31));
        txt_mapping = new QLabel(tab_main);
        txt_mapping->setObjectName(QString::fromUtf8("txt_mapping"));
        txt_mapping->setGeometry(QRect(40, 290, 151, 61));
        txt_mapping->setPixmap(QPixmap(QString::fromUtf8(":img/mapping.png")));
        txt_mapping->setScaledContents(true);
        txt_mapping->setAlignment(Qt::AlignCenter);
        txt_mapping_2 = new QLabel(tab_main);
        txt_mapping_2->setObjectName(QString::fromUtf8("txt_mapping_2"));
        txt_mapping_2->setGeometry(QRect(360, 290, 151, 61));
        txt_mapping_2->setPixmap(QPixmap(QString::fromUtf8(":img/autoclean.png")));
        txt_mapping_2->setScaledContents(true);
        txt_mapping_2->setAlignment(Qt::AlignCenter);
        txt_mapping_3 = new QLabel(tab_main);
        txt_mapping_3->setObjectName(QString::fromUtf8("txt_mapping_3"));
        txt_mapping_3->setGeometry(QRect(690, 290, 151, 61));
        txt_mapping_3->setPixmap(QPixmap(QString::fromUtf8(":img/manualclean.png")));
        txt_mapping_3->setScaledContents(true);
        txt_mapping_3->setAlignment(Qt::AlignCenter);
        txt_mapping_4 = new QLabel(tab_main);
        txt_mapping_4->setObjectName(QString::fromUtf8("txt_mapping_4"));
        txt_mapping_4->setGeometry(QRect(1000, 290, 151, 61));
        txt_mapping_4->setPixmap(QPixmap(QString::fromUtf8(":img/setting.png")));
        txt_mapping_4->setScaledContents(true);
        txt_mapping_4->setAlignment(Qt::AlignCenter);
        btn_sucktion = new QPushButton(tab_main);
        btn_sucktion->setObjectName(QString::fromUtf8("btn_sucktion"));
        btn_sucktion->setGeometry(QRect(440, 370, 271, 151));
        btn_sucktion->setStyleSheet(QString::fromUtf8("border: 1px solid white;"));
        QIcon icon4;
        icon4.addFile(QString::fromUtf8(":img/sucktion1_1-removebg-preview.png"), QSize(), QIcon::Normal, QIcon::On);
        btn_sucktion->setIcon(icon4);
        btn_sucktion->setIconSize(QSize(250, 300));
        btn_brush = new QPushButton(tab_main);
        btn_brush->setObjectName(QString::fromUtf8("btn_brush"));
        btn_brush->setGeometry(QRect(720, 370, 271, 151));
        btn_brush->setStyleSheet(QString::fromUtf8("border: 1px solid white;"));
        QIcon icon5;
        icon5.addFile(QString::fromUtf8(":img/brush1_2-removebg-preview.png"), QSize(), QIcon::Normal, QIcon::On);
        btn_brush->setIcon(icon5);
        btn_brush->setIconSize(QSize(250, 300));
        label_11 = new QLabel(tab_main);
        label_11->setObjectName(QString::fromUtf8("label_11"));
        label_11->setGeometry(QRect(10, 10, 51, 41));
        label_11->setPixmap(QPixmap(QString::fromUtf8(":img/pic_setting.jpg")));
        label_11->setScaledContents(true);
        Zeus_Widget->addTab(tab_main, QString());
        btn_brush->raise();
        btn_mapping->raise();
        btn_auto->raise();
        btn_manual->raise();
        btn_setting->raise();
        lable_logo->raise();
        water_clean->raise();
        water_dirt->raise();
        lbl_robot_speed->raise();
        lbl_status_cleaning->raise();
        lbl_status_comeback->raise();
        lbl_status_charging->raise();
        lbl_status_water_out->raise();
        lbl_status_water_in->raise();
        btn_change_brush->raise();
        bottle_clean->raise();
        bottle_dirt->raise();
        txt_mapping->raise();
        txt_mapping_2->raise();
        txt_mapping_3->raise();
        txt_mapping_4->raise();
        btn_sucktion->raise();
        group_manual->raise();
        label_11->raise();
        label_2 = new QLabel(Widget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(1129, 14, 111, 31));
        label_2->setWordWrap(true);
        label_3 = new QLabel(Widget);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(1254, 10, 41, 41));
        label_3->setPixmap(QPixmap(QString::fromUtf8(":img/free-icon-no-wifi-1686538.png")));
        label_3->setScaledContents(true);
        lbl_charge_gage = new QLabel(Widget);
        lbl_charge_gage->setObjectName(QString::fromUtf8("lbl_charge_gage"));
        lbl_charge_gage->setGeometry(QRect(1314, 4, 75, 51));
        lbl_charge_gage->setPixmap(QPixmap(QString::fromUtf8(":img/empty-battery-512-removebg-preview.png")));
        lbl_charge_gage->setScaledContents(true);
        lbl_charge1 = new QLabel(Widget);
        lbl_charge1->setObjectName(QString::fromUtf8("lbl_charge1"));
        lbl_charge1->setGeometry(QRect(1322, 20, 15, 20));
        lbl_charge1->setPixmap(QPixmap(QString::fromUtf8(":img/battery_green.png")));
        lbl_charge1->setScaledContents(true);
        lbl_charge2 = new QLabel(Widget);
        lbl_charge2->setObjectName(QString::fromUtf8("lbl_charge2"));
        lbl_charge2->setGeometry(QRect(1341, 20, 15, 20));
        lbl_charge2->setPixmap(QPixmap(QString::fromUtf8(":img/battery_green.png")));
        lbl_charge2->setScaledContents(true);
        lbl_charge3 = new QLabel(Widget);
        lbl_charge3->setObjectName(QString::fromUtf8("lbl_charge3"));
        lbl_charge3->setGeometry(QRect(1359, 20, 15, 20));
        lbl_charge3->setPixmap(QPixmap(QString::fromUtf8(":img/battery_green.png")));
        lbl_charge3->setScaledContents(true);
        btn_reset = new QPushButton(Widget);
        btn_reset->setObjectName(QString::fromUtf8("btn_reset"));
        btn_reset->setGeometry(QRect(1030, 16, 89, 31));

        retranslateUi(Widget);

        Zeus_Widget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(Widget);
    } // setupUi

    void retranslateUi(QWidget *Widget)
    {
        Widget->setWindowTitle(QApplication::translate("Widget", "Widget", nullptr));
        label_10->setText(QApplication::translate("Widget", "Main", nullptr));
        control_group->setTitle(QApplication::translate("Widget", "Controller", nullptr));
        Front->setText(QApplication::translate("Widget", "F", nullptr));
        Right_Front->setText(QApplication::translate("Widget", "RF", nullptr));
        Back->setText(QApplication::translate("Widget", "B", nullptr));
        Left->setText(QApplication::translate("Widget", "L", nullptr));
        Stop->setText(QApplication::translate("Widget", "S", nullptr));
        Left_Back->setText(QApplication::translate("Widget", "LB", nullptr));
        Right->setText(QApplication::translate("Widget", "R", nullptr));
        Right_Back->setText(QApplication::translate("Widget", "RB", nullptr));
        Left_Front->setText(QApplication::translate("Widget", "LF", nullptr));
        control_group_waypoint->setTitle(QApplication::translate("Widget", "Waypoint", nullptr));
        waypoint_1->setText(QApplication::translate("Widget", "1", nullptr));
        waypoint_3->setText(QApplication::translate("Widget", "3", nullptr));
        waypoint_4->setText(QApplication::translate("Widget", "4", nullptr));
        waypoint_2->setText(QApplication::translate("Widget", "2", nullptr));
        waypoint_5->setText(QApplication::translate("Widget", "5", nullptr));
        lbl_map_R->setText(QApplication::translate("Widget", "R: Goal", nullptr));
        lbl_map_G->setText(QApplication::translate("Widget", "G: Robot", nullptr));
        lbl_map_B->setText(QApplication::translate("Widget", "B : ref Frame", nullptr));
        lbl_map_img->setText(QString());
        pump_on->setText(QApplication::translate("Widget", "Water \n"
"On", nullptr));
        Brush_OFF->setText(QApplication::translate("Widget", "Brush \n"
"OFF", nullptr));
        Pump_OFF->setText(QApplication::translate("Widget", "Water \n"
"OFF", nullptr));
        brush_on->setText(QApplication::translate("Widget", "Brush \n"
"On", nullptr));
        vacumm_on->setText(QApplication::translate("Widget", "Vacumm \n"
"On", nullptr));
        vacumm_OFF->setText(QApplication::translate("Widget", "Vacumm \n"
"OFF", nullptr));
        control_group_waypoint_location->setTitle(QApplication::translate("Widget", "Location (m)", nullptr));
        gb_location_1->setTitle(QApplication::translate("Widget", "1", nullptr));
        label_4->setText(QApplication::translate("Widget", "X: ", nullptr));
        label_5->setText(QApplication::translate("Widget", "Y: ", nullptr));
        le_loaction_x_1->setText(QApplication::translate("Widget", "-14.75", nullptr));
        le_loaction_y_1->setText(QApplication::translate("Widget", "-0.5", nullptr));
        gb_location_2->setTitle(QApplication::translate("Widget", "2", nullptr));
        label_6->setText(QApplication::translate("Widget", "X: ", nullptr));
        label_7->setText(QApplication::translate("Widget", "Y: ", nullptr));
        le_loaction_x_2->setText(QApplication::translate("Widget", "0.0", nullptr));
        le_loaction_y_2->setText(QApplication::translate("Widget", "0.0", nullptr));
        gb_location_3->setTitle(QApplication::translate("Widget", "3", nullptr));
        label_8->setText(QApplication::translate("Widget", "X: ", nullptr));
        label_9->setText(QApplication::translate("Widget", "Y: ", nullptr));
        le_loaction_x_3->setText(QApplication::translate("Widget", "10.30", nullptr));
        le_loaction_y_3->setText(QApplication::translate("Widget", "0.0", nullptr));
        gb_location_4->setTitle(QApplication::translate("Widget", "4", nullptr));
        label_12->setText(QApplication::translate("Widget", "X: ", nullptr));
        label_13->setText(QApplication::translate("Widget", "Y: ", nullptr));
        le_loaction_x_4->setText(QApplication::translate("Widget", "29.75", nullptr));
        le_loaction_y_4->setText(QApplication::translate("Widget", "0.05", nullptr));
        gb_location_5->setTitle(QApplication::translate("Widget", "5", nullptr));
        label_14->setText(QApplication::translate("Widget", "X: ", nullptr));
        label_15->setText(QApplication::translate("Widget", "Y: ", nullptr));
        le_loaction_x_5->setText(QApplication::translate("Widget", "29.75", nullptr));
        le_loaction_y_5->setText(QApplication::translate("Widget", "8.30", nullptr));
        gb_speed->setTitle(QApplication::translate("Widget", "Speed (m/s)", nullptr));
        label_16->setText(QApplication::translate("Widget", "Max: ", nullptr));
        btn_refresh_speed->setText(QApplication::translate("Widget", "Refresh", nullptr));
        btn_apply_speed->setText(QApplication::translate("Widget", "Apply", nullptr));
        le_speed->setText(QApplication::translate("Widget", "0.25", nullptr));
        lbl_set_bbox->setText(QApplication::translate("Widget", "Set \n"
"Area", nullptr));
        lbl_start_clean->setText(QApplication::translate("Widget", "Start \n"
"Auto Clean", nullptr));
        lbl_reset_area->setText(QApplication::translate("Widget", "Reset\n"
"Area", nullptr));
        control_group_elevator_button->setTitle(QApplication::translate("Widget", "Elevator Button", nullptr));
        lbl_selected_button->setText(QApplication::translate("Widget", "down", nullptr));
        btn_send_button_cmd->setText(QApplication::translate("Widget", "Send", nullptr));
        Zeus_Widget->setTabText(Zeus_Widget->indexOf(tab_setting), QApplication::translate("Widget", "Main", nullptr));
        btn_mapping->setText(QString());
        btn_auto->setText(QString());
        btn_manual->setText(QString());
        btn_setting->setText(QString());
        lable_logo->setText(QString());
        water_clean->setText(QString());
        water_dirt->setText(QString());
        lbl_robot_speed->setText(QString());
        lbl_status_cleaning->setText(QString());
        lbl_status_comeback->setText(QString());
        lbl_status_charging->setText(QString());
        lbl_status_water_out->setText(QString());
        lbl_status_water_in->setText(QString());
        btn_change_brush->setText(QString());
        bottle_clean->setText(QString());
        bottle_dirt->setText(QString());
        group_manual->setTitle(QApplication::translate("Widget", "Manual Setting", nullptr));
        group_clean->setTitle(QApplication::translate("Widget", "Cleaning_Setting", nullptr));
        btn_Dry->setText(QApplication::translate("Widget", "Dry", nullptr));
        btn_wet->setText(QApplication::translate("Widget", "Wet", nullptr));
        btn_both_dry_wet->setText(QApplication::translate("Widget", "Dry && Wet", nullptr));
        btn_sterilization->setText(QApplication::translate("Widget", "Sterilization", nullptr));
        setup_group->setTitle(QApplication::translate("Widget", "Brush_Setting", nullptr));
        btn_brush_up->setText(QApplication::translate("Widget", "Brush UP", nullptr));
        btn_brush_down->setText(QApplication::translate("Widget", "Brush DN", nullptr));
        btn_manual_setting->setText(QApplication::translate("Widget", "Setting", nullptr));
        txt_mapping->setText(QString());
        txt_mapping_2->setText(QString());
        txt_mapping_3->setText(QString());
        txt_mapping_4->setText(QString());
        btn_sucktion->setText(QString());
        btn_brush->setText(QString());
        label_11->setText(QString());
        Zeus_Widget->setTabText(Zeus_Widget->indexOf(tab_main), QApplication::translate("Widget", "Setting", nullptr));
        label_2->setText(QApplication::translate("Widget", "Not Connected", nullptr));
        label_3->setText(QString());
        lbl_charge_gage->setText(QString());
        lbl_charge1->setText(QString());
        lbl_charge2->setText(QString());
        lbl_charge3->setText(QString());
        btn_reset->setText(QApplication::translate("Widget", "Reset", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Widget: public Ui_Widget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_WIDGET_H
