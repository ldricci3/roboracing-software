#ifndef PROJECT_RESETPANEL_H
#define PROJECT_RESETPANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <QSlider>
#include <QDoubleSpinBox>

/*
 * All of our panels need to be under the rr_rviz_plugins namespace.
 */
namespace rr_rviz_plugins {

class ColorPanel : public rviz::Panel {
Q_OBJECT
public:
    ColorPanel(QWidget *parent = 0);

protected:
    QSlider *color_slider_min;
    QDoubleSpinBox *color_spinner_min;
    ros::NodeHandle nh;
    ros::Publisher color_pub;

private slots:
    void colorCallback();
};

}

#endif
