#ifndef PROJECT_RESETPANEL_H
#define PROJECT_RESETPANEL_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <QSlider>
#include <QSpinBox>

/*
 * All of our panels need to be under the rr_rviz_plugins namespace.
 */
namespace rr_rviz_plugins {

class ColorPanel : public rviz::Panel {
Q_OBJECT
public:
    ColorPanel(QWidget *parent = 0);
    int hue_min;
    int hue_max;

protected:
    QSlider *color_slider_min;
    QSpinBox *color_spinner_min;
    QSlider *color_slider_max;
    QSpinBox *color_spinner_max;
    ros::NodeHandle nh;
    ros::Publisher color_pub;

    void publish();

private slots:
    void colorCallback();
    void hueMinCallback(int value);
    void hueMaxCallback(int value);
};

}

#endif
