#include <rr_rviz_plugins/ColorPanel.h>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <pluginlib/class_list_macros.h>

namespace rr_rviz_plugins {

ColorPanel::ColorPanel(QWidget *parent)
  : rviz::Panel(parent) // Base class constructor
{
    color_slider_min = new QSlider(Qt::Horizontal, 0);
    color_spinner_min = new QDoubleSpinBox();
    color_spinner_min->setRange(0,100);
    color_spinner_min->setSingleStep(1);
    color_spinner_min->setValue(0); //#TODO: this should load from file saved (#TODO add save to file button)
    //color_pub = nh.advertise<rr_platform::race_reset>("/reset_detected", 0); //#TODO
    //connect(reset_btn, SIGNAL (released()), this, SLOT (ColorCallback()));

    QHBoxLayout *hbox_1 = new QHBoxLayout;
    hbox_1->addWidget(color_slider_min);
    hbox_1->addWidget(color_spinner_min);


    QVBoxLayout *layout = new QVBoxLayout;
    layout->addLayout(hbox_1);
    //layout->addWidget(color_slider_min);
    //layout->addWidget(color_spinner_min);
    setLayout(layout);
}

void ColorPanel::colorCallback() {
    //rr_platform::race_reset reset;
    //color_pub.publish(reset);
}

}

PLUGINLIB_EXPORT_CLASS( rr_rviz_plugins::ColorPanel, rviz::Panel)
