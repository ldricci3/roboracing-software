#include <rr_rviz_plugins/ColorPanel.h>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <pluginlib/class_list_macros.h>

namespace rr_rviz_plugins {

ColorPanel::ColorPanel(QWidget *parent)
  : rviz::Panel(parent) // Base class constructor
{
    hue_min = 0;
    hue_max = 0;


    color_slider_min = new QSlider(Qt::Horizontal, 0);
    //color_slider_min->setInvertedAppearance(true);
    color_spinner_min = new QSpinBox();
    color_spinner_min->setRange(0,100);
    color_spinner_min->setSingleStep(1);
    color_spinner_min->setValue(0); //#TODO: this should load from file saved (#TODO add save to file button)
    connect(color_slider_min, SIGNAL(valueChanged(int)), color_spinner_min, SLOT(setValue(int)));
    connect(color_spinner_min, SIGNAL(valueChanged(int)), this, SLOT(hueMinCallback(int)));


    //color_pub = nh.advertise<rr_platform::race_reset>("/reset_detected", 0); //#TODO
    //connect(reset_btn, SIGNAL (released()), this, SLOT (ColorCallback()));


    color_slider_max = new QSlider(Qt::Horizontal, 0);
    color_spinner_max = new QSpinBox();
    color_spinner_max->setRange(0,100);
    color_spinner_max->setSingleStep(1);
    color_spinner_max->setValue(0); //#TODO: this should load from file saved (#TODO add save to file button)
    connect(color_slider_max, SIGNAL(valueChanged(int)), color_spinner_max, SLOT(setValue(int)));
    connect(color_spinner_max, SIGNAL(valueChanged(int)), this, SLOT(hueMaxCallback(int)));


    QHBoxLayout *hbox_1 = new QHBoxLayout;
    hbox_1->addWidget(color_spinner_min);
    hbox_1->addWidget(color_slider_min);
    hbox_1->addWidget(color_slider_max);
    hbox_1->addWidget(color_spinner_max);


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

void ColorPanel::hueMinCallback(int value) {
  hue_min = value;
  if (hue_min > hue_max) {
    color_slider_max->setValue(hue_min);
  }
}
void ColorPanel::hueMaxCallback(int value) {
  hue_max = value;
  if (hue_max < hue_min) {
    color_slider_min->setValue(hue_max);
  }
}

}

PLUGINLIB_EXPORT_CLASS( rr_rviz_plugins::ColorPanel, rviz::Panel)
