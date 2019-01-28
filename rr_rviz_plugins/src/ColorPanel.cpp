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


    hue_slider_min = new QSlider(Qt::Horizontal, 0);
    //color_slider_min->setInvertedAppearance(true);
    hue_spinner_min = new QSpinBox();
    hue_spinner_min->setRange(0,100);
    hue_spinner_min->setSingleStep(1);
    hue_spinner_min->setValue(0); //#TODO: this should load from file saved (#TODO add save to file button)
    connect(hue_slider_min, SIGNAL(valueChanged(int)), hue_spinner_min, SLOT(setValue(int)));
    connect(hue_spinner_min, SIGNAL(valueChanged(int)), this, SLOT(hueMinCallback(int)));


    //connect(reset_btn, SIGNAL (released()), this, SLOT (ColorCallback()));


    hue_slider_max = new QSlider(Qt::Horizontal, 0);
    hue_spinner_max = new QSpinBox();
    hue_spinner_max->setRange(0,100);
    hue_spinner_max->setSingleStep(1);
    hue_spinner_max->setValue(0); //#TODO: this should load from file saved (#TODO add save to file button)
    connect(hue_slider_max, SIGNAL(valueChanged(int)), hue_spinner_max, SLOT(setValue(int)));
    connect(hue_spinner_max, SIGNAL(valueChanged(int)), this, SLOT(hueMaxCallback(int)));


    QHBoxLayout *hbox_1 = new QHBoxLayout;
    hbox_1->addWidget(hue_spinner_min);
    hbox_1->addWidget(hue_slider_min);
    hbox_1->addWidget(hue_slider_max);
    hbox_1->addWidget(hue_spinner_max);


    QVBoxLayout *layout = new QVBoxLayout;
    layout->addLayout(hbox_1);
    //layout->addWidget(color_slider_min);
    //layout->addWidget(color_spinner_min);
    setLayout(layout);


    color_pub = nh.advertise<rr_platform::color>("/color_tuning", 0); //#TODO
}

void ColorPanel::publishColor() {
  rr_platform::color msg;
  msg.header.frame_id = "hls_tuned";
  msg.hue_min = hue_min;
  msg.hue_max = hue_max;

  color_pub.publish(msg);
}

void ColorPanel::colorCallback() {
    //rr_platform::race_reset reset;
    //color_pub.publish(reset);
}

void ColorPanel::hueMinCallback(int value) {
  hue_min = value;
  if (hue_min > hue_max) {
    hue_slider_max->setValue(hue_min);
  }
  publishColor();
}
void ColorPanel::hueMaxCallback(int value) {
  hue_max = value;
  if (hue_max < hue_min) {
    hue_slider_min->setValue(hue_max);
  }
  publishColor();
}

}

PLUGINLIB_EXPORT_CLASS( rr_rviz_plugins::ColorPanel, rviz::Panel)
