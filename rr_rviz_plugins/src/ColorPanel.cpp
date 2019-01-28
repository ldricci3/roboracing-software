#include <rr_rviz_plugins/ColorPanel.h>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <pluginlib/class_list_macros.h>

namespace rr_rviz_plugins {

ColorPanel::ColorPanel(QWidget *parent)
  : rviz::Panel(parent) // Base class constructor
{
    hue_min = 0; //#TODO: load these values from save file (and add a save button)
    hue_max = 0;
    lightness_min = 0;
    lightness_max = 0;

    //#TODO: put all these panels into a single object and pass in ref to variable they change.

    //Hue stuff
    hue_label = new QLabel("H:");
    hue_slider_min = new QSlider(Qt::Horizontal, 0);
    hue_spinner_min = new QSpinBox();
    hue_spinner_min->setRange(0, 360);
    hue_spinner_min->setSingleStep(1);
    hue_spinner_min->setValue(hue_min);
    connect(hue_slider_min, SIGNAL(valueChanged(int)), hue_spinner_min, SLOT(setValue(int)));
    connect(hue_spinner_min, SIGNAL(valueChanged(int)), this, SLOT(hueMinCallback(int)));


    hue_slider_max = new QSlider(Qt::Horizontal, 0);
    hue_spinner_max = new QSpinBox();
    hue_spinner_max->setRange(0,360);
    hue_spinner_max->setSingleStep(1);
    hue_spinner_max->setValue(hue_max);
    connect(hue_slider_max, SIGNAL(valueChanged(int)), hue_spinner_max, SLOT(setValue(int)));
    connect(hue_spinner_max, SIGNAL(valueChanged(int)), this, SLOT(hueMaxCallback(int)));


    QHBoxLayout *hbox_1 = new QHBoxLayout;
    hbox_1->addWidget(hue_label);
    hbox_1->addWidget(hue_spinner_min);
    hbox_1->addWidget(hue_slider_min);
    hbox_1->addWidget(hue_slider_max);
    hbox_1->addWidget(hue_spinner_max);


    //Lightness stuff
    lightness_label = new QLabel("L:");
    lightness_slider_min = new QSlider(Qt::Horizontal, 0);
    lightness_spinner_min = new QSpinBox();
    lightness_spinner_min->setRange(0, 300);
    lightness_spinner_min->setSingleStep(1);
    lightness_spinner_min->setValue(lightness_min);
    connect(lightness_slider_min, SIGNAL(valueChanged(int)), lightness_spinner_min, SLOT(setValue(int)));
    connect(lightness_spinner_min, SIGNAL(valueChanged(int)), this, SLOT(lightnessMinCallback(int)));


    lightness_slider_max = new QSlider(Qt::Horizontal, 0);
    lightness_spinner_max = new QSpinBox();
    lightness_spinner_max->setRange(0,300);
    lightness_spinner_max->setSingleStep(1);
    lightness_spinner_max->setValue(lightness_max);
    connect(lightness_slider_max, SIGNAL(valueChanged(int)), lightness_spinner_max, SLOT(setValue(int)));
    connect(lightness_spinner_max, SIGNAL(valueChanged(int)), this, SLOT(lightnessMaxCallback(int)));


    QHBoxLayout *hbox_2 = new QHBoxLayout;
    hbox_2->addWidget(lightness_label);
    hbox_2->addWidget(lightness_spinner_min);
    hbox_2->addWidget(lightness_slider_min);
    hbox_2->addWidget(lightness_slider_max);
    hbox_2->addWidget(lightness_spinner_max);


    QVBoxLayout *layout = new QVBoxLayout;
    layout->addLayout(hbox_1);
    layout->addLayout(hbox_2);


    setLayout(layout);

    color_pub = nh.advertise<rr_platform::color>("/hls_tuner", 0); //#TODO
}

void ColorPanel::publishColor() {
  rr_platform::color msg;
  msg.header.frame_id = "hls_tuner";
  msg.hue_min = hue_min;
  msg.hue_max = hue_max;
  msg.lightness_min = lightness_min;
  msg.lightness_max = lightness_max;

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
void ColorPanel::lightnessMinCallback(int value) {
  lightness_min = value;
  if (lightness_min > lightness_max) {
    lightness_slider_max->setValue(lightness_min);
  }
  publishColor();
}
void ColorPanel::lightnessMaxCallback(int value) {
  lightness_max = value;
  if (lightness_max < lightness_min) {
    lightness_slider_min->setValue(lightness_max);
  }
  publishColor();
}


}

PLUGINLIB_EXPORT_CLASS( rr_rviz_plugins::ColorPanel, rviz::Panel)
