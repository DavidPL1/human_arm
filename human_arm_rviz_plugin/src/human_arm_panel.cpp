#include <ros/console.h>
#include <ros/package.h>
#include <stdio.h>

#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPushButton>
#include <QThread>
#include <QTimer>
#include <QVBoxLayout>

#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"
#include "human_arm_panel.h"
#include "std_msgs/String.h"

namespace human_arm_rviz_plugin {

MotionDirectorPanel::MotionDirectorPanel(QWidget *parent)
    : rviz::Panel(parent) {
  QHBoxLayout *config_layout = new QHBoxLayout;

  start_button_ = new QPushButton("Run Config");
  refresh_button_ = new QPushButton("Refresh Available Configs");
  config_name_ = "";

  config_layout->addWidget(new QLabel("Config Name:"));
  config_name_editor_ = new QComboBox;
  config_layout->addWidget(config_name_editor_);

  QVBoxLayout *layout = new QVBoxLayout;
  layout->addLayout(config_layout);
  layout->addWidget(refresh_button_);
  layout->addWidget(start_button_);
  setLayout(layout);

  setup_publisher();
  fetch_configs();

  connect(config_name_editor_, SIGNAL(activated(int)), this,
          SLOT(update_config()));
  connect(start_button_, SIGNAL(clicked()), this, SLOT(run_config()));
  connect(refresh_button_, SIGNAL(clicked()), this, SLOT(fetch_configs()));
}

void MotionDirectorPanel::fetch_configs() {
  std::string config_subdir = "default";
  
  ros::param::get("~config_subdir", config_subdir);
  ROS_INFO_STREAM("config_subdir param: " << config_subdir);
  std::string path =
      ros::package::getPath("human_arm_motion_server") + "/config/" + config_subdir + "/recfiles";

  config_name_editor_->clear();

  for (const auto &entry : boost::filesystem::directory_iterator(path)) {
    ROS_DEBUG_STREAM("Found config file: " << entry.path().filename().string());
    config_name_editor_->insertItem(
        0, QString::fromStdString(entry.path().stem().string()));
  }
  config_name_editor_->insertItem(0, QString::fromStdString(""));
}

void MotionDirectorPanel::update_config() {
  config_name_ = config_name_editor_->currentText();
}

void MotionDirectorPanel::setup_publisher() {
  // TODO should be ns agnostic
  run_config_pub_ = nh_.advertise<std_msgs::String>("arm/play_config", 1);
}

void MotionDirectorPanel::run_config() {
  if (ros::ok() && run_config_pub_) {
    std_msgs::String msg;
    msg.data = config_name_.toStdString();

    run_config_pub_.publish(msg);
  }
}

void MotionDirectorPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
}

void MotionDirectorPanel::load(const rviz::Config &config) {
  rviz::Panel::load(config);
}

} // namespace human_arm_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(human_arm_rviz_plugin::MotionDirectorPanel, rviz::Panel)
// END_TUTORIAL
