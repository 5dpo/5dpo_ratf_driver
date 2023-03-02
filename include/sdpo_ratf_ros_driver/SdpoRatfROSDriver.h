#pragma once

#include <ros/ros.h>
#include <sdpo_ros_interfaces_hw/mot_enc_array.h>
#include <sdpo_ros_interfaces_hw/mot_ref.h>

#include "sdpo_ratf_ros_driver/Robot5dpoRatf.h"

namespace sdpo_ratf_ros_driver {

const double kWatchdogMotWRef = 0.2;

class SdpoRatfROSDriver {
 private:
  ros::NodeHandle nh;
  ros::Rate loop_rate_;

  ros::Publisher pub_mot_enc_;
  ros::Subscriber sub_mot_ref_;

  ros::Time sample_time_;
  ros::Time sample_time_prev_;

  Robot5dpoRatf rob_;

  std::string serial_port_name_;

 public:
  SdpoRatfROSDriver();
  ~SdpoRatfROSDriver() = default;

  void run();

 private:
  bool readParam();

  void pubMotEnc();
  void subMotRef(const sdpo_ros_interfaces_hw::mot_ref& msg);
};

} // namespace sdpo_ratf_ros_driver
