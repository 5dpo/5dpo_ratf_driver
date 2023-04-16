#include "sdpo_ratf_ros_driver/SdpoRatfROSDriver.h"

#include <std_msgs/Bool.h>

namespace sdpo_ratf_ros_driver {

SdpoRatfROSDriver::SdpoRatfROSDriver() : loop_rate_(kCtrlFreq) {
  readParam();

  rob_.setSerialPortName(serial_port_name_);
  rob_.openSerial();
  rob_.init();

  pub_mot_enc_ = nh.advertise<sdpo_ros_interfaces_hw::mot_enc_array>(
      "motors_encoders", 1);
  pub_switch_ = nh.advertise<std_msgs::Bool>("switch_state", 1);
  sub_mot_ref_ = nh.subscribe("motors_ref", 1,
                              &SdpoRatfROSDriver::subMotRef, this);

  srv_solenoid_ = nh.advertiseService("set_solenoid_state",
      &SdpoRatfROSDriver::srvSolenoid, this);
}

void SdpoRatfROSDriver::run() {
  sample_time_prev_ = ros::Time::now() - ros::Duration(kWatchdogMotWRef * 2);
  sample_time_ = ros::Time::now();

  while (ros::ok()) {
    if (sample_time_ - sample_time_prev_ > ros::Duration(kWatchdogMotWRef)) {
      /*ROS_WARN("[sdpo_ratf_ros_driver] Watchdog timer to monitor the motors "
               "angular speed reference triggered (action: stop motors)");*/
      rob_.mtx_.lock();
      rob_.stopMotors();
      rob_.mtx_.unlock();
    }

    if (!rob_.isSerialOpen()) {
      /*ROS_WARN("[sdpo_ratf_ros_driver] Serial port %s not available "
               "(action: stop motors + trying close and open port)",
               serial_port_name_.c_str());*/
      rob_.mtx_.lock();
      rob_.stopMotors();
      rob_.mtx_.unlock();
      rob_.closeSerial();
      rob_.openSerial();
      if (rob_.isSerialOpen()) {
        rob_.init();
        ROS_INFO("[sdpo_ratf_ros_driver] Serial port %s successfully reconnected",
                 serial_port_name_.c_str());
      }
    } else {
      pubMotEnc();
      pubSwitch();
    }

    ros::spinOnce();
    loop_rate_.sleep();
  }
}

bool SdpoRatfROSDriver::readParam() {
  ros::NodeHandle nh_private("~");

  for (auto& m : rob_.mot) {
    nh_private.param<double>("encoder_res", m.encoder_res, 48.0);
    nh_private.param<double>("gear_reduction", m.gear_reduction, 64.0);
  }
  nh_private.param<std::string>("serial_port_name", serial_port_name_,
                                "/dev/ttyACM0");

  auto print_is_default_param_set =
      [&nh_private](const std::string& param_name) {
    if (!nh_private.hasParam(param_name)) {
      ROS_INFO("[sdpo_ratf_ros_driver] Parameter %s not set in the "
               "parameter server (using default value)",
               param_name.c_str());
    }
  };

  print_is_default_param_set("encoder_res");
  ROS_INFO("[sdpo_ratf_ros_driver] Encoder resolution: %lf (ticks/rev)",
           rob_.mot[0].encoder_res);

  print_is_default_param_set("gear_reduction");
  ROS_INFO("[sdpo_ratf_ros_driver] Gear reduction ratio: %lf (n:1)",
           rob_.mot[0].gear_reduction);

  print_is_default_param_set("serial_port_name");
  ROS_INFO("[sdpo_ratf_ros_driver] Serial port: %s", serial_port_name_.c_str());

  return true;
}

void SdpoRatfROSDriver::pubMotEnc() {
  sdpo_ros_interfaces_hw::mot_enc_array msg;

  msg.stamp = ros::Time::now();
  msg.mot_enc_array_data.resize(4);

  rob_.mtx_.lock();
  for (int i = 0; i < 4; i++) {
    msg.mot_enc_array_data[i].encoder_delta = rob_.mot[i].getEncTicksDeltaPub();
    msg.mot_enc_array_data[i].ticks_per_rev =
        rob_.mot[i].encoder_res * rob_.mot[i].gear_reduction;
    msg.mot_enc_array_data[i].angular_speed = rob_.mot[i].w;
  }
  rob_.mtx_.unlock();

  pub_mot_enc_.publish(msg);
}

void SdpoRatfROSDriver::pubSwitch() {
  std_msgs::Bool msg;

  rob_.mtx_.lock();
  msg.data = rob_.switch_state;
  rob_.mtx_.unlock();

  pub_switch_.publish(msg);
}

void SdpoRatfROSDriver::subMotRef(const sdpo_ros_interfaces_hw::mot_ref& msg) {
  if (msg.angular_speed_ref.size() >= 4) {
    rob_.mtx_.lock();
    for (int i = 0; i < 4; i++) {
      rob_.mot[i].w_r = msg.angular_speed_ref[i];
    }
    rob_.mtx_.unlock();

    sample_time_prev_ = sample_time_;
    sample_time_ = ros::Time::now();
  }
}

bool SdpoRatfROSDriver::srvSolenoid(std_srvs::SetBool::Request& request,
    std_srvs::SetBool::Response& response) {
  rob_.mtx_.lock();
  rob_.solenoid_state = request.data;
  rob_.mtx_.unlock();

  response.success = true;
  response.message = "";
  return true;
}

} // namespace sdpo_ratf_ros_driver
