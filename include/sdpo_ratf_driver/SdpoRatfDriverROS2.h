#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sdpo_drivers_interfaces/msg/mot_enc_array.hpp>
#include <sdpo_drivers_interfaces/msg/mot_ref_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include "sdpo_ratf_driver/Robot5dpoRatf.h"





namespace sdpo_ratf_driver
{



const double kWatchdogMotWRef = 0.2;





class SdpoRatfDriverROS2 : public rclcpp::Node
{

 private:

  rclcpp::Publisher<sdpo_drivers_interfaces::msg::MotEncArray>::SharedPtr pub_mot_enc_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                       pub_switch_;
  rclcpp::Subscription<sdpo_drivers_interfaces::msg::MotRefArray>::SharedPtr sub_mot_ref_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_solenoid_;

  rclcpp::TimerBase::SharedPtr serial_port_timer_;

  rclcpp::Time sample_time_;

  Robot5dpoRatf rob_;

  double encoder_res_;
  double gear_reduction_;
  std::string serial_port_name_;

  bool serial_comms_first_fault_;



 public:

  SdpoRatfDriverROS2();
  ~SdpoRatfDriverROS2() = default;

 private:

  void getParam();

  void checkSerialComms();

  void run();

  void pubMotEnc();
  void pubSwitch();
  void subMotRef(
      const sdpo_drivers_interfaces::msg::MotRefArray::SharedPtr msg);

  void srvSolenoid(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                   const std::shared_ptr<std_srvs::srv::SetBool::Response> response);

};// class SdpoRatfDriverROS2 : public rclcpp::Node



} // namespace sdpo_ratf_driver
