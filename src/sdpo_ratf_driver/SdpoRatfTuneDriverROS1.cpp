#include "sdpo_ratf_driver/SdpoRatfTuneDriverROS1.h"

#include <std_msgs/Bool.h>





namespace sdpo_ratf_driver
{



SdpoRatfTuneDriverROS1::SdpoRatfTuneDriverROS1()
    : nh_priv_("~") , serial_comms_first_fault_(true)
{

  try
  {
    getParam();
  }
  catch (std::exception& e)
  {
    ROS_FATAL("[%s] Error reading the node parameters (%s)",
              ros::this_node::getName().c_str(),
              e.what());

    ros::shutdown();

    return;
  }

  sample_time_ = ros::Time::now();

  pub_mot_data_ = nh_.advertise
      <sdpo_drivers_interfaces::MotDataArrayROS1>("motors_data", 10);
  pub_switch_ = nh_.advertise<std_msgs::Bool>("switch_state", 10);

  sub_mot_ref_ = nh_.subscribe
      <sdpo_drivers_interfaces::MotRefArrayROS1>(
          "motors_ref", 1,
          &SdpoRatfTuneDriverROS1::subMotRef, this);



  rob_.setSerialPortName(serial_port_name_);
  rob_.openSerial();

  rob_.run = std::bind(&SdpoRatfTuneDriverROS1::run, this);
  rob_.init();



  srv_solenoid_ = nh_.advertiseService("set_solenoid_state",
      &SdpoRatfTuneDriverROS1::srvSolenoid, this);



  serial_port_timer_ = nh_.createTimer(ros::Duration(1),
      boost::bind(&SdpoRatfTuneDriverROS1::checkSerialComms, this));


} // SdpoRatfTuneDriverROS1::SdpoRatfTuneDriverROS1()





void SdpoRatfTuneDriverROS1::getParam()
{

  nh_priv_.param("encoder_res", encoder_res_, 64.0);
  nh_priv_.param("gear_reduction", gear_reduction_, 43.8);
  nh_priv_.param<std::string>(
      "serial_port_name", serial_port_name_, "/dev/ttyACM0");

  for (auto& m : rob_.mot)
  {
    m.encoder_res = encoder_res_;
    m.gear_reduction = gear_reduction_;
  }

  ROS_INFO("[%s] Encoder resolution: %lf (ticks/rev)",
           ros::this_node::getName().c_str(),
           rob_.mot[0].encoder_res);

  ROS_INFO("[%s] Gear reduction ratio: %lf (n:1)",
           ros::this_node::getName().c_str(),
           rob_.mot[0].gear_reduction);

  ROS_INFO("[%s] Serial port: %s",
           ros::this_node::getName().c_str(),
           serial_port_name_.c_str());

} // void SdpoRatfTuneDriverROS1::getParam()





void SdpoRatfTuneDriverROS1::checkSerialComms()
{

  if (!rob_.isSerialOpen())
  {
    if (serial_comms_first_fault_)
    {
      serial_comms_first_fault_ = false;

      ROS_INFO("[%s] Couldn't open the serial port %s. Will retry every second.",
               ros::this_node::getName().c_str(),
               serial_port_name_.c_str());
    }

    rob_.mtx_.lock();
    rob_.stopMotors();
    rob_.mtx_.unlock();

    rob_.closeSerial();
    rob_.openSerial();

    if (rob_.isSerialOpen())
    {
      serial_comms_first_fault_ = true;

      ROS_INFO("[%s] Opened serial port %s.",
               ros::this_node::getName().c_str(),
               serial_port_name_.c_str());

      rob_.init();
    }
  }

} // void SdpoRatfTuneDriverROS1::checkSerialComms()





void SdpoRatfTuneDriverROS1::run()
{

  try
  {
    if (ros::Duration(ros::Time::now() - sample_time_) >
        ros::Duration(kWatchdogMotWRef))
    {
      rob_.mtx_.lock();
      rob_.stopMotors();
      rob_.mtx_.unlock();
    }
  }
  catch (std::exception& e)
  {
    ROS_WARN("[%s] Not possible to check the driver timeout condition (%s)",
             ros::this_node::getName().c_str(),
             e.what());

    sample_time_ = ros::Time::now();

    return;
  }

  pubMotData();
  pubSwitch();

} // void SdpoRatfTuneDriverROS1::run()





void SdpoRatfTuneDriverROS1::pubMotData()
{

  sdpo_drivers_interfaces::MotDataArrayROS1 msg;



  msg.stamp = ros::Time::now();
  msg.mot_data.resize(4);



  rob_.mtx_.lock();

  for (int i = 0; i < 4; i++)
  {

    msg.mot_data[i].sample_period = rob_.mot[i].sample_time;

    msg.mot_data[i].pwm = rob_.mot[i].pwm;

    msg.mot_data[i].enc_delta = rob_.mot[i].getEncTicksDeltaPub();

    msg.mot_data[i].ticks_per_rev =
        rob_.mot[i].encoder_res * rob_.mot[i].gear_reduction;

    msg.mot_data[i].ang_speed = rob_.mot[i].w;

  }

  rob_.mtx_.unlock();



  pub_mot_data_.publish(msg);

} // void SdpoRatfTuneDriverROS1::pubMotEnc()





void SdpoRatfTuneDriverROS1::pubSwitch()
{
  std_msgs::Bool msg;

  rob_.mtx_.lock();
  msg.data = rob_.switch_state;
  rob_.mtx_.unlock();

  pub_switch_.publish(msg);
} // void SdpoRatfTuneDriverROS1::pubSwitch()





void SdpoRatfTuneDriverROS1::subMotRef(
    const sdpo_drivers_interfaces::MotRefArrayROS1::ConstPtr& msg)
{

  if (msg->ang_speed_ref.size() >= 4)
  {
    rob_.mtx_.lock();
    for (int i = 0; i < 4; i++)
    {
      rob_.mot[i].w_r = msg->ang_speed_ref[i].ref;
    }
    rob_.mtx_.unlock();

    sample_time_ = msg->stamp;
  }

} // void SdpoRatfTuneDriverROS1::subMotRef(const sdpo_drivers_interfaces::MotRefArrayROS1::ConstPtr& msg)





bool SdpoRatfTuneDriverROS1::srvSolenoid(std_srvs::SetBool::Request& request,
                                     std_srvs::SetBool::Response& response)
{
  rob_.mtx_.lock();
  rob_.solenoid_state = request.data;
  rob_.mtx_.unlock();

  response.success = true;
  response.message = "";
  return true;
}



} // namespace sdpo_ratf_driver
