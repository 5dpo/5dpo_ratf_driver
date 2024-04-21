#include "sdpo_ratf_driver/SdpoRatfTuneDriverROS2.h"





namespace sdpo_ratf_driver
{



using namespace std::chrono_literals;





SdpoRatfTuneDriverROS2::SdpoRatfTuneDriverROS2()
    : Node("sdpo_ratf_driver") , serial_comms_first_fault_(true)
{

  try
  {
    getParam();
  }
  catch (std::exception& e)
  {
    RCLCPP_FATAL(this->get_logger(),
                 "Error reading the node parameters (%s)", e.what());

    rclcpp::shutdown();

    return;
  }

  sample_time_ = this->now();

  pub_mot_data_ = this->create_publisher
      <sdpo_drivers_interfaces::msg::MotDataArray>("motors_data", 10);
  pub_switch_ = this->create_publisher<std_msgs::msg::Bool>("switch_state", 10);

  sub_mot_ref_ = this->create_subscription
      <sdpo_drivers_interfaces::msg::MotRefArray>(
          "motors_ref", 1,
          std::bind(&SdpoRatfTuneDriverROS2::subMotRef, this,
              std::placeholders::_1));



  rob_.setSerialPortName(serial_port_name_);
  rob_.openSerial();

  rob_.run = std::bind(&SdpoRatfTuneDriverROS2::run, this);
  rob_.init();



  srv_solenoid_ = this->create_service
      <std_srvs::srv::SetBool>("set_solenoid_state",
      std::bind(&SdpoRatfTuneDriverROS2::srvSolenoid, this,
                std::placeholders::_1, std::placeholders::_2));



  serial_port_timer_ = this->create_wall_timer(
      1s, std::bind(&SdpoRatfTuneDriverROS2::checkSerialComms, this));

} // SdpoRatfTuneDriverROS2::SdpoRatfTuneDriverROS2()





void SdpoRatfTuneDriverROS2::getParam()
{

  encoder_res_    = this->declare_parameter<double>("encoder_res", 64.0);
  gear_reduction_ = this->declare_parameter<double>("gear_reduction", 43.8);
  serial_port_name_ =
      this->declare_parameter<std::string>("serial_port_name", "/dev/ttyACM0");

  for (auto& m : rob_.mot)
  {
    m.encoder_res = encoder_res_;
    m.gear_reduction = gear_reduction_;
  }

  RCLCPP_INFO(this->get_logger(),
              "Encoder resolution: %lf (ticks/rev)",
              rob_.mot[0].encoder_res);

  RCLCPP_INFO(this->get_logger(),
              "Gear reduction ratio: %lf (n:1)",
              rob_.mot[0].gear_reduction);

  RCLCPP_INFO(this->get_logger(),
              "Serial port: %s",
              serial_port_name_.c_str());

} // void SdpoRatfTuneDriverROS2::getParam()





void SdpoRatfTuneDriverROS2::checkSerialComms()
{

  if (!rob_.isSerialOpen())
  {
    if (serial_comms_first_fault_)
    {
      serial_comms_first_fault_ = false;

      RCLCPP_INFO(this->get_logger(),
                  "Couldn't open the serial port %s. Will retry every second.",
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

      RCLCPP_INFO(this->get_logger(),
                  "Opened serial port %s.",
                  serial_port_name_.c_str());

      rob_.init();
    }
  }

} // void SdpoRatfTuneDriverROS2::checkSerialComms()





void SdpoRatfTuneDriverROS2::run()
{

  try
  {
    if (rclcpp::Duration(this->now() - sample_time_) >
        rclcpp::Duration::from_seconds(kWatchdogMotWRef))
    {
      rob_.mtx_.lock();
      rob_.stopMotors();
      rob_.mtx_.unlock();
    }
  }
  catch (std::exception& e)
  {
    RCLCPP_WARN(this->get_logger(),
                "Not possible to check the driver timeout condition (%s)",
                e.what());

    sample_time_ = this->now();

    return;
  }

  pubMotData();
  pubSwitch();

} // void SdpoRatfTuneDriverROS2::run()





void SdpoRatfTuneDriverROS2::pubMotData()
{

  sdpo_drivers_interfaces::msg::MotDataArray msg;



  msg.stamp = this->now();
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



  pub_mot_data_->publish(msg);

} // void SdpoRatfTuneDriverROS2::pubMotEnc()





void SdpoRatfTuneDriverROS2::pubSwitch()
{
  std_msgs::msg::Bool msg;

  rob_.mtx_.lock();
  msg.data = rob_.switch_state;
  rob_.mtx_.unlock();

  pub_switch_->publish(msg);
} // void SdpoRatfTuneDriverROS2::pubSwitch()





void SdpoRatfTuneDriverROS2::subMotRef(
    const sdpo_drivers_interfaces::msg::MotRefArray::SharedPtr msg)
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

} // void SdpoRatfTuneDriverROS2::subMotRef(const sdpo_drivers_interfaces::msg::MotRefArray::SharedPtr msg)





void SdpoRatfTuneDriverROS2::srvSolenoid(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  rob_.mtx_.lock();
  rob_.solenoid_state = request->data;
  rob_.mtx_.unlock();

  response->success = true;
  response->message = "";

} // void SdpoRatfTuneDriverROS2::srvSolenoid(...)



} // namespace sdpo_ratf_driver
