#include "sdpo_ratf_driver/SdpoRatfTuneDriverROS2.h"





int main(int argc, char* argv[])
{

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<sdpo_ratf_driver::SdpoRatfTuneDriverROS2>());

  rclcpp::shutdown();

  return 0;

} // int main(int argc, char* argv[])
