#pragma once

#include <iostream>
#include <functional>
#include <mutex>

#include <sdpo_serial_port/AsyncSerial.h>

#include "sdpo_ratf_driver/SerialChannelsConfig.h"





namespace sdpo_ratf_driver
{



struct Motor
{

 public:

  double encoder_res = 1;
  double gear_reduction = 1;
  int32_t enc_ticks = 0;
  int32_t enc_ticks_prev = 0;
  int32_t enc_ticks_delta = 0;
  int32_t enc_ticks_delta_pub = 0;
  int16_t pwm = 0;
  double w_r = 0;
  double w = 0;
  double sample_time = 0;
  double sample_time_prev = 0;
  double sample_period = 0;

  int mot_ctrl_freq   = 100;
  int16_t max_mot_pwm = 1023;



 public:

  void setEncoderRes(const double& enc_res);
  void setGearReduction(const double& gear_ratio);

  void setEncTicksDelta(const int32_t& delta_enc_ticks);
  void setEncTicks(const int32_t& total_enc_ticks);
  double getEncTicksDeltaPub();

  void setPWM(const int16_t& pwm_mot);

  void setWr(const double& w_ref);

  void setSampleTime(const int32_t& time_sample);

  void reset();

 protected:

  void setW();

};// struct Motor





class Robot5dpoRatf
{

 public:

  Motor mot[4];
  std::mutex mtx_;

  bool switch_1_state = false;
  bool switch_2_state = false;
  bool solenoid_1_state = false;
  bool solenoid_2_state = false;

  std::function<void()> run;

 protected:

  std::string serial_port_name_;
  SerialChannelsConfig *serial_cfg_;
  CallbackAsyncSerial *serial_async_;



 public:

  Robot5dpoRatf();
  Robot5dpoRatf(std::string serial_port_name);
  ~Robot5dpoRatf();

  void init();

  bool openSerial(const bool dbg = false);
  void closeSerial(const bool dbg = false);
  bool isSerialOpen();

  void setSerialPortName(const std::string& serial_port_name);

  void reset();

  virtual void stopMotors();

 protected:

  virtual void rcvSerialData(const char *data, unsigned int len);
  virtual void sendSerialData();

};// class Robot5dpoRatf



} // namespace sdpo_ratf_driver
