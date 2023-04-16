# [5dpo_ratf_ros_driver](https://github.com/5dpo/5dpo_ratf_ros_driver)

This repository implements a driver within a ROS package to communicate with the
firmware present in the four-wheeled omnidirectional robotic platform used by
the [5dpo FEUP Robotics Team](https://github.com/5dpo) in the
[Robot@Factory](https://www.festivalnacionalrobotica.pt/) competition. The
driver is required for communicating with the robot and have available all its
different functions.

The serial communication is handled by
[Boost.Asio](https://www.boost.org/doc/libs/1_80_0/doc/html/boost_asio.html).
This communication is based on the example `4_callback` provided in the
[serial-port](https://github.com/fedetft/serial-port) GitHub repository.

**Version 1.0.2**

**With this version, it is possible to do:**

- Communicate with Arduino Mega 2560 using Boost.Asio
  ([sdpo_ros_serial_port](https://github.com/5dpo/5dpo_ros_serial_port))
- Subscribe motors angular speed reference
- Publish encoders data (encoders + wheels angular speed)
- Read encoders
- Set motors speed
- Reset driver upon reset signal
- Watchdog timer to monitor the motors angular speed reference
- Send serial message to the firmware upon reconnection of the serial port
  communication

**The next version will add these features:**

- Read switch
- Set solenoid
- Publish optionally the odometry data

## ROS

**Current version:**

- [Ubuntu 20.04.5 LTS](https://releases.ubuntu.com/focal/)
- [ROS Noetic](https://wiki.ros.org/noetic)

### Dependencies

- [roscpp](https://wiki.ros.org/roscpp)
- [sdpo_ros_interfaces_hw](https://github.com/5dpo/5dpo_ros_interfaces)
- [sdpo_ros_serial_port](https://github.com/5dpo/5dpo_ros_serial_port)
- [serial_communication_channels](https://github.com/5dpo/serial_communication_channels)
- [std_msgs](https://wiki.ros.org/std_msgs)

### Parameters

- enc_ticks_per_rev (`float = 64.0`): resolution of the encoder (ticks/rot)
- gear_reduction (`float = 43.8`): reduction ratio of the transmissions
  (\[gear_reduction:1\])
- serial_port_name (`std::string = "/dev/ttyACM0"`): name of the serial port

### Subscribes

- motors_ref
  ([mot_ref.msg](https://github.com/5dpo/5dpo_ros_interfaces/blob/main/5dpo_ros_interfaces_hw/msg/mot_ref.msg))

### Publishes

- motors_encoders
  ([mot_enc_array.msg](https://github.com/5dpo/5dpo_ros_interfaces/blob/main/5dpo_ros_interfaces_hw/msg/mot_enc_array.msg))
- switch_state
  ([Bool.msg](https://docs.ros.org/en/api/std_msgs/html/msg/Bool.html))

### Services

None.

### Actions

None.

## Usage

### Build

```sh
# Create catkin workspace
mkdir -p ~/catkin_ws/src

# Clone repository
cd ~/catkin_ws/src
git clone git@github.com:5dpo/5dpo_ratf_ros_driver.git

# Build
cd ..
catkin build
```

### Launch

```sh
roslaunch sdpo_ratf_ros_driver sdpo_ratf_ros_driver.launch
```

## Contacts

If you have any questions or you want to know more about this work, please
contact one of the contributors of this package:

- Héber Miguel Sobreira ([gitlab](https://gitlab.inesctec.pt/heber.m.sobreira),
  [inesctec](mailto:heber.m.sobreira@inesctec.pt))
- Ricardo B. Sousa ([github](https://github.com/sousarbarb/),
  [gitlab](https://gitlab.com/sousarbarb/),
  [personal](mailto:sousa.ricardob@outlook.com),
  [feup:professor](mailto:rbs@fe.up.pt),
  [feup:student](mailto:up201503004@edu.fe.up.pt),
  [inesctec](mailto:ricardo.b.sousa@inesctec.pt))
