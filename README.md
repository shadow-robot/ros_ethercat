ros_ethercat
------------
## CI Statuses

Check | Status
---|---
Build|[<img src="https://codebuild.eu-west-2.amazonaws.com/badges?uuid=eyJlbmNyeXB0ZWREYXRhIjoiNHlPY3dOQTFFVzNEbzJSOTUvSGF2RU1UQll2bExhR09BdVF1elB0Uktjc2tOVFVGS2xyVXBLdW00R25IMzBNSFIxSTBuOXJkM2srYUwydHIxUXppTlI4PSIsIml2UGFyYW1ldGVyU3BlYyI6IlAvMFBwN1dsM1I4bElLZWciLCJtYXRlcmlhbFNldFNlcmlhbCI6MX0%3D&branch=melodic-devel"/>](https://eu-west-2.console.aws.amazon.com/codesuite/codebuild/projects/auto_ros_ethercat_melodic-devel_install_check/)
Style|[<img src="https://codebuild.eu-west-2.amazonaws.com/badges?uuid=eyJlbmNyeXB0ZWREYXRhIjoiVHpnQ2RQNWtGbXVEV2MwcFFWY3FSUFZMZXlRMk1hbDdSN2x0NjdzczNTWC9nRUR5MHc2NjZEWFhBd0t2dkRPQ3I3UnA5RzVNVE1MNU1uZEJ0MGVOWnp3PSIsIml2UGFyYW1ldGVyU3BlYyI6IlN0a2ZVUlA1MHZnbGprQ2giLCJtYXRlcmlhbFNldFNlcmlhbCI6MX0%3D&branch=melodic-devel"/>](https://eu-west-2.console.aws.amazon.com/codesuite/codebuild/projects/auto_ros_ethercat_melodic-devel_style_check/)
Code Coverage|[<img src="https://codebuild.eu-west-2.amazonaws.com/badges?uuid=eyJlbmNyeXB0ZWREYXRhIjoiMkFZSXo5dWt0WVcvS2JibDRqNzAvYW5CTFh6ZnZCd1ZWQTNGOE1lUkhFcUhpQWRvZmZwMWVRVGprK3ppemdRUmdqZXg4SmE5TEVocFFBYTk2NFkwdFZnPSIsIml2UGFyYW1ldGVyU3BlYyI6IlVDcGpTQW9rWlhGMzBUZWgiLCJtYXRlcmlhbFNldFNlcmlhbCI6MX0%3D&branch=melodic-devel"/>](https://eu-west-2.console.aws.amazon.com/codesuite/codebuild/projects/auto_ros_ethercat_melodic-devel_code_coverage/)

## Overview

  This is a reimplementation of the main loop of pr2_ethercat without dependencies on PR2 software. It was mainly developed to be used by Shadow Robot Company. It may be useful to anyone people that has developed ROS software for their own robot and used etherCAT for communication. Most likely such software would be based on the software for Willow Garage's PR2. ros_ethercat reuses existing etherCAT drivers (eml package) and instantiates a ros_control controller manager. Part of the software in this repository comes from repositories pr2_mechanism, pr2_robot, pr2_ethercat_drivers. The parts specific to the pr2 robot only have been removed. Therefore, the same license (BSD) has been used.

*pr2 packages no longer required*

  1. ethercat_hardware
  2. pr2_bringup
  3. pr2_ethercat
  4. pr2_mechanism
  5. pr2_controller_interface
  6. pr2_controller_manager
  7. pr2_hardware_interface
  8. pr2_mechanism_diagnostics
  9. pr2_mechanism_model

You can find the architecture diagram below for a closer look at how this works.

![Architecture Diagram](doc/RosEtherCAT.png)

## Compatibility

In software with previous pr2 dependencies that is switched to this package, the following modifications are required.

### Controllers

 1. `#include <controller_interface/controller.h>`<br>
instead of<br>`#include <pr2_controller_interface/controller.h>`
 2. `class MyController : public controller_interface::Controller<ros_ethercat_mechanism_model::RobotState>`<br>
instead of<br>`class MyController : public pr2_controller_interface::Controller`
 3. Controller's update function should be declared as <br>`virtual void update(const ros::Time&, const ros::Duration&)`<br> and starting function as <br> `virtual void starting(const ros::Time& time)`,br.
 4. `controller_manager` and `controller_interface` should be used as dependencies in `CMakeLists.txt` and `package.xml` files instead of `pr2_controller_manager` and `pr2_controller_interface` respectively.
 5. In `package.xml` file in export tag use `<controller_interface plugin="${prefix}/controller_plugins.xml"/>`
 6. In `controller_plugins.xml` file use `base_class_type="controller_interface::ControllerBase" />`

### launch files

 1. Replace `pr2_ethercat` with `ros_ethercat_loop` in launch files
 2. Since pr2_controller_manager is no longer used, joint_states or mechanism_statistics are no longer published by it. Joint states are now  published with the join_state_controller from ros_controllers. To start this controller this line need to be added in the main launch file <br> `<include file="$(find ros_ethercat_model)/launch/joint_state_publisher.launch"/>`
 3. `calibrate.py` file from pr2_bringup is now included in ros_ethercat_model package and renamed to just `calibrate` according to ROS convention. Launch files that execute `calibrate.py` should be modified to find it there.

### transmissions

 Transmissions are still defined in urdf in the pr2 style and inherit from `ros_ethercat_model::Transmission`. This class will initialize the actuators associated with this transmission.

## New features
 1. ros_ethercat accepts a new argument `--period` which is the period of main ethercat loop in msec. If not given the default value is 1ms.
 2. There is a helper bash script called ethercat_grant. This will grant to the ros_ethercat_loop executable the ability to be ran from a normal user without root privileges. E.g. of use<br>`rosrun ros_ethercat_loop ethercat_grant`
