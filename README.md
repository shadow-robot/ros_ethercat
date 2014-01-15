ros_ethercat
------------

  This is a reimplementation of the main loop of pr2_ethercat with reduced dependencies on PR2 software. It may be useful to people that have developed ROS software for their own hardware that uses etherCAT for communication and inexorably based their work on the software for Willow Garage's PR2. By reusing the existing etherCAT drivers and pr2_hardware_interface and instantiating a ros_control controller manager 

*Remaining pr2 dependencies*

 1. pr2_hardware_interface
 2. ethercat_hardware
 3. pr2_mechanism_model
 4. pr2_msgs
  
*Removed pr2 dependencies*

 1. pr2_robot
 2. pr2_ethercat
 3. pr2_bringup
 4. pr2_controller_interface
 5. pr2_controller_manager


**Compatibility**

Some modifications are required in software that will use ros_ethercat instead of pr2_ethercat.

*Controllers*

 1. ```#include <controller_interface/controller.h>```<br>
   instead of<br>```#include <pr2_controller_interface/controller.h>```
 2. ```class MyController : public controller_interface::Controller<pr2_mechanism_model::RobotState>```<br>
   instead of<br>```class MyController : public pr2_controller_interface::Controller```
 3. Modify update function like so<br>```virtual void update(const ros::Time&, const ros::Duration&)```<br>
 4. 
