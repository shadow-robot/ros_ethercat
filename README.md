ros_ethercat
------------

  This is a reimplementation of the main loop of pr2_ethercat with reduced dependencies on PR2 software. It may be useful to people that have developed ROS software for their own hardware that uses etherCAT for communication and inexorably based their work on the software for Willow Garage's PR2. By reusing the existing etherCAT drivers, replacing pr2_hardware_interface and instantiating a ros_control controller manager 

*Remaining pr2 dependencies*

 1. ethercat_hardware 
 2. pr2_mechanism_msgs  
  
*Removed pr2 dependencies*

 1. pr2_robot
 2. pr2_ethercat
 3. pr2_bringup
 4. pr2_controller_interface
 5. pr2_controller_manager
 6. pr2_msgs
 7. pr2_mechanism_model
 8. pr2_hardware_interface


**Compatibility**

Some modifications are required in software that will use ros_ethercat instead of pr2_ethercat.

*Controllers*

 1. `#include <controller_interface/controller.h>`<br>
instead of<br>`#include <pr2_controller_interface/controller.h>`
 2. `class MyController : public controller_interface::Controller<ros_ethercat_mechanism_model::RobotState>`<br>
instead of<br>`class MyController : public pr2_controller_interface::Controller```
 3. Controller's update function should be declared `virtual void update(const ros::Time&, const ros::Duration&)`<br>
 4. `controller_manager` and `controller_interface` should be used as dependencies in `CMakeLists.txt` and `package.xml`  files instead of `pr2_controller_manager` and `pr2_controller_interface` respectively.
 5. In `package.xml` file in export tag use `<controller_interface plugin="${prefix}/controller_plugins.xml"/>` 
 6. In `controller_plugins.xml` file use `base_class_type="controller_interface::ControllerBase" />`
 
*launch files*

 1. Replace `pr2_ethercat` with `ros_ethercat` in launch files
 2. Since pr2_controller_manager is no longer used it no longer publishes joint_states or mechanism_statistics. Joint states can be published with the join_state_controller in ros_controllers. Automatically starting this controller requires changes in launch files and the addition of a yaml file.
 3. `calibrate.py` file from pr2_bringup must be replicated in another package and modified to use controller_manager (from ros_control). Launch files that execute `calibrate.py` should be modified with the correct package name.
 
*transmissions*

 Objects of type ros_ethercat_mechanism_model::Transmission are allowed and actually needed to keep RobotState updated.
