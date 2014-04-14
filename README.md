ros_ethercat
------------

  This is a reimplementation of the main loop of pr2_ethercat without dependencies on PR2 software. It was mainly developed to be used by Shadow Robot Company. It may be useful to anyone people that have developed ROS software for their own robot and use etherCAT for communication. Most likely such would be based on the software for Willow Garage's PR2. ros_ethercat reuses existing etherCAT drivers (eml package)and instantiates a ros_control controller manager 
  
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


**Compatibility**

In software with previous pr2 dependencies that is switched to this package, the following modifications are required.

*Controllers*

 1. `#include <controller_interface/controller.h>`<br>
instead of<br>`#include <pr2_controller_interface/controller.h>`
 2. `class MyController : public controller_interface::Controller<ros_ethercat_mechanism_model::RobotState>`<br>
instead of<br>`class MyController : public pr2_controller_interface::Controller```
 3. Controller's update function should be declared as`virtual void update(const ros::Time&, const ros::Duration&)`<br>
 4. `controller_manager` and `controller_interface` should be used as dependencies in `CMakeLists.txt` and `package.xml`  files instead of `pr2_controller_manager` and `pr2_controller_interface` respectively.
 5. In `package.xml` file in export tag use `<controller_interface plugin="${prefix}/controller_plugins.xml"/>` 
 6. In `controller_plugins.xml` file use `base_class_type="controller_interface::ControllerBase" />`
 
*launch files*

 1. Replace `pr2_ethercat` with `ros_ethercat_loop` in launch files
 2. Since pr2_controller_manager is no longer used it no longer publishes joint_states or mechanism_statistics. Joint states can be published with the join_state_controller in ros_controllers. Automatically starting this controller requires changes in launch files and possibly adding a yaml file.
 3. `calibrate.py` file from pr2_bringup must be replicated in another package and modified to use controller_manager (from ros_control). Launch files that execute `calibrate.py` should be modified with the correct package name.
 
*transmissions*

 Transmissions are still defined in urdf in the pr2 style and inherit from ros_ethercat_model::Transmission.
 
 **New features**
 1. ros_ethercat accepts a new argument --period which is the period of main ethercat loop in msec. If not given the default value is 1ms.
 2. There is a helper bash script called ros_grant. This will grant to the ros_ethercat_loop executable the ability to be ran from a normal (nonroot) user. E.g rosrun ros_ethercat_loop ros_grant
 
 
