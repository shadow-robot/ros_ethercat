# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(ros_ethercat)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
   roscpp 
   std_srvs 
   std_msgs
   controller_manager 
   ethercat_hardware 
   realtime_tools 
   diagnostic_updater
   diagnostic_msgs
   pr2_hardware_interface
   pr2_mechanism_model
   pr2_mechanism_diagnostics
   sensor_msgs
   pluginlib
)

find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package(
CATKIN_DEPENDS
   roscpp 
   std_srvs 
   std_msgs
   controller_manager 
   ethercat_hardware 
   realtime_tools 
   diagnostic_updater
   pr2_hardware_interface
   pr2_mechanism_model
   pr2_mechanism_diagnostics
   diagnostic_msgs
   sensor_msgs
   pluginlib
INCLUDE_DIRS
   include
CFG_EXTRAS
   ros_ethercat-extras.cmake.em
)

add_definitions(-O3)

add_executable(ros_ethercat src/main.cpp src/ros_ethercat.cpp)
target_link_libraries(ros_ethercat rt ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(ros_ethercat ${catkin_EXPORTED_TARGETS})
pr2_enable_rpath(ros_ethercat)

install(TARGETS ros_ethercat
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

file(GLOB PYTHON_SCRIPTS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" 
   "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*")
install(PROGRAMS ${PYTHON_SCRIPTS}
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
