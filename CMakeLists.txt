# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(sr_ethercat)

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
   pr2_controller_interface
   pr2_mechanism_msgs
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
   pr2_controller_interface
   pr2_mechanism_msgs
   diagnostic_msgs
   sensor_msgs
   pluginlib
INCLUDE_DIRS
   include
CFG_EXTRAS
   sr_ethercat-extras.cmake.em
)

add_definitions(-O3)

add_executable(sr_ethercat src/main.cpp src/sr_ethercat.cpp)
target_link_libraries(sr_ethercat rt ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(sr_ethercat ${catkin_EXPORTED_TARGETS})
pr2_enable_rpath(sr_ethercat)

install(TARGETS sr_ethercat
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

file(GLOB PYTHON_SCRIPTS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" 
   "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*")
install(PROGRAMS ${PYTHON_SCRIPTS}
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
