# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(ros_ethercat)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
    roscpp
    std_msgs
    controller_manager 
    ethercat_hardware
    realtime_tools
    diagnostic_updater
    diagnostic_msgs
    sensor_msgs
    pluginlib
    geometry_msgs
    urdf
    kdl_parser
    angles
)

find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package(
DEPENDS
    tinyxml
    urdfdom
CATKIN_DEPENDS
    roscpp
    std_msgs
    controller_manager 
    ethercat_hardware 
    realtime_tools
    diagnostic_updater
    diagnostic_msgs
    sensor_msgs
    pluginlib
    geometry_msgs
    urdf
    kdl_parser
    angles
INCLUDE_DIRS
    include
CFG_EXTRAS
    ros_ethercat-extras.cmake.em
LIBRARIES
    ros_ethercat_mechanism_model
)

add_definitions(-O3)

add_library(ros_ethercat_mechanism_model
    src/joint.cpp
    src/robot.cpp
    src/chain.cpp
    src/tree.cpp
    src/joint_calibration_simulator.cpp
)
pr2_enable_rpath(ros_ethercat_mechanism_model)
target_link_libraries(ros_ethercat_mechanism_model ${catkin_LIBRARIES} )
add_dependencies(ros_ethercat_mechanism_model ${catkin_EXPORTED_TARGETS})

add_executable(ros_ethercat
   src/main.cpp
   src/ros_ethercat.cpp)
target_link_libraries(ros_ethercat ros_ethercat_mechanism_model rt ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(ros_ethercat ${catkin_EXPORTED_TARGETS})
pr2_enable_rpath(ros_ethercat)

install(DIRECTORY include/ros_ethercat/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ros_ethercat ros_ethercat_mechanism_model
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

file(GLOB PYTHON_SCRIPTS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" 
   "${CMAKE_CURRENT_SOURCE_DIR}/scripts/*")

install(PROGRAMS ${PYTHON_SCRIPTS}
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
   
install(FILES transmission_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
