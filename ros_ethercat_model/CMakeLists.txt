# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(ros_ethercat_model)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    roscpp
    hardware_interface
    pluginlib
    urdf
    kdl_parser
    sr_robot_msgs
    cmake_modules
    realtime_tools
    ros_ethercat_hardware
)

find_package(Boost REQUIRED COMPONENTS system thread)
find_package(TinyXML REQUIRED)

catkin_python_setup()

include_directories(include
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${TinyXML_INCLUDE_DIRS})

catkin_package(
DEPENDS
    tinyxml
    urdfdom
    Boost
CATKIN_DEPENDS
    roscpp
    hardware_interface
    pluginlib
    urdf
    kdl_parser
    sr_robot_msgs
    realtime_tools
    ros_ethercat_hardware
INCLUDE_DIRS
    include
CFG_EXTRAS
    ${PROJECT_NAME}-extras.cmake
)

add_library(${PROJECT_NAME} src/ros_ethercat.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(PROGRAMS scripts/${PROJECT_NAME}/calibrate
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
