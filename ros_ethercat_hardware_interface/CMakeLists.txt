cmake_minimum_required(VERSION 2.8.3)
project(ros_ethercat_hardware_interface)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    geometry_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
CATKIN_DEPENDS
    roscpp
    geometry_msgs
INCLUDE_DIRS
    include
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
