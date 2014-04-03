# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(ros_ethercat_mechanism_model)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
    roscpp
    ros_ethercat_hardware
    ros_ethercat_hardware_interface
    pluginlib
    urdf
    kdl_parser
    angles
    hardware_interface
)

find_package(Boost REQUIRED COMPONENTS system thread)
include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package(
DEPENDS
    tinyxml
    urdfdom
CATKIN_DEPENDS
    roscpp
    ros_ethercat_hardware
    ros_ethercat_hardware_interface
    pluginlib
    urdf
    kdl_parser
    angles
    hardware_interface
INCLUDE_DIRS
    include
LIBRARIES
    ${PROJECT_NAME}
)

add_definitions(-O3)

add_library(${PROJECT_NAME}
    src/chain.cpp
    src/joint_calibration_simulator.cpp
    src/joint.cpp
    src/robot.cpp
    src/tree.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} )
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
