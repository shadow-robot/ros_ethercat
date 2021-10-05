# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 3.0.2)
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
find_package(TinyXML2 REQUIRED)

set( TinyXML2_INCLUDE_DIRS ${TinyXML2_INCLUDE_DIR} )
set( TinyXML2_LIBRARIES ${TinyXML2_LIBRARY} )

include_directories(include
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${TinyXML2_INCLUDE_DIRS})

catkin_package(
DEPENDS
    TinyXML2
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
LIBRARIES
    ${PROJECT_NAME}
CFG_EXTRAS
    ${PROJECT_NAME}-extras.cmake
)

add_library(${PROJECT_NAME} src/ros_ethercat.cpp)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${TinyXML2_LIBRARIES})


install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(FILES ethercat_robot_hw_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
