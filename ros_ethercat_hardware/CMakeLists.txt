cmake_minimum_required(VERSION 2.8.3)
project(ros_ethercat_hardware)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    realtime_tools
    diagnostic_msgs
    diagnostic_updater
    pluginlib
    hardware_interface
    ros_ethercat_eml
)

include_directories(include
    ${catkin_INCLUDE_DIRS}
)

catkin_package(
    CATKIN_DEPENDS
        roscpp
        realtime_tools
        diagnostic_msgs
        diagnostic_updater
        pluginlib
        hardware_interface
        ros_ethercat_eml
    INCLUDE_DIRS include
    LIBRARIES ros_ethercat_hardware
    CFG_EXTRAS
        ${PROJECT_NAME}-extras.cmake
)

add_library(${PROJECT_NAME}
    src/ethercat_com.cpp
    src/ethercat_device.cpp
    src/ethercat_hardware.cpp
    src/ethernet_interface_info.cpp
)
add_dependencies(${PROJECT_NAME}
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

include(cmake/${PROJECT_NAME}-extras.cmake.em)
ros_enable_rpath(${PROJECT_NAME})


install(TARGETS ${PROJECT_NAME}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
