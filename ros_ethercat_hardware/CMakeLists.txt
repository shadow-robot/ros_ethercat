cmake_minimum_required(VERSION 2.8.3)
project(ros_ethercat_hardware)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    roscpp
    realtime_tools
    diagnostic_msgs
    diagnostic_updater
    pluginlib
    ros_ethercat_model
)

find_package(EML REQUIRED)

include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${EML_INCLUDE_DIRS}
)

add_definitions(-Wall)

catkin_package(
    DEPENDS EML
    CATKIN_DEPENDS
        roscpp
        realtime_tools
        diagnostic_msgs
        diagnostic_updater
        pluginlib
        ros_ethercat_model
    INCLUDE_DIRS include
    LIBRARIES ros_ethercat_hardware
)

add_definitions(-O3)

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


install(TARGETS ${PROJECT_NAME}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

