cmake_minimum_required(VERSION 2.8.3)
project(ros_ethercat_loop)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    controller_manager
    ros_ethercat_hardware
    ros_ethercat_model
    realtime_tools
    diagnostic_updater
    diagnostic_msgs
    pluginlib
    rosconsole_bridge
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
DEPENDS
CATKIN_DEPENDS
    roscpp
    std_msgs
    controller_manager
    ros_ethercat_hardware
    ros_ethercat_model
    realtime_tools
    diagnostic_updater
    diagnostic_msgs
    pluginlib
    rosconsole_bridge
)

# needed to circumvent LD_LIBRARY_PATH being emptied through ethercat_grant
# in addition to not propagating ros_ethercat_loop RUNPATH to dependencies, in contrast to RPATH
SET(GCC_NEWDTAGS_LINK_FLAGS "-Wl,--disable-new-dtags")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${GCC_NEWDTAGS_LINK_FLAGS}")

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} rt ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
ros_enable_rpath(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
