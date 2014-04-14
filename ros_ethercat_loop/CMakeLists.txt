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
    ros_ethercat_hardware
    ros_ethercat_model
    realtime_tools
    diagnostic_updater
    diagnostic_msgs
    pluginlib
)

add_definitions(-O3)

add_executable(${PROJECT_NAME}
   src/main.cpp)
target_link_libraries(${PROJECT_NAME} rt ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY scripts/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
