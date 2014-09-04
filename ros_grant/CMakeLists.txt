cmake_minimum_required(VERSION 2.8.3)
project(ros_grant)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_grant
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(ros_grant src/ros_grant.cpp)
target_link_libraries(ros_grant cap)

install(TARGETS ros_grant
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
