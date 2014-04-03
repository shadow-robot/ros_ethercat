cmake_minimum_required(VERSION 2.8.3)
project(ros_ethercat_loop)

function(ros_enable_rpath target)
   set_target_properties(${target} PROPERTIES SKIP_BUILD_RPATH FALSE)
   set(RPATH "${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}")
   if(DEFINED ENV{LD_LIBRARY_PATH})
      set(RPATH "${RPATH}:$ENV{LD_LIBRARY_PATH}")
   endif()
   message("Install RPATH for ${target} is ${RPATH}")
   set_target_properties(${target} PROPERTIES INSTALL_RPATH ${RPATH})
   set_target_properties(${target} PROPERTIES BUILD_WITH_INSTALL_RPATH FALSE)
endfunction()

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
    roscpp
    std_msgs
    controller_manager 
    ros_ethercat_hardware
    ros_ethercat_mechanism_model
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
    ros_ethercat_mechanism_model
    realtime_tools
    diagnostic_updater
    diagnostic_msgs
    pluginlib
INCLUDE_DIRS
    include
)

add_definitions(-O3)

add_executable(${PROJECT_NAME}
   src/main.cpp
   src/ros_ethercat.cpp)
target_link_libraries(${PROJECT_NAME} rt ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
ros_enable_rpath(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
