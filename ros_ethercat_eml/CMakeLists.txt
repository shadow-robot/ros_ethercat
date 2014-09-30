cmake_minimum_required(VERSION 2.8.3)
project(ros_ethercat_eml)

find_package(catkin REQUIRED roscpp)

include_directories(include
    ${catkin_INCLUDE_DIRS}
)

catkin_package(
    CATKIN_DEPENDS roscpp
    INCLUDE_DIRS include
    LIBRARIES ros_ethercat_eml
)

add_library(${PROJECT_NAME}
    src/ethercat_AL.cxx
    src/ethercat_FSM.cxx
    src/ethercat_master.cxx
    src/ethercat_mbx.cxx
    src/ethercat_process_data.cxx
    src/ethercat_router.cxx
    src/ethercat_slave_conf.cxx
    src/ethercat_slave_handler.cxx
    src/ethercat_device_addressed_telegram.cxx
    src/ethercat_dll.cxx
    src/ethercat_frame.cxx
    src/ethercat_logical_addressed_telegram.cxx
    src/ethercat_slave_memory.cxx
    src/ethercat_telegram.cxx
    src/ethercat_posix_drv.cxx
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
