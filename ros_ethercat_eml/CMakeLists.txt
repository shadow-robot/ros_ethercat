cmake_minimum_required(VERSION 2.8.3)
project(ros_ethercat_eml)

find_package(catkin REQUIRED)

include_directories(include
    ${catkin_INCLUDE_DIRS}
)

catkin_package(
    CATKIN_DEPENDS
    INCLUDE_DIRS include
    LIBRARIES ros_ethercat_eml
)

add_library(${PROJECT_NAME}
    src/al/ethercat_AL.cxx
    src/al/ethercat_FSM.cxx
    src/al/ethercat_master.cxx
    src/al/ethercat_mbx.cxx
    src/al/ethercat_process_data.cxx
    src/al/ethercat_router.cxx
    src/al/ethercat_slave_conf.cxx
    src/al/ethercat_slave_handler.cxx
    src/dll/ethercat_device_addressed_telegram.cxx
    src/dll/ethercat_dll.cxx
    src/dll/ethercat_frame.cxx
    src/dll/ethercat_logical_addressed_telegram.cxx
    src/dll/ethercat_slave_memory.cxx
    src/dll/ethercat_telegram.cxx
    src/posix/ethercat_posix_drv.cxx
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME} 
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
