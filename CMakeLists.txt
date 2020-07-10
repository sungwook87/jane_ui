cmake_minimum_required(VERSION 2.8.3)
project(jane_ui)
set (CMAKE_CXX_STANDARD 11)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    message_generation
    roscpp
    rospy
    nav_msgs
    sensor_msgs
    std_msgs
    cv_bridge
)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


add_service_files(
  FILES
  start_trig.srv
  take_off.srv
  return_home.srv
  request_mode_info.srv
  landing.srv
  next_mission.srv
  previous_mission.srv
  pause_mission.srv
  emergency_landing.srv
  send_mission_info.srv
  request_gps_mission.srv
  record_start.srv
  record_stop.srv
  Srv_key.srv
  move_control.srv
  rotate_control.srv
  linenumin_waypointout.srv
  automatic_mission_start.srv
  ui_mission_request.srv
  manual_control.srv
  test_srv.srv
)

add_message_files(
        FILES
        estimated_gps.msg
        log_data.msg
        mission_struct.msg
        rpyThrust.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV)

include(FindCURL)
if(NOT CURL_FOUND)
  message("CURL not found!  Aborting...")
  fail()
endif(NOT CURL_FOUND)
include_directories(${CURL_INCLUDE_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs
  DEPENDS system-lib
)


qt5_add_resources(QT_RESOURCES_CPP qt_resources/arrows.qrc)
qt5_wrap_cpp(QT_MOC_HPP src/UIwindow.h src/rosthread.h)

include_directories(${Qt5Widgets_INCLUDE_DIRS})

add_executable(jane_ui_node src/UIwindow.cpp src/rosthread.cpp src/main.cpp
                                  ${QT_RESOURCES_CPP} ${QT_MOC_HPP})
add_dependencies(jane_ui_node jane_ui_generate_messages_cpp)
target_link_libraries(jane_ui_node ${QT_LIBRARIES} ${catkin_LIBRARIES} Qt5::Widgets)
install(TARGETS jane_ui_node  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
