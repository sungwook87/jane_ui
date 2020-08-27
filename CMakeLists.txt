cmake_minimum_required(VERSION 2.8.3)
project(jane_ui)
set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    message_generation
    roscpp
    rospy
    nav_msgs
    sensor_msgs
    std_msgs
    cv_bridge
    diagnostic_updater
    std_srvs
)

catkin_python_setup()

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
        JaneGimbalOrientation.msg
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
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs message_runtime std_msgs
  DEPENDS system-lib
)


#########################
##         GUI         ##
#########################
#qt5_add_resources(QT_RESOURCES_CPP qt_resources/arrows.qrc)
qt5_wrap_cpp(QT_MOC_HPP src/UIwindow.h src/rosthread.h)
set(gauge_source
    src/qcgaugewidget.cpp
    src/qcgaugewidget.h
)

include_directories(${Qt5Widgets_INCLUDE_DIRS})

add_executable(jane_ui_node src/UIwindow.cpp src/rosthread.cpp src/main.cpp
                                  ${QT_RESOURCES_CPP} ${QT_MOC_HPP})
add_dependencies(jane_ui_node jane_ui_generate_messages_cpp)
target_link_libraries(jane_ui_node ${QT_LIBRARIES} ${catkin_LIBRARIES} Qt5::Widgets )
install(TARGETS jane_ui_node  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


#########################
##   2D visualization  ##
#########################
set(geo_source
    jane_2dviz/GeoCoordiConv/GeoCoordConv.cpp
    jane_2dviz/GeoCoordiConv/GeoCoordConv.h
    )
set(2dviz_source
    jane_2dviz/src/2dviz.cpp
    jane_2dviz/src/2dviz.h
)

add_executable(jane_2dviz jane_2dviz/jane_2dviz.cpp ${2dviz_source} ${geo_source})
target_link_libraries(jane_2dviz ${catkin_LIBRARIES} ${CURL_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(jane_2dviz jane_ui_generate_messages_cpp)

#########################
##       Control       ##
#########################
set(control_source
    jane_control/src/fncs_header.hpp
    jane_control/src/srv_fncs.cpp
    jane_control/src/sub_fncs.cpp
    jane_control/src/cont_fncs.cpp
    include/UTM_lib/UTM.cpp
    include/UTM_lib/UTM.h
)
add_executable(jane_control jane_control/jane_control.cpp ${control_source})
target_link_libraries(jane_control
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
add_dependencies(jane_control jane_ui_generate_messages_cpp)

#########################
##      WP reader      ##
#########################
add_executable(jane_waypoint_reader jane_waypoint_reader/src/wp_reader_integration.cpp)
add_dependencies(jane_waypoint_reader jane_ui_generate_messages_cpp)
target_link_libraries(jane_waypoint_reader
                ${catkin_LIBRARIES}
                )

#########################
##       Gimbal        ##
#########################
catkin_install_python(
  PROGRAMS jane_gimbal/storm32_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


#########################
##       Save Log      ##
#########################
add_executable(jane_savelog src/jane_savelog.cpp)
add_dependencies(jane_savelog jane_ui_generate_messages_cpp)
target_link_libraries(jane_savelog
                ${catkin_LIBRARIES}
                )

