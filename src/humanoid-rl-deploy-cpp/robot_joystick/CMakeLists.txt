cmake_minimum_required(VERSION 3.5)

project(robot_joystick)

find_package(ament_cmake REQUIRED)

include_directories(
  include
  ${CMAKE_INSTALL_PREFIX}/include
)

link_directories(
  ${CMAKE_INSTALL_PREFIX}/lib
)

add_library(robot_joystick SHARED 
  src/RobotJoystick.cpp
)

target_link_libraries(robot_joystick 
  limxsdk_lowlevel
  yaml-cpp 
  pthread
  dl
)

install(TARGETS robot_joystick DESTINATION lib)

ament_package()
