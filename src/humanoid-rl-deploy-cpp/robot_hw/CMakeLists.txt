cmake_minimum_required(VERSION 3.5)

project(robot_hw)

find_package(ament_cmake REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_INSTALL_PREFIX}/include
)

link_directories(
  ${CMAKE_INSTALL_PREFIX}/lib
)

add_executable(robot_hw main.cpp)

target_link_libraries(robot_hw limxsdk_lowlevel yaml-cpp pthread dl)

install(TARGETS robot_hw DESTINATION bin)
install(FILES robot_controllers.yaml DESTINATION etc/robot_hw)

ament_package()
