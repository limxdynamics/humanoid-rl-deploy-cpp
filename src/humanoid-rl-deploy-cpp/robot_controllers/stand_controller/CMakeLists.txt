cmake_minimum_required(VERSION 3.5)

project(stand_controller)

find_package(ament_cmake REQUIRED)

find_package(Eigen 3 QUIET)
if(NOT Eigen_FOUND)
  # Fallback to cmake_modules
  include(../cmake/FindEigen3.cmake)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_INSTALL_PREFIX}/include
)

link_directories(
  ${CMAKE_INSTALL_PREFIX}/lib
)

add_library(stand_controller SHARED 
  src/StandController.cpp
)

target_link_libraries(stand_controller 
  limxsdk_lowlevel
  yaml-cpp 
  pthread
  dl
)

install(TARGETS stand_controller DESTINATION lib)

install(DIRECTORY config/. DESTINATION etc/stand_controller)

ament_package()
