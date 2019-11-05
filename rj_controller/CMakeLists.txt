cmake_minimum_required(VERSION 2.8.3)
project(rj_controller)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
nav_msgs
  genmsg
  bolt_msgs
  dynamic_reconfigure
)
find_package(cmake_modules REQUIRED)

generate_dynamic_reconfigure_options(
cfg/long_gains.cfg
)

#C++ 11 support
#if(CMAKE_COMPILER_IS_GNUCXX)
#  add_definitions(-std=gnu++0x)
#endif()

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

#file(GLOB_RECURSE SOURCES
#    include/*.h
#    src/*.cpp
#)

#MESSAGE("${SOURCES}")


add_executable(rj_velocity_node src/velocity_node.cpp src/velocity.cpp)
target_link_libraries(rj_velocity_node ${catkin_LIBRARIES})
add_dependencies(rj_velocity_node ${PROJECT_NAME}_gencfg)

#add_executable(rj_controller_node ${SOURCES})
add_executable(rj_controller_node src/control.cpp src/CombinedROSNode.cpp src/CarConstants.cpp src/LateralFeedback.cpp src/LateralFeedforward.cpp src/Longitudinal.cpp src/Filter.cpp)
target_link_libraries(rj_controller_node ${catkin_LIBRARIES})
add_dependencies(rj_controller_node ${PROJECT_NAME}_gencfg)


