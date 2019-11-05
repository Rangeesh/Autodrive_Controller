cmake_minimum_required(VERSION 2.8.3)
project(bolt)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nav_msgs
  std_msgs
  geometry_msgs
  message_generation
)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bolt_2dnav
#  CATKIN_DEPENDS move_base my_odom_configuration_dep my_sensor_configuration_dep my_tf_configuration_dep
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
#add_executable(points_planner src/waypointstopoints.cpp)
#target_link_libraries(points_planner src/waypointstopoints.cpp)
#add_dependencies(points_planner src/waypointstopoints.cpp)


#add_executable(rj_straight src/straight_point_cpp.cpp)
#target_link_libraries(rj_straight ${catkin_LIBRARIES})
#add_dependencies(rj_straight ${PROJECT_NAME}_gencfg)

#add_executable(testing src/testing_cpp.cpp)
#target_link_libraries(testing ${catkin_LIBRARIES})
#add_dependencies(testing ${PROJECT_NAME}_gencfg)
