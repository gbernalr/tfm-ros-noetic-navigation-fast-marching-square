cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_fm2_nav)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tf2_ros
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES turtlebot3_fm2_nav
  CATKIN_DEPENDS geometry_msgs nav_msgs rospy sensor_msgs std_msgs tf2_ros
  # DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Instalar scripts Python como nodos ROS
catkin_install_python(
  PROGRAMS
    src/fm2_nav_node.py
    src/fm2_costmap_node.py
    src/simple_nav_node.py
    src/fm2_planner_node.py
    src/fm2_controller_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Ejemplos de tests (comentados)
# catkin_add_gtest(${PROJECT_NAME}-test test/test_turtlebot3_fm2_nav.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

# catkin_add_nosetests(test)
