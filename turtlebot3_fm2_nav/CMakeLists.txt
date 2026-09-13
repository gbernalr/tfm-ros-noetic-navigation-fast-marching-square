cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_fm2_nav)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rgbd_person_tracker
  rospy
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
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
  CATKIN_DEPENDS geometry_msgs nav_msgs rgbd_person_tracker rospy sensor_msgs std_msgs tf2_geometry_msgs tf2_ros
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
    src/fm2_costmap_node.py
    src/simple_nav_node.py
    src/fm2_planner_node.py
    src/fm2_controller_node.py
    src/person_patrol_mover.py
    src/person_track_publisher.py
    src/zed_body_tracking_sim_node.py
    src/initial_pose_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# These are importable modules, not executable ROS nodes. Catkin's Python
# relay scripts cannot be imported, so copy the source modules verbatim to the
# devel space and install them alongside the nodes.
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})
foreach(module nav_validation navigation_utils)
  configure_file(
    src/${module}.py
    ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/${module}.py
    COPYONLY
  )
endforeach()
install(
  FILES src/nav_validation.py src/navigation_utils.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_nav_validation.py)
  catkin_add_nosetests(test/test_planner_geometry.py)
  add_rostest(test/controller_watchdog.test)
endif()
