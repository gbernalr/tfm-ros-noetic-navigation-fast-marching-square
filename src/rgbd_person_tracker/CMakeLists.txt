cmake_minimum_required(VERSION 3.0.2)
project(rgbd_person_tracker)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  cv_bridge
  message_filters
  tf2_ros
  tf2_geometry_msgs
  image_transport
)

catkin_python_setup()

catkin_package()

catkin_install_python(PROGRAMS
  scripts/rgbd_person_tracker_node.py
  scripts/generate_world.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
