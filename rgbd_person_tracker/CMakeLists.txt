cmake_minimum_required(VERSION 3.0.2)
project(rgbd_person_tracker)

find_package(catkin REQUIRED COMPONENTS
  message_generation
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

add_message_files(
  FILES
    PersonTrack.msg
    PersonTrackArray.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    nav_msgs
    rospy
    sensor_msgs
    std_msgs
    tf2_geometry_msgs
    tf2_ros
    visualization_msgs
)

catkin_install_python(PROGRAMS
  scripts/rgbd_person_tracker_node.py
  scripts/generate_world.py
  scripts/generate_multi_person_world.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
