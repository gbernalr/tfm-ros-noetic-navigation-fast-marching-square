cmake_minimum_required(VERSION 3.0.2)
project(rgbd_sim)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  gazebo_msgs
  gazebo_ros
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/move_person.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch worlds models config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
