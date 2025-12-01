# Install script for directory: /home/gbernalr/tfm/intro_ws/src/turtlebot3_fm2_nav

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/gbernalr/tfm/intro_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/gbernalr/tfm/intro_ws/build/turtlebot3_fm2_nav/catkin_generated/installspace/turtlebot3_fm2_nav.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot3_fm2_nav/cmake" TYPE FILE FILES
    "/home/gbernalr/tfm/intro_ws/build/turtlebot3_fm2_nav/catkin_generated/installspace/turtlebot3_fm2_navConfig.cmake"
    "/home/gbernalr/tfm/intro_ws/build/turtlebot3_fm2_nav/catkin_generated/installspace/turtlebot3_fm2_navConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot3_fm2_nav" TYPE FILE FILES "/home/gbernalr/tfm/intro_ws/src/turtlebot3_fm2_nav/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/gbernalr/tfm/intro_ws/build/turtlebot3_fm2_nav/catkin_generated/installspace/turtlebot3_fm2_nav.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot3_fm2_nav/cmake" TYPE FILE FILES
    "/home/gbernalr/tfm/intro_ws/build/turtlebot3_fm2_nav/catkin_generated/installspace/turtlebot3_fm2_navConfig.cmake"
    "/home/gbernalr/tfm/intro_ws/build/turtlebot3_fm2_nav/catkin_generated/installspace/turtlebot3_fm2_navConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/turtlebot3_fm2_nav" TYPE FILE FILES "/home/gbernalr/tfm/intro_ws/src/turtlebot3_fm2_nav/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/turtlebot3_fm2_nav" TYPE PROGRAM FILES "/home/gbernalr/tfm/intro_ws/build/turtlebot3_fm2_nav/catkin_generated/installspace/fm2_nav_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/turtlebot3_fm2_nav" TYPE PROGRAM FILES "/home/gbernalr/tfm/intro_ws/build/turtlebot3_fm2_nav/catkin_generated/installspace/fm2_costmap_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/turtlebot3_fm2_nav" TYPE PROGRAM FILES "/home/gbernalr/tfm/intro_ws/build/turtlebot3_fm2_nav/catkin_generated/installspace/simple_nav_node.py")
endif()

