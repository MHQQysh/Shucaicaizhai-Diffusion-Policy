# Install script for directory: /home/lsg/ros_ws/src/bmirobot_pkg/general-message-pkgs-master/path_navigation_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/lsg/ros_ws/install")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/path_navigation_msgs/action" TYPE FILE FILES
    "/home/lsg/ros_ws/src/bmirobot_pkg/general-message-pkgs-master/path_navigation_msgs/action/PathExecution.action"
    "/home/lsg/ros_ws/src/bmirobot_pkg/general-message-pkgs-master/path_navigation_msgs/action/TransformPathExecution.action"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/path_navigation_msgs/msg" TYPE FILE FILES
    "/home/lsg/ros_ws/devel/share/path_navigation_msgs/msg/PathExecutionAction.msg"
    "/home/lsg/ros_ws/devel/share/path_navigation_msgs/msg/PathExecutionActionGoal.msg"
    "/home/lsg/ros_ws/devel/share/path_navigation_msgs/msg/PathExecutionActionResult.msg"
    "/home/lsg/ros_ws/devel/share/path_navigation_msgs/msg/PathExecutionActionFeedback.msg"
    "/home/lsg/ros_ws/devel/share/path_navigation_msgs/msg/PathExecutionGoal.msg"
    "/home/lsg/ros_ws/devel/share/path_navigation_msgs/msg/PathExecutionResult.msg"
    "/home/lsg/ros_ws/devel/share/path_navigation_msgs/msg/PathExecutionFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/path_navigation_msgs/msg" TYPE FILE FILES
    "/home/lsg/ros_ws/devel/share/path_navigation_msgs/msg/TransformPathExecutionAction.msg"
    "/home/lsg/ros_ws/devel/share/path_navigation_msgs/msg/TransformPathExecutionActionGoal.msg"
    "/home/lsg/ros_ws/devel/share/path_navigation_msgs/msg/TransformPathExecutionActionResult.msg"
    "/home/lsg/ros_ws/devel/share/path_navigation_msgs/msg/TransformPathExecutionActionFeedback.msg"
    "/home/lsg/ros_ws/devel/share/path_navigation_msgs/msg/TransformPathExecutionGoal.msg"
    "/home/lsg/ros_ws/devel/share/path_navigation_msgs/msg/TransformPathExecutionResult.msg"
    "/home/lsg/ros_ws/devel/share/path_navigation_msgs/msg/TransformPathExecutionFeedback.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/path_navigation_msgs/cmake" TYPE FILE FILES "/home/lsg/ros_ws/build/bmirobot_pkg/general-message-pkgs-master/path_navigation_msgs/catkin_generated/installspace/path_navigation_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/lsg/ros_ws/devel/include/path_navigation_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/lsg/ros_ws/devel/share/roseus/ros/path_navigation_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/lsg/ros_ws/devel/share/common-lisp/ros/path_navigation_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/lsg/ros_ws/devel/share/gennodejs/ros/path_navigation_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/lsg/ros_ws/devel/lib/python3/dist-packages/path_navigation_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/lsg/ros_ws/devel/lib/python3/dist-packages/path_navigation_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/lsg/ros_ws/build/bmirobot_pkg/general-message-pkgs-master/path_navigation_msgs/catkin_generated/installspace/path_navigation_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/path_navigation_msgs/cmake" TYPE FILE FILES "/home/lsg/ros_ws/build/bmirobot_pkg/general-message-pkgs-master/path_navigation_msgs/catkin_generated/installspace/path_navigation_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/path_navigation_msgs/cmake" TYPE FILE FILES
    "/home/lsg/ros_ws/build/bmirobot_pkg/general-message-pkgs-master/path_navigation_msgs/catkin_generated/installspace/path_navigation_msgsConfig.cmake"
    "/home/lsg/ros_ws/build/bmirobot_pkg/general-message-pkgs-master/path_navigation_msgs/catkin_generated/installspace/path_navigation_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/path_navigation_msgs" TYPE FILE FILES "/home/lsg/ros_ws/src/bmirobot_pkg/general-message-pkgs-master/path_navigation_msgs/package.xml")
endif()

