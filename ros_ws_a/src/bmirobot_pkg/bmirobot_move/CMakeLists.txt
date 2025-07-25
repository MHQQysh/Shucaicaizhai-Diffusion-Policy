cmake_minimum_required(VERSION 3.3.2)
project(bmirobot_move)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  eigen_conversions
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_core
  moveit_visual_tools
  tf
  tf_conversions
  trajectory_msgs
  message_generation
  geometry_msgs
  actionlib
  actionlib_msgs
  moveit_msgs
  cmake_modules
  sensor_msgs
  #pcl_ros
  bmirobot_msg
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED thread system)
#find_package(PCL REQUIRED)
SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++11" )
## Generate services in the 'srv' folder
add_service_files(
   FILES
   GraspServo.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

# Catkin
catkin_package(
    LIBRARIES
    ${PROJECT_NAME}
    ${PROJECT_NAME}_filter
    CATKIN_DEPENDS
    actionlib_msgs
    geometry_msgs
    moveit_msgs
    trajectory_msgs
    std_msgs
    message_runtime
    moveit_visual_tools
    object_recognition_msgs
    INCLUDE_DIRS include
)

## Build
include_directories(
    ${catkin_INCLUDE_DIRS}
)

#include_directories(include${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})
# Test filter executable

#add_executable(bmirobotv5_moveit       src/bmirobot_moveit.cpp)
#target_link_libraries(bmirobotv5_moveit
#   ${catkin_LIBRARIES} ${Boost_LIBRARIES}
#)


add_executable(bmirobotv5_Polhemus_control src/Polhemus_control.cpp)
target_link_libraries(bmirobotv5_Polhemus_control
   ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

#add_executable(bmirobotv5_Polhemus_control_left src/Polhemus_control_left.cpp)
#target_link_libraries(bmirobotv5_Polhemus_control_left
#   ${catkin_LIBRARIES} ${Boost_LIBRARIES}
#)

add_executable(bmirobotv5_IK_control src/Polhemus_IK_control.cpp)
target_link_libraries(bmirobotv5_IK_control
   ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

#add_executable(bmirobotv5_IK_one      src/Polhemus_IK_one.cpp)
#target_link_libraries(bmirobotv5_IK_one
#   ${catkin_LIBRARIES} ${Boost_LIBRARIES}
#)

add_executable(bmirobotv5_IK_motion       src/bmirobot_IK_motion.cpp)
target_link_libraries(bmirobotv5_IK_motion
   ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

#add_executable(bmirobotv5_IK_key       src/bmirobot_IK_key.cpp)
#target_link_libraries(bmirobotv5_IK_key
#   ${catkin_LIBRARIES} ${Boost_LIBRARIES}
#)




## Install

# Install executables

