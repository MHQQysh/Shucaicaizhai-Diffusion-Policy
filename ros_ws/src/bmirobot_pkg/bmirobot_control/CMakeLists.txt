cmake_minimum_required(VERSION 2.8.3)
project(bmirobot_control)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  realtime_tools 
  hardware_interface 
  controller_interface  
  control_toolbox
  eigen_conversions
  kdl_parser
  bmirobot_hw
)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
add_library(bmirobot_control src/IK_controller.cpp )

catkin_package(
    DEPENDS 
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS include
    LIBRARIES # TODO
    realtime_tools  
    hardware_interface 
    controller_interface  
    control_toolbox 
    kdl_parser
    bmirobot_hw
    eigen_conversions
)




