# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "actionlib_msgs;geometry_msgs;moveit_msgs;trajectory_msgs;std_msgs;message_runtime;moveit_visual_tools;object_recognition_msgs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lbmirobot_move;-lbmirobot_move_filter".split(';') if "-lbmirobot_move;-lbmirobot_move_filter" != "" else []
PROJECT_NAME = "bmirobot_move"
PROJECT_SPACE_DIR = "/home/jok/ros_ws/install"
PROJECT_VERSION = "1.3.1"
