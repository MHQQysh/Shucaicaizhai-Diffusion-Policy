# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "bmirobot_msg: 8 messages, 0 services")

set(MSG_I_FLAGS "-Ibmirobot_msg:/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(bmirobot_msg_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_jointfd.msg" NAME_WE)
add_custom_target(_bmirobot_msg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bmirobot_msg" "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_jointfd.msg" ""
)

get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_ctr.msg" NAME_WE)
add_custom_target(_bmirobot_msg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bmirobot_msg" "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_ctr.msg" ""
)

get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_fdctr.msg" NAME_WE)
add_custom_target(_bmirobot_msg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bmirobot_msg" "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_fdctr.msg" ""
)

get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_fdstatus.msg" NAME_WE)
add_custom_target(_bmirobot_msg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bmirobot_msg" "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_fdstatus.msg" ""
)

get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_mpu.msg" NAME_WE)
add_custom_target(_bmirobot_msg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bmirobot_msg" "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_mpu.msg" ""
)

get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_distance.msg" NAME_WE)
add_custom_target(_bmirobot_msg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bmirobot_msg" "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_distance.msg" ""
)

get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/motorv2_fd.msg" NAME_WE)
add_custom_target(_bmirobot_msg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bmirobot_msg" "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/motorv2_fd.msg" ""
)

get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/motorv2_ctr.msg" NAME_WE)
add_custom_target(_bmirobot_msg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "bmirobot_msg" "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/motorv2_ctr.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_jointfd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bmirobot_msg
)
_generate_msg_cpp(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_ctr.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bmirobot_msg
)
_generate_msg_cpp(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_fdctr.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bmirobot_msg
)
_generate_msg_cpp(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_fdstatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bmirobot_msg
)
_generate_msg_cpp(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_mpu.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bmirobot_msg
)
_generate_msg_cpp(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_distance.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bmirobot_msg
)
_generate_msg_cpp(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/motorv2_fd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bmirobot_msg
)
_generate_msg_cpp(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/motorv2_ctr.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bmirobot_msg
)

### Generating Services

### Generating Module File
_generate_module_cpp(bmirobot_msg
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bmirobot_msg
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(bmirobot_msg_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(bmirobot_msg_generate_messages bmirobot_msg_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_jointfd.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_cpp _bmirobot_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_ctr.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_cpp _bmirobot_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_fdctr.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_cpp _bmirobot_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_fdstatus.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_cpp _bmirobot_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_mpu.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_cpp _bmirobot_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_distance.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_cpp _bmirobot_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/motorv2_fd.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_cpp _bmirobot_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/motorv2_ctr.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_cpp _bmirobot_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bmirobot_msg_gencpp)
add_dependencies(bmirobot_msg_gencpp bmirobot_msg_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bmirobot_msg_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_jointfd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bmirobot_msg
)
_generate_msg_eus(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_ctr.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bmirobot_msg
)
_generate_msg_eus(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_fdctr.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bmirobot_msg
)
_generate_msg_eus(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_fdstatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bmirobot_msg
)
_generate_msg_eus(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_mpu.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bmirobot_msg
)
_generate_msg_eus(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_distance.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bmirobot_msg
)
_generate_msg_eus(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/motorv2_fd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bmirobot_msg
)
_generate_msg_eus(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/motorv2_ctr.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bmirobot_msg
)

### Generating Services

### Generating Module File
_generate_module_eus(bmirobot_msg
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bmirobot_msg
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(bmirobot_msg_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(bmirobot_msg_generate_messages bmirobot_msg_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_jointfd.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_eus _bmirobot_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_ctr.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_eus _bmirobot_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_fdctr.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_eus _bmirobot_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_fdstatus.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_eus _bmirobot_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_mpu.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_eus _bmirobot_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_distance.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_eus _bmirobot_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/motorv2_fd.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_eus _bmirobot_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/motorv2_ctr.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_eus _bmirobot_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bmirobot_msg_geneus)
add_dependencies(bmirobot_msg_geneus bmirobot_msg_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bmirobot_msg_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_jointfd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bmirobot_msg
)
_generate_msg_lisp(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_ctr.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bmirobot_msg
)
_generate_msg_lisp(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_fdctr.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bmirobot_msg
)
_generate_msg_lisp(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_fdstatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bmirobot_msg
)
_generate_msg_lisp(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_mpu.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bmirobot_msg
)
_generate_msg_lisp(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_distance.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bmirobot_msg
)
_generate_msg_lisp(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/motorv2_fd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bmirobot_msg
)
_generate_msg_lisp(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/motorv2_ctr.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bmirobot_msg
)

### Generating Services

### Generating Module File
_generate_module_lisp(bmirobot_msg
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bmirobot_msg
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(bmirobot_msg_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(bmirobot_msg_generate_messages bmirobot_msg_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_jointfd.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_lisp _bmirobot_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_ctr.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_lisp _bmirobot_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_fdctr.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_lisp _bmirobot_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_fdstatus.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_lisp _bmirobot_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_mpu.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_lisp _bmirobot_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_distance.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_lisp _bmirobot_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/motorv2_fd.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_lisp _bmirobot_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/motorv2_ctr.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_lisp _bmirobot_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bmirobot_msg_genlisp)
add_dependencies(bmirobot_msg_genlisp bmirobot_msg_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bmirobot_msg_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_jointfd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bmirobot_msg
)
_generate_msg_nodejs(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_ctr.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bmirobot_msg
)
_generate_msg_nodejs(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_fdctr.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bmirobot_msg
)
_generate_msg_nodejs(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_fdstatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bmirobot_msg
)
_generate_msg_nodejs(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_mpu.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bmirobot_msg
)
_generate_msg_nodejs(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_distance.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bmirobot_msg
)
_generate_msg_nodejs(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/motorv2_fd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bmirobot_msg
)
_generate_msg_nodejs(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/motorv2_ctr.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bmirobot_msg
)

### Generating Services

### Generating Module File
_generate_module_nodejs(bmirobot_msg
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bmirobot_msg
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(bmirobot_msg_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(bmirobot_msg_generate_messages bmirobot_msg_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_jointfd.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_nodejs _bmirobot_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_ctr.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_nodejs _bmirobot_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_fdctr.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_nodejs _bmirobot_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_fdstatus.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_nodejs _bmirobot_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_mpu.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_nodejs _bmirobot_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_distance.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_nodejs _bmirobot_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/motorv2_fd.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_nodejs _bmirobot_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/motorv2_ctr.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_nodejs _bmirobot_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bmirobot_msg_gennodejs)
add_dependencies(bmirobot_msg_gennodejs bmirobot_msg_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bmirobot_msg_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_jointfd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bmirobot_msg
)
_generate_msg_py(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_ctr.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bmirobot_msg
)
_generate_msg_py(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_fdctr.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bmirobot_msg
)
_generate_msg_py(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_fdstatus.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bmirobot_msg
)
_generate_msg_py(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_mpu.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bmirobot_msg
)
_generate_msg_py(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_distance.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bmirobot_msg
)
_generate_msg_py(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/motorv2_fd.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bmirobot_msg
)
_generate_msg_py(bmirobot_msg
  "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/motorv2_ctr.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bmirobot_msg
)

### Generating Services

### Generating Module File
_generate_module_py(bmirobot_msg
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bmirobot_msg
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(bmirobot_msg_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(bmirobot_msg_generate_messages bmirobot_msg_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_jointfd.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_py _bmirobot_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_ctr.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_py _bmirobot_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_fdctr.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_py _bmirobot_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_fdstatus.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_py _bmirobot_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_mpu.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_py _bmirobot_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/Robot_distance.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_py _bmirobot_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/motorv2_fd.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_py _bmirobot_msg_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jok/ros_ws/src/bmirobot_pkg/bmirobot_msg/msg/motorv2_ctr.msg" NAME_WE)
add_dependencies(bmirobot_msg_generate_messages_py _bmirobot_msg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(bmirobot_msg_genpy)
add_dependencies(bmirobot_msg_genpy bmirobot_msg_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS bmirobot_msg_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bmirobot_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/bmirobot_msg
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(bmirobot_msg_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bmirobot_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/bmirobot_msg
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(bmirobot_msg_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bmirobot_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/bmirobot_msg
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(bmirobot_msg_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bmirobot_msg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/bmirobot_msg
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(bmirobot_msg_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bmirobot_msg)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bmirobot_msg\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/bmirobot_msg
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(bmirobot_msg_generate_messages_py std_msgs_generate_messages_py)
endif()
