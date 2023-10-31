# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ds4_driver: 4 messages, 0 services")

set(MSG_I_FLAGS "-Ids4_driver:/home/rnd/moobot_ws/src/ds4_driver/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ds4_driver_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/rnd/moobot_ws/src/ds4_driver/msg/Feedback.msg" NAME_WE)
add_custom_target(_ds4_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ds4_driver" "/home/rnd/moobot_ws/src/ds4_driver/msg/Feedback.msg" ""
)

get_filename_component(_filename "/home/rnd/moobot_ws/src/ds4_driver/msg/Report.msg" NAME_WE)
add_custom_target(_ds4_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ds4_driver" "/home/rnd/moobot_ws/src/ds4_driver/msg/Report.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/rnd/moobot_ws/src/ds4_driver/msg/Status.msg" NAME_WE)
add_custom_target(_ds4_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ds4_driver" "/home/rnd/moobot_ws/src/ds4_driver/msg/Status.msg" "geometry_msgs/Vector3:std_msgs/Header:ds4_driver/Trackpad:geometry_msgs/Quaternion:sensor_msgs/Imu"
)

get_filename_component(_filename "/home/rnd/moobot_ws/src/ds4_driver/msg/Trackpad.msg" NAME_WE)
add_custom_target(_ds4_driver_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ds4_driver" "/home/rnd/moobot_ws/src/ds4_driver/msg/Trackpad.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(ds4_driver
  "/home/rnd/moobot_ws/src/ds4_driver/msg/Feedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ds4_driver
)
_generate_msg_cpp(ds4_driver
  "/home/rnd/moobot_ws/src/ds4_driver/msg/Report.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ds4_driver
)
_generate_msg_cpp(ds4_driver
  "/home/rnd/moobot_ws/src/ds4_driver/msg/Status.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/rnd/moobot_ws/src/ds4_driver/msg/Trackpad.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ds4_driver
)
_generate_msg_cpp(ds4_driver
  "/home/rnd/moobot_ws/src/ds4_driver/msg/Trackpad.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ds4_driver
)

### Generating Services

### Generating Module File
_generate_module_cpp(ds4_driver
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ds4_driver
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ds4_driver_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ds4_driver_generate_messages ds4_driver_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rnd/moobot_ws/src/ds4_driver/msg/Feedback.msg" NAME_WE)
add_dependencies(ds4_driver_generate_messages_cpp _ds4_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rnd/moobot_ws/src/ds4_driver/msg/Report.msg" NAME_WE)
add_dependencies(ds4_driver_generate_messages_cpp _ds4_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rnd/moobot_ws/src/ds4_driver/msg/Status.msg" NAME_WE)
add_dependencies(ds4_driver_generate_messages_cpp _ds4_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rnd/moobot_ws/src/ds4_driver/msg/Trackpad.msg" NAME_WE)
add_dependencies(ds4_driver_generate_messages_cpp _ds4_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ds4_driver_gencpp)
add_dependencies(ds4_driver_gencpp ds4_driver_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ds4_driver_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(ds4_driver
  "/home/rnd/moobot_ws/src/ds4_driver/msg/Feedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ds4_driver
)
_generate_msg_eus(ds4_driver
  "/home/rnd/moobot_ws/src/ds4_driver/msg/Report.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ds4_driver
)
_generate_msg_eus(ds4_driver
  "/home/rnd/moobot_ws/src/ds4_driver/msg/Status.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/rnd/moobot_ws/src/ds4_driver/msg/Trackpad.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ds4_driver
)
_generate_msg_eus(ds4_driver
  "/home/rnd/moobot_ws/src/ds4_driver/msg/Trackpad.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ds4_driver
)

### Generating Services

### Generating Module File
_generate_module_eus(ds4_driver
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ds4_driver
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(ds4_driver_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(ds4_driver_generate_messages ds4_driver_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rnd/moobot_ws/src/ds4_driver/msg/Feedback.msg" NAME_WE)
add_dependencies(ds4_driver_generate_messages_eus _ds4_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rnd/moobot_ws/src/ds4_driver/msg/Report.msg" NAME_WE)
add_dependencies(ds4_driver_generate_messages_eus _ds4_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rnd/moobot_ws/src/ds4_driver/msg/Status.msg" NAME_WE)
add_dependencies(ds4_driver_generate_messages_eus _ds4_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rnd/moobot_ws/src/ds4_driver/msg/Trackpad.msg" NAME_WE)
add_dependencies(ds4_driver_generate_messages_eus _ds4_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ds4_driver_geneus)
add_dependencies(ds4_driver_geneus ds4_driver_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ds4_driver_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(ds4_driver
  "/home/rnd/moobot_ws/src/ds4_driver/msg/Feedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ds4_driver
)
_generate_msg_lisp(ds4_driver
  "/home/rnd/moobot_ws/src/ds4_driver/msg/Report.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ds4_driver
)
_generate_msg_lisp(ds4_driver
  "/home/rnd/moobot_ws/src/ds4_driver/msg/Status.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/rnd/moobot_ws/src/ds4_driver/msg/Trackpad.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ds4_driver
)
_generate_msg_lisp(ds4_driver
  "/home/rnd/moobot_ws/src/ds4_driver/msg/Trackpad.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ds4_driver
)

### Generating Services

### Generating Module File
_generate_module_lisp(ds4_driver
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ds4_driver
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(ds4_driver_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(ds4_driver_generate_messages ds4_driver_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rnd/moobot_ws/src/ds4_driver/msg/Feedback.msg" NAME_WE)
add_dependencies(ds4_driver_generate_messages_lisp _ds4_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rnd/moobot_ws/src/ds4_driver/msg/Report.msg" NAME_WE)
add_dependencies(ds4_driver_generate_messages_lisp _ds4_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rnd/moobot_ws/src/ds4_driver/msg/Status.msg" NAME_WE)
add_dependencies(ds4_driver_generate_messages_lisp _ds4_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rnd/moobot_ws/src/ds4_driver/msg/Trackpad.msg" NAME_WE)
add_dependencies(ds4_driver_generate_messages_lisp _ds4_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ds4_driver_genlisp)
add_dependencies(ds4_driver_genlisp ds4_driver_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ds4_driver_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(ds4_driver
  "/home/rnd/moobot_ws/src/ds4_driver/msg/Feedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ds4_driver
)
_generate_msg_nodejs(ds4_driver
  "/home/rnd/moobot_ws/src/ds4_driver/msg/Report.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ds4_driver
)
_generate_msg_nodejs(ds4_driver
  "/home/rnd/moobot_ws/src/ds4_driver/msg/Status.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/rnd/moobot_ws/src/ds4_driver/msg/Trackpad.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ds4_driver
)
_generate_msg_nodejs(ds4_driver
  "/home/rnd/moobot_ws/src/ds4_driver/msg/Trackpad.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ds4_driver
)

### Generating Services

### Generating Module File
_generate_module_nodejs(ds4_driver
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ds4_driver
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(ds4_driver_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(ds4_driver_generate_messages ds4_driver_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rnd/moobot_ws/src/ds4_driver/msg/Feedback.msg" NAME_WE)
add_dependencies(ds4_driver_generate_messages_nodejs _ds4_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rnd/moobot_ws/src/ds4_driver/msg/Report.msg" NAME_WE)
add_dependencies(ds4_driver_generate_messages_nodejs _ds4_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rnd/moobot_ws/src/ds4_driver/msg/Status.msg" NAME_WE)
add_dependencies(ds4_driver_generate_messages_nodejs _ds4_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rnd/moobot_ws/src/ds4_driver/msg/Trackpad.msg" NAME_WE)
add_dependencies(ds4_driver_generate_messages_nodejs _ds4_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ds4_driver_gennodejs)
add_dependencies(ds4_driver_gennodejs ds4_driver_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ds4_driver_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(ds4_driver
  "/home/rnd/moobot_ws/src/ds4_driver/msg/Feedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ds4_driver
)
_generate_msg_py(ds4_driver
  "/home/rnd/moobot_ws/src/ds4_driver/msg/Report.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ds4_driver
)
_generate_msg_py(ds4_driver
  "/home/rnd/moobot_ws/src/ds4_driver/msg/Status.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/home/rnd/moobot_ws/src/ds4_driver/msg/Trackpad.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Imu.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ds4_driver
)
_generate_msg_py(ds4_driver
  "/home/rnd/moobot_ws/src/ds4_driver/msg/Trackpad.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ds4_driver
)

### Generating Services

### Generating Module File
_generate_module_py(ds4_driver
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ds4_driver
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ds4_driver_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ds4_driver_generate_messages ds4_driver_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rnd/moobot_ws/src/ds4_driver/msg/Feedback.msg" NAME_WE)
add_dependencies(ds4_driver_generate_messages_py _ds4_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rnd/moobot_ws/src/ds4_driver/msg/Report.msg" NAME_WE)
add_dependencies(ds4_driver_generate_messages_py _ds4_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rnd/moobot_ws/src/ds4_driver/msg/Status.msg" NAME_WE)
add_dependencies(ds4_driver_generate_messages_py _ds4_driver_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/rnd/moobot_ws/src/ds4_driver/msg/Trackpad.msg" NAME_WE)
add_dependencies(ds4_driver_generate_messages_py _ds4_driver_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ds4_driver_genpy)
add_dependencies(ds4_driver_genpy ds4_driver_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ds4_driver_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ds4_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ds4_driver
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(ds4_driver_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(ds4_driver_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ds4_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ds4_driver
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(ds4_driver_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(ds4_driver_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ds4_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ds4_driver
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(ds4_driver_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(ds4_driver_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ds4_driver)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ds4_driver
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(ds4_driver_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(ds4_driver_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ds4_driver)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ds4_driver\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ds4_driver
    DESTINATION ${genpy_INSTALL_DIR}
    # skip all init files
    PATTERN "__init__.py" EXCLUDE
    PATTERN "__init__.pyc" EXCLUDE
  )
  # install init files which are not in the root folder of the generated code
  string(REGEX REPLACE "([][+.*()^])" "\\\\\\1" ESCAPED_PATH "${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ds4_driver")
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ds4_driver
    DESTINATION ${genpy_INSTALL_DIR}
    FILES_MATCHING
    REGEX "${ESCAPED_PATH}/.+/__init__.pyc?$"
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(ds4_driver_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(ds4_driver_generate_messages_py sensor_msgs_generate_messages_py)
endif()