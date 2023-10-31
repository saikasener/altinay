# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "moobot_pgv: 1 messages, 0 services")

set(MSG_I_FLAGS "-Imoobot_pgv:/home/rnd/moobot_ws/src/moobot_pgv/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(moobot_pgv_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/rnd/moobot_ws/src/moobot_pgv/msg/pgv_scan_data.msg" NAME_WE)
add_custom_target(_moobot_pgv_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "moobot_pgv" "/home/rnd/moobot_ws/src/moobot_pgv/msg/pgv_scan_data.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(moobot_pgv
  "/home/rnd/moobot_ws/src/moobot_pgv/msg/pgv_scan_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/moobot_pgv
)

### Generating Services

### Generating Module File
_generate_module_cpp(moobot_pgv
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/moobot_pgv
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(moobot_pgv_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(moobot_pgv_generate_messages moobot_pgv_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rnd/moobot_ws/src/moobot_pgv/msg/pgv_scan_data.msg" NAME_WE)
add_dependencies(moobot_pgv_generate_messages_cpp _moobot_pgv_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(moobot_pgv_gencpp)
add_dependencies(moobot_pgv_gencpp moobot_pgv_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS moobot_pgv_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(moobot_pgv
  "/home/rnd/moobot_ws/src/moobot_pgv/msg/pgv_scan_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/moobot_pgv
)

### Generating Services

### Generating Module File
_generate_module_eus(moobot_pgv
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/moobot_pgv
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(moobot_pgv_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(moobot_pgv_generate_messages moobot_pgv_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rnd/moobot_ws/src/moobot_pgv/msg/pgv_scan_data.msg" NAME_WE)
add_dependencies(moobot_pgv_generate_messages_eus _moobot_pgv_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(moobot_pgv_geneus)
add_dependencies(moobot_pgv_geneus moobot_pgv_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS moobot_pgv_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(moobot_pgv
  "/home/rnd/moobot_ws/src/moobot_pgv/msg/pgv_scan_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/moobot_pgv
)

### Generating Services

### Generating Module File
_generate_module_lisp(moobot_pgv
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/moobot_pgv
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(moobot_pgv_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(moobot_pgv_generate_messages moobot_pgv_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rnd/moobot_ws/src/moobot_pgv/msg/pgv_scan_data.msg" NAME_WE)
add_dependencies(moobot_pgv_generate_messages_lisp _moobot_pgv_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(moobot_pgv_genlisp)
add_dependencies(moobot_pgv_genlisp moobot_pgv_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS moobot_pgv_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(moobot_pgv
  "/home/rnd/moobot_ws/src/moobot_pgv/msg/pgv_scan_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/moobot_pgv
)

### Generating Services

### Generating Module File
_generate_module_nodejs(moobot_pgv
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/moobot_pgv
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(moobot_pgv_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(moobot_pgv_generate_messages moobot_pgv_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rnd/moobot_ws/src/moobot_pgv/msg/pgv_scan_data.msg" NAME_WE)
add_dependencies(moobot_pgv_generate_messages_nodejs _moobot_pgv_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(moobot_pgv_gennodejs)
add_dependencies(moobot_pgv_gennodejs moobot_pgv_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS moobot_pgv_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(moobot_pgv
  "/home/rnd/moobot_ws/src/moobot_pgv/msg/pgv_scan_data.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/moobot_pgv
)

### Generating Services

### Generating Module File
_generate_module_py(moobot_pgv
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/moobot_pgv
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(moobot_pgv_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(moobot_pgv_generate_messages moobot_pgv_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rnd/moobot_ws/src/moobot_pgv/msg/pgv_scan_data.msg" NAME_WE)
add_dependencies(moobot_pgv_generate_messages_py _moobot_pgv_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(moobot_pgv_genpy)
add_dependencies(moobot_pgv_genpy moobot_pgv_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS moobot_pgv_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/moobot_pgv)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/moobot_pgv
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(moobot_pgv_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/moobot_pgv)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/moobot_pgv
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(moobot_pgv_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/moobot_pgv)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/moobot_pgv
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(moobot_pgv_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/moobot_pgv)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/moobot_pgv
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(moobot_pgv_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/moobot_pgv)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/moobot_pgv\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/moobot_pgv
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(moobot_pgv_generate_messages_py std_msgs_generate_messages_py)
endif()
