# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "Car_Spraying: 3 messages, 1 services")

set(MSG_I_FLAGS "-ICar_Spraying:/home/lzq/lzq_ws/src/Car_Spraying/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(Car_Spraying_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/lzq/lzq_ws/src/Car_Spraying/msg/SetTwoAngle.msg" NAME_WE)
add_custom_target(_Car_Spraying_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "Car_Spraying" "/home/lzq/lzq_ws/src/Car_Spraying/msg/SetTwoAngle.msg" ""
)

get_filename_component(_filename "/home/lzq/lzq_ws/src/Car_Spraying/msg/SetServoAngle.msg" NAME_WE)
add_custom_target(_Car_Spraying_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "Car_Spraying" "/home/lzq/lzq_ws/src/Car_Spraying/msg/SetServoAngle.msg" ""
)

get_filename_component(_filename "/home/lzq/lzq_ws/src/Car_Spraying/msg/SetServoDamping.msg" NAME_WE)
add_custom_target(_Car_Spraying_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "Car_Spraying" "/home/lzq/lzq_ws/src/Car_Spraying/msg/SetServoDamping.msg" ""
)

get_filename_component(_filename "/home/lzq/lzq_ws/src/Car_Spraying/srv/QueryServoAngle.srv" NAME_WE)
add_custom_target(_Car_Spraying_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "Car_Spraying" "/home/lzq/lzq_ws/src/Car_Spraying/srv/QueryServoAngle.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(Car_Spraying
  "/home/lzq/lzq_ws/src/Car_Spraying/msg/SetTwoAngle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/Car_Spraying
)
_generate_msg_cpp(Car_Spraying
  "/home/lzq/lzq_ws/src/Car_Spraying/msg/SetServoAngle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/Car_Spraying
)
_generate_msg_cpp(Car_Spraying
  "/home/lzq/lzq_ws/src/Car_Spraying/msg/SetServoDamping.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/Car_Spraying
)

### Generating Services
_generate_srv_cpp(Car_Spraying
  "/home/lzq/lzq_ws/src/Car_Spraying/srv/QueryServoAngle.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/Car_Spraying
)

### Generating Module File
_generate_module_cpp(Car_Spraying
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/Car_Spraying
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(Car_Spraying_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(Car_Spraying_generate_messages Car_Spraying_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lzq/lzq_ws/src/Car_Spraying/msg/SetTwoAngle.msg" NAME_WE)
add_dependencies(Car_Spraying_generate_messages_cpp _Car_Spraying_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lzq/lzq_ws/src/Car_Spraying/msg/SetServoAngle.msg" NAME_WE)
add_dependencies(Car_Spraying_generate_messages_cpp _Car_Spraying_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lzq/lzq_ws/src/Car_Spraying/msg/SetServoDamping.msg" NAME_WE)
add_dependencies(Car_Spraying_generate_messages_cpp _Car_Spraying_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lzq/lzq_ws/src/Car_Spraying/srv/QueryServoAngle.srv" NAME_WE)
add_dependencies(Car_Spraying_generate_messages_cpp _Car_Spraying_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(Car_Spraying_gencpp)
add_dependencies(Car_Spraying_gencpp Car_Spraying_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS Car_Spraying_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(Car_Spraying
  "/home/lzq/lzq_ws/src/Car_Spraying/msg/SetTwoAngle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/Car_Spraying
)
_generate_msg_eus(Car_Spraying
  "/home/lzq/lzq_ws/src/Car_Spraying/msg/SetServoAngle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/Car_Spraying
)
_generate_msg_eus(Car_Spraying
  "/home/lzq/lzq_ws/src/Car_Spraying/msg/SetServoDamping.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/Car_Spraying
)

### Generating Services
_generate_srv_eus(Car_Spraying
  "/home/lzq/lzq_ws/src/Car_Spraying/srv/QueryServoAngle.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/Car_Spraying
)

### Generating Module File
_generate_module_eus(Car_Spraying
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/Car_Spraying
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(Car_Spraying_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(Car_Spraying_generate_messages Car_Spraying_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lzq/lzq_ws/src/Car_Spraying/msg/SetTwoAngle.msg" NAME_WE)
add_dependencies(Car_Spraying_generate_messages_eus _Car_Spraying_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lzq/lzq_ws/src/Car_Spraying/msg/SetServoAngle.msg" NAME_WE)
add_dependencies(Car_Spraying_generate_messages_eus _Car_Spraying_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lzq/lzq_ws/src/Car_Spraying/msg/SetServoDamping.msg" NAME_WE)
add_dependencies(Car_Spraying_generate_messages_eus _Car_Spraying_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lzq/lzq_ws/src/Car_Spraying/srv/QueryServoAngle.srv" NAME_WE)
add_dependencies(Car_Spraying_generate_messages_eus _Car_Spraying_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(Car_Spraying_geneus)
add_dependencies(Car_Spraying_geneus Car_Spraying_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS Car_Spraying_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(Car_Spraying
  "/home/lzq/lzq_ws/src/Car_Spraying/msg/SetTwoAngle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/Car_Spraying
)
_generate_msg_lisp(Car_Spraying
  "/home/lzq/lzq_ws/src/Car_Spraying/msg/SetServoAngle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/Car_Spraying
)
_generate_msg_lisp(Car_Spraying
  "/home/lzq/lzq_ws/src/Car_Spraying/msg/SetServoDamping.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/Car_Spraying
)

### Generating Services
_generate_srv_lisp(Car_Spraying
  "/home/lzq/lzq_ws/src/Car_Spraying/srv/QueryServoAngle.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/Car_Spraying
)

### Generating Module File
_generate_module_lisp(Car_Spraying
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/Car_Spraying
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(Car_Spraying_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(Car_Spraying_generate_messages Car_Spraying_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lzq/lzq_ws/src/Car_Spraying/msg/SetTwoAngle.msg" NAME_WE)
add_dependencies(Car_Spraying_generate_messages_lisp _Car_Spraying_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lzq/lzq_ws/src/Car_Spraying/msg/SetServoAngle.msg" NAME_WE)
add_dependencies(Car_Spraying_generate_messages_lisp _Car_Spraying_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lzq/lzq_ws/src/Car_Spraying/msg/SetServoDamping.msg" NAME_WE)
add_dependencies(Car_Spraying_generate_messages_lisp _Car_Spraying_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lzq/lzq_ws/src/Car_Spraying/srv/QueryServoAngle.srv" NAME_WE)
add_dependencies(Car_Spraying_generate_messages_lisp _Car_Spraying_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(Car_Spraying_genlisp)
add_dependencies(Car_Spraying_genlisp Car_Spraying_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS Car_Spraying_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(Car_Spraying
  "/home/lzq/lzq_ws/src/Car_Spraying/msg/SetTwoAngle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/Car_Spraying
)
_generate_msg_nodejs(Car_Spraying
  "/home/lzq/lzq_ws/src/Car_Spraying/msg/SetServoAngle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/Car_Spraying
)
_generate_msg_nodejs(Car_Spraying
  "/home/lzq/lzq_ws/src/Car_Spraying/msg/SetServoDamping.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/Car_Spraying
)

### Generating Services
_generate_srv_nodejs(Car_Spraying
  "/home/lzq/lzq_ws/src/Car_Spraying/srv/QueryServoAngle.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/Car_Spraying
)

### Generating Module File
_generate_module_nodejs(Car_Spraying
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/Car_Spraying
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(Car_Spraying_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(Car_Spraying_generate_messages Car_Spraying_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lzq/lzq_ws/src/Car_Spraying/msg/SetTwoAngle.msg" NAME_WE)
add_dependencies(Car_Spraying_generate_messages_nodejs _Car_Spraying_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lzq/lzq_ws/src/Car_Spraying/msg/SetServoAngle.msg" NAME_WE)
add_dependencies(Car_Spraying_generate_messages_nodejs _Car_Spraying_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lzq/lzq_ws/src/Car_Spraying/msg/SetServoDamping.msg" NAME_WE)
add_dependencies(Car_Spraying_generate_messages_nodejs _Car_Spraying_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lzq/lzq_ws/src/Car_Spraying/srv/QueryServoAngle.srv" NAME_WE)
add_dependencies(Car_Spraying_generate_messages_nodejs _Car_Spraying_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(Car_Spraying_gennodejs)
add_dependencies(Car_Spraying_gennodejs Car_Spraying_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS Car_Spraying_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(Car_Spraying
  "/home/lzq/lzq_ws/src/Car_Spraying/msg/SetTwoAngle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/Car_Spraying
)
_generate_msg_py(Car_Spraying
  "/home/lzq/lzq_ws/src/Car_Spraying/msg/SetServoAngle.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/Car_Spraying
)
_generate_msg_py(Car_Spraying
  "/home/lzq/lzq_ws/src/Car_Spraying/msg/SetServoDamping.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/Car_Spraying
)

### Generating Services
_generate_srv_py(Car_Spraying
  "/home/lzq/lzq_ws/src/Car_Spraying/srv/QueryServoAngle.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/Car_Spraying
)

### Generating Module File
_generate_module_py(Car_Spraying
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/Car_Spraying
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(Car_Spraying_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(Car_Spraying_generate_messages Car_Spraying_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lzq/lzq_ws/src/Car_Spraying/msg/SetTwoAngle.msg" NAME_WE)
add_dependencies(Car_Spraying_generate_messages_py _Car_Spraying_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lzq/lzq_ws/src/Car_Spraying/msg/SetServoAngle.msg" NAME_WE)
add_dependencies(Car_Spraying_generate_messages_py _Car_Spraying_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lzq/lzq_ws/src/Car_Spraying/msg/SetServoDamping.msg" NAME_WE)
add_dependencies(Car_Spraying_generate_messages_py _Car_Spraying_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lzq/lzq_ws/src/Car_Spraying/srv/QueryServoAngle.srv" NAME_WE)
add_dependencies(Car_Spraying_generate_messages_py _Car_Spraying_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(Car_Spraying_genpy)
add_dependencies(Car_Spraying_genpy Car_Spraying_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS Car_Spraying_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/Car_Spraying)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/Car_Spraying
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(Car_Spraying_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/Car_Spraying)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/Car_Spraying
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(Car_Spraying_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/Car_Spraying)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/Car_Spraying
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(Car_Spraying_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/Car_Spraying)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/Car_Spraying
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(Car_Spraying_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/Car_Spraying)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/Car_Spraying\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/Car_Spraying
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(Car_Spraying_generate_messages_py std_msgs_generate_messages_py)
endif()
