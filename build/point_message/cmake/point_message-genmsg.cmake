# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "point_message: 2 messages, 0 services")

set(MSG_I_FLAGS "-Ipoint_message:/home/quinn/brokenGlasses/src/point_message/msg;-Istd_msgs:/opt/ros/jade/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(point_message_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/quinn/brokenGlasses/src/point_message/msg/statsMsg.msg" NAME_WE)
add_custom_target(_point_message_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "point_message" "/home/quinn/brokenGlasses/src/point_message/msg/statsMsg.msg" ""
)

get_filename_component(_filename "/home/quinn/brokenGlasses/src/point_message/msg/pointMsg.msg" NAME_WE)
add_custom_target(_point_message_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "point_message" "/home/quinn/brokenGlasses/src/point_message/msg/pointMsg.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(point_message
  "/home/quinn/brokenGlasses/src/point_message/msg/statsMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/point_message
)
_generate_msg_cpp(point_message
  "/home/quinn/brokenGlasses/src/point_message/msg/pointMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/point_message
)

### Generating Services

### Generating Module File
_generate_module_cpp(point_message
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/point_message
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(point_message_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(point_message_generate_messages point_message_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/quinn/brokenGlasses/src/point_message/msg/statsMsg.msg" NAME_WE)
add_dependencies(point_message_generate_messages_cpp _point_message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/quinn/brokenGlasses/src/point_message/msg/pointMsg.msg" NAME_WE)
add_dependencies(point_message_generate_messages_cpp _point_message_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(point_message_gencpp)
add_dependencies(point_message_gencpp point_message_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS point_message_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(point_message
  "/home/quinn/brokenGlasses/src/point_message/msg/statsMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/point_message
)
_generate_msg_eus(point_message
  "/home/quinn/brokenGlasses/src/point_message/msg/pointMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/point_message
)

### Generating Services

### Generating Module File
_generate_module_eus(point_message
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/point_message
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(point_message_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(point_message_generate_messages point_message_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/quinn/brokenGlasses/src/point_message/msg/statsMsg.msg" NAME_WE)
add_dependencies(point_message_generate_messages_eus _point_message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/quinn/brokenGlasses/src/point_message/msg/pointMsg.msg" NAME_WE)
add_dependencies(point_message_generate_messages_eus _point_message_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(point_message_geneus)
add_dependencies(point_message_geneus point_message_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS point_message_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(point_message
  "/home/quinn/brokenGlasses/src/point_message/msg/statsMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/point_message
)
_generate_msg_lisp(point_message
  "/home/quinn/brokenGlasses/src/point_message/msg/pointMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/point_message
)

### Generating Services

### Generating Module File
_generate_module_lisp(point_message
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/point_message
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(point_message_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(point_message_generate_messages point_message_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/quinn/brokenGlasses/src/point_message/msg/statsMsg.msg" NAME_WE)
add_dependencies(point_message_generate_messages_lisp _point_message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/quinn/brokenGlasses/src/point_message/msg/pointMsg.msg" NAME_WE)
add_dependencies(point_message_generate_messages_lisp _point_message_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(point_message_genlisp)
add_dependencies(point_message_genlisp point_message_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS point_message_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(point_message
  "/home/quinn/brokenGlasses/src/point_message/msg/statsMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/point_message
)
_generate_msg_py(point_message
  "/home/quinn/brokenGlasses/src/point_message/msg/pointMsg.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/point_message
)

### Generating Services

### Generating Module File
_generate_module_py(point_message
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/point_message
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(point_message_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(point_message_generate_messages point_message_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/quinn/brokenGlasses/src/point_message/msg/statsMsg.msg" NAME_WE)
add_dependencies(point_message_generate_messages_py _point_message_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/quinn/brokenGlasses/src/point_message/msg/pointMsg.msg" NAME_WE)
add_dependencies(point_message_generate_messages_py _point_message_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(point_message_genpy)
add_dependencies(point_message_genpy point_message_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS point_message_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/point_message)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/point_message
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(point_message_generate_messages_cpp std_msgs_generate_messages_cpp)

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/point_message)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/point_message
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
add_dependencies(point_message_generate_messages_eus std_msgs_generate_messages_eus)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/point_message)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/point_message
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(point_message_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/point_message)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/point_message\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/point_message
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(point_message_generate_messages_py std_msgs_generate_messages_py)
