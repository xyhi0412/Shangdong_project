# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "eskf_navi: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ieskf_navi:/home/xiaomi1/Shandong_project/navi_eskf/src/eskf_navi/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(eskf_navi_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/xiaomi1/Shandong_project/navi_eskf/src/eskf_navi/msg/hedge_pos_a.msg" NAME_WE)
add_custom_target(_eskf_navi_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "eskf_navi" "/home/xiaomi1/Shandong_project/navi_eskf/src/eskf_navi/msg/hedge_pos_a.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(eskf_navi
  "/home/xiaomi1/Shandong_project/navi_eskf/src/eskf_navi/msg/hedge_pos_a.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/eskf_navi
)

### Generating Services

### Generating Module File
_generate_module_cpp(eskf_navi
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/eskf_navi
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(eskf_navi_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(eskf_navi_generate_messages eskf_navi_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xiaomi1/Shandong_project/navi_eskf/src/eskf_navi/msg/hedge_pos_a.msg" NAME_WE)
add_dependencies(eskf_navi_generate_messages_cpp _eskf_navi_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(eskf_navi_gencpp)
add_dependencies(eskf_navi_gencpp eskf_navi_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS eskf_navi_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(eskf_navi
  "/home/xiaomi1/Shandong_project/navi_eskf/src/eskf_navi/msg/hedge_pos_a.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/eskf_navi
)

### Generating Services

### Generating Module File
_generate_module_eus(eskf_navi
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/eskf_navi
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(eskf_navi_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(eskf_navi_generate_messages eskf_navi_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xiaomi1/Shandong_project/navi_eskf/src/eskf_navi/msg/hedge_pos_a.msg" NAME_WE)
add_dependencies(eskf_navi_generate_messages_eus _eskf_navi_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(eskf_navi_geneus)
add_dependencies(eskf_navi_geneus eskf_navi_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS eskf_navi_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(eskf_navi
  "/home/xiaomi1/Shandong_project/navi_eskf/src/eskf_navi/msg/hedge_pos_a.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/eskf_navi
)

### Generating Services

### Generating Module File
_generate_module_lisp(eskf_navi
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/eskf_navi
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(eskf_navi_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(eskf_navi_generate_messages eskf_navi_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xiaomi1/Shandong_project/navi_eskf/src/eskf_navi/msg/hedge_pos_a.msg" NAME_WE)
add_dependencies(eskf_navi_generate_messages_lisp _eskf_navi_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(eskf_navi_genlisp)
add_dependencies(eskf_navi_genlisp eskf_navi_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS eskf_navi_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(eskf_navi
  "/home/xiaomi1/Shandong_project/navi_eskf/src/eskf_navi/msg/hedge_pos_a.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/eskf_navi
)

### Generating Services

### Generating Module File
_generate_module_nodejs(eskf_navi
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/eskf_navi
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(eskf_navi_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(eskf_navi_generate_messages eskf_navi_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xiaomi1/Shandong_project/navi_eskf/src/eskf_navi/msg/hedge_pos_a.msg" NAME_WE)
add_dependencies(eskf_navi_generate_messages_nodejs _eskf_navi_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(eskf_navi_gennodejs)
add_dependencies(eskf_navi_gennodejs eskf_navi_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS eskf_navi_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(eskf_navi
  "/home/xiaomi1/Shandong_project/navi_eskf/src/eskf_navi/msg/hedge_pos_a.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/eskf_navi
)

### Generating Services

### Generating Module File
_generate_module_py(eskf_navi
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/eskf_navi
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(eskf_navi_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(eskf_navi_generate_messages eskf_navi_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/xiaomi1/Shandong_project/navi_eskf/src/eskf_navi/msg/hedge_pos_a.msg" NAME_WE)
add_dependencies(eskf_navi_generate_messages_py _eskf_navi_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(eskf_navi_genpy)
add_dependencies(eskf_navi_genpy eskf_navi_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS eskf_navi_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/eskf_navi)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/eskf_navi
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(eskf_navi_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/eskf_navi)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/eskf_navi
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(eskf_navi_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/eskf_navi)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/eskf_navi
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(eskf_navi_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/eskf_navi)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/eskf_navi
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(eskf_navi_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/eskf_navi)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/eskf_navi\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/eskf_navi
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(eskf_navi_generate_messages_py std_msgs_generate_messages_py)
endif()
