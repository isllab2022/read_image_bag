# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "read_image_bag: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(read_image_bag_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/isl/catkin_ws/src/read_image_bag/srv/Rgbimage.srv" NAME_WE)
add_custom_target(_read_image_bag_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "read_image_bag" "/home/isl/catkin_ws/src/read_image_bag/srv/Rgbimage.srv" "std_msgs/Header:sensor_msgs/Image"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(read_image_bag
  "/home/isl/catkin_ws/src/read_image_bag/srv/Rgbimage.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/read_image_bag
)

### Generating Module File
_generate_module_cpp(read_image_bag
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/read_image_bag
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(read_image_bag_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(read_image_bag_generate_messages read_image_bag_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/isl/catkin_ws/src/read_image_bag/srv/Rgbimage.srv" NAME_WE)
add_dependencies(read_image_bag_generate_messages_cpp _read_image_bag_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(read_image_bag_gencpp)
add_dependencies(read_image_bag_gencpp read_image_bag_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS read_image_bag_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(read_image_bag
  "/home/isl/catkin_ws/src/read_image_bag/srv/Rgbimage.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/read_image_bag
)

### Generating Module File
_generate_module_eus(read_image_bag
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/read_image_bag
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(read_image_bag_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(read_image_bag_generate_messages read_image_bag_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/isl/catkin_ws/src/read_image_bag/srv/Rgbimage.srv" NAME_WE)
add_dependencies(read_image_bag_generate_messages_eus _read_image_bag_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(read_image_bag_geneus)
add_dependencies(read_image_bag_geneus read_image_bag_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS read_image_bag_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(read_image_bag
  "/home/isl/catkin_ws/src/read_image_bag/srv/Rgbimage.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/read_image_bag
)

### Generating Module File
_generate_module_lisp(read_image_bag
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/read_image_bag
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(read_image_bag_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(read_image_bag_generate_messages read_image_bag_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/isl/catkin_ws/src/read_image_bag/srv/Rgbimage.srv" NAME_WE)
add_dependencies(read_image_bag_generate_messages_lisp _read_image_bag_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(read_image_bag_genlisp)
add_dependencies(read_image_bag_genlisp read_image_bag_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS read_image_bag_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(read_image_bag
  "/home/isl/catkin_ws/src/read_image_bag/srv/Rgbimage.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/read_image_bag
)

### Generating Module File
_generate_module_nodejs(read_image_bag
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/read_image_bag
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(read_image_bag_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(read_image_bag_generate_messages read_image_bag_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/isl/catkin_ws/src/read_image_bag/srv/Rgbimage.srv" NAME_WE)
add_dependencies(read_image_bag_generate_messages_nodejs _read_image_bag_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(read_image_bag_gennodejs)
add_dependencies(read_image_bag_gennodejs read_image_bag_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS read_image_bag_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(read_image_bag
  "/home/isl/catkin_ws/src/read_image_bag/srv/Rgbimage.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/sensor_msgs/cmake/../msg/Image.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/read_image_bag
)

### Generating Module File
_generate_module_py(read_image_bag
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/read_image_bag
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(read_image_bag_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(read_image_bag_generate_messages read_image_bag_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/isl/catkin_ws/src/read_image_bag/srv/Rgbimage.srv" NAME_WE)
add_dependencies(read_image_bag_generate_messages_py _read_image_bag_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(read_image_bag_genpy)
add_dependencies(read_image_bag_genpy read_image_bag_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS read_image_bag_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/read_image_bag)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/read_image_bag
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(read_image_bag_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(read_image_bag_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/read_image_bag)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/read_image_bag
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(read_image_bag_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(read_image_bag_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/read_image_bag)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/read_image_bag
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(read_image_bag_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(read_image_bag_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/read_image_bag)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/read_image_bag
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(read_image_bag_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(read_image_bag_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/read_image_bag)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/read_image_bag\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/read_image_bag
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(read_image_bag_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(read_image_bag_generate_messages_py sensor_msgs_generate_messages_py)
endif()
