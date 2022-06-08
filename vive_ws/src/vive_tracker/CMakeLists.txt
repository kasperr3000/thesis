cmake_minimum_required(VERSION 3.5)
project(vive_tracker)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions(-DLINUX -DLINUX64 -DPOSIX)

## set definition, path, compileflags
set(OPENVR "$ENV{HOME}/libraries/openvr")
add_definitions(-DOPENVR_PATH="${OPENVR}") ## for source code to get sample textures and shader files path
set(THIRDPARTY_DIR ${OPENVR}/samples/thirdparty)
set(SHARED_SRC_DIR ${OPENVR}/samples/shared)
set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS} -std=c++11 -include ${SHARED_SRC_DIR}/compat.h")
set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wextra -pedantic -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")

## find openvr
find_library(OPENVR_LIBRARIES
  NAMES
    openvr_api
  PATHS
    ${OPENVR}/bin
    ${OPENVR}/lib
  PATH_SUFFIXES
    linux64
  NO_DEFAULT_PATH
  NO_CMAKE_FIND_ROOT_PATH
)
set(OPENVR_INCLUDE_DIR ${OPENVR}/headers ${OPENVR}/samples)
## common src from openvr sample
file(GLOB SHARED_SRC_FILES
  ${SHARED_SRC_DIR}/*.cpp
  ${SHARED_SRC_DIR}/*.h
)

include_directories(
  include
  ${ament_INCLUDE_DIR}
)




# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(boost REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)



# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)

#INTERFACE: list of interfaces; message generation; exporting on runtime because it is used in this build(in this build)
set(srv_files "srv/Calibrate.srv")
rosidl_generate_interfaces(${PROJECT_NAME} ${srv_files} DEPENDENCIES builtin_interfaces geometry_msgs)
ament_export_dependencies(rosidl_default_runtime)


set(CMAKE_BUILD_TYPE "Release")

add_executable(vive_node
  ${SHARED_SRC_FILES}
  src/vive_node.cpp
  src/vr_interface.cpp
)              #making a node
ament_target_dependencies(vive_node rclcpp cv_bridge sensor_msgs geometry_msgs tf2 tf2_ros) #receiving (hier cvbridge en opencv)

rosidl_target_interfaces(vive_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)


target_include_directories(vive_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OPENVR_INCLUDE_DIR}
)

install(
  TARGETS vive_node
  DESTINATION lib/${PROJECT_NAME}
)

target_link_libraries(vive_node
  ${ament_LIBRARIES}
  ${OPENVR_LIBRARIES}
  ${CMAKE_DL_LIBS}
  ${EXTRA_LIBS}
)

ament_package()
