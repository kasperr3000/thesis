cmake_minimum_required(VERSION 3.5)
project(pose_writer_aruco)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})


# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)

#INTERFACE: list of interfaces; message generation; exporting on runtime because it is used in this build(in this build)
set(srv_files "srv/FileDetails.srv")
rosidl_generate_interfaces(${PROJECT_NAME} ${srv_files} DEPENDENCIES builtin_interfaces geometry_msgs)
ament_export_dependencies(rosidl_default_runtime)

add_executable(tf_file_printer src/tf_file_printerV3.cpp)
#linking/providing INTERFACE of same package
rosidl_target_interfaces(tf_file_printer
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

ament_target_dependencies(tf_file_printer rclcpp builtin_interfaces geometry_msgs tf2 tf2_ros Boost)
target_include_directories(tf_file_printer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


TARGET_LINK_LIBRARIES(tf_file_printer ${Boost_LIBRARIES})

install(TARGETS tf_file_printer
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
