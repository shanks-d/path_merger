cmake_minimum_required(VERSION 3.5)
project(path_merger)

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
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Path.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

# find headers in the include folder
include_directories(include)

add_executable(merger src/merger_node.cpp src/merger_obj.cpp)
ament_target_dependencies(merger rclcpp geometry_msgs)
rosidl_target_interfaces(merger ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(input src/input_paths.cpp)
ament_target_dependencies(input rclcpp geometry_msgs)
rosidl_target_interfaces(input ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(visualize src/visualize_paths.cpp)
ament_target_dependencies(visualize rclcpp visualization_msgs)
rosidl_target_interfaces(visualize ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(generate src/generate_paths.cpp)
ament_target_dependencies(generate rclcpp)

install(TARGETS
  merger
  input
  visualize
  generate
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
