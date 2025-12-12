cmake_minimum_required(VERSION 3.8)
project(test_lifecycle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Node1
add_executable(node1 src/node1.cpp)
ament_target_dependencies(node1 rclcpp rclcpp_lifecycle lifecycle_msgs std_msgs)

# Node2
add_executable(node2 src/node2.cpp)
ament_target_dependencies(node2 rclcpp rclcpp_lifecycle lifecycle_msgs std_msgs)

# Node3
add_executable(node3 src/node3.cpp)
ament_target_dependencies(node3 rclcpp rclcpp_lifecycle lifecycle_msgs std_msgs)

# Node4
add_executable(node4 src/node4.cpp)
ament_target_dependencies(node4 rclcpp rclcpp_lifecycle lifecycle_msgs std_msgs)

# Lifecycle Manager
add_executable(lifecycle_manager src/lifecycle_manager.cpp)
ament_target_dependencies(lifecycle_manager rclcpp lifecycle_msgs)

# GCS Node
add_executable(gcs src/gcs.cpp)
ament_target_dependencies(gcs rclcpp std_msgs)

# Install executables
install(TARGETS
  node1
  node2
  node3
  node4
  lifecycle_manager
  gcs
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
