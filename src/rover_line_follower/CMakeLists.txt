cmake_minimum_required(VERSION 3.8)
project(rover_line_follower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(camera_viewer src/camera_viewer.cpp)
ament_target_dependencies(camera_viewer
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

install(TARGETS
  camera_viewer
  DESTINATION lib/${PROJECT_NAME}
)

# ament_index에 패키지 등록을 위한 resource 파일 설치
# ament_package() 호출 전에 설치되어야 함
install(
  FILES
    resource/${PROJECT_NAME}
  DESTINATION
    share/ament_index/resource_index/packages
)

# launch 파일 설치 (하이브리드 패키지에서는 CMakeLists.txt에서 설치)
install(
  DIRECTORY
    launch
  DESTINATION
    share/${PROJECT_NAME}
)

# Python 패키지 설치 (하이브리드 패키지)
# ament_python_install_package가 setup.py를 실행하지만
# entry_points가 제대로 설치되지 않을 수 있으므로
# Python 스크립트를 직접 실행 가능하게 설치
ament_python_install_package(${PROJECT_NAME})

# entry_points를 위한 Python 스크립트 직접 설치
# setup.py의 entry_points가 제대로 작동하지 않을 경우를 대비
# Python 스크립트에 shebang이 있으므로 PROGRAMS로 설치하면 실행 가능
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/line_follower_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME line_follower_node
)
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/tune_bev.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME tune_bev
)
install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}/teleop_keyboard.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME teleop_keyboard
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
