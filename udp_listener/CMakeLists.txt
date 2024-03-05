cmake_minimum_required(VERSION 3.8)
project(udp_listener)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)

#add_executable(udp_listener src/udp_listener.cpp)
#ament_target_dependencies(udp_listener rclcpp std_msgs)

add_executable(udp_new src/udp_new.cpp)
ament_target_dependencies(udp_new rclcpp std_msgs)

# Install launch files
#DESTINATION share/${PROJECT_NAME}
#install(
#  DIRECTORY launch
#)

# Install nodes
install(
  TARGETS
  udp_new
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
