cmake_minimum_required(VERSION 3.8)
project(self_drive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)

  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
add_executable(self_drive src/self_drive.cpp)
ament_target_dependencies(self_drive
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
)

install(TARGETS
  self_drive
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


