cmake_minimum_required(VERSION 3.5)
project(timelimitros-native)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(timelimitros-native-clock src/timelimitros-native-clock.cpp)
ament_target_dependencies(timelimitros-native-clock rclcpp std_msgs)

add_executable(timelimitros-native-dest src/timelimitros-native-dest.cpp)
ament_target_dependencies(timelimitros-native-dest rclcpp std_msgs)

install(TARGETS
   timelimitros-native-clock
   timelimitros-native-dest
   DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
