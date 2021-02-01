cmake_minimum_required(VERSION 3.5)
project(TimeLimitROSNative)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(TimeLimitROSNative-Clock src/TimeLimitROSNative-Clock.cpp)
ament_target_dependencies(TimeLimitROSNative-Clock rclcpp std_msgs)

add_executable(TimeLimitROSNative-Dest src/TimeLimitROSNative-Dest.cpp)
ament_target_dependencies(TimeLimitROSNative-Dest rclcpp std_msgs)

install(TARGETS
   TimeLimitROSNative-Clock
   TimeLimitROSNative-Dest
   DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
