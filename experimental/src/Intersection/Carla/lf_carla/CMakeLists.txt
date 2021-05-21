cmake_minimum_required(VERSION 3.5)
project(lf_carla)

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
find_package(carla_msgs REQUIRED)

include_directories(src/core)

include_directories(src/core/platform)

add_library(platform_support src/core/platform/lf_linux_support.cpp)

add_executable(lf_carla_publisher src/CarlaIntersection.cpp)

target_link_libraries(lf_carla_publisher platform_support)

ament_target_dependencies(lf_carla_publisher rclcpp carla_msgs)

install(TARGETS
  platform_support
  DESTINATION lib
)

install(TARGETS
  lf_carla_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY src/core/ DESTINATION core)

ament_package()
