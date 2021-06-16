cmake_minimum_required(VERSION 3.5)
project(lf_carla)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(carla_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include/core)

include_directories(include/core/platform)

add_compile_definitions(NUMBER_OF_WORKERS=4)

add_library(platform_support src/core/platform/lf_linux_support.cpp)

add_executable(lf_carla_publisher src/CarlaIntersection.cpp)

target_link_libraries(lf_carla_publisher platform_support)

ament_target_dependencies(lf_carla_publisher rclcpp carla_msgs geometry_msgs sensor_msgs)


install(TARGETS
  platform_support
  DESTINATION lib
)

install(TARGETS
  lf_carla_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
          DESTINATION share/${PROJECT_NAME}/)

install(DIRECTORY config/ 
          DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY src/core/ DESTINATION core)

ament_package()
