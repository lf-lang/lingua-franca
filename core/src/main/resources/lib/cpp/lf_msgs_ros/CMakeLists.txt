cmake_minimum_required(VERSION 3.5)
project(lf_msgs_ros)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Tag.msg"
 )

ament_package()


