cmake_minimum_required(VERSION 3.8)
project(booster_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(booster_msgs
  "msg/HandDdsMsg.msg"
  "msg/HandCommand.msg"
  "msg/HandParam.msg"
)

install(
  DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(booster_interface)

ament_package()
