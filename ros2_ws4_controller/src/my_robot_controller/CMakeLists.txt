cmake_minimum_required(VERSION 3.8)
project(my_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(example_interface REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

###########################################################################
add_library(ma_robot_controller SHARED
  src/ma_robot_controller.cpp
)
ament_target_dependencies(ma_robot_controller rclcpp rclcpp_lifecycle controller_interface pluginlib)
pluginlib_export_plugin_description_file(controller_interface ma_robot_controller.xml)

install(TARGETS  ma_robot_controller
  DESTINATION lib
)
install(
  DIRECTORY   include
  DESTINATION include
)
ament_export_libraries(
  ma_robot_controller
)
ament_export_dependencies(rclcpp rclcpp_lifecycle controller_interface pluginlib)
###########################################################################


ament_package()
