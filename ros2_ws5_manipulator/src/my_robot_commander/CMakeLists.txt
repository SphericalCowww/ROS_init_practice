cmake_minimum_required(VERSION 3.8)
project(my_robot_commander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

###########################################################################
add_executable(my_robot_moveit_basic src/my_robot_moveit_basic.cpp)
ament_target_dependencies(my_robot_moveit_basic rclcpp moveit_ros_planning_interface)

install(TARGETS my_robot_moveit_basic
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
