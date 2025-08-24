cmake_minimum_required(VERSION 3.8)
project(my_robot_firmware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

###########################################################################
add_library(pca9685_driver STATIC
  include/pca9685/I2C.cpp
  include/pca9685/PCA9685.cpp
)
set_target_properties(pca9685_driver PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(pca9685_driver PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
)
add_executable(testRaspPi5_pca9685_channel0 src/testRaspPi5_pca9685_channel0.cpp)
ament_target_dependencies(testRaspPi5_pca9685_channel0 rclcpp)
target_link_libraries(testRaspPi5_pca9685_channel0 pca9685_driver)

add_executable(testRaspPi5_pca9685_channel01 src/testRaspPi5_pca9685_channel01.cpp)
ament_target_dependencies(testRaspPi5_pca9685_channel01 rclcpp)
target_link_libraries(testRaspPi5_pca9685_channel01 pca9685_driver)

install(TARGETS pca9685_driver
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(TARGETS testRaspPi5_pca9685_channel0 testRaspPi5_pca9685_channel01
    DESTINATION lib/${PROJECT_NAME}
)

###########################################################################
add_library(hardware_interface_pca9685 SHARED
  src/hardware_interface_pca9685.cpp
)
ament_target_dependencies(hardware_interface_pca9685 rclcpp rclcpp_lifecycle hardware_interface pluginlib)
target_link_libraries(hardware_interface_pca9685 pca9685_driver)
pluginlib_export_plugin_description_file(hardware_interface my_robot_hardware_interface.xml)
install(TARGETS hardware_interface_pca9685
  DESTINATION lib
)
install(
  DIRECTORY   include
  DESTINATION include
)
ament_export_libraries(
  $(PROJECT_NAME)
)
ament_export_dependencies(rclcpp rclcpp_lifecycle hardware_interface pluginlib)
###########################################################################


ament_package()
