cmake_minimum_required(VERSION 3.10)
project(harper_control)

# add compiler warnings and pedantic flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(controller_manager REQUIRED) 
find_package(dynamixel_sdk REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)


add_library(harper_dynamixel_system SHARED
  src/dynamixel_bus.cpp
  src/dynamixel_config.cpp
  src/harper_dynamixel_system.cpp
)

# set compiler features so that the library is compiled with C++17
target_compile_features(harper_dynamixel_system PUBLIC cxx_std_17)

target_include_directories(harper_dynamixel_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(harper_dynamixel_system
  ament_index_cpp
  dynamixel_sdk
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# this is necessary because the library uses yaml-cpp to read the configuration files.
target_link_libraries(harper_dynamixel_system yaml-cpp Threads::Threads)

# export the plugin description file so that the library can be used by the hardware interface.
pluginlib_export_plugin_description_file(hardware_interface harper_control_plugins.xml)

# install the library, plugin description file, and configuration files.
install(
  TARGETS harper_dynamixel_system
  EXPORT export_harper_dynamixel_system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_harper_dynamixel_system HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_index_cpp
  dynamixel_sdk
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
