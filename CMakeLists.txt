cmake_minimum_required(VERSION 3.16)
project(gz_transport_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)

# Gazebo Transport (gz-transport13 for Jazzy/Harmonic)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)


add_library(${PROJECT_NAME} SHARED
  src/gz_transport_system_interface.cpp
)

target_compile_definitions(${PROJECT_NAME}
  PRIVATE "GZ_TRANSPORT_HARDWARE_INTERFACE_BUILDING_DLL"
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  realtime_tools
)

target_link_libraries(${PROJECT_NAME}
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)


pluginlib_export_plugin_description_file(hardware_interface gz_transport_hardware_interface.xml)


install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  realtime_tools
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
