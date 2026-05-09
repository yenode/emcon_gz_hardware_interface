cmake_minimum_required(VERSION 3.16)
project(emcon_gz_hardware_interface)

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

find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/emcon_gz_system_interface.cpp
)

target_compile_definitions(${PROJECT_NAME}
  PRIVATE "EMCON_GZ_HARDWARE_INTERFACE_BUILDING_DLL"
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

pluginlib_export_plugin_description_file(hardware_interface emcon_gz_hardware_interface.xml)

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

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_emcon_gz_system_interface
    test/test_emcon_gz_system_interface.cpp
  )

  ament_target_dependencies(test_emcon_gz_system_interface
    hardware_interface
    rclcpp
    rclcpp_lifecycle
  )

  target_link_libraries(test_emcon_gz_system_interface
    ${PROJECT_NAME}
  )
endif()

ament_package()
