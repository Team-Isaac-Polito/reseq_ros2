cmake_minimum_required(VERSION 3.8)
project(reseq_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# BUILD steps for the Hardware Interface
add_library(reseq_hardware SHARED
  hardware/reseq.cpp
  hardware/canbus.cpp
  hardware/msg_buffer.cpp
  hardware/config_parser.cpp
)
target_compile_features(reseq_hardware PUBLIC cxx_std_17)
target_include_directories(reseq_hardware PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/reseq_hardware>
)
ament_target_dependencies(reseq_hardware PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ament_index_cpp
)
target_link_libraries(reseq_hardware PUBLIC yaml-cpp)

pluginlib_export_plugin_description_file(hardware_interface reseq_hardware.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/reseq_hardware
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(TARGETS reseq_hardware
  EXPORT export_reseq_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# TEST
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)
  ament_add_gmock(test_reseq_hardware test/test_reseq_hardware.cpp)
  target_link_libraries(test_reseq_hardware reseq_hardware)
  ament_target_dependencies(test_reseq_hardware
    pluginlib
    ros2_control_test_assets
    hardware_interface
  )
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_reseq_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ament_index_cpp
  yaml-cpp
)

ament_package()
