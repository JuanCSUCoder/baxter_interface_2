cmake_minimum_required(VERSION 3.8)
project(baxter_interface_2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

# Executable Test Program
add_executable(baxter_tests src/baxter_tests.cpp)

add_dependencies(baxter_tests baxter_interface_2)

target_include_directories(baxter_tests PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(baxter_tests PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_link_libraries(baxter_tests baxter_interface_2)

ament_target_dependencies(
  baxter_tests
  "moveit_ros_planning_interface"
  "rclcpp"
)

install(TARGETS baxter_tests
  DESTINATION lib/${PROJECT_NAME})

# Library Compilation
add_library(baxter_interface_2 src/ik_interface.cpp src/robot.cpp include/baxter_interface_2/robot.hpp)

target_include_directories(baxter_interface_2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(baxter_interface_2 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  baxter_interface_2
  "moveit_ros_planning_interface"
  "rclcpp"
)

install(TARGETS baxter_interface_2
  DESTINATION lib/${PROJECT_NAME})

# Auto-generated testing block
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
