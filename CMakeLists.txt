cmake_minimum_required(VERSION 3.8)
project(robotic_platform)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED)

find_package(robot_controller_msgs REQUIRED)
find_package(robotic_platform_msgs REQUIRED)

include_directories(
  include
  ${YAML_CPP_INCLUDEDIR}
)

add_executable(manipulation_server 
  src/manipulation/main.cpp
  src/manipulation/collision_planner/collision_planner.cpp

  src/manipulation/workflow_planner/workflow_planner.cpp
  src/manipulation/workflow_planner/action_server/scan_sku.cpp

)
target_include_directories(manipulation_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(manipulation_server
  rclcpp
  rclcpp_action
  sensor_msgs

  geometry_msgs
  moveit_msgs

  geometric_shapes
  Boost

  robot_controller_msgs
  robotic_platform_msgs
)
target_link_libraries(manipulation_server 
  ${YAML_CPP_LIBRARIES})
target_compile_features(manipulation_server PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS 
  manipulation_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  params
  DESTINATION share/${PROJECT_NAME}/
)

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
