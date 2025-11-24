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
find_package(lifecycle_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Boost REQUIRED)

find_package(robot_controller_msgs REQUIRED)
find_package(robotic_platform_msgs REQUIRED)
find_package(robostore_msgs REQUIRED)

include_directories(
  include
  ${YAML_CPP_INCLUDEDIR}
)

set(common_files
  src/node_base.cpp
  src/planner_base.cpp
  src/tf_broadcaster.cpp
)

set(manager_files
  src/manager/main.cpp
  src/manager/manager.cpp
  src/manager/action_server/new_order.cpp
  src/manager/execution.cpp
  src/manager/utils.cpp
  src/manager/pick.cpp
  src/manager/place.cpp
)

set(manipulation_files
  src/manipulation/end_effector_ctrl/vacuum_gripper.cpp
  src/manipulation/end_effector_ctrl/finger_gripper.cpp

  src/manipulation/collision_planner/collision_planner.cpp

  src/manipulation/motion_planner/motion_planner.cpp
  src/manipulation/motion_planner/finger_gripper_impl.cpp
  src/manipulation/motion_planner/vac_gripper_impl.cpp
  src/manipulation/motion_planner/motion.cpp
  src/manipulation/motion_planner/motion_impl.cpp
  src/manipulation/motion_planner/utils.cpp

  src/manipulation/workflow_planner/workflow_planner.cpp
  src/manipulation/workflow_planner/transformation.cpp
  src/manipulation/workflow_planner/utils.cpp
  
  src/manipulation/workflow_planner/strategy/elevation.cpp
  src/manipulation/workflow_planner/strategy/pick.cpp
  src/manipulation/workflow_planner/strategy/place.cpp
  src/manipulation/workflow_planner/strategy/scan.cpp

  src/manipulation/workflow_planner/action_server/pick.cpp
  src/manipulation/workflow_planner/action_server/place.cpp
  src/manipulation/workflow_planner/action_server/scan_sku.cpp
  src/manipulation/workflow_planner/action_server/replenish.cpp

  src/elevation/fold_elevator_driver.cpp
  src/elevation/impl.cpp
  src/elevation/utils.cpp
)

add_executable(manager 
  ${common_files}
  ${manager_files}
)
target_include_directories(manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(manager
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  std_srvs
  geometry_msgs
  robot_controller_msgs
  robotic_platform_msgs
  robostore_msgs
)
target_compile_features(manager PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(action_planner 
  ${common_files}
  src/manipulation/action_planner/action_planner.cpp
  src/manipulation/action_planner/service_server/plan.cpp
)
target_include_directories(action_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(action_planner
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  std_srvs
  geometry_msgs
  robotic_platform_msgs
  robot_controller_msgs
)
target_compile_features(action_planner PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(platform_server 
  src/platform_server.cpp

  ${common_files}
  ${manipulation_files}
)
target_include_directories(platform_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(platform_server
  rclcpp
  rclcpp_action
  rcl_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  std_srvs
  lifecycle_msgs
  sensor_msgs
  geometry_msgs
  moveit_msgs
  geometric_shapes
  Boost
  robot_controller_msgs
  robotic_platform_msgs
  robostore_msgs
)
target_link_libraries(platform_server 
  ${YAML_CPP_LIBRARIES})
target_compile_features(platform_server PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS 
  manager
  platform_server
  action_planner
  DESTINATION lib/${PROJECT_NAME}
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
