#ifndef MOTION_PLANNER_HPP__
#define MOTION_PLANNER_HPP__

#pragma once

#include <functional>
#include <future>
#include <memory>
#include <chrono>
#include <algorithm>
#include <string_view>

#include <yaml-cpp/yaml.h>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "sensor_msgs/msg/range.hpp"

#include "geometry_msgs/msg/pose.hpp"

#include "robotic_platform_msgs/msg/pick_plan_result.hpp"
#include "robotic_platform_msgs/msg/place_plan_result.hpp"

#include "robot_controller_msgs/srv/execute_waypoints.hpp"
#include "robot_controller_msgs/srv/get_current_pose.hpp"
#include "robot_controller_msgs/srv/robot_speed.hpp"

#include "planner_base.hpp"
#include "manipulation/poses_loader.hpp"
#include "manipulation/end_effector_ctrl/vacuum_gripper_ctlr.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class MotionPlanner : public PlannerBase
{
  using Empty = std_msgs::msg::Empty;

  using SetBool = std_srvs::srv::SetBool;
  using Trigger = std_srvs::srv::Trigger;

  using Pose = geometry_msgs::msg::Pose;
  using Range = sensor_msgs::msg::Range;

  using PickPlanResult = robotic_platform_msgs::msg::PickPlanResult;
  using PlacePlanResult = robotic_platform_msgs::msg::PlacePlanResult;

  using ExecuteWaypoints = robot_controller_msgs::srv::ExecuteWaypoints;
  using GetCurrentPose = robot_controller_msgs::srv::GetCurrentPose;
  using RobotSpeed = robot_controller_msgs::srv::RobotSpeed;

public:
  explicit MotionPlanner(
    const rclcpp::NodeOptions& options,
    std::shared_ptr<VacuumGripperCtlr> gripper);
  ~MotionPlanner();

  bool pick(const PickPlanResult& plan, const float speed);
  bool place(const PlacePlanResult& plan, const float speed);
  bool move_from_pick_to_place(const float speed);

  bool move_to_home_pose(const float speed);
  bool move_to_middle_pose(const float speed);
  bool move_to_scan_pose(const float speed);
  bool move_to_pre_scan_pose(const float speed);
  bool move_to_before_pick_poses(const float speed);
  bool move_to_before_place_poses(const float speed);
  bool move_to_lifted_pick_poses(const float speed);
  bool move_to_lifted_place_poses(const float speed);
  bool move_to_pre_pick_pose(const float speed);
  bool move_to_pre_place_pose(const float speed);
  bool move_to(const Pose& waypoint, const float speed);
  bool move_to(const std::vector<Pose>& waypoints, const float speed);

  bool try_to_pick(
    const Pose& pre_pick_pose, 
    const std::vector<Pose>& pick_poses, 
    const std::chrono::milliseconds leak_check_duration = std::chrono::milliseconds(500));
  bool try_to_place(
    const Pose& pre_place_pose, 
    const std::vector<Pose>& place_poses,
    const uint8_t max_retries = 3);

  bool gripper_action(const bool cmd);

  void move_to_cb(
    const std::shared_ptr<Trigger::Request> request, 
    std::shared_ptr<Trigger::Response> response,
    std::string_view pose);

  std::chrono::duration<double> get_cli_req_timeout() const override 
  {
    return CLI_REQ_TIMEOUT;
  }

private:
  std::mutex mutex_;
  uint16_t max_pick_attempt_;
  
  std::shared_ptr<VacuumGripperCtlr> gripper_;

  Pose home_pose_;
  Pose middle_pose_;
  Pose scan_pose_;
  Pose pre_scan_pose_;

  Pose pre_pick_pose_;
  Pose pre_place_pose_;

  std::vector<Pose> before_pick_waypoints_; 
  std::vector<Pose> lifted_pick_waypoints_;
  std::vector<Pose> before_place_waypoints_; 
  std::vector<Pose> lifted_place_waypoints_;

  std::vector<std::string> move_to_srv_names_;
  std::map<std::string_view, std::function<bool(float)>, std::less<>> pose_actions;
  
  rclcpp::CallbackGroup::SharedPtr srv_ser_cbg_;
  rclcpp::CallbackGroup::SharedPtr exec_srv_cli_cbg_;

  rclcpp::Client<ExecuteWaypoints>::SharedPtr exec_wps_cli_;

  // ============== Services ==============
  std::vector<rclcpp::Service<Trigger>::SharedPtr> move_to_srv_;

  constexpr static std::chrono::duration CLI_REQ_TIMEOUT = std::chrono::seconds(60);
};

#endif // MOTION_PLANNER_HPP__