#ifndef MOTION_PLANNER_HPP__
#define MOTION_PLANNER_HPP__

#pragma once

#include <functional>
#include <future>
#include <memory>
#include <chrono>
#include <algorithm>
#include <string_view>
#include <random>

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

#include "robot_controller_msgs/srv/execute_joints.hpp"
#include "robot_controller_msgs/srv/execute_pose.hpp"
#include "robot_controller_msgs/srv/execute_waypoints.hpp"
#include "robot_controller_msgs/srv/get_pose.hpp"
#include "robot_controller_msgs/srv/robot_speed.hpp"

#include "planner_base.hpp"
#include "manipulation/poses_loader.hpp"
#include "manipulation/end_effector_ctrl/vacuum_gripper.hpp"
#include "manipulation/end_effector_ctrl/finger_gripper.hpp"

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

  using GetJointLimits = robot_controller_msgs::srv::GetJointLimits;
  using ExecuteJoints = robot_controller_msgs::srv::ExecuteJoints;
  using ExecutePose = robot_controller_msgs::srv::ExecutePose;
  using ExecuteWaypoints = robot_controller_msgs::srv::ExecuteWaypoints;
  using GetPose = robot_controller_msgs::srv::GetPose;
  using RobotSpeed = robot_controller_msgs::srv::RobotSpeed;

public:
  explicit MotionPlanner(
    const rclcpp::NodeOptions& options,
    std::shared_ptr<VacuumGripper> vac_gripper,
    std::shared_ptr<FingerGripper> finger_gripper);
  ~MotionPlanner();

  
  RobotArm arm_remove_rotation(RobotArm arm);

  bool pick(RobotArm arm, const PickPlanResult& plan, const float speed);
  bool place(RobotArm arm, const PlacePlanResult& plan, const float speed);

  bool move_to_zero_pose(RobotArm arm, const float speed);
  bool move_to_home_pose(RobotArm arm, const float speed);
  bool move_to_holding_pose(RobotArm arm, const float speed);
  bool move_to_action_pose(RobotArm arm, const float speed);

  bool move_to(RobotArm arm, const std::vector<double>& joints, const float speed);
  bool move_to(RobotArm arm, const Pose& pose, const float speed);
  bool move_to(RobotArm arm, const std::vector<Pose>& waypoints, const float speed);

  void testing_cb(
    const std::shared_ptr<Trigger::Request> request, 
    std::shared_ptr<Trigger::Response> response);

  bool try_to_pick(
    RobotArm arm,
    const Pose& pre_pick_pose, 
    const std::vector<Pose>& pick_poses);
  
  // left arm
  bool try_to_pick_by_vac(
    RobotArm arm, 
    const Pose& pre_pick_pose, 
    const std::vector<Pose>& pick_poses, 
    const std::chrono::milliseconds leak_check_duration = std::chrono::milliseconds(500));

  // right arm
  bool try_to_pick_by_finger(
    RobotArm arm, 
    const Pose& pre_pick_pose, 
    const std::vector<Pose>& pick_poses,
    const std::chrono::milliseconds holding_check_duration = std::chrono::milliseconds(500));

  bool try_to_place(
    RobotArm arm,
    const Pose& pre_place_pose, 
    const std::vector<Pose>& place_poses,
    const uint8_t max_retries = 3);

  // left arm
  bool try_to_place_by_vac(
    RobotArm arm, 
    const Pose& pre_place_pose, 
    const std::vector<Pose>& place_poses,
    const uint8_t max_retries);

  // right arm
  bool try_to_place_by_finger(
    RobotArm arm, 
    const Pose& pre_place_pose, 
    const std::vector<Pose>& place_poses,
    const uint8_t max_retries);

  bool gripper_action(RobotArm arm, const bool cmd);

  void move_to_cb(
    const std::shared_ptr<Trigger::Request> request, 
    std::shared_ptr<Trigger::Response> response,
    RobotArm arm,
    std::string_view pose);

  std::chrono::duration<double> get_cli_req_timeout() const override 
  {
    return CLI_REQ_TIMEOUT;
  }

private:
  bool simulation_;
  bool enable_ultrasonic_;
  
  std::mutex mutex_;

  uint16_t max_pick_attempt_;
  
  std::shared_ptr<VacuumGripper> vac_gripper_;
  std::shared_ptr<FingerGripper> finger_gripper_;

  std::map<RobotArm, std::vector<double>> zero_joint_pose_;
  std::map<RobotArm, std::vector<double>> home_joint_pose_;
  std::map<RobotArm, std::vector<double>> holding_joint_pose_;
  std::map<RobotArm, std::vector<double>> action_joint_pose_;

  std::map<RobotArm, std::map<std::string_view, std::function<bool(RobotArm, float)>, std::less<>>> pose_actions;
  
  // ============== Callback Groups ==============

  rclcpp::CallbackGroup::SharedPtr srv_ser_cbg_;
  rclcpp::CallbackGroup::SharedPtr exec_srv_cli_cbg_;

  // ============== Services Clients ==============

  std::map<RobotArm, rclcpp::Client<GetJointLimits>::SharedPtr>  get_joint_limits_cli_;

  std::map<RobotArm, rclcpp::Client<ExecuteJoints>::SharedPtr> exec_joints_cli_;
  std::map<RobotArm, rclcpp::Client<ExecutePose>::SharedPtr> exec_pose_cli_;
  std::map<RobotArm, rclcpp::Client<ExecuteWaypoints>::SharedPtr> exec_wps_cli_;

  // ============== Services Servers ==============

  rclcpp::Service<Trigger>::SharedPtr testing_srv_;
  std::vector<rclcpp::Service<Trigger>::SharedPtr> move_to_srv_;

  constexpr static std::chrono::duration CLI_REQ_TIMEOUT = std::chrono::seconds(30);
};

#endif // MOTION_PLANNER_HPP__