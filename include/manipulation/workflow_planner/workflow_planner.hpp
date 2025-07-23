#ifndef WORKFLOW_PLANNER_HPP__
#define WORKFLOW_PLANNER_HPP__

#pragma once

#include <functional>
#include <future>
#include <memory>
#include <chrono>
#include <algorithm>
#include <tuple>
#include <utility>
#include <unordered_map>

// #include <yaml-cpp/yaml.h>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose.hpp"

#include "robotic_platform_msgs/action/scan_sku.hpp"
#include "robotic_platform_msgs/action/replenish.hpp"

#include "robotic_platform_msgs/msg/pick_plan_result.hpp"
#include "robotic_platform_msgs/msg/place_plan_result.hpp"
#include "robotic_platform_msgs/msg/replenish_pose.hpp"
#include "robotic_platform_msgs/msg/replenish_quantity.hpp"
#include "robotic_platform_msgs/msg/robot_status.hpp"
#include "robotic_platform_msgs/msg/localization_param.hpp"

#include "robotic_platform_msgs/srv/get_slot_state_trigger.hpp"
#include "robotic_platform_msgs/srv/get_object_pose_trigger.hpp"
#include "robotic_platform_msgs/srv/pick_plan.hpp"
#include "robotic_platform_msgs/srv/place_plan.hpp"

#include "robot_controller_msgs/srv/get_current_pose.hpp"

#include "manipulation/planner_base.hpp"
#include "manipulation/poses_loader.hpp"
#include "manipulation/motion_planner/motion_planner.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class WorkflowPlanner : public PlannerBase
{
  using Pose = geometry_msgs::msg::Pose;

  using GetCurrentPose = robot_controller_msgs::srv::GetCurrentPose;

  using PickPlanResult = robotic_platform_msgs::msg::PickPlanResult;
  using PlacePlanResult = robotic_platform_msgs::msg::PlacePlanResult;
  using ReplenishPose = robotic_platform_msgs::msg::ReplenishPose;
  using ReplenishQuantity = robotic_platform_msgs::msg::ReplenishQuantity;
  using RobotStatus = robotic_platform_msgs::msg::RobotStatus;
  using LocalizationParam = robotic_platform_msgs::msg::LocalizationParam;

  using GetSlotStateTrigger = robotic_platform_msgs::srv::GetSlotStateTrigger;
  using GetObjectPoseTrigger = robotic_platform_msgs::srv::GetObjectPoseTrigger;
  using PickPlan = robotic_platform_msgs::srv::PickPlan;
  using PlacePlan = robotic_platform_msgs::srv::PlacePlan;

  using ScanSku = robotic_platform_msgs::action::ScanSku;
  using Replenish = robotic_platform_msgs::action::Replenish;

  using GoalHandlerScanSku = rclcpp_action::ServerGoalHandle<ScanSku>;
  using GoalHandlerReplenish = rclcpp_action::ServerGoalHandle<Replenish>;
  
public:
  explicit WorkflowPlanner(
    const rclcpp::NodeOptions& options,
    std::shared_ptr<MotionPlanner> motion_planner);
  ~WorkflowPlanner();

  void clear_workflow_data();
  void push_tf_buf(const std::tuple<Pose, std::string, std::string>& tf);
  void clear_tf_buf();

  std::optional<PickPlanResult> get_pick_plan(const Pose& object_pose);
  std::optional<PlacePlanResult> get_place_plan(const Pose& place_pose);
  std::optional<Pose> get_curr_pose(const std::string& joint_name);

  void tf_pub_cb(void);

  rclcpp_action::GoalResponse scan_sku_goal_cb(
    const rclcpp_action::GoalUUID & uuid, 
    std::shared_ptr<const ScanSku::Goal> goal);
  rclcpp_action::CancelResponse scan_sku_cancel_cb(
    const std::shared_ptr<GoalHandlerScanSku> goal_handle);
  void scan_sku_accepted(const std::shared_ptr<GoalHandlerScanSku> goal_handle);
  void scan_sku_execution(const std::shared_ptr<GoalHandlerScanSku> goal_handle);

  rclcpp_action::GoalResponse replenish_goal_cb(
    const rclcpp_action::GoalUUID & uuid, 
    std::shared_ptr<const Replenish::Goal> goal);
  rclcpp_action::CancelResponse replenish_cancel_cb(
    const std::shared_ptr<GoalHandlerReplenish> goal_handle);
  void replenish_accepted(const std::shared_ptr<GoalHandlerReplenish> goal_handle);
  void replenish_execution(const std::shared_ptr<GoalHandlerReplenish> goal_handle);

private:
  std::mutex mutex_;
  std::mutex tf_mutex_;

  std::unordered_map<int, Pose> scan_poses_; // sku_id, pose
  std::vector<std::pair<int, int>> scan_order_; // order, sku_id
  std::unordered_map<int, Pose> place_poses_; // sku_id, pose

  std::shared_ptr<MotionPlanner> motion_planner_;
  
  uint8_t state_ = 0; // FIXME

  std::vector<std::tuple<Pose, std::string, std::string>> tf_buf_;
  
  rclcpp::CallbackGroup::SharedPtr action_ser_cbg_;
  rclcpp::CallbackGroup::SharedPtr vision_srv_cli_cbg_;
  rclcpp::CallbackGroup::SharedPtr exec_srv_cli_cbg_;
  rclcpp::CallbackGroup::SharedPtr plan_srv_cli_cbg_;
  rclcpp::CallbackGroup::SharedPtr exec_timer_cbg_;
  rclcpp::CallbackGroup::SharedPtr tf_timer_cbg_;
  rclcpp::CallbackGroup::SharedPtr srv_ser_cbg_;

  rclcpp::TimerBase::SharedPtr tf_pub_timer;

  rclcpp::Client<GetCurrentPose>::SharedPtr get_curr_pose_cli_;
  rclcpp::Client<GetSlotStateTrigger>::SharedPtr get_slot_state_tri_cli_;
  rclcpp::Client<GetObjectPoseTrigger>::SharedPtr get_obj_pose_tri_cli_;

  rclcpp::Client<PickPlan>::SharedPtr pick_plan_cli_;
  rclcpp::Client<PlacePlan>::SharedPtr place_plan_cli_;

  rclcpp_action::Server<ScanSku>::SharedPtr scan_sku_action_ser_;
  rclcpp_action::Server<Replenish>::SharedPtr replenish_action_ser_;

};

#endif // WORKFLOW_PLANNER_HPP__