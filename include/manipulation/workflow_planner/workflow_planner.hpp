#ifndef WORKFLOW_PLANNER_HPP__
#define WORKFLOW_PLANNER_HPP__

#pragma once

#include <functional>
#include <future>
#include <memory>
#include <chrono>
#include <algorithm>

#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose.hpp"

#include "robotic_platform_msgs/action/scan_sku.hpp"

#include "robotic_platform_msgs/msg/replenish_pose.hpp"
#include "robotic_platform_msgs/msg/replenish_quantity.hpp"
#include "robotic_platform_msgs/msg/robot_status.hpp"
#include "robotic_platform_msgs/msg/localization_param.hpp"

#include "robotic_platform_msgs/srv/get_slot_state_trigger.hpp"
#include "robotic_platform_msgs/srv/get_object_pose_trigger.hpp"

#include "robot_controller_msgs/srv/execute_waypoints.hpp"
#include "robot_controller_msgs/srv/get_current_pose.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class WorkflowPlanner : public rclcpp::Node
{
  using ExecuteWaypoints = robot_controller_msgs::srv::ExecuteWaypoints;
  using GetCurrentPose = robot_controller_msgs::srv::GetCurrentPose;

  using ReplenishPose = robotic_platform_msgs::msg::ReplenishPose;
  using ReplenishQuantity = robotic_platform_msgs::msg::ReplenishQuantity;
  using RobotStatus = robotic_platform_msgs::msg::RobotStatus;
  using LocalizationParam = robotic_platform_msgs::msg::LocalizationParam;

  using GetSlotStateTrigger = robotic_platform_msgs::srv::GetSlotStateTrigger;
  using GetObjectPoseTrigger = robotic_platform_msgs::srv::GetObjectPoseTrigger;

  using ScanSku = robotic_platform_msgs::action::ScanSku;

  using GoalHandlerScanSku = rclcpp_action::ServerGoalHandle<ScanSku>;
  
public:
  explicit WorkflowPlanner(const rclcpp::NodeOptions& options);
  ~WorkflowPlanner();

  rclcpp_action::GoalResponse scan_sku_goal_cb(
    const rclcpp_action::GoalUUID & uuid, 
    std::shared_ptr<const ScanSku::Goal> goal);
  rclcpp_action::CancelResponse scan_sku_cancel_cb(
    const std::shared_ptr<GoalHandlerScanSku> goal_handle);
  void scan_sku_accepted(const std::shared_ptr<GoalHandlerScanSku> goal_handle);
  void scan_sku_execution(const std::shared_ptr<GoalHandlerScanSku> goal_handle);

  geometry_msgs::msg::Pose compose_pose_msg(const std::vector<double>& pose_vec);
  bool are_poses_equal(
    const geometry_msgs::msg::Pose& pose_1,
    const geometry_msgs::msg::Pose& pose_2, 
    double pos_thd = 1e-5, 
    double ori_thd = 1e-5);

  template <typename T>
  bool send_sync_req(
    typename rclcpp::Client<T>::SharedPtr cli, 
    const typename T::Request::SharedPtr request,
    typename T::Response::SharedPtr& response,
    const std::string srv_name) const;

  template <typename T>
  bool cli_wait_for_srv(
    typename rclcpp::Client<T>::SharedPtr cli, 
    const std::string srv_name) const;

  template <typename T>
  void reset_req_res(
    typename T::Request::SharedPtr request,
    typename T::Response::SharedPtr response) const;

private:
  std::mutex mutex_;

  uint8_t state_ = 0;
  
  rclcpp::CallbackGroup::SharedPtr action_ser_cbg_;
  rclcpp::CallbackGroup::SharedPtr srv_cli_cbg_;
  rclcpp::CallbackGroup::SharedPtr vison_srv_cli_cbg_;
  rclcpp::CallbackGroup::SharedPtr exec_timer_cbg_;

  rclcpp::Client<ExecuteWaypoints>::SharedPtr exec_wps_cli_;
  rclcpp::Client<GetCurrentPose>::SharedPtr get_curr_pose_cli_;
  rclcpp::Client<GetSlotStateTrigger>::SharedPtr get_slot_state_tri_cli_;
  rclcpp::Client<GetObjectPoseTrigger>::SharedPtr get_obj_pose_tri_cli_;


  rclcpp_action::Server<ScanSku>::SharedPtr scan_sku_action_ser_;

  constexpr static uint8_t SRV_CLI_MAX_RETIES = 5;
  constexpr static std::chrono::duration CLI_REQ_TIMEOUT = std::chrono::seconds(5);
};

#endif // WORKFLOW_PLANNER_HPP__