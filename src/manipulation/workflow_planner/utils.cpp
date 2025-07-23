#include "manipulation/workflow_planner/workflow_planner.hpp"

std::optional<robotic_platform_msgs::msg::PickPlanResult> WorkflowPlanner::get_pick_plan(const Pose& object_pose)
{
  auto request = std::make_shared<PickPlan::Request>();
  request->object_pose = object_pose;

  PickPlan::Response::SharedPtr response;
  if (!(send_sync_req<PickPlan>(pick_plan_cli_, std::move(request), response) && response))
  {
    RCLCPP_ERROR(get_logger(), "Sent PickPlan request failed");
    return std::nullopt;
  }

  if (!response->success)
    return std::nullopt;

  return std::make_optional(std::move(response->result));
}

std::optional<robotic_platform_msgs::msg::PlacePlanResult> WorkflowPlanner::get_place_plan(const Pose& place_pose)
{
  auto request = std::make_shared<PlacePlan::Request>();
  request->place_pose = place_pose;

  PlacePlan::Response::SharedPtr response;
  if (!(send_sync_req<PlacePlan>(place_plan_cli_, std::move(request), response) && response))
  {
    RCLCPP_ERROR(get_logger(), "Sent PlacePlan request failed");
    return std::nullopt;
  }

  if (!response->success)
    return std::nullopt;

  return std::make_optional(std::move(response->result));
}


std::optional<geometry_msgs::msg::Pose> WorkflowPlanner::get_curr_pose(const std::string& joint_name)
{
  if (joint_name.empty()) 
  {
    RCLCPP_ERROR(get_logger(), "Joint name is empty!");
    return std::nullopt;
  } 
  
  auto request = std::make_shared<GetCurrentPose::Request>();
  request->joint_name = joint_name;
  GetCurrentPose::Response::SharedPtr response;
  if (!(send_sync_req<GetCurrentPose>(get_curr_pose_cli_, std::move(request), response) && response))
  {
    RCLCPP_ERROR(get_logger(), "Sent GetCurrentPose request failed");
    return std::nullopt;
  }

  if (!response->success)
    return std::nullopt; 

  return std::make_optional(std::move(response->pose));
}

void WorkflowPlanner::clear_workflow_data()
{
  clear_tf_buf();
}

void WorkflowPlanner::push_tf_buf(const std::tuple<Pose, std::string, std::string>& tf)
{
  std::lock_guard<std::mutex> lock(tf_mutex_);

  tf_buf_.emplace_back(tf);
}

void WorkflowPlanner::clear_tf_buf()
{
  std::lock_guard<std::mutex> lock(tf_mutex_);

  tf_buf_.clear();
}

