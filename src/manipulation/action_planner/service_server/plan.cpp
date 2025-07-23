#include "manipulation/action_planner/action_planner.hpp"

void ActionPlanner::pick_plan_cb(
  const std::shared_ptr<PickPlan::Request> request, 
  std::shared_ptr<PickPlan::Response> response)
{
  PickPlanResult result;

  auto pick_pose = std::make_shared<Pose>(request->object_pose);
  pose_translation(pick_pose, eef_offset_[0], eef_offset_[1], eef_offset_[2]); 
  print_pose(*pick_pose);
  
  auto pre_pick_pose = std::make_shared<Pose>(*pick_pose);
  pose_translation(pre_pick_pose, 0, 0, -pre_obj_pose_shift_);
  result.pre_pick_pose = *pre_pick_pose;

  result.pick_poses.push_back(*pre_pick_pose);
  result.pick_poses.push_back(*pick_pose);
  
  auto retract_pose = std::make_shared<Pose>(*pick_pose);
  pose_translation(retract_pose, -post_pick_lift_offset_, 0, 0);
  auto retract_back_pose = std::make_shared<Pose>(*retract_pose);
  pose_translation(retract_back_pose, 0, 0, -post_pick_back_offset_);

  result.lifted_poses.push_back(*pick_pose);
  result.lifted_poses.push_back(*retract_pose);
  result.lifted_poses.push_back(*retract_back_pose);

  RCLCPP_INFO(get_logger(), "Pre Pick pose:");
  print_pose(result.pre_pick_pose);

  RCLCPP_INFO(get_logger(), "Pick poses:");
  print_pose_arr(result.pick_poses);

  RCLCPP_INFO(get_logger(), "Lefted poses:");
  print_pose_arr(result.lifted_poses);

  response->result = std::move(result);
  RCLCPP_INFO(get_logger(), "Pick plan completed successfully");
  response->success = true;
}

void ActionPlanner::place_plan_cb(
  const std::shared_ptr<PlacePlan::Request> request, 
  std::shared_ptr<PlacePlan::Response> response)
{
  PlacePlanResult result;

  auto place_pose = std::make_shared<Pose>(request->place_pose);

  auto pre_place_pose = std::make_shared<Pose>(request->place_pose);
  pose_translation(pre_place_pose, 0, 0, -pre_place_pose_shift_); 
  print_pose(*pre_place_pose);
  result.pre_place_pose = *pre_place_pose;

  auto place_down_pose = std::make_shared<Pose>(request->place_pose);
  pose_translation(place_down_pose, post_place_down_offset_, 0, 0); 

  result.place_poses.push_back(*pre_place_pose);
  result.place_poses.push_back(*place_pose);
  result.place_poses.push_back(*place_down_pose);

  auto place_down_lift_pose = std::make_shared<Pose>(*place_down_pose);
  pose_translation(place_down_lift_pose, 0, 0, -pre_place_pose_shift_); 
  result.lifted_poses.push_back(*place_down_pose);
  result.lifted_poses.push_back(*place_down_lift_pose);

  RCLCPP_INFO(get_logger(), "Pre Place pose:");
  print_pose(result.pre_place_pose);

  RCLCPP_INFO(get_logger(), "Place poses:");
  print_pose_arr(result.place_poses);

  RCLCPP_INFO(get_logger(), "Lefted poses:");
  print_pose_arr(result.lifted_poses);

  response->result = std::move(result);
  RCLCPP_INFO(get_logger(), "Place plan completed successfully");
  response->success = true;
}