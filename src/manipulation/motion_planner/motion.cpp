#include "manipulation/motion_planner/motion_planner.hpp"

bool MotionPlanner::pick(const PickPlanResult& plan, const float speed)
{
  if (!move_to(plan.pre_pick_pose, speed))
  {
    return false;
  }

  if (!try_to_pick(plan.pre_pick_pose, plan.pick_poses))
  {
    return false;
  }
  RCLCPP_WARN(get_logger(), "Picked the item");

  if (!move_to(plan.lifted_poses, speed * 0.75f))
  {
    return false;
  }
  RCLCPP_WARN(get_logger(), "Lefted the object pose");

  return true;
}

bool MotionPlanner::move_from_pick_to_place(const float speed)
{
  (void) speed;

  if (!move_to_lifted_pick_poses(speed))
  {
    return false;
  }
  RCLCPP_WARN(get_logger(), "Lifted the pick pose");

  if (!move_to_before_place_poses(speed))
  {
    return false;
  }
  RCLCPP_WARN(get_logger(), "Moved to pre place pose");

  return true;
}

bool MotionPlanner::place(const PlacePlanResult& plan, const float speed)
{
  if (!move_to(plan.pre_place_pose, speed))
  {
    return false;
  }
  RCLCPP_WARN(get_logger(), "Moved to the pre place pose");

  if (!try_to_place(plan.pre_place_pose, plan.place_poses))
  {
    return false;
  }
  RCLCPP_WARN(get_logger(), "Placed the item");

  if (!move_to(plan.lifted_poses, speed * 0.75f))
  {
    return false;
  }
  RCLCPP_WARN(get_logger(), "Lifted the place pose");

  return true;
}

bool MotionPlanner::gripper_action(const bool cmd)
{
  return gripper_->gripper_action(cmd);
}

bool MotionPlanner::move_to_home_pose(const float speed)
{
  return move_to(home_pose_, speed);
}

bool MotionPlanner::move_to_middle_pose(const float speed)
{
  return move_to(middle_pose_, speed);
}

bool MotionPlanner::move_to_scan_pose(const float speed)
{
  return move_to(scan_pose_, speed);
}

bool MotionPlanner::move_to_pre_scan_pose(const float speed)
{
  return move_to(pre_scan_pose_, speed);
}

bool MotionPlanner::move_to_pre_pick_pose(const float speed)
{
  return move_to(pre_pick_pose_, speed);
}

bool MotionPlanner::move_to_pre_place_pose(const float speed)
{
  return move_to(pre_place_pose_, speed);
}

bool MotionPlanner::move_to_before_pick_poses(const float speed)
{
  return move_to(before_pick_waypoints_, speed);
}

bool MotionPlanner::move_to_before_place_poses(const float speed)
{
  return move_to(before_place_waypoints_, speed);
}

bool MotionPlanner::move_to_lifted_pick_poses(const float speed)
{
  return move_to(lifted_pick_waypoints_, speed);
}

bool MotionPlanner::move_to_lifted_place_poses(const float speed)
{
  return move_to(lifted_place_waypoints_, speed);
}