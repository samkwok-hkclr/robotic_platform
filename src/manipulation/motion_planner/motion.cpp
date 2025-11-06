#include "manipulation/motion_planner/motion_planner.hpp"

bool MotionPlanner::pick(RobotArm arm, const PickPlanResult& plan, const float speed)
{
  if (!move_to(arm, plan.pre_pick_pose, speed))
  {
    return false;
  }

  if (!try_to_pick(arm, plan.pre_pick_pose, plan.pick_poses))
  {
    return false;
  }
  RCLCPP_WARN(get_logger(), "Picked the item");

  if (!move_to(arm, plan.lifted_poses, speed))
  {
    return false;
  }
  RCLCPP_WARN(get_logger(), "Lefted the object pose");

  return true;
}

bool MotionPlanner::place(RobotArm arm, const PlacePlanResult& plan, const float speed)
{
  if (!move_to(arm, plan.pre_place_pose, speed))
  {
    return false;
  }
  RCLCPP_WARN(get_logger(), "Moved to the pre place pose");

  if (!try_to_place(arm, plan.pre_place_pose, plan.place_poses))
  {
    return false;
  }
  RCLCPP_WARN(get_logger(), "Placed the item");

  if (!move_to(arm, plan.lifted_poses, speed))
  {
    return false;
  }
  RCLCPP_WARN(get_logger(), "Lifted the place pose");

  return true;
}

bool MotionPlanner::move_to_zero_pose(RobotArm arm, const float speed)
{
  if (zero_joint_pose_.find(arm) != zero_joint_pose_.end()) 
  {
    return move_to(arm, zero_joint_pose_[arm], speed);
  }
  
  return false;
}


bool MotionPlanner::move_to_home_pose(RobotArm arm, const float speed)
{
  if (home_joint_pose_.find(arm) != home_joint_pose_.end()) 
  {
    return move_to(arm, home_joint_pose_[arm], speed);
  }
  
  return false;
}

bool MotionPlanner::move_to_holding_pose(RobotArm arm, const float speed)
{
  if (holding_joint_pose_.find(arm) != holding_joint_pose_.end()) 
  {
    return move_to(arm, holding_joint_pose_[arm], speed);
  }
  
  return false;
}

bool MotionPlanner::move_to_action_pose(RobotArm arm, const float speed)
{
  if (action_joint_pose_.find(arm) != action_joint_pose_.end()) 
  {
    return move_to(arm, action_joint_pose_[arm], speed);
  }
  
  return false;
}
