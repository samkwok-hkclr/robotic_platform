#include "manipulation/motion_planner/motion_planner.hpp"

bool MotionPlanner::pick(RobotArm arm, const PickPlanResult& plan, const float speed)
{
  RCLCPP_WARN(get_logger(), "Move to pre-pick pose");
  if (!move_to(arm, plan.pre_pick_pose, speed / 2.0))
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
  RobotArm arm_wo_rototion = arm_remove_rotation(arm);

  if (home_joint_pose_.find(arm_wo_rototion) != home_joint_pose_.end()) 
  {
    return move_to(arm_wo_rototion, home_joint_pose_[arm_wo_rototion], speed);
  }
  
  return false;
}

bool MotionPlanner::move_to_holding_pose(RobotArm arm, const float speed)
{
  RobotArm arm_wo_rototion = arm_remove_rotation(arm);

  if (holding_joint_pose_.find(arm_wo_rototion) != holding_joint_pose_.end()) 
  {
    return move_to(arm_wo_rototion, holding_joint_pose_[arm_wo_rototion], speed);
  }
  
  return false;
}

bool MotionPlanner::move_to_action_pose(RobotArm arm, const float speed)
{
  RobotArm arm_wo_rototion = arm_remove_rotation(arm);

  if (action_joint_pose_.find(arm_wo_rototion) != action_joint_pose_.end()) 
  {
    return move_to(arm_wo_rototion, action_joint_pose_[arm_wo_rototion], speed);
  }
  
  return false;
}

RobotArm MotionPlanner::arm_remove_rotation(RobotArm arm)
{
  RobotArm arm_wo_rototion;

  switch (arm)
  { 
    case RobotArm::LEFT_ACTION:
      arm_wo_rototion = RobotArm::LEFT;
      break;
    case RobotArm::RIGHT_ACTION:
      arm_wo_rototion = RobotArm::RIGHT;
      break;
    default:
      arm_wo_rototion = arm;
      break;
  }

  return arm_wo_rototion;
}