#include "manipulation/motion_planner/motion_planner.hpp"

bool MotionPlanner::try_to_pick(
  RobotArm arm,
  const Pose& pre_pick_pose, 
  const std::vector<Pose>& pick_poses)
{
  switch (arm)
  {
    case RobotArm::LEFT:  
      return try_to_pick_by_vac(RobotArm::LEFT, pre_pick_pose, pick_poses);
    case RobotArm::LEFT_ACTION:  
      return try_to_pick_by_vac(RobotArm::LEFT_ACTION, pre_pick_pose, pick_poses);
    case RobotArm::RIGHT:
      return try_to_pick_by_finger(RobotArm::RIGHT, pre_pick_pose, pick_poses);
    case RobotArm::RIGHT_ACTION:
      return try_to_pick_by_finger(RobotArm::RIGHT_ACTION, pre_pick_pose, pick_poses);
    default:
      return false;
  }
}

bool MotionPlanner::try_to_place(
  RobotArm arm,
  const Pose& pre_pick_pose, 
  const std::vector<Pose>& pick_poses,
  const uint8_t max_retries)
{
  switch (arm)
  {
    case RobotArm::LEFT:
      return try_to_place_by_vac(RobotArm::LEFT, pre_pick_pose, pick_poses, max_retries);
    case RobotArm::LEFT_ACTION:  
      return try_to_place_by_vac(RobotArm::LEFT_ACTION, pre_pick_pose, pick_poses, max_retries);
    case RobotArm::RIGHT:
      return try_to_place_by_finger(RobotArm::RIGHT, pre_pick_pose, pick_poses, max_retries);
    case RobotArm::RIGHT_ACTION:
      return try_to_place_by_finger(RobotArm::RIGHT_ACTION, pre_pick_pose, pick_poses, max_retries);
    default:
      return false;
  }
}

bool MotionPlanner::gripper_action(RobotArm arm, const bool cmd)
{
  switch(arm)
  {
    case RobotArm::LEFT:  
    case RobotArm::LEFT_ACTION:  
      return vac_gripper_->gripper_action(cmd);
    case RobotArm::RIGHT:
    case RobotArm::RIGHT_ACTION:
      return finger_gripper_->gripper_action(100, cmd ? 0.0 : 95.0);
    default:
      return false;
  }
}