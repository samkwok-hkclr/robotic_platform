#include "manipulation/motion_planner/motion_planner.hpp"

bool MotionPlanner::try_to_pick(
  RobotArm arm,
  const Pose& pre_pick_pose, 
  const std::vector<Pose>& pick_poses)
{
  switch (arm)
  {
    case RobotArm::LEFT:  
      return try_to_pick_by_vac(pre_pick_pose, pick_poses);
    case RobotArm::RIGHT:
      return false;
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
      return try_to_place_by_vac(pre_pick_pose, pick_poses, max_retries);
    case RobotArm::RIGHT:
      return false;
    default:
      return false;
  }
}

bool MotionPlanner::gripper_action(RobotArm arm, const bool cmd)
{
  switch(arm)
  {
    case RobotArm::LEFT:  
      return vac_gripper_->gripper_action(cmd);
    case RobotArm::RIGHT:
      return false;
    default:
      return false;
  }
}