#include "manipulation/motion_planner/motion_planner.hpp"

bool MotionPlanner::pick(RobotArm arm, const PickPlanResult& plan, const float speed)
{
  RCLCPP_WARN(get_logger(), "Move to pre-pick pose");
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
  RobotArm arm_wo_rotation = arm_remove_rotation(arm);

  if (home_joint_pose_.find(arm_wo_rotation) != home_joint_pose_.end()) 
  {
    return move_to(arm_wo_rotation, home_joint_pose_[arm_wo_rotation], speed);
  }
  
  return false;
}

bool MotionPlanner::move_to_holding_pose(RobotArm arm, const float speed)
{
  RobotArm arm_wo_rotation = arm_remove_rotation(arm);

  auto move_to_holding_joint = [this](RobotArm arm_wo_rotation, float speed) -> bool {
    if (holding_joint_pose_.find(arm_wo_rotation) != holding_joint_pose_.end()) 
    {
      bool success = move_to(arm_wo_rotation, holding_joint_pose_[arm_wo_rotation], speed);
      return success;
    }

    return false;
  };
  
  auto tf_stamped = get_tf(MAP_FRAME, BASE_FOOTPRINT);
  if (!tf_stamped.has_value())
  { 
    return move_to_holding_joint(arm_wo_rotation, speed);
  }

  tf2::Transform g_m__b;
  tf2::fromMsg(tf_stamped.value().transform, g_m__b);

  tf_stamped = get_tf(MAP_FRAME, ELEV_FLAT_LINK);
  if (!tf_stamped.has_value())
  { 
    return move_to_holding_joint(arm_wo_rotation, speed);
  }

  tf2::Transform g_m__ef;
  tf2::fromMsg(tf_stamped.value().transform, g_m__ef);

  const double x = 0.15;
  const double y = 0.15;
  const double z = -0.05;
  const double roll = 0.0;
  const double pitch = -M_PI / 2.0;
  const double yaw = M_PI;

  tf2::Transform g_ef__h;

  if (arm_wo_rotation == RobotArm::LEFT)
    g_ef__h = get_g(x, y, z, roll, pitch, yaw);
  else
    g_ef__h = get_g(x, -y, z, roll, pitch, yaw);

  tf2::Transform g_b__h = g_m__b.inverse() * g_m__ef * g_ef__h;
  Pose pose = cvt_g_to_pose(g_b__h);
  bool success = move_to(arm_wo_rotation, pose, speed);

  if (success) 
  {
    RCLCPP_INFO(get_logger(), "Move to holding pose by waypoint!");
    return true;
  }
  else
    return move_to_holding_joint(arm_wo_rotation, speed);
}

bool MotionPlanner::move_to_action_pose(RobotArm arm, const float speed)
{
  RobotArm arm_wo_rotation = arm_remove_rotation(arm);

  if (action_joint_pose_.find(arm_wo_rotation) != action_joint_pose_.end()) 
  {
    return move_to(arm_wo_rotation, action_joint_pose_[arm_wo_rotation], speed);
  }
  
  return false;
}

RobotArm MotionPlanner::arm_remove_rotation(RobotArm arm)
{
  RobotArm arm_wo_rotation;

  switch (arm)
  { 
    case RobotArm::LEFT_ACTION:
      arm_wo_rotation = RobotArm::LEFT;
      break;
    case RobotArm::RIGHT_ACTION:
      arm_wo_rotation = RobotArm::RIGHT;
      break;
    default:
      arm_wo_rotation = arm;
      break;
  }

  return arm_wo_rotation;
}