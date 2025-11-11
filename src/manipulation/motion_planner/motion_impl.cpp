#include "manipulation/motion_planner/motion_planner.hpp"

bool MotionPlanner::move_to(RobotArm arm, const std::vector<double>& joints, const float speed)
{
  auto request = std::make_shared<ExecuteJoints::Request>();
  request->joints.clear();
  request->joints.insert(request->joints.end(), joints.begin(), joints.end());
  request->speed = speed;

  ExecuteJoints::Response::SharedPtr response;
  if (!send_sync_req<ExecuteJoints>(exec_joints_cli_[arm], std::move(request), response) )
  {
    RCLCPP_ERROR(get_logger(), "Sent ExecuteJoints request failed");
    return false;
  }

  if (!response->success)
  {
    RCLCPP_ERROR(get_logger(), "Unsuccessful: %s", response->message.c_str());
    return false;
  }

  return true;
}

bool MotionPlanner::move_to(RobotArm arm, const Pose& pose, const float speed)
{
  std::vector<Pose> poses{ pose };

  return move_to(arm, poses, speed);
}

bool MotionPlanner::move_to(RobotArm arm, const std::vector<Pose>& waypoints, const float speed)
{
  auto request = std::make_shared<ExecuteWaypoints::Request>();
  
  request->waypoints.clear();
  request->waypoints.insert(request->waypoints.end(), waypoints.begin(), waypoints.end());
  request->speed = speed;

  ExecuteWaypoints::Response::SharedPtr response;
  if (!send_sync_req<ExecuteWaypoints>(exec_wps_cli_[arm], std::move(request), response))
  {
    RCLCPP_ERROR(get_logger(), "Sent ExecuteWaypoints request failed");
    return false;
  }

  if (!response->success)
  {
    RCLCPP_ERROR(get_logger(), "Unsuccessful: %s", response->message.c_str());
    return false;
  }

  return true;
}