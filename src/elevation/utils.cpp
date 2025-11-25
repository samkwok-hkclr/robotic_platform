#include "elevation/fold_elevator_driver.hpp"

bool FoldElevatorDriver::rotate(double yaw, std::string* message)
{
  if (rotate_by_joints(yaw, message))
  {
    return true;
  }
  else if (rotate_by_pose(yaw, message))
  {
    return true;
  }

  return false;
}

bool FoldElevatorDriver::move_to_home_joint(void)
{
  auto req = std::make_shared<ExecuteJoints::Request>();
  req->joints.insert(req->joints.end(), home_joints_.begin(), home_joints_.end());

  ExecuteJoints::Response::SharedPtr res;
  if (!send_sync_req<ExecuteJoints>(exec_joints_cli_, std::move(req), res, __FUNCTION__)) 
  {
    RCLCPP_ERROR(get_logger(), "Sent ExecuteJoints request failed");
    return false;
  }

  if (!res->success) 
  {
    RCLCPP_WARN(get_logger(), "ExecuteJoints rejected");
    return false;
  }

  RCLCPP_INFO(get_logger(), "Successfully elevated fold elevator");
  return true;
}

double FoldElevatorDriver::get_last_joint_diff(void)
{
  std::lock_guard<std::mutex> lock(joint_states_mutex_);

  return joint_states_[ELEV_FLAT_JOINT];
}

double FoldElevatorDriver::normalize_angle(double angle)
{
  while (angle > M_PI) 
    angle -= 2.0 * M_PI;
  while (angle < -M_PI) 
    angle += 2.0 * M_PI;

  return angle;
}

double FoldElevatorDriver::shortest_angular_distance(double from, double to)
{
  double diff = to - from;
  return normalize_angle(diff);
}

bool FoldElevatorDriver::rotate_to(RotationDirection direction)
{
  double yaw = -1.0 * get_last_joint_diff();

  switch (direction) 
  {
    case RotationDirection::FRONT:
      return rotate(yaw);
    case RotationDirection::LEFT:
      return rotate(yaw + M_PI / 2.0);
    case RotationDirection::RIGHT:
      return rotate(yaw - M_PI / 2.0);
    default:
      RCLCPP_ERROR(get_logger(), "Unknown rotation direction");
      return false;
  }
}

bool FoldElevatorDriver::rotate_to_abs_front(void) 
{
  return rotate_to(RotationDirection::FRONT);
}

bool FoldElevatorDriver::rotate_to_abs_left(void) 
{
  return rotate_to(RotationDirection::LEFT);
}

bool FoldElevatorDriver::rotate_to_abs_right(void) 
{
  return rotate_to(RotationDirection::RIGHT);
}

bool FoldElevatorDriver::rotate_to_abs_back(void)
{
  double yaw = -1.0 * get_last_joint_diff();
  
  double back_clockwise = yaw + M_PI;      
  double back_counterclockwise = yaw - M_PI;

  back_clockwise = normalize_angle(back_clockwise);
  back_counterclockwise = normalize_angle(back_counterclockwise);
  
  double diff_clockwise = shortest_angular_distance(yaw, back_clockwise);
  double diff_counterclockwise = shortest_angular_distance(yaw, back_counterclockwise);
  
  if (std::abs(diff_clockwise) <= std::abs(diff_counterclockwise)) 
  {
    return rotate(back_clockwise);
  } 
  else 
  {
    return rotate(back_counterclockwise);
  }
}

bool FoldElevatorDriver::rotate_to_relative_back(void)
{
  return rotate_relative(M_PI, -M_PI, "back");
}

bool FoldElevatorDriver::rotate_to_relative_left(void)
{
  return rotate_relative(M_PI_2, -3 * M_PI_2, "left");
}

bool FoldElevatorDriver::rotate_to_relative_right(void)
{
  return rotate_relative(-M_PI_2, 3 * M_PI_2, "right");
}