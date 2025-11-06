#include "elevation/fold_elevator_driver.hpp"

bool FoldElevatorDriver::elevate(double x, double z, double yaw)
{
  auto stamped_opt = get_tf(BASE_FOOTPRINT, ELEV_FLAT_LINK);

  if (!stamped_opt.has_value()) 
  {
    RCLCPP_WARN(get_logger(), "TF lookup failed: %s -> %s", BASE_FOOTPRINT.c_str(), ELEV_FLAT_LINK.c_str());
    return false;
  }

  const auto& stamped = stamped_opt.value();
  tf2::Transform g_base__link4;
  tf2::fromMsg(stamped.transform, g_base__link4);
  print_g(g_base__link4);

  g_base__link4.setRotation(clean_quat(g_base__link4.getRotation(), 1e-2));
  print_g(g_base__link4);

  auto try_yaw = [&](double sign) -> bool {
    tf2::Transform g_link4__target = get_g(x, 0, z, 0, 0, sign * yaw);
    print_g(g_link4__target);

    tf2::Transform g_base__target = g_base__link4 * g_link4__target;

    Pose p = cvt_g_to_pose(g_base__target);

    std::vector<Pose> wps;
    wps.push_back(p);

    return exec_wps(std::move(wps));
  };

  if (!try_yaw(+1.0)) 
  {
    if (!try_yaw(-1.0)) 
    {
      RCLCPP_ERROR(get_logger(), "Both +yaw and -yaw failed. giving up");
      return false;
    }
  }

  RCLCPP_INFO(get_logger(), "Successfully elevated fold elevator");
  return true;
}

bool FoldElevatorDriver::rotate_by_joints(double yaw, std::string* message)
{
  auto req_lim = std::make_shared<GetJointLimits::Request>();

  GetJointLimits::Response::SharedPtr res_lim;
  if (!send_sync_req<GetJointLimits>(get_joint_limits_cli_, std::move(req_lim), res_lim, __FUNCTION__))
  {
    RCLCPP_ERROR(get_logger(), "Sent ExecuteJoints request failed");
    return false;
  }

  if (!res_lim->success) 
  {
    RCLCPP_ERROR(get_logger(), "ExecuteJoints rejected rotation: %s", res_lim->message.c_str());
    return false;
  }

  std::unordered_map<std::string, const moveit_msgs::msg::JointLimits*> limits_map;
  for (const auto& limit : res_lim->joint_limits)
  {
    limits_map[limit.joint_name] = &limit;
  }

  auto req = std::make_shared<ExecuteJoints::Request>();

  std::ostringstream debug_stream;
  debug_stream << std::fixed << std::setprecision(6);

  {
    std::lock_guard<std::mutex> lock(joint_states_mutex_);
    for (const auto& k : keys)
    {
      double value = joint_states_[k];

      if (k == ELEV_FLAT_JOINT)
        value += yaw;
      
      auto it = limits_map.find(k);
      if (it == limits_map.end())
      {
        RCLCPP_ERROR(get_logger(), "Joint %s does not found!", k.c_str());
        return false;
      }

      const auto& limit = *it->second;
      if (limit.has_position_limits)
      {
        if (value < limit.min_position || value > limit.max_position)
        {
          if (message != nullptr)
            *message = "Joint value is outside limits";
          RCLCPP_ERROR(get_logger(), "Joint %s value %.6f is outside limits [%.6f, %.6f]", k.c_str(), value, limit.min_position, limit.max_position);
          return false;
        }
      }
      
      req->joints.push_back(value);
      debug_stream << k << ": " << value << "  ";
    }
  }

  RCLCPP_WARN(get_logger(), "Sending ExecuteJoints with %zu joints -> %s", req->joints.size(), debug_stream.str().c_str());

  ExecuteJoints::Response::SharedPtr res;
  if (!send_sync_req<ExecuteJoints>(exec_joints_cli_, std::move(req), res, __FUNCTION__))
  {
    RCLCPP_ERROR(get_logger(), "Sent ExecuteJoints request failed");
    return false;
  }

  if (!res->success) 
  {
    RCLCPP_ERROR(get_logger(), "ExecuteJoints rejected rotation: %s", res->message.c_str());
    return false;
  }

  return true;
}

bool FoldElevatorDriver::rotate_by_pose(double yaw, std::string* message)
{
  auto stamped_opt = get_tf(BASE_FOOTPRINT, ELEV_FLAT_LINK);

  if (!stamped_opt.has_value()) 
  {
    *message = "TF lookup failed";
    RCLCPP_WARN(get_logger(), "TF lookup failed: %s -> %s", BASE_FOOTPRINT.c_str(), ELEV_FLAT_LINK.c_str());
    return false;
  }

  const auto& stamped = stamped_opt.value();
  tf2::Transform g_base__link4;
  tf2::fromMsg(stamped.transform, g_base__link4);
  print_g(g_base__link4);

  tf2::Transform g_link4__link4_rot = get_g(0, 0, 0, 0, 0, yaw);
  print_g(g_link4__link4_rot);

  g_base__link4.setRotation(clean_quat(g_base__link4.getRotation(), 1e-2));
  print_g(g_base__link4);
  
  tf2::Transform g_base__link4_rot = g_base__link4 * g_link4__link4_rot;

  auto req = std::make_shared<ExecutePose::Request>();
  req->pose = cvt_g_to_pose(g_base__link4_rot);
  print_pose(req->pose);

  ExecutePose::Response::SharedPtr res;
  if (!send_sync_req<ExecutePose>(exec_pose_cli_, req, res, __FUNCTION__))
  {
    RCLCPP_ERROR(get_logger(), "Sent ExecutePose request failed");
    return false;
  }

  if (!res->success) 
  {
    RCLCPP_ERROR(get_logger(), "ExecutePose rejected rotation: %s", res->message.c_str());
    return false;
  }

  RCLCPP_INFO(get_logger(), "Successfully rotated fold elevator by %.6f rad (%.1f°)", yaw, yaw * 180.0 / M_PI);
  return true;
} 


bool FoldElevatorDriver::rotate_relative(
  double primary_delta,
  double fallback_delta,
  const std::string& direction_name)
{
  auto req = std::make_shared<GetJointLimits::Request>();
  GetJointLimits::Response::SharedPtr res;

  if (!send_sync_req<GetJointLimits>(get_joint_limits_cli_, std::move(req), res, __FUNCTION__))
  {
    RCLCPP_ERROR(get_logger(), "Failed to get joint limits for %s rotation", direction_name.c_str());
    return false;
  }

  if (!res->success)
  {
    RCLCPP_ERROR(get_logger(), "Joint limits service rejected %s rotation: %s",
      direction_name.c_str(), res->message.c_str());
    return false;
  }

  std::unordered_map<std::string, const moveit_msgs::msg::JointLimits*> limits_map;
  for (const auto& limit : res->joint_limits)
  {
    limits_map[limit.joint_name] = &limit;
  }

  auto it = limits_map.find(ELEV_FLAT_JOINT);
  if (it == limits_map.end())
  {
    RCLCPP_ERROR(get_logger(), "Joint '%s' not found in limits", ELEV_FLAT_JOINT.c_str());
    return false;
  }

  const auto& limit = *it->second;
  const double current_yaw = get_last_joint_diff();

  double target_yaw = normalize_angle(current_yaw + primary_delta);

  auto try_rotation = [&](double delta, double yaw, const std::string& label) -> bool {
    if (!limit.has_position_limits || (yaw > limit.min_position && yaw < limit.max_position))
    {
      RCLCPP_INFO(get_logger(), "Rotating %s: %.1f degrees (target yaw: %.6f) within limits",
        direction_name.c_str(), delta * 180.0 / M_PI, yaw);
      return rotate(delta);
    }
    else
    {
      RCLCPP_WARN(get_logger(), "%s rotation (%.1f degrees) outside limits [%.6f, %.6f]",
        label.c_str(), delta * 180.0 / M_PI, limit.min_position, limit.max_position);
      return false;
    }
  };
  
  if (try_rotation(primary_delta, target_yaw, "Primary"))
  {
    return true;
  }

  if (std::abs(fallback_delta) > 0.0)
  {
    double alt_yaw = normalize_angle(current_yaw + fallback_delta);
    RCLCPP_WARN(get_logger(), "Trying fallback %s rotation: %.1f degrees", direction_name.c_str(), fallback_delta * 180.0 / M_PI);

    if (try_rotation(fallback_delta, alt_yaw, "Fallback"))
    {
      return true;
    }

    RCLCPP_ERROR(get_logger(), "Both primary and fallback rotations for %s failed.", direction_name.c_str());
    return false;
  }

  RCLCPP_ERROR(get_logger(), "Primary rotations for %s failed.", direction_name.c_str());
  return false;
}

tf2::Quaternion FoldElevatorDriver::clean_quat(const tf2::Quaternion& quat_in, double threshold)
{
  tf2::Quaternion quat = quat_in;

  if (!quat.normalized()) 
    quat.normalize();
  
  if (std::abs(quat.x()) < threshold && 
      std::abs(quat.y()) < threshold && 
      std::abs(quat.z()) < threshold && 
      std::abs(quat.w() - 1.0) < threshold) 
  {
    quat.setX(0.0);
    quat.setY(0.0);
    quat.setZ(0.0);
    quat.setW(1.0);
  }

  return quat;
}