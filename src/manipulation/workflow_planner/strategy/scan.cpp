#include "manipulation/workflow_planner/workflow_planner.hpp"

std::optional<std::vector<robotic_platform_msgs::msg::ObjectPose>> WorkflowPlanner::try_to_scan(
  const uint32_t sku_id, 
  const uint8_t camera_id)
{
  auto scan_and_check = [&](uint32_t attempt_num) -> std::optional<std::vector<ObjectPose>> {
    // wait for image streaming become stable
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::optional<std::vector<ObjectPose>> obj_poses = get_obj_poses(sku_id, camera_id);
    
    if (!obj_poses.has_value()) 
    {
      RCLCPP_INFO(get_logger(), "attempt [%d]: have no value", attempt_num);
      return std::nullopt;
    }
    
    std::vector<ObjectPose>& poses_in_camera = obj_poses.value();
    RCLCPP_INFO(get_logger(), "attempt [%d]: pose length: %ld", attempt_num, poses_in_camera.size());
    
    if (poses_in_camera.size() > 0) 
    {
      bool valid = std::any_of(poses_in_camera.begin(), poses_in_camera.end(), 
        [this](const ObjectPose& obj_pose) {
          return obj_pose.pose.position.z >= valid_z_threshold_;
        });

      if (valid)
      {
        RCLCPP_INFO(get_logger(), "attempt [%d]: object_poses ok", attempt_num);
        return obj_poses;
      }
    }
    
    return std::nullopt;
  };

  // 1st attempt
  if (scan_poses_.find(sku_id) != scan_poses_.end())
  {
    if (!motion_planner_->move_to(scan_poses_[sku_id], 20.0f)) 
    {
      RCLCPP_ERROR(get_logger(), "Failed to move to SKU [%d] position", sku_id);
      return std::nullopt;
    }

    if (auto result = scan_and_check(1); result.has_value()) 
    {
      return result;
    }
  }
  else if (auto result = scan_and_check(1); result.has_value()) 
  {
    return result;
  }

  // Get current pose for offset calculations
  std::optional<Pose> curr_pose = get_curr_pose("tcp");
  if (!curr_pose.has_value()) 
  {
    RCLCPP_ERROR(get_logger(), "Failed to get current pose");
    return std::nullopt;
  }

  tf2::Transform g_b__tcp = get_g(curr_pose.value()); 

  for (uint16_t i = 0; i < max_scan_attempt_; i++) 
  {
    float offset = ((i % 2) * 2 - 1) * ((i / 2) + 1) * re_scan_translation_;
    RCLCPP_WARN(get_logger(), "translate offset: %.4f", offset);

    tf2::Transform g_tcp__tcp_offset = get_g(0, offset, 0, 0, 0, 0);
    tf2::Transform g_b__tcp_offset = g_b__tcp * g_tcp__tcp_offset;
    
    Pose p = cvt_g_to_pose(g_b__tcp_offset);
    
    if (!motion_planner_->move_to(p, 20.0f)) 
    {
      RCLCPP_ERROR(get_logger(), "Failed to move to offset position");
      continue; // Try next offset instead of giving up
    }
    
    // +2 because attempt 1 already done
    if (auto result = scan_and_check(i + 2); result.has_value()) 
    { 
      return result;
    }
  }
  
  RCLCPP_ERROR(get_logger(), "Failed to scan <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
  return std::nullopt;
}

