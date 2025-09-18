#include "manipulation/workflow_planner/workflow_planner.hpp"

std::optional<std::vector<robotic_platform_msgs::msg::ObjectPose>> WorkflowPlanner::try_to_scan(
  RobotArm arm,
  const int32_t sku_id, 
  const uint8_t camera_id,
  const Pose& scan_pose)
{
  auto scan_and_check = [&](uint32_t attempt_num) -> std::optional<std::vector<ObjectPose>> {
    // wait for image streaming become stable
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

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

  auto try_translation = [&](const std::string& axis, 
                              float x_scale, 
                              float y_scale, 
                              uint32_t start_attempt,
                              const tf2::Transform& g_b__tcp) -> std::optional<std::vector<ObjectPose>> {
    const float translation_step = (axis == "X") ? re_scan_x_translation_ : re_scan_y_translation_;
    
    for (uint16_t i = 0; i < max_scan_attempt_; i++) 
    {
      // Negative first: -d, +d, -2d, +2d, -3d, +3d...
      // float multiplier = ((i % 2) * 2 - 1) * ((i / 2) + 1);
      // Positive offsets first: +d, -d, +2d, -2d, +3d, -3d...
      float multiplier = ((i % 2) == 0) ? ((i / 2) + 1) : -((i / 2) + 1);
      float x_offset = x_scale * multiplier * translation_step;
      float y_offset = y_scale * multiplier * translation_step;
      
      RCLCPP_WARN(get_logger(), "%s-axis translate - x: %.4f, y: %.4f", axis.c_str(), x_offset, y_offset);

      tf2::Transform g_tcp__tcp_offset = get_g(x_offset, y_offset, 0, 0, 0, 0);
      tf2::Transform g_b__tcp_offset = g_b__tcp * g_tcp__tcp_offset;
      
      Pose p = cvt_g_to_pose(g_b__tcp_offset);
      
      if (!motion_planner_->move_to(arm, p, 100.0f)) 
      {
        RCLCPP_ERROR(get_logger(), "Failed to move to %s-offset position", axis.c_str());
        continue; // Try next offset instead of giving up
      }
      
      if (auto result = scan_and_check(start_attempt + i); result.has_value()) 
      { 
        return result;
      }
    }
    
    return std::nullopt;
  };

  if (!motion_planner_->move_to_action_pose(arm, 100.0f)) 
  {
    RCLCPP_ERROR(get_logger(), "Failed to move to action pose");
    return std::nullopt;
  }

  if (!motion_planner_->move_to(arm, scan_pose, 100.0f)) 
  {
    RCLCPP_ERROR(get_logger(), "Failed to move to SKU [%d] position", sku_id);
    return std::nullopt;
  }
  
  std::string tcp;
  if (arm == RobotArm::LEFT || arm == RobotArm::LEFT_ACTION)
  {
    tcp = "left_tcp";
    RCLCPP_ERROR(get_logger(), "Left arm move to SKU [%d] scan position", sku_id);
  }
  else if (arm == RobotArm::RIGHT || arm == RobotArm::RIGHT_ACTION)
  {
    tcp = "right_tcp";
    RCLCPP_ERROR(get_logger(), "Right arm move to SKU [%d] scan position", sku_id);
  }

  // 1st attempt
  if (auto result = scan_and_check(1); result.has_value()) 
  {
    return result;
  }

  // Get current pose for offset calculations
  auto tf_stamped = get_tf(ARM_REF_FRAME, tcp);
  if (!tf_stamped.has_value())
  {
    return std::nullopt;
  }

  tf2::Transform g_b__tcp;
  tf2::fromMsg(tf_stamped.value().transform, g_b__tcp);
  print_g(g_b__tcp);

  // Y-axis translation attempts
  if (auto result = try_translation("Y", 0.0f, 1.0f, 2, g_b__tcp); result.has_value()) 
  {
    return result;
  }

  // X-axis translation attempts
  if (auto result = try_translation("X", 1.0f, 0.0f, max_scan_attempt_ + 2, g_b__tcp); result.has_value())
  {
    return result;
  }

  RCLCPP_ERROR(get_logger(), "Failed to scan <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
  return std::nullopt;
}



  // for (uint16_t i = 0; i < max_scan_attempt_; i++) 
  // {
  //   float offset = ((i % 2) * 2 - 1) * ((i / 2) + 1) * re_scan_y_translation_;
  //   RCLCPP_WARN(get_logger(), "X-axis translate offset: %.4f", offset);

  //   tf2::Transform g_tcp__tcp_offset = get_g(0, offset, 0, 0, 0, 0);
  //   tf2::Transform g_b__tcp_offset = g_b__tcp * g_tcp__tcp_offset;
    
  //   Pose p = cvt_g_to_pose(g_b__tcp_offset);
    
  //   if (!motion_planner_->move_to(arm, p, 100.0f)) 
  //   {
  //     RCLCPP_ERROR(get_logger(), "Failed to move to offset position");
  //     continue; // Try next offset instead of giving up
  //   }
    
  //   // +2 because attempt 1 already done
  //   if (auto result = scan_and_check(i + 2); result.has_value()) 
  //   { 
  //     return result;
  //   }
  // }

  // for (uint16_t i = 0; i < max_scan_attempt_; i++) 
  // {
  //   float offset = ((i % 2) * 2 - 1) * ((i / 2) + 1) * re_scan_x_translation_;
  //   RCLCPP_WARN(get_logger(), "X-axis translate offset: %.4f", offset);

  //   tf2::Transform g_tcp__tcp_offset = get_g(offset, 0, 0, 0, 0, 0);
  //   tf2::Transform g_b__tcp_offset = g_b__tcp * g_tcp__tcp_offset;
    
  //   Pose p = cvt_g_to_pose(g_b__tcp_offset);
    
  //   if (!motion_planner_->move_to(arm, p, 100.0f)) 
  //   {
  //     RCLCPP_ERROR(get_logger(), "Failed to move to X-offset position");
  //     continue; // Try next offset instead of giving up
  //   }
    
  //   // + max_scan_attempt_ + 2 because Y-axis attempts and attempt 1 already done
  //   if (auto result = scan_and_check(i + max_scan_attempt_ + 2); result.has_value()) 
  //   { 
  //     return result;
  //   }
  // }