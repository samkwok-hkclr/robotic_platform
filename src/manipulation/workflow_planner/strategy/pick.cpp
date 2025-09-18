#include "manipulation/workflow_planner/workflow_planner.hpp"

std::optional<double> WorkflowPlanner::try_to_pick_up(
  RobotArm arm, const int32_t sku_id, const uint8_t camera_id,
  const RackInfo& rack)
{
  RCLCPP_WARN(get_logger(), "try to pick up item %d", sku_id);
  uint8_t attempt = 1;
  bool success = false;

  clear_tf_buf();

  const std::string flat_frame = get_flat_link(rack.id, rack.shelf_level);

  for (; attempt <= max_pick_attempt_; attempt++)
  {
    RCLCPP_INFO(get_logger(), "Attempt [%d]: try to pick up item %d", attempt, sku_id);

    RCLCPP_WARN(get_logger(), "get scan pose <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
    std::optional<geometry_msgs::msg::Pose> scan_pose = get_scan_pose(rack.id, rack.shelf_level, rack.shelf_slot);
    if (!scan_pose.has_value())
    {
      RCLCPP_ERROR(get_logger(), "Get scan pose failed");
      continue;
    }

    RCLCPP_WARN(get_logger(), "start to scan <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
    std::optional<std::vector<ObjectPose>> obj_poses = try_to_scan(arm, sku_id, camera_id, scan_pose.value());
    
    // FIXME: debug now
    if (!obj_poses.has_value())
    {
      RCLCPP_INFO(get_logger(), "vision algorithm have no value");
      return std::nullopt;
    }

    std::vector<ObjectPose> poses_in_camera;

    if (simulation_)
    {
      ObjectPose sim_msg;
      sim_msg.pose = cvt_g_to_pose(get_g(0.08, 0, 0.28, 0, 0, 0));
      poses_in_camera.push_back(sim_msg);
      RCLCPP_INFO(get_logger(), "Simulation pose length: %ld", poses_in_camera.size());
    }
    else
    {
      poses_in_camera = std::move(obj_poses.value());
    }

    std::optional<Pose> obj_pose = extract_object_pose(arm, poses_in_camera);
    if (!obj_pose.has_value())
    {
      RCLCPP_INFO(get_logger(), "obj_pose have no value");
      return std::nullopt;
    }

    std::optional<PickPlanResult> pick_plan_result = get_pick_plan(obj_pose.value(), rack, flat_frame);
    if (!pick_plan_result.has_value())
    {
      return std::nullopt;
    }

    push_tf_buf(std::make_tuple(pick_plan_result.value().pre_pick_pose, ARM_REF_FRAME, "pre_pick_pose"));
    auto& pick_poses = pick_plan_result.value().pick_poses;

    std::for_each(pick_poses.begin(), pick_poses.end(), [this, i = 0](auto& pose) mutable {
      push_tf_buf(std::make_tuple(pose, ARM_REF_FRAME, "pick_poses_" + std::to_string(i++)));
    });

    if (motion_planner_->pick(arm, pick_plan_result.value(), 50.0f))
    {
      success = true;
      break;
    }
    
    RCLCPP_ERROR(get_logger(), "attempt [%d]: failed to pick up item %d", attempt, sku_id);
  }
  
  if (success)
  {
    std::optional<TransformStamped> g_obj_flat = get_tf(flat_frame, OBJECT_POSE);
    if (!g_obj_flat.has_value())
    {
      RCLCPP_INFO(get_logger(), "have no value");
      return std::nullopt;
    }

    const double height = std::abs(g_obj_flat.value().transform.translation.z);
    RCLCPP_WARN(get_logger(), "height: %.6f above [rack: %d, shelf level: %d, shlef slot: %d]", 
      height, rack.id, rack.shelf_level, rack.shelf_slot);

    return std::make_optional(height);
  }

  return std::nullopt;
}

