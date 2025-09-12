#include "manipulation/workflow_planner/workflow_planner.hpp"

bool WorkflowPlanner::try_to_pick_up(const uint32_t sku_id, const uint8_t camera_id)
{
  uint8_t attempt = 1;

  for (; attempt <= max_pick_attampt_; attempt++)
  {
    RCLCPP_INFO(get_logger(), "attempt [%d]: try to pick up item %d", attempt, sku_id);

    if (!motion_planner_->move_to_scan_pose(25.0))
    {
      return false;
    }

    RCLCPP_WARN(get_logger(), "start to scan <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
    std::optional<std::vector<ObjectPose>> obj_poses = try_to_scan(sku_id, camera_id);
    if (!obj_poses.has_value())
    {
      RCLCPP_INFO(get_logger(), "have no value");
      return false;
    }

    std::vector<ObjectPose>& poses_in_camera = obj_poses.value();
    RCLCPP_INFO(get_logger(), "pose length: %ld", poses_in_camera.size());

    std::optional<Pose> obj_pose = extract_object_pose(poses_in_camera);
    if (!obj_pose.has_value())
    {
      RCLCPP_INFO(get_logger(), "obj_pose have no value");
      return false;
    }

    std::optional<PickPlanResult> pick_plan_result = get_pick_plan(obj_pose.value());
    if (!pick_plan_result.has_value())
    {
      return false;
    }

    push_tf_buf(std::make_tuple(pick_plan_result.value().pre_pick_pose, "base_footprint", "pre_pick_pose"));
    auto& pick_poses = pick_plan_result.value().pick_poses;

    std::for_each(pick_poses.begin(), pick_poses.end(), [this, i = 0](auto& pose) mutable {
      push_tf_buf(std::make_tuple(pose, "base_footprint", "pick_poses_" + std::to_string(i++)));
    });

    if (motion_planner_->pick(pick_plan_result.value(), 20.0))
    {
      break;
    }
    
    clear_tf_buf();
    RCLCPP_ERROR(get_logger(), "attempt [%d]: failed to pick up item %d", attempt, sku_id);
  }

  clear_tf_buf();
  return attempt <= max_pick_attampt_ ? true : false;
}