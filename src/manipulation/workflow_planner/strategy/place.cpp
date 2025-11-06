#include "manipulation/workflow_planner/workflow_planner.hpp"

bool WorkflowPlanner::try_to_place_down(
  RobotArm arm, double height, const TableInfo& table)
{
  bool success = false;
  const std::string place_frame = get_place_link(table.id, table.index);

  RCLCPP_WARN(get_logger(), "place frame: %s", place_frame.c_str());

  std::optional<TransformStamped> tf_stamped = get_tf(ARM_REF_FRAME, place_frame);
  if (!tf_stamped.has_value())
  {
    RCLCPP_INFO(get_logger(), "tf_stamped [%s] have no value", place_frame.c_str());
    return success;
  }
  
  tf2::Transform g_b__ref;
  tf2::fromMsg(tf_stamped.value().transform, g_b__ref);
  print_g(g_b__ref);

  double x = g_b__ref.getOrigin().getX();
  double y = g_b__ref.getOrigin().getY();
  double z = g_b__ref.getOrigin().getZ();
  double roll = 0;
  double pitch = -M_PI / 2;
  double yaw = -M_PI;

  tf2::Transform g_b__place_flat = get_g(x, y, z, roll, pitch, yaw);

  RCLCPP_INFO(get_logger(), "<%s> height: %.3f", __FUNCTION__, height);
  RCLCPP_INFO(get_logger(), "<%s> height + offset: %.3f", __FUNCTION__, height + place_offset_);

  tf2::Transform g_place_flat__place = get_g(std::abs(height + place_offset_), 0, 0, 0, 0, 0);
  tf2::Transform g_b__place = g_b__place_flat * g_place_flat__place;
  print_g(g_b__place);

  const Pose place_pose = cvt_g_to_pose(g_b__place);
  push_tf_buf(std::make_tuple(place_pose, ARM_REF_FRAME, "place_pose"));

  std::optional<PlacePlanResult> place_plan_result = get_place_plan(place_pose);
  if (!place_plan_result.has_value())
  {
    return success;
  }

  if (motion_planner_->place(arm, place_plan_result.value(), 100.0))
  {
    success = true;
  }

  return success;
}