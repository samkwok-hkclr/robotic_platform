#include "manipulation/workflow_planner/workflow_planner.hpp"

bool WorkflowPlanner::try_to_place_down(
  RobotArm arm, double height, const TableInfo& table)
{
  bool success = false;
  const std::string place_frame = get_place_link(table.id, table.index);

  RCLCPP_WARN(get_logger(), "place frame: %s", place_frame.c_str());

  std::optional<TransformStamped> tf_stamped = get_tf(MAP_FRAME, place_frame);
  if (!tf_stamped.has_value())
  {
    RCLCPP_INFO(get_logger(), "tf_stamped [%s] have no value", place_frame.c_str());
    return success;
  }
  
  // pf: place frame
  tf2::Transform g_m__pf;
  tf2::fromMsg(tf_stamped.value().transform, g_m__pf);
  print_g(g_m__pf);

  double x = 0;
  double y = 0;
  double z = std::abs(height + place_offset_);
  double roll = 0;
  double pitch = -M_PI / 2;
  double yaw = -M_PI / 2;

  // pp: place pose
  tf2::Transform g_pf__pp = get_g(x, y, z, roll, pitch, yaw);

  tf_stamped = get_tf(MAP_FRAME, ARM_REF_FRAME);
  if (!tf_stamped.has_value())
  {
    RCLCPP_INFO(get_logger(), "tf_stamped [%s] have no value", place_frame.c_str());
    return success;
  }
  
  tf2::Transform g_m__b;
  tf2::fromMsg(tf_stamped.value().transform, g_m__b);

  RCLCPP_INFO(get_logger(), "<%s> height: %.3f", __FUNCTION__, height);
  RCLCPP_INFO(get_logger(), "<%s> height + offset: %.3f", __FUNCTION__, height + place_offset_);

  tf2::Transform g_b__pp = g_m__b.inverse() * g_m__pf * g_pf__pp;
  print_g(g_b__pp);

  const Pose place_pose = cvt_g_to_pose(g_b__pp);
  push_tf_buf(std::make_tuple(place_pose, ARM_REF_FRAME, "place_pose"));

  std::optional<PlacePlanResult> place_plan_result = get_place_plan(place_pose);
  if (!place_plan_result.has_value())
  {
    return success;
  }

  if (motion_planner_->place(arm, place_plan_result.value(), 75.0))
  {
    success = true;
  }

  return success;
}
