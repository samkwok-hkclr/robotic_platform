#include "manipulation/workflow_planner/workflow_planner.hpp"

bool WorkflowPlanner::optimal_pick_elvation(const RackInfo& rack)
{
  // FIXME: fixed poses for rapid development
  Pose p;
  p.position.y = -0.00665;
  p.orientation.x = 0;
  p.orientation.y = 0;
  p.orientation.z = 0;
  p.orientation.w = 1.0;

  switch (rack.shelf_level)
  {
    case 2:
      p.position.x = -0.1;
      p.position.z = 0.75;
      break;
    case 3:
      p.position.x = -0.15;
      p.position.z = 0.85;
      break;
    case 4:
      p.position.x = -0.12;
      p.position.z = 1.0;
      break;
    default:
      RCLCPP_WARN(get_logger(), "unknown shelf_level");
      return false;
  }

  std::vector<Pose> wps;
  wps.push_back(p);
  
  RCLCPP_WARN(get_logger(), "try to elevate the fold elvator for optimal pick action");
  return fold_elev_driver_->exec_wps(std::move(wps));
}

bool WorkflowPlanner::optimal_place_elvation(const TableInfo& table)
{
  // FIXME: fixed poses for rapid development
  auto stamped_tf = get_tf(BASE_FOOTPRINT, get_place_link(table.id, table.index));
  if (!stamped_tf.has_value())
  {
    return false;
  }

  tf2::Transform g_b__table_index;
  tf2::fromMsg(stamped_tf.value().transform, g_b__table_index);

  stamped_tf = get_tf(BASE_FOOTPRINT, ARM_REF_FRAME);
  if (!stamped_tf.has_value())
  {
    return false;
  }
   tf2::Transform g_b__elev_float;
  tf2::fromMsg(stamped_tf.value().transform, g_b__elev_float);

  // double x = 0.0;
  double x = std::abs(g_b__table_index.getOrigin().getX()) - table_front_offset_ - std::abs(g_b__elev_float.getOrigin().getX());
  double z = std::abs(g_b__table_index.getOrigin().getZ()) + table_height_offset_ - std::abs(g_b__elev_float.getOrigin().getZ());
  double yaw = 0.0;

  RCLCPP_WARN(get_logger(), "try to elevate the fold elvator for optimal place action");
  return fold_elev_driver_->elevate(x, z, yaw);
}