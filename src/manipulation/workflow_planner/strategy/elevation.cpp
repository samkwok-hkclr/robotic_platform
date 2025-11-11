#include "manipulation/workflow_planner/workflow_planner.hpp"

bool WorkflowPlanner::optimal_pick_elvation(const RackInfo& rack)
{
  if (fold_elev_driver_->rotate_to_abs_front())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  else
  {
    return false;
  }

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
      p.position.x = -0.08;
      p.position.z = 0.99;
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
  const std::string place_link = get_place_link(table.id, table.index);

  auto stamped_tf = get_tf(BASE_FOOTPRINT, place_link);
  if (!stamped_tf.has_value())
  {
    return false;
  }
  // ti: table_index
  tf2::Transform g_b_ti;
  tf2::fromMsg(stamped_tf.value().transform, g_b_ti);

  stamped_tf = get_tf(BASE_FOOTPRINT, ELEV_FLAT_LINK);
  if (!stamped_tf.has_value())
  {
    return false;
  }
  // ep: elevation plane
  tf2::Transform g_b__ep;
  tf2::fromMsg(stamped_tf.value().transform, g_b__ep);

  stamped_tf = get_tf(ELEV_FLAT_LINK, place_link);
  if (!stamped_tf.has_value())
  {
    return false;
  }
  tf2::Transform g_ep__ti;
  tf2::fromMsg(stamped_tf.value().transform, g_ep__ti);

  double x = 0.0, z = 0.0, yaw = 0.0;

  // double target_x = table_front_offset_;
  // double curr_x = std::abs(g_ep__ti.getOrigin().getX());

  // if (curr_x > table_front_offset_)
  //   x = table_front_offset_ - curr_x;
  // else if (curr_x < table_front_offset_)
  //   x = -(target_x - curr_x);

  // if (std::abs(x) < 0.01)
  //   x = 0.0;

  double target_z = std::abs(g_b_ti.getOrigin().getZ()) + table_height_offset_;
  double curr_z = std::abs(g_b__ep.getOrigin().getZ());

  if (curr_z > target_z)
    z = -(curr_z - target_z);
  else
    z = target_z - curr_z;

  if (std::abs(z) < 0.01)
    z = 0.0;

  // tf2::Vector3 table_pos = g_ep__ti.getOrigin();

  // Calculate the angle between elevator's x-axis and the vector to the table
  // if (table_pos.length() > 0.001)
  // {
  //   tf2::Vector3 direction = table_pos.normalized();
  //   yaw = atan2(direction.getY(), direction.getX());
  // }
  // else
  // {
  //   RCLCPP_WARN(get_logger(), "Table is very close to elevator, using zero yaw");
  //   yaw = 0.0;
  // }
  
  RCLCPP_WARN(get_logger(), "try to elevate the fold elvator for optimal place action");
  RCLCPP_WARN(get_logger(), "x: %.3f, z: %.3f, yaw: %.3f", x, z, yaw);
  return fold_elev_driver_->elevate(x, z, yaw);
}