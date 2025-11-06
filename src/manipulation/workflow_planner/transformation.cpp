#include "manipulation/workflow_planner/workflow_planner.hpp"

std::optional<geometry_msgs::msg::Pose> WorkflowPlanner::extract_object_pose(
  RobotArm arm,
  const std::vector<ObjectPose>& poses_in_camera)
{
  auto tf_stamped = get_tf(ARM_REF_FRAME, arm == RobotArm::LEFT ? "left_tcp" : "right_tcp");

  if (!tf_stamped.has_value())
  {
    RCLCPP_ERROR(get_logger(), "Failed to get current pose");
    return std::nullopt;
  }

  tf2::Transform g_b__tcp;
  tf2::fromMsg(tf_stamped.value().transform, g_b__tcp);
  RCLCPP_INFO(get_logger(), "g_b__tcp OK");
  
  auto min_z_it = std::min_element(poses_in_camera.begin(), poses_in_camera.end(),
    [this](const ObjectPose& a, const ObjectPose& b) {
      return a.pose.position.z >= valid_z_threshold_ && a.pose.position.z < b.pose.position.z;
    });

  int min_z_index = std::distance(poses_in_camera.begin(), min_z_it);

  if (min_z_it == poses_in_camera.end()) 
  {
    RCLCPP_ERROR(get_logger(), "Failed to find object with minimum Z");
    return std::nullopt;
  }

  tf2::Transform g_cam__obj = get_g(poses_in_camera[min_z_index].pose);
  
  tf2::Transform g_b__obj = g_b__tcp * g_tcp__cam_.at(arm) * g_cam__obj;
  print_g(g_b__obj);
  RCLCPP_INFO(get_logger(), "g_b__obj OK");

  Pose object_pose = cvt_g_to_pose(g_b__obj);

  push_tf_buf(std::make_tuple(object_pose, ARM_REF_FRAME, OBJECT_POSE));
  RCLCPP_INFO(get_logger(), "pushed %s to tf", OBJECT_POSE.c_str());

  push_tf_buf(std::make_tuple(cvt_g_to_pose(g_cam__obj), "camera_color_optical_frame", "detected_pose"));
  RCLCPP_INFO(get_logger(), "pushed detected_pose to tf");

  RCLCPP_WARN(get_logger(), "sense object pose ended <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");

  return std::make_optional(std::move(object_pose));
}