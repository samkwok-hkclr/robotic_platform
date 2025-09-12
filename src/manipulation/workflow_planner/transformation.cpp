#include "manipulation/workflow_planner/workflow_planner.hpp"

std::optional<geometry_msgs::msg::Pose> WorkflowPlanner::extract_object_pose(
  const std::vector<ObjectPose>& poses_in_camera)
{
  std::optional<Pose> curr_pose = get_curr_pose("tcp");

  if (!curr_pose.has_value())
  {
    RCLCPP_ERROR(get_logger(), "Failed to get current pose");
    return std::nullopt;
  }
  
  tf2::Transform g_b__tcp = get_g(curr_pose.value());
  RCLCPP_INFO(this->get_logger(), "g_b__tcp OK");
  
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

  tf2::Transform g_cam__obj_rot = get_g(poses_in_camera[min_z_index].pose);
  RCLCPP_INFO(this->get_logger(), "g_cam__obj_rot OK");
  
  // FIXME: the detected pose rotation is mis-matched.
  // manually rotate the object pose
  tf2::Transform g_obj_rot__obj = get_g(0, 0, 0, 0, 0, -M_PI / 2.0);

  tf2::Transform g_b__obj = g_b__tcp * g_tcp__left_cam_ * g_cam__obj_rot * g_obj_rot__obj;
  RCLCPP_INFO(this->get_logger(), "g_b__obj OK");

  Pose object_pose = cvt_g_to_pose(g_b__obj);

  push_tf_buf(std::make_tuple(object_pose, "base_footprint", object_pose_));
  RCLCPP_INFO(this->get_logger(), "pushed %s to tf", object_pose_.c_str());

  push_tf_buf(std::make_tuple(cvt_g_to_pose(g_cam__obj_rot), "camera_color_optical_frame", "detected_pose"));
  RCLCPP_INFO(this->get_logger(), "pushed detected_pose to tf");

  RCLCPP_WARN(get_logger(), "sense object pose ended <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");

  return std::make_optional(std::move(object_pose));
}