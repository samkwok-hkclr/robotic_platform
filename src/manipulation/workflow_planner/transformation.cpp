#include "manipulation/workflow_planner/workflow_planner.hpp"

std::optional<geometry_msgs::msg::Pose> WorkflowPlanner::extract_object_pose(
  RobotArm arm,
  const std::vector<ObjectPose>& poses_in_camera)
{
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

  std::string tcp;
  if (arm == RobotArm::LEFT || arm == RobotArm::LEFT_ACTION)
    tcp = "left_tcp";
  else if (arm == RobotArm::RIGHT || arm == RobotArm::RIGHT_ACTION)
    tcp = "right_tcp";
  
  std::optional<TransformStamped> tf_stamped = get_tf(ARM_REF_FRAME, tcp);

  if (!tf_stamped.has_value())
  {
    RCLCPP_ERROR(get_logger(), "Failed to get current pose");
    return std::nullopt;
  }

  tf2::Transform g_b__tcp;
  tf2::fromMsg(tf_stamped.value().transform, g_b__tcp);

  RobotArm origin_arm;
  switch (arm)
  {
    case RobotArm::LEFT_ACTION:
      origin_arm = RobotArm::LEFT;
      break;
    case RobotArm::RIGHT_ACTION:
      origin_arm = RobotArm::RIGHT;
      break;    
    default:
      origin_arm = arm;
      break;
  }
  
  if (auto it = g_tcp__cam_.find(origin_arm); it == g_tcp__cam_.end())
    return std::nullopt;

  tf2::Transform g_obj_rot__obj;
  switch (arm)
  {
    case RobotArm::LEFT:
    case RobotArm::LEFT_ACTION:
      g_obj_rot__obj = get_g(0, 0, 0, 0, 0, M_PI/2.0);
      break;
    case RobotArm::RIGHT:
    case RobotArm::RIGHT_ACTION:
      g_obj_rot__obj = get_g(0, 0, 0, 0, 0, 0);
      break;    
    default:
      RCLCPP_ERROR(get_logger(), "WHOLE does not handle");
      return std::nullopt;
  }

  tf2::Transform g_cam__obj = g_cam__obj_rot * g_obj_rot__obj;
  RCLCPP_INFO(get_logger(), "g_cam__obj:");
  print_g(g_cam__obj);

  tf2::Transform g_b__obj = g_b__tcp * g_tcp__cam_.at(origin_arm) * g_cam__obj;
  RCLCPP_INFO(get_logger(), "g_b__obj:");
  print_g(g_b__obj);

  Pose object_pose = cvt_g_to_pose(g_b__obj);

  push_tf_buf(std::make_tuple(object_pose, ARM_REF_FRAME, OBJECT_POSE));
  RCLCPP_INFO(get_logger(), "pushed %s to tf", OBJECT_POSE.c_str());

  std::string cam_optical_frame;
  if (origin_arm == RobotArm::LEFT)
    cam_optical_frame = "left_camera_color_optical_frame";
  else
    cam_optical_frame = "right_camera_color_optical_frame";

  push_tf_buf(std::make_tuple(cvt_g_to_pose(g_cam__obj), cam_optical_frame, "detected_pose"));
  RCLCPP_INFO(get_logger(), "pushed detected_pose to tf");

  RCLCPP_WARN(get_logger(), "sense object pose ended <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");

  return std::make_optional(std::move(object_pose));
}
