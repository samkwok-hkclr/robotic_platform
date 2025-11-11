#include "planner_base.hpp"

PlannerBase::PlannerBase(
  std::string node_name, 
  const rclcpp::NodeOptions& options)
: NodeBase(node_name, options)
{
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);
  
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  transform_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
}

tf2::Transform PlannerBase::get_g(const Pose& pose)
{
  const auto& [px, py, pz] = pose.position;
  const auto& [qx, qy, qz, qw] = pose.orientation;

  tf2::Transform g;
  tf2::Quaternion q(qx, qy, qz, qw);

  g.setOrigin(tf2::Vector3(px, py, pz));
  g.setRotation(q);

  RCLCPP_DEBUG(get_logger(), "px = %f, py= %f, pz = %f, qx = %f, qy = %f, qz = %f, qw = %f", 
    px, py, pz, qx, qy, qz, qw); 
  return g;
}

tf2::Transform PlannerBase::get_g(double px, double py, double pz, double roll, double pitch, double yaw)
{
  tf2::Transform g;
  tf2::Quaternion q;

  g.setOrigin(tf2::Vector3(px, py, pz));
  q.setRPY(roll, pitch, yaw);
  q.normalize();

  g.setRotation(q);

  RCLCPP_DEBUG(get_logger(), "px = %f, py= %f, pz = %f, roll = %f, pitch = %f, yaw = %f", 
    px, py, pz, roll, pitch, yaw);  
  return g;
}

tf2::Transform PlannerBase::get_g(double px, double py, double pz, double qx, double qy, double qz, double qw)
{
  tf2::Transform g;
  tf2::Quaternion q(qx, qy, qz, qw);
  q.normalize();

  g.setOrigin(tf2::Vector3(px, py, pz));
  g.setRotation(q);

  RCLCPP_DEBUG(get_logger(), "px = %f, py= %f, pz = %f, qx = %f, qy = %f, qz = %f, qw = %f", 
    px, py, pz, qx, qy, qz, qw);  
  return g;
}

void PlannerBase::print_g(const tf2::Transform& g) const
{
  RCLCPP_WARN(get_logger(), "px = %f, py= %f, pz = %f, qx = %f, qy = %f, qz = %f, qw = %f",
    g.getOrigin().getX(),
    g.getOrigin().getY(),
    g.getOrigin().getZ(),
    g.getRotation().getX(),
    g.getRotation().getY(),
    g.getRotation().getZ(),
    g.getRotation().getW());
}

geometry_msgs::msg::Pose PlannerBase::cvt_g_to_pose(const tf2::Transform& g) const
{
  Pose msg;

  msg.position.x = g.getOrigin().getX();
  msg.position.y = g.getOrigin().getY();
  msg.position.z = g.getOrigin().getZ();
  msg.orientation.x = g.getRotation().getX();
  msg.orientation.y = g.getRotation().getY();
  msg.orientation.z = g.getRotation().getZ();
  msg.orientation.w = g.getRotation().getW();

  return msg;
}

void PlannerBase::pose_translation(Pose::SharedPtr pose, float x, float y, float z)
{
  // 1. quaternion to rotation matrix (3 by 3)
  Eigen::Quaternionf quat_original;
  Eigen::Matrix3f mat_rotation;
  quat_original.x() = pose->orientation.x;
  quat_original.y() = pose->orientation.y;
  quat_original.z() = pose->orientation.z;
  quat_original.w() = pose->orientation.w;
  mat_rotation = quat_original.normalized().toRotationMatrix();

  // 2. Create homogeneous transformation matrix (4 by 4)
  Eigen::Matrix4f mat_transform = Eigen::Matrix4f::Identity();
  // Set rotation part
  mat_transform.block<3, 3>(0, 0) = mat_rotation;
  // Set translation part
  mat_transform(0, 3) = pose->position.x;
  mat_transform(1, 3) = pose->position.y;
  mat_transform(2, 3) = pose->position.z;

  // 3. Create translation matrix
  Eigen::Matrix4f Txyz = Eigen::Matrix4f::Identity();
  Txyz(0, 3) = x;
  Txyz(1, 3) = y;
  Txyz(2, 3) = z;  

  // 4. Apply transformation: first translate, then rotate+translate
  mat_transform = mat_transform * Txyz;
  
  // 5. Update pose
  pose->position.x = mat_transform(0, 3);
  pose->position.y = mat_transform(1, 3);
  pose->position.z = mat_transform(2, 3);
}

void PlannerBase::print_pose(const Pose& pose) const
{
  RCLCPP_WARN(get_logger(), "[p.x: %.6f, p.y: %.6f, p.z: %.6f, q.x: %.6f, q.y: %.6f, q.z: %.6f, q.w: %.6f]", 
    pose.position.x, pose.position.y, pose.position.z, 
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
}

void PlannerBase::print_pose_arr(const std::vector<Pose>& poses) const
{
  for (const auto& pose : poses)
  {
    print_pose(pose);
  }
}

geometry_msgs::msg::Pose PlannerBase::compose_pose_msg(const std::vector<double>& pose_vec) const
{
  if (pose_vec.size() != 7) 
  {
    RCLCPP_ERROR(get_logger(), "Invalid dimensions for pose, vector size: %ld", pose_vec.size());
    return geometry_msgs::msg::Pose();
  }

  geometry_msgs::msg::Pose msg;

  msg.position.x = pose_vec[0];
  msg.position.y = pose_vec[1];
  msg.position.z = pose_vec[2];
  msg.orientation.x = pose_vec[3];
  msg.orientation.y = pose_vec[4];
  msg.orientation.z = pose_vec[5];
  msg.orientation.w = pose_vec[6];

  return msg; 
}

bool PlannerBase::are_poses_closed(
  const geometry_msgs::msg::Pose& pose_1,
  const geometry_msgs::msg::Pose& pose_2, 
  double pos_thd,
  double ori_thd) const
{
  Eigen::Vector3d pos1(pose_1.position.x, pose_1.position.y, pose_1.position.z);
  Eigen::Vector3d pos2(pose_2.position.x, pose_2.position.y, pose_2.position.z);

  if ((pos1 - pos2).norm() > pos_thd) 
    return false;
  
  // Quaternion check (normalized)
  Eigen::Quaterniond q1(
    pose_1.orientation.w,
    pose_1.orientation.x,
    pose_1.orientation.y,
    pose_1.orientation.z
  );
  Eigen::Quaterniond q2(
    pose_2.orientation.w,
    pose_2.orientation.x,
    pose_2.orientation.y,
    pose_2.orientation.z
  );

  // Check if q1 and q2 represent the same rotation (allowing for sign flips)
  return (q1.angularDistance(q2) < ori_thd);
}

geometry_msgs::msg::TransformStamped PlannerBase::create_transform(
  const Pose& msg, 
  const std::string& parent_frame,
  const std::string& child_frame)
{
  TransformStamped t;

  t.header.stamp = get_clock()->now();
  t.header.frame_id = parent_frame;
  t.child_frame_id = child_frame;

  t.transform.translation.x = msg.position.x;
  t.transform.translation.y = msg.position.y;
  t.transform.translation.z = msg.position.z;

  t.transform.rotation.x = msg.orientation.x;
  t.transform.rotation.y = msg.orientation.y;
  t.transform.rotation.z = msg.orientation.z;
  t.transform.rotation.w = msg.orientation.w;

  return t;
}

void PlannerBase::send_transform(
  const Pose& msg, 
  const std::string& parent_frame,
  const std::string& child_frame)
{
  auto transform = create_transform(msg, parent_frame, child_frame);
  tf_broadcaster_->sendTransform(transform);
}

void PlannerBase::send_static_transform(
  const Pose& msg, 
  const std::string& parent_frame,
  const std::string& child_frame)
{
  auto transform = create_transform(msg, parent_frame, child_frame);
  tf_static_broadcaster_->sendTransform(transform);
}

std::optional<geometry_msgs::msg::TransformStamped> PlannerBase::get_tf(
  const std::string& to_frame, 
  const std::string& from_frame)
{
  TransformStamped tf_stamped;

  try 
  {
    // Wait up to 1.0 second
    tf_stamped = tf_buffer_->lookupTransform(to_frame, from_frame, tf2::TimePointZero, tf2::durationFromSec(1.0));
  } 
  catch (tf2::TransformException & ex) 
  {
    RCLCPP_ERROR(get_logger(), "Could not transform %s to %s: %s", to_frame.c_str(), from_frame.c_str(), ex.what());
    return std::nullopt;
  }
  catch (...) 
  {
    RCLCPP_ERROR(get_logger(), "Unknown exception in transform %s to %s", to_frame.c_str(), from_frame.c_str());
    return std::nullopt;
  }

  RCLCPP_WARN(get_logger(), "get_tf OK! to_frame: %s, from_frame: %s", to_frame.c_str(), from_frame.c_str());

  return std::make_optional(std::move(tf_stamped));
}

