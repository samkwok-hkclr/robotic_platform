#ifndef PLANNER_BASE_HPP__
#define PLANNER_BASE_HPP__

#pragma once

#include <memory>
#include <string>
#include <chrono>

#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "tf2/exceptions.h"
#include "tf2/convert.h"

#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "std_srvs/srv/set_bool.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "robotic_platform_msgs/srv/get_slot_state_trigger.hpp"
#include "robotic_platform_msgs/srv/get_object_pose_trigger.hpp"
#include "robotic_platform_msgs/srv/pick_plan.hpp"
#include "robotic_platform_msgs/srv/place_plan.hpp"

#include "robot_controller_msgs/srv/execute_waypoints.hpp"
#include "robot_controller_msgs/srv/get_current_pose.hpp"
#include "robot_controller_msgs/srv/add_collision_objects.hpp"
#include "robot_controller_msgs/srv/apply_attached_collision_objects.hpp"
#include "robot_controller_msgs/srv/get_collision_objects_from_scene.hpp"
#include "robot_controller_msgs/srv/move_collision_objects.hpp"
#include "robot_controller_msgs/srv/remove_collision_objects.hpp"
#include "robot_controller_msgs/srv/robot_speed.hpp"

#include "node_base.hpp"

class PlannerBase : public NodeBase
{
  using Pose = geometry_msgs::msg::Pose;
  using TransformStamped = geometry_msgs::msg::TransformStamped;

public:
	explicit PlannerBase(std::string node_name, const rclcpp::NodeOptions& options);
	~PlannerBase() = default;

  tf2::Transform get_g(const Pose& pose);
  tf2::Transform get_g(double px, double py, double pz, double roll, double pitch, double yaw);
  tf2::Transform get_g(double px, double py, double pz, double qx, double qy, double qz, double qw);
  void print_g(const tf2::Transform& g) const;
  Pose cvt_g_to_pose(const tf2::Transform& g) const;

	virtual Pose compose_pose_msg(const std::vector<double>& pose_vec);

	virtual void pose_translation(Pose::SharedPtr pose, float x, float y, float z);

  virtual void print_pose(const Pose& pose);
  virtual void print_pose_arr(const std::vector<Pose>& poses);

  virtual bool are_poses_closed(
    const Pose& pose_1,
    const Pose& pose_2, 
    double pos_thd = 1e-5, 
    double ori_thd = 1e-5);

  virtual void send_transform(
    const Pose& pose, 
    const std::string& parent_frame,
    const std::string& child_frame);

  virtual std::optional<TransformStamped> get_tf(
    const std::string& to_frame, 
    const std::string& from_frame);

private:  
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

};

#endif // PLANNER_BASE_HPP__
