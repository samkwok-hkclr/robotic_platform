#ifndef PLANNER_BASE_HPP__
#define PLANNER_BASE_HPP__

#pragma once

#include <memory>
#include <chrono>

#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

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

private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

};

#endif // PLANNER_BASE_HPP__
