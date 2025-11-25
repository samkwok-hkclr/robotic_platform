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

#include "node_base.hpp"

enum RobotArm : uint8_t
{
  LEFT = 1,
  LEFT_ACTION,
  RIGHT,
  RIGHT_ACTION,
  WHOLE,

  LAST // should not be assigned
};

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

	virtual Pose compose_pose_msg(const std::vector<double>& pose_vec) const;

  // deprecated
	// virtual void pose_translation(Pose::SharedPtr pose, float x, float y, float z);

  virtual void print_pose(const Pose& pose) const;
  virtual void print_pose_arr(const std::vector<Pose>& poses) const;

  virtual bool are_poses_closed(
    const Pose& pose_1,
    const Pose& pose_2, 
    double pos_thd = 1e-5, 
    double ori_thd = 1e-5) const;

  TransformStamped create_transform(
    const Pose& msg, 
    const std::string& parent_frame,
    const std::string& child_frame);

  virtual void send_transform(
    const Pose& pose, 
    const std::string& parent_frame,
    const std::string& child_frame);

  virtual void send_static_transform(
    const Pose& pose, 
    const std::string& parent_frame,
    const std::string& child_frame);

  virtual std::optional<TransformStamped> get_tf(
    const std::string& to_frame, 
    const std::string& from_frame);

  std::vector<tf2::Transform> interpolate_tf(
    const tf2::Transform& start, 
    const tf2::Transform& end, 
    double max_step_distance = 0.1);

  inline static const std::map<RobotArm, std::string> arm_to_str = {
    { RobotArm::LEFT, "left_arm" },
    { RobotArm::LEFT_ACTION, "left_action_arm" },
    { RobotArm::RIGHT, "right_arm" },
    { RobotArm::RIGHT_ACTION, "right_action_arm" },
    { RobotArm::WHOLE, "whole_arm" }
  };

  inline static const std::map<std::string, RobotArm> str_to_arm = {
    { "left", RobotArm::LEFT } ,
    { "left_arm", RobotArm::LEFT } ,
    { "left_action_arm", RobotArm::LEFT_ACTION } ,

    { "right", RobotArm::RIGHT },
    { "right_arm", RobotArm::RIGHT },
    { "right_action_arm", RobotArm::RIGHT_ACTION },

    { "whole", RobotArm::WHOLE },
    { "whole_arm", RobotArm::WHOLE }
  };

  const std::array<RobotArm, 5> all_arms = {
    RobotArm::LEFT, 
    RobotArm::LEFT_ACTION, 
    RobotArm::RIGHT, 
    RobotArm::RIGHT_ACTION, 
    RobotArm::WHOLE
  };

  const std::string BASE_LINK = "base_link";
  const std::string BASE_FOOTPRINT = "base_footprint";
  const std::string OBJECT_POSE = "object_pose";
  const std::string ARM_REF_FRAME = "base_footprint";
  const std::string ELEV_FLAT_JOINT = "joint4";
  const std::string ELEV_FLAT_LINK = "link4";
  const std::string MAP_FRAME = "map";

private:  
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

  std::unique_ptr<tf2_ros::TransformListener> transform_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

};

#endif // PLANNER_BASE_HPP__
