#ifndef NODE_BASE_HPP__
#define NODE_BASE_HPP__

#pragma once

#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/int32.hpp"

#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "robotic_platform_msgs/srv/get_slot_state_trigger.hpp"
#include "robotic_platform_msgs/srv/get_object_pose_trigger.hpp"
#include "robotic_platform_msgs/srv/pick_plan.hpp"
#include "robotic_platform_msgs/srv/place_plan.hpp"
#include "robotic_platform_msgs/srv/control_gripper.hpp"

#include "robot_controller_msgs/srv/execute_waypoints.hpp"
#include "robot_controller_msgs/srv/execute_joints.hpp"
#include "robot_controller_msgs/srv/execute_pose.hpp"
#include "robot_controller_msgs/srv/get_pose.hpp"
#include "robot_controller_msgs/srv/get_joint_states.hpp"
#include "robot_controller_msgs/srv/get_joint_limits.hpp"
#include "robot_controller_msgs/srv/add_collision_objects.hpp"
#include "robot_controller_msgs/srv/apply_attached_collision_objects.hpp"
#include "robot_controller_msgs/srv/get_collision_objects_from_scene.hpp"
#include "robot_controller_msgs/srv/move_collision_objects.hpp"
#include "robot_controller_msgs/srv/remove_collision_objects.hpp"
#include "robot_controller_msgs/srv/robot_speed.hpp"


class NodeBase : public rclcpp::Node
{
public:
	explicit NodeBase(
    std::string node_name, 
    const rclcpp::NodeOptions& options)
  : Node(node_name, options)
  {

  }

	~NodeBase() = default;

  virtual std::chrono::duration<double> get_cli_req_timeout() const 
  {
    return CLI_REQ_TIMEOUT;
  }

	template <typename T>
  void reset_req_res(
    typename T::Request::SharedPtr& request,
    typename T::Response::SharedPtr& response) const;

	template <typename T>
  bool send_sync_req(
    typename rclcpp::Client<T>::SharedPtr cli, 
    const typename T::Request::SharedPtr request,
    typename T::Response::SharedPtr& response,
    const std::string srv_name = "") const;

  template <typename T>
  bool cli_wait_for_srv(
    typename rclcpp::Client<T>::SharedPtr cli, 
    const std::string srv_name = "") const;
    
private:
  constexpr static uint8_t SRV_CLI_MAX_RETIES = 5;
  constexpr static std::chrono::duration WAIT_SER_TIMEOUT = std::chrono::milliseconds(100);
  constexpr static std::chrono::duration CLI_REQ_TIMEOUT = std::chrono::seconds(3);

};

#endif // NODE_BASE_HPP__
