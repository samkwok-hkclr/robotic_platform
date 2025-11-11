#ifndef FINGER_GRIPPER_HPP__
#define FINGER_GRIPPER_HPP__

#pragma once

#include <functional>
#include <future>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/bool.hpp"

#include "std_srvs/srv/set_bool.hpp"

#include "robotic_platform_msgs/msg/gripper_status.hpp"
#include "robotic_platform_msgs/srv/control_gripper.hpp"

#include "node_base.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

enum OBJECT_STATE : uint8_t
{
  CLAMPED = 2,
  DROPPED = 3,
  
  NONE
};

class FingerGripper : public NodeBase
{
  using Empty = std_msgs::msg::Empty;
  using Bool = std_msgs::msg::Bool;

  using SetBool = std_srvs::srv::SetBool;

  using GripperStatus = robotic_platform_msgs::msg::GripperStatus;
  using ControlGripper = robotic_platform_msgs::srv::ControlGripper;

public:
  FingerGripper(const rclcpp::NodeOptions& options);
  ~FingerGripper() = default;

  void gripper_status_cb(const GripperStatus::SharedPtr msg);

  bool gripper_action(const uint8_t force, const float position);

  void obj_state_cb(void);

private:
  std::atomic<bool> is_initialized_{false};
  std::atomic<OBJECT_STATE> obj_state_{OBJECT_STATE::NONE};

  rclcpp::CallbackGroup::SharedPtr srv_cli_cbg_;

  rclcpp::TimerBase::SharedPtr obj_dropped_timer;

  rclcpp::Publisher<Empty>::SharedPtr obj_dropped_pub_;

  rclcpp::Subscription<GripperStatus>::SharedPtr gripper_status_sub_;

  rclcpp::Client<ControlGripper>::SharedPtr gripper_ctrl_cli_;

};

#endif // FINGER_GRIPPER_HPP__