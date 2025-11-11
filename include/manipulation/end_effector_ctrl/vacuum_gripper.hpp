#ifndef VACUUM_GRIPPER_HPP__
#define VACUUM_GRIPPER_HPP__

#pragma once

#include <functional>
#include <future>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/bool.hpp"

#include "std_srvs/srv/set_bool.hpp"

#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/range.hpp"

#include "node_base.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class VacuumGripper : public NodeBase
{
  using Empty = std_msgs::msg::Empty;
  using Bool = std_msgs::msg::Bool;

  using FluidPressure = sensor_msgs::msg::FluidPressure;
  using Temperature = sensor_msgs::msg::Temperature;
  using Range = sensor_msgs::msg::Range;

  using SetBool = std_srvs::srv::SetBool;

public:
  VacuumGripper(const rclcpp::NodeOptions& options);
  ~VacuumGripper() = default;

  void state_cb(const Bool::SharedPtr msg);
  void range_cb(const Range::SharedPtr msg);
  void temp_cb(const Temperature::SharedPtr msg);
  void pressure_cb(const FluidPressure::SharedPtr msg);

  bool gripper_action(const bool cmd);

  void leak_validation_cb(void);

private:
  std::atomic<bool> state_{false};
  std::atomic<float> pressure_{0.0f};
  std::atomic<float> distance_{0.0f};
  std::atomic<float> temperature_{0.0f};

  float leak_threshold_;

  rclcpp::CallbackGroup::SharedPtr srv_cli_cbg_;

  rclcpp::TimerBase::SharedPtr leak_valid_timer;

  rclcpp::Publisher<Empty>::SharedPtr leak_pub_;

  rclcpp::Subscription<Bool>::SharedPtr state_sub_;
  rclcpp::Subscription<Range>::SharedPtr range_sub_;
  rclcpp::Subscription<Temperature>::SharedPtr temp_sub_;
  rclcpp::Subscription<FluidPressure>::SharedPtr pressure_sub_;

  rclcpp::Client<SetBool>::SharedPtr pump_ctrl_cli_;

};

#endif // VACUUM_GRIPPER_HPP__