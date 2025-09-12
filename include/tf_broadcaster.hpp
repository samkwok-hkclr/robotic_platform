#ifndef TF_BROADCASTER_HPP__
#define TF_BROADCASTER_HPP__

#pragma once

#include <cmath>
#include <functional>
#include <future>
#include <memory>
#include <chrono>
#include <algorithm>
#include <tuple>
#include <utility>
#include <unordered_map>

#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose.hpp"

#include "planner_base.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class TfBroadcaster : public PlannerBase
{
  using Pose = geometry_msgs::msg::Pose;
  using TransformStamped = geometry_msgs::msg::TransformStamped;

public:
  explicit TfBroadcaster(
    std::string node_name,
    const rclcpp::NodeOptions& options);
  ~TfBroadcaster() = default;

  void push_tf_static_buf(const std::tuple<Pose, std::string, std::string>& tf);
  void push_tf_buf(const std::tuple<Pose, std::string, std::string>& tf);
  void clear_tf_buf();

  void tf_pub_cb(void);

private:
  std::mutex tf_mutex_;
  
  std::vector<std::tuple<Pose, std::string, std::string>> tf_buf_;

  rclcpp::CallbackGroup::SharedPtr tf_timer_cbg_;

  rclcpp::TimerBase::SharedPtr tf_pub_timer;
};

#endif // TF_BROADCASTER_HPP__