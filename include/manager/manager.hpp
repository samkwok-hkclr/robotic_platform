#ifndef MANAGER_HPP__
#define MANAGER_HPP__

#pragma once

#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose.hpp"

#include "node_base.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class Manager : public NodeBase
{
  using Pose = geometry_msgs::msg::Pose;
  
public:
	explicit Manager(std::string node_name, const rclcpp::NodeOptions& options);
	~Manager() = default;

private:
  std::mutex mutex_;

};

#endif // MANAGER_HPP__
