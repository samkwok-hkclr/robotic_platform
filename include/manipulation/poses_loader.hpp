#ifndef POSES_LOADER_HPP__
#define POSES_LOADER_HPP__

#pragma once

#include <optional>

#include "rclcpp/rclcpp.hpp"

#include <yaml-cpp/yaml.h>
#include <Eigen/Geometry>

#include "geometry_msgs/msg/pose.hpp"

class PosesLoader : public rclcpp::Node
{
  using Pose = geometry_msgs::msg::Pose;

public:
  PosesLoader() : Node("poses_loader")
  {

  }

  ~PosesLoader() = default;

  std::optional<geometry_msgs::msg::Pose> parse_pose(const YAML::Node& node)
  {
    if (!node || !node["position"] || !node["orientation"]) 
    {
      RCLCPP_ERROR(get_logger(), "The pose node is empty");
      return std::nullopt;
    }

    try 
    {
      Pose pose;
      pose.position.x = node["position"][0].as<double>();
      pose.position.y = node["position"][1].as<double>();
      pose.position.z = node["position"][2].as<double>();
      pose.orientation.x = node["orientation"][0].as<double>();
      pose.orientation.y = node["orientation"][1].as<double>();
      pose.orientation.z = node["orientation"][2].as<double>();
      pose.orientation.w = node["orientation"][3].as<double>();
      
      return std::make_optional(std::move(pose));
    } 
    catch (const YAML::Exception& e) 
    {
      RCLCPP_ERROR(get_logger(), "Failed to parse pose: %s", e.what());
      return std::nullopt;
    }
  }

  void load_pose_from_yaml(const YAML::Node& node, const std::string& name, Pose& target_pose)
  {
    if (!node || !node[name]) 
    {
      RCLCPP_ERROR(get_logger(), "Invalid name: %s", name.c_str());
      return;
    }

    std::optional<Pose> pose = parse_pose(node[name]);
    if (pose.has_value()) 
      target_pose = std::move(pose.value());
    else
      RCLCPP_ERROR(get_logger(), "Invalid %s", name.c_str());
  }

  void load_poses_from_yaml(
    const YAML::Node& node, 
    const std::string& name, 
    std::unordered_map<int, Pose>& target_poses)
  {
    if (!node || !node[name]) 
    {
      RCLCPP_ERROR(get_logger(), "Invalid name: %s", name.c_str());
      return;
    }

    target_poses.clear();

    for (const auto& scan_pose : node[name]) 
    {
      int sku_id = scan_pose["sku_id"].as<int>();
      
      std::optional<Pose> pose = parse_pose(scan_pose["pose"]);
      if (!pose.has_value()) 
      {
        RCLCPP_ERROR(get_logger(), "Invalid pose for item %d", sku_id);
        continue;
      }
      
      target_poses[sku_id] = pose.value();
    }
  }

  void load_ordered_poses_from_yaml(
    const YAML::Node& node, 
    const std::string& name, 
    std::unordered_map<int, Pose>& target_poses,
    std::vector<std::pair<int, int>>& order)
  {
    if (!node || !node[name]) 
    {
      RCLCPP_ERROR(get_logger(), "Invalid name: %s", name.c_str());
      return;
    }

    target_poses.clear();

    for (const auto& scan_pose : node[name]) 
    {
      int scan_order = -1;
      scan_order = scan_pose["scan_order"].as<int>();
      int sku_id = scan_pose["sku_id"].as<int>();

      std::optional<Pose> pose = parse_pose(scan_pose["pose"]);
      if (!pose.has_value()) 
      {
        RCLCPP_ERROR(get_logger(), "Invalid pose for item %d (scan_order %d)", sku_id, scan_order);
        continue;
      }
      
      target_poses[sku_id] = pose.value();
      order.emplace_back(scan_order, sku_id);
    }
    
    std::sort(order.begin(), order.end(), 
      [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
          return a.first < b.first;
      });
  }

  void load_waypoints_from_yaml(const YAML::Node& node, const std::string& name, std::vector<Pose>& target_waypoints)
  {
    if (!node || !node[name]) 
    {
      RCLCPP_ERROR(get_logger(), "Invalid name: %s", name.c_str());
      return;
    }
    
    target_waypoints.clear();
    std::vector<std::pair<int, Pose>> ordered_waypoints;

    for (const auto& waypoint : node[name]) 
    {
      int order = waypoint["order"].as<int>();

      std::optional<Pose> pose = parse_pose(waypoint["pose"]);
      if (pose.has_value()) 
        target_waypoints.emplace_back(std::move(pose.value()));
      else
        RCLCPP_ERROR(get_logger(), "Invalid pose in %s (order %d)", name.c_str(), order);
    }

    std::sort(ordered_waypoints.begin(), ordered_waypoints.end(),
      [](const auto& a, const auto& b) {
          return a.first < b.first;
      });

    for (auto& [order, pose] : ordered_waypoints) 
    {
      target_waypoints.emplace_back(std::move(pose));
    }
  }

  std::optional<YAML::Node> parse_yaml(const std::string& file_path)
  {
    if (file_path.empty())
    {
      RCLCPP_ERROR(get_logger(), "file does not exist");
      return std::nullopt;
    }

    RCLCPP_WARN(get_logger(), "Path: %s", file_path.c_str());

    try 
    {
      return std::make_optional(std::move(YAML::LoadFile(file_path)));
    } 
    catch (const YAML::Exception& e) 
    {
      RCLCPP_ERROR(get_logger(), "Failed to parse YAML file: %s", e.what());
      return std::nullopt;
    }

    return std::nullopt;
  }
};

#endif // POSES_LOADER_HPP__
