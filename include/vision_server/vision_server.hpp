#ifndef VISION_SERVER_HPP__
#define VISION_SERVER_HPP__

#pragma once

#include <functional>
#include <future>
#include <memory>
#include <chrono>
#include <iterator>
#include <deque>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

enum CameraId: uint8_t
{
  LEFT = 1,
  RIGHT = 2
};

class VisionServer : public rclcpp::Node
{
  using Image = sensor_msgs::msg::Image;
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  
public:
  explicit VisionServer(const rclcpp::NodeOptions& options);
  ~VisionServer();

  void image_recv_cb(const Image::SharedPtr msg, const CameraId id);
  void pc_recv_cb(const PointCloud2::SharedPtr msg, const CameraId id);

private:
  std::mutex mutex_;

  std::unordered_map<CameraId, std::mutex> image_mutexes_;
  std::unordered_map<CameraId, std::mutex> pc_mutexes_;

  std::unordered_map<CameraId, std::string> image_topic_;
  std::unordered_map<CameraId, std::string> pc_topic_;

  size_t history_max_size_;

  std::unordered_map<CameraId, std::deque<Image>> image_history_;
  std::unordered_map<CameraId, std::deque<PointCloud2>> pc_history_;
 
  std::unordered_map<CameraId, std::function<void(const Image::SharedPtr msg)>> image_cb_;
  std::unordered_map<CameraId, std::function<void(const PointCloud2::SharedPtr msg)>> pc_cb_;

  rclcpp::CallbackGroup::SharedPtr image_sub_cbg_;
  rclcpp::CallbackGroup::SharedPtr pc_sub_cbg_;

  std::unordered_map<CameraId, rclcpp::Subscription<Image>::SharedPtr> image_sub_;
  std::unordered_map<CameraId, rclcpp::Subscription<PointCloud2>::SharedPtr> pc_sub_;
};

#endif // VISION_SERVER_HPP__