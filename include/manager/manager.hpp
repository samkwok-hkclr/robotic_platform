#ifndef MANAGER_HPP__
#define MANAGER_HPP__

#pragma once

#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"


#include "geometry_msgs/msg/pose.hpp"

#include "robostore_msgs/msg/order_item.hpp"
#include "robostore_msgs/srv/new_order.hpp"

#include "robotic_platform_msgs/msg/pick_task.hpp"
#include "robotic_platform_msgs/msg/pick_result.hpp"
#include "robotic_platform_msgs/msg/place_task.hpp"
#include "robotic_platform_msgs/msg/place_result.hpp"

#include "robotic_platform_msgs/action/pick.hpp"
#include "robotic_platform_msgs/action/place.hpp"
#include "robotic_platform_msgs/action/scan_sku.hpp"
#include "robotic_platform_msgs/action/replenish.hpp"

#include "planner_base.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class Manager : public PlannerBase
{
  using Trigger = std_srvs::srv::Trigger;
  using Pose = geometry_msgs::msg::Pose;

  using OrderItem = robostore_msgs::msg::OrderItem;
  using NewOrder = robostore_msgs::srv::NewOrder;

  using PickTask = robotic_platform_msgs::msg::PickTask;
  using PickResult = robotic_platform_msgs::msg::PickResult;
  using PlaceTask = robotic_platform_msgs::msg::PlaceTask;
  using PlaceResult = robotic_platform_msgs::msg::PlaceResult;

  using Pick = robotic_platform_msgs::action::Pick;
  using Place = robotic_platform_msgs::action::Place;
  using ScanSku = robotic_platform_msgs::action::ScanSku;
  using Replenish = robotic_platform_msgs::action::Replenish;

  using PickGoalHandle = rclcpp_action::ClientGoalHandle<Pick>;
  using PlaceGoalHandle = rclcpp_action::ClientGoalHandle<Place>;
  
public:
	explicit Manager(std::string node_name, const rclcpp::NodeOptions& options);
	~Manager() = default;

  std::vector<size_t> select_next_items(
    const std::vector<OrderItem>& items, 
    const std::vector<bool>& items_completed);

  std::optional<std::map<uint8_t, double>> send_pick_goal(
    const std::vector<OrderItem>& order_items,
    const std::vector<size_t>& items_selected);
  bool send_place_goal(
    const std::vector<OrderItem>& order_items,
    const std::vector<size_t>& items_selected,
    const uint8_t table_id,
    const std::map<uint8_t, double>& place_height);

  bool rotate_to(std::string direction);
  bool move_to_basic_pose(RobotArm arm, const std::string& pose);

  std::chrono::duration<double> get_cli_req_timeout() const override 
  {
    return CLI_REQ_TIMEOUT;
  }

private:
  std::mutex mutex_;

  std::vector<std::string> rotation_name_;
  std::vector<std::string> arm_poses_name_;

  rclcpp::CallbackGroup::SharedPtr srv_ser_cbg_;
  rclcpp::CallbackGroup::SharedPtr srv_cli_cbg_;
  rclcpp::CallbackGroup::SharedPtr action_cli_cbg_;

  rclcpp::Service<NewOrder>::SharedPtr new_order_srv_;

  std::unordered_map<std::string, rclcpp::Client<Trigger>::SharedPtr> fold_elev_rotate_cli_;
  std::unordered_map<RobotArm, std::unordered_map<std::string, rclcpp::Client<Trigger>::SharedPtr>> arm_basic_ctrl_cli_;
  
  rclcpp_action::Client<Pick>::SharedPtr pick_cli_;
  rclcpp_action::Client<Place>::SharedPtr place_cli_;
  
  void new_order_cb(
    const std::shared_ptr<NewOrder::Request> request, 
    std::shared_ptr<NewOrder::Response> response);

  void pick_feedback_cb(PickGoalHandle::SharedPtr, const std::shared_ptr<const Pick::Feedback> feedback);
  void place_feedback_cb(PlaceGoalHandle::SharedPtr, const std::shared_ptr<const Place::Feedback> feedback);

  constexpr static std::chrono::duration ACTION_TIMEOUT = std::chrono::seconds(180);
  constexpr static std::chrono::duration CLI_REQ_TIMEOUT = std::chrono::seconds(30);
};

#endif // MANAGER_HPP__
