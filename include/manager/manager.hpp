#ifndef MANAGER_HPP__
#define MANAGER_HPP__

#pragma once

#include <memory>
#include <chrono>
#include <random>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose.hpp"

#include "robostore_msgs/msg/robot_status.hpp"
#include "robostore_msgs/msg/item_status.hpp"
#include "robostore_msgs/msg/item_result.hpp"
#include "robostore_msgs/msg/order_item.hpp"
#include "robostore_msgs/srv/get_robot_status.hpp"
#include "robostore_msgs/srv/new_order.hpp"
#include "robostore_msgs/srv/occupy.hpp"
#include "robostore_msgs/action/new_order.hpp"

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
  using Int32 = std_msgs::msg::Int32;
  using Trigger = std_srvs::srv::Trigger;
  using Pose = geometry_msgs::msg::Pose;

  using RobotStatus = robostore_msgs::msg::RobotStatus;
  using ItemStatus = robostore_msgs::msg::ItemStatus;
  using ItemResult = robostore_msgs::msg::ItemResult;
  using OrderItem = robostore_msgs::msg::OrderItem;
  using GetRobotStatus = robostore_msgs::srv::GetRobotStatus;
  using NewOrderSrv = robostore_msgs::srv::NewOrder;
  using Occupy = robostore_msgs::srv::Occupy;
  using NewOrderAction = robostore_msgs::action::NewOrder;

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
  
  using NewOrderGoalHandle = rclcpp_action::ServerGoalHandle<NewOrderAction>;
  
public:
	explicit Manager(const rclcpp::NodeOptions& options);
	~Manager() = default;

  void testing_cb(void);
  void robot_status_cb(void);
  void clear_occupancy_cb(void);

  std::vector<size_t> select_next_items(
    const std::vector<OrderItem>& items, 
    const std::vector<bool>& items_completed);

  std::optional<std::map<uint8_t, double>> send_pick_goal(
    const std::vector<OrderItem>& order_items,
    const std::vector<size_t>& items_selected);
  bool send_place_goal(
    const std::vector<OrderItem>& order_items,
    const std::vector<size_t>& items_selected,
    const uint8_t port_id,
    const std::map<uint8_t, double>& place_height,
    std::vector<bool>& place_occupancy_map);

  bool rotate_to(std::string direction);
  bool move_to_basic_pose(RobotArm arm, const std::string& pose);

  std::chrono::duration<double> get_cli_req_timeout() const override 
  {
    return CLI_REQ_TIMEOUT;
  }

private:
  std::mutex mutex_;
  std::atomic<uint8_t> robot_status_;
  std::string client_name_;

  std::vector<std::string> rotation_name_;
  std::vector<std::string> arm_poses_name_;

  rclcpp::CallbackGroup::SharedPtr srv_ser_cbg_;
  rclcpp::CallbackGroup::SharedPtr srv_cli_cbg_;
  rclcpp::CallbackGroup::SharedPtr action_cli_cbg_;
  rclcpp::CallbackGroup::SharedPtr action_ser_cbg_;

  rclcpp::TimerBase::SharedPtr testing_timer_;
  rclcpp::TimerBase::SharedPtr robot_status_timer_;
  rclcpp::TimerBase::SharedPtr clear_occupy_timer_;

  rclcpp::Publisher<Int32>::SharedPtr testing_pub_;
  rclcpp::Publisher<RobotStatus>::SharedPtr robot_status_pub_;

  rclcpp::Service<GetRobotStatus>::SharedPtr get_robot_status_srv_;
  rclcpp::Service<Occupy>::SharedPtr occupy_srv_;
  rclcpp::Service<NewOrderSrv>::SharedPtr new_order_srv_;
  rclcpp_action::Server<NewOrderAction>::SharedPtr new_order_action_ser_;

  std::unordered_map<std::string, rclcpp::Client<Trigger>::SharedPtr> fold_elev_rotate_cli_;
  std::unordered_map<RobotArm, std::unordered_map<std::string, rclcpp::Client<Trigger>::SharedPtr>> arm_basic_ctrl_cli_;
  
  rclcpp_action::Client<Pick>::SharedPtr pick_cli_;
  rclcpp_action::Client<Place>::SharedPtr place_cli_;
  
  void get_robot_status_cb(
    const std::shared_ptr<GetRobotStatus::Request> request, 
    std::shared_ptr<GetRobotStatus::Response> response);
  void occupy_cb(
    const std::shared_ptr<Occupy::Request> request, 
    std::shared_ptr<Occupy::Response> response);
  void new_order_cb(
    const std::shared_ptr<NewOrderSrv::Request> request, 
    std::shared_ptr<NewOrderSrv::Response> response);

  rclcpp_action::GoalResponse new_order_goal_cb(
    const rclcpp_action::GoalUUID & uuid, 
    std::shared_ptr<const NewOrderAction::Goal> goal);
  rclcpp_action::CancelResponse new_order_cancel_cb(
    const std::shared_ptr<NewOrderGoalHandle> goal_handle);
  void new_order_accepted_cb(const std::shared_ptr<NewOrderGoalHandle> goal_handle);

  void order_execution(const std::shared_ptr<NewOrderGoalHandle> goal_handle);

  void pick_feedback_cb(PickGoalHandle::SharedPtr, const std::shared_ptr<const Pick::Feedback> feedback);
  void place_feedback_cb(PlaceGoalHandle::SharedPtr, const std::shared_ptr<const Place::Feedback> feedback);

  constexpr static std::chrono::duration ACTION_TIMEOUT = std::chrono::seconds(180);
  constexpr static std::chrono::duration CLI_REQ_TIMEOUT = std::chrono::seconds(30);
  constexpr static uint8_t MAX_ORDER_ITEMS = 6;
  const std::vector<uint8_t> LEFT_ARM_PLACE_ORDER{1, 2, 4, 5, 3, 6};
  const std::vector<uint8_t> RIGHT_ARM_PLACE_ORDER{3, 2, 6, 5, 1, 4};

};

#endif // MANAGER_HPP__
