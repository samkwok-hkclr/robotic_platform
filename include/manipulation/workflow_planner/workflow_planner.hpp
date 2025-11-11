#ifndef WORKFLOW_PLANNER_HPP__
#define WORKFLOW_PLANNER_HPP__

#pragma once

#include <cmath>
#include <functional>
#include <future>
#include <memory>
#include <chrono>
#include <algorithm>
#include <tuple>
#include <utility>
#include <map>
#include <unordered_map>

#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "robotic_platform_msgs/action/pick.hpp"
#include "robotic_platform_msgs/action/place.hpp"
#include "robotic_platform_msgs/action/scan_sku.hpp"
#include "robotic_platform_msgs/action/replenish.hpp"

#include "robotic_platform_msgs/msg/pick_plan_result.hpp"
#include "robotic_platform_msgs/msg/place_plan_result.hpp"
#include "robotic_platform_msgs/msg/replenish_pose.hpp"
#include "robotic_platform_msgs/msg/replenish_quantity.hpp"
#include "robotic_platform_msgs/msg/robot_status.hpp"
#include "robotic_platform_msgs/msg/localization_param.hpp"
#include "robotic_platform_msgs/msg/object_pose.hpp"
#include "robotic_platform_msgs/msg/rack_info.hpp"
#include "robotic_platform_msgs/msg/table_info.hpp"

#include "robotic_platform_msgs/srv/get_slot_state_trigger.hpp"
#include "robotic_platform_msgs/srv/get_object_pose_trigger.hpp"
#include "robotic_platform_msgs/srv/pick_plan.hpp"
#include "robotic_platform_msgs/srv/place_plan.hpp"

#include "planner_base.hpp"
#include "elevation/fold_elevator_driver.hpp"
#include "manipulation/poses_loader.hpp"
#include "manipulation/motion_planner/motion_planner.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class WorkflowPlanner : public PlannerBase
{
  using Float32 = std_msgs::msg::Float32;
  using Pose = geometry_msgs::msg::Pose;
  using TransformStamped = geometry_msgs::msg::TransformStamped;

  using Transition = lifecycle_msgs::msg::Transition;
  using ChangeState = lifecycle_msgs::srv::ChangeState;

  using PickPlanResult = robotic_platform_msgs::msg::PickPlanResult;
  using PlacePlanResult = robotic_platform_msgs::msg::PlacePlanResult;

  using PickResult = robotic_platform_msgs::msg::PickResult;
  using PickTask = robotic_platform_msgs::msg::PickTask;
  using PlaceResult = robotic_platform_msgs::msg::PlaceResult;
  using PlaceTask = robotic_platform_msgs::msg::PlaceTask;

  using ReplenishPose = robotic_platform_msgs::msg::ReplenishPose;
  using ReplenishQuantity = robotic_platform_msgs::msg::ReplenishQuantity;
  using RobotStatus = robotic_platform_msgs::msg::RobotStatus;
  using LocalizationParam = robotic_platform_msgs::msg::LocalizationParam;
  using ObjectPose = robotic_platform_msgs::msg::ObjectPose;
  using RackInfo = robotic_platform_msgs::msg::RackInfo;
  using TableInfo = robotic_platform_msgs::msg::TableInfo;

  using GetSlotStateTrigger = robotic_platform_msgs::srv::GetSlotStateTrigger;
  using GetObjectPoseTrigger = robotic_platform_msgs::srv::GetObjectPoseTrigger;
  using PickPlan = robotic_platform_msgs::srv::PickPlan;
  using PlacePlan = robotic_platform_msgs::srv::PlacePlan;

  using Pick = robotic_platform_msgs::action::Pick;
  using Place = robotic_platform_msgs::action::Place;
  using ScanSku = robotic_platform_msgs::action::ScanSku;
  using Replenish = robotic_platform_msgs::action::Replenish;

  using GoalHandlerPick = rclcpp_action::ServerGoalHandle<Pick>;
  using GoalHandlerPlace = rclcpp_action::ServerGoalHandle<Place>;
  using GoalHandlerScanSku = rclcpp_action::ServerGoalHandle<ScanSku>;
  using GoalHandlerReplenish = rclcpp_action::ServerGoalHandle<Replenish>;
  
public:
  explicit WorkflowPlanner(
    const rclcpp::NodeOptions& options,
    std::shared_ptr<FoldElevatorDriver> fold_elev_driver, 
    std::shared_ptr<MotionPlanner> motion_planner);
  ~WorkflowPlanner();

  void setup_camera_transform(RobotArm arm, const std::string& cam_name);

  void push_tf_buf(const std::tuple<Pose, std::string, std::string>& tf);
  void clear_tf_buf();

  std::string get_flat_link(uint8_t rack_id, uint8_t shelf_level) const;
  std::string get_place_link(uint8_t table_id, uint8_t index) const;

  bool set_camera_lifecycle(RobotArm arm, bool activate);

  bool optimal_pick_elvation(const RackInfo& rack);
  bool optimal_place_elvation(const TableInfo& table);

  std::optional<Pose> get_scan_pose(uint8_t rack_id, uint8_t shelf_level, uint8_t shelf_slot);

  std::optional<double> try_to_pick_up(
    RobotArm arm, const int32_t sku_id, const uint8_t camera_id, const RackInfo& rack);

  bool try_to_place_down(
    RobotArm arm, double height, const TableInfo& table);

  std::optional<std::vector<ObjectPose>> try_to_scan(
    RobotArm arm, const int32_t sku_id, const uint8_t camera_id, const Pose& scan_pose);

  std::optional<Pose> extract_object_pose(RobotArm arm, const std::vector<ObjectPose>& poses_in_camera);

  std::optional<std::vector<ObjectPose>> get_obj_poses(const int32_t sku_id, const uint8_t camera_id);
  std::optional<PickPlanResult> get_pick_plan(const Pose& object_pose, const RackInfo& rack, const std::string& flat_frame);
  std::optional<PlacePlanResult> get_place_plan(const Pose& place_pose);

  void tf_pub_cb(void);
  void debug_cb(const Float32::SharedPtr msg);

private:
  std::mutex mutex_;
  std::mutex tf_mutex_;
  
  std::vector<std::tuple<Pose, std::string, std::string>> tf_buf_;
  std::map<RobotArm, tf2::Transform> g_tcp__cam_;

  std::shared_ptr<FoldElevatorDriver> fold_elev_driver_;
  std::shared_ptr<MotionPlanner> motion_planner_;
  
  uint8_t max_pick_attempt_;
  double valid_z_threshold_;
  uint16_t max_scan_attempt_;
  double re_scan_translation_;
  double place_offset_;
  double scan_distance_;

  double optimal_arm_flat_height_distance_;
  double optimal_arm_flat_front_distance_;

  double table_front_offset_;
  double table_height_offset_;
  uint8_t state_ = 0; // FIXME

  // ============== Callback Groups ==============

  rclcpp::CallbackGroup::SharedPtr action_ser_cbg_;
  rclcpp::CallbackGroup::SharedPtr vision_srv_cli_cbg_;
  rclcpp::CallbackGroup::SharedPtr plan_srv_cli_cbg_;
  rclcpp::CallbackGroup::SharedPtr cam_srv_cli_cbg_;
  rclcpp::CallbackGroup::SharedPtr exec_timer_cbg_;
  rclcpp::CallbackGroup::SharedPtr tf_timer_cbg_;

  // ============== Timers ==============

  rclcpp::TimerBase::SharedPtr tf_pub_timer;

  // ============== Subscriber ==============

  rclcpp::Subscription<Float32>::SharedPtr debug_sub_;

  // ============== Services Clients ==============

  rclcpp::Client<GetSlotStateTrigger>::SharedPtr get_slot_state_tri_cli_;
  rclcpp::Client<GetObjectPoseTrigger>::SharedPtr get_obj_pose_tri_cli_;

  rclcpp::Client<PickPlan>::SharedPtr pick_plan_cli_;
  rclcpp::Client<PlacePlan>::SharedPtr place_plan_cli_;

  std::map<RobotArm, rclcpp::Client<ChangeState>::SharedPtr> camera_cli_;

  // ============== Action Sersers ==============

  rclcpp_action::Server<Pick>::SharedPtr pick_action_ser_;
  rclcpp_action::Server<Place>::SharedPtr place_action_ser_;
  rclcpp_action::Server<ScanSku>::SharedPtr scan_sku_action_ser_;
  rclcpp_action::Server<Replenish>::SharedPtr replenish_action_ser_;

  rclcpp_action::GoalResponse pick_goal_cb(
    const rclcpp_action::GoalUUID & uuid, 
    std::shared_ptr<const Pick::Goal> goal);
  rclcpp_action::CancelResponse pick_cancel_cb(
    const std::shared_ptr<GoalHandlerPick> goal_handle);
  void pick_accepted(const std::shared_ptr<GoalHandlerPick> goal_handle);
  void pick_execution(const std::shared_ptr<GoalHandlerPick> goal_handle);

  rclcpp_action::GoalResponse place_goal_cb(
    const rclcpp_action::GoalUUID & uuid, 
    std::shared_ptr<const Place::Goal> goal);
  rclcpp_action::CancelResponse place_cancel_cb(
    const std::shared_ptr<GoalHandlerPlace> goal_handle);
  void place_accepted(const std::shared_ptr<GoalHandlerPlace> goal_handle);
  void place_execution(const std::shared_ptr<GoalHandlerPlace> goal_handle);

  rclcpp_action::GoalResponse scan_sku_goal_cb(
    const rclcpp_action::GoalUUID & uuid, 
    std::shared_ptr<const ScanSku::Goal> goal);
  rclcpp_action::CancelResponse scan_sku_cancel_cb(
    const std::shared_ptr<GoalHandlerScanSku> goal_handle);
  void scan_sku_accepted(const std::shared_ptr<GoalHandlerScanSku> goal_handle);
  void scan_sku_execution(const std::shared_ptr<GoalHandlerScanSku> goal_handle);

  rclcpp_action::GoalResponse replenish_goal_cb(
    const rclcpp_action::GoalUUID & uuid, 
    std::shared_ptr<const Replenish::Goal> goal);
  rclcpp_action::CancelResponse replenish_cancel_cb(
    const std::shared_ptr<GoalHandlerReplenish> goal_handle);
  void replenish_accepted(const std::shared_ptr<GoalHandlerReplenish> goal_handle);
  void replenish_execution(const std::shared_ptr<GoalHandlerReplenish> goal_handle);

};

#endif // WORKFLOW_PLANNER_HPP__