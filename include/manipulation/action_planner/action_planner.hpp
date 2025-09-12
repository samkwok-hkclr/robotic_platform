#ifndef ACTION_PLANNER_HPP__
#define ACTION_PLANNER_HPP__

#pragma once

#include <functional>
#include <future>
#include <memory>
#include <chrono>
#include <algorithm>

#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose.hpp"

#include "robotic_platform_msgs/msg/pick_plan_result.hpp"
#include "robotic_platform_msgs/msg/place_plan_result.hpp"

#include "robotic_platform_msgs/srv/pick_plan.hpp"
#include "robotic_platform_msgs/srv/place_plan.hpp"

#include "planner_base.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class ActionPlanner : public PlannerBase
{
  using Pose = geometry_msgs::msg::Pose;

  using PickPlanResult = robotic_platform_msgs::msg::PickPlanResult;
  using PlacePlanResult = robotic_platform_msgs::msg::PlacePlanResult;

  using PickPlan = robotic_platform_msgs::srv::PickPlan;
  using PlacePlan = robotic_platform_msgs::srv::PlacePlan;

public:
  explicit ActionPlanner(const rclcpp::NodeOptions& options);
  ~ActionPlanner();

  void tf_pub_cb(void);

  void pick_plan_cb(
    const std::shared_ptr<PickPlan::Request> request, 
    std::shared_ptr<PickPlan::Response> response);
  void place_plan_cb(
    const std::shared_ptr<PlacePlan::Request> request, 
    std::shared_ptr<PlacePlan::Response> response);

private:
  std::mutex mutex_;
  std::mutex tf_mutex_;
  
  std::vector<std::tuple<Pose, std::string, std::string>> tf_buf_;

  rclcpp::CallbackGroup::SharedPtr tf_timer_cbg_;
  rclcpp::CallbackGroup::SharedPtr srv_ser_cbg_;

  rclcpp::TimerBase::SharedPtr tf_pub_timer;

  // ============== Services ==============
  rclcpp::Service<PickPlan>::SharedPtr pick_plan_srv_;
  rclcpp::Service<PlacePlan>::SharedPtr place_plan_srv_;

  std::vector<double> eef_offset_;
  double pre_obj_pose_shift_; 
  double pre_place_pose_shift_;
  double post_pick_lift_offset_;
  double post_pick_back_offset_;
  double post_place_down_offset_;
};

#endif // ACTION_PLANNER_HPP__