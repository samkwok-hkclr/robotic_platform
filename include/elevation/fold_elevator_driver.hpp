#ifndef FOLD_ELEVATOR_DRIVER_HPP__
#define FOLD_ELEVATOR_DRIVER_HPP__

#pragma once

#include <unordered_map>
#include <string>
#include <mutex>

#include "rclcpp/rclcpp.hpp"

#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "robotic_platform_msgs/srv/rotate.hpp"
#include "robotic_platform_msgs/srv/elevate.hpp"

#include "planner_base.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

enum class RotationDirection 
{
  FRONT = 1,
  LEFT, 
  RIGHT,
  BACK
};

class FoldElevatorDriver : public PlannerBase
{
  using Trigger = std_srvs::srv::Trigger;

  using Pose = geometry_msgs::msg::Pose;
  using TransformStamped = geometry_msgs::msg::TransformStamped;
  using JointState = sensor_msgs::msg::JointState;

  using Rotate = robotic_platform_msgs::srv::Rotate;
  using Elevate = robotic_platform_msgs::srv::Elevate;

  using GetJointLimits = robot_controller_msgs::srv::GetJointLimits;
  using ExecuteJoints = robot_controller_msgs::srv::ExecuteJoints;
  using ExecutePose = robot_controller_msgs::srv::ExecutePose;
  using ExecuteWaypoints = robot_controller_msgs::srv::ExecuteWaypoints;

public:
  explicit FoldElevatorDriver(const rclcpp::NodeOptions& options);
  ~FoldElevatorDriver();

  bool move_to_home_joint(void);
  double get_last_joint_diff(void);
  tf2::Quaternion clean_quat(const tf2::Quaternion& quat, double threshold = 1e-6);
  bool exec_wps(std::vector<Pose> wps);

  void testing_cb(void);
  void joint_states_cb(const JointState::SharedPtr msg);

  void move_to_cb(
    const std::shared_ptr<Trigger::Request> request, 
    std::shared_ptr<Trigger::Response> response);
  void rotate_cb(
    const std::shared_ptr<Rotate::Request> request, 
    std::shared_ptr<Rotate::Response> response);
  void elevate_cb(
    const std::shared_ptr<Elevate::Request> request, 
    std::shared_ptr<Elevate::Response> response);
  void rotate_to_cb(
    const std::shared_ptr<Trigger::Request> request, 
    std::shared_ptr<Trigger::Response> response,
    std::string_view pose);

  bool rotate_to(RotationDirection direction);
  bool rotate_to_abs_front(void);
  bool rotate_to_abs_left(void);
  bool rotate_to_abs_right(void);
  bool rotate_to_abs_back(void);

  bool rotate_relative(double primary_delta, double fallback_delta, const std::string& direction_name);
  bool rotate_to_relative_back(void);
  bool rotate_to_relative_left(void);
  bool rotate_to_relative_right(void);

  bool rotate(double yaw, std::string* message = nullptr);

  bool rotate_by_joints(double yaw, std::string* message = nullptr);
  bool rotate_by_pose(double yaw, std::string* message = nullptr);

  bool elevate(double x, double z, double yaw);

  double normalize_angle(double angle);
  double shortest_angular_distance(double from, double to);

  std::chrono::duration<double> get_cli_req_timeout() const override 
  {
    return CLI_REQ_TIMEOUT;
  }

private:
  std::mutex mutex_;
  std::mutex joint_states_mutex_;

  std::vector<double> home_joints_;

  std::unordered_map<std::string, double> joint_states_;
  std::unordered_map<std::string, size_t> joint_index_;

  std::map<std::string_view, std::function<bool(void)>, std::less<>> rotate_actions;

  rclcpp::CallbackGroup::SharedPtr sub_cbg_;
  rclcpp::CallbackGroup::SharedPtr timer_cbg_;
  rclcpp::CallbackGroup::SharedPtr srv_cli_cbg_;
  rclcpp::CallbackGroup::SharedPtr srv_ser_cbg_;

  rclcpp::TimerBase::SharedPtr testing_timer_;

  rclcpp::Subscription<JointState>::SharedPtr joint_states_sub_;

  rclcpp::Client<GetJointLimits>::SharedPtr get_joint_limits_cli_;
  rclcpp::Client<ExecuteJoints>::SharedPtr exec_joints_cli_;
  rclcpp::Client<ExecutePose>::SharedPtr exec_pose_cli_;
  rclcpp::Client<ExecuteWaypoints>::SharedPtr exec_wps_cli_;

  rclcpp::Service<Trigger>::SharedPtr home_srv_;
  rclcpp::Service<Rotate>::SharedPtr rotate_srv_;
  rclcpp::Service<Elevate>::SharedPtr elevate_srv_;
  std::vector<rclcpp::Service<Trigger>::SharedPtr> rotate_actions_srv_;

  constexpr static std::chrono::duration CLI_REQ_TIMEOUT = std::chrono::seconds(30);

  const std::vector<std::string> keys = {"joint1", "joint2", "joint3", "joint4"};


  // It is because tf2 does not provide high accuracy of floating value
  constexpr static double ABS_Y_DISTANCE = -0.00665;
};

#endif // FOLD_ELEVATOR_DRIVER_HPP__