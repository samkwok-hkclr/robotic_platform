#include "elevation/fold_elevator_driver.hpp"

FoldElevatorDriver::FoldElevatorDriver(
  const rclcpp::NodeOptions& options)
: PlannerBase("fold_elevator_driver", options)
{
  declare_parameter<std::vector<double>>("home", std::vector<double>{});

  home_joints_ = get_parameter("home").as_double_array();

  sub_cbg_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  timer_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_cli_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_ser_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = sub_cbg_;

  testing_timer_ = create_wall_timer(
    std::chrono::seconds(1), 
    std::bind(&FoldElevatorDriver::testing_cb, this), 
    timer_cbg_);
  testing_timer_->cancel();

  joint_states_sub_ = create_subscription<JointState>(
    "/joint_states", 
    10, 
    std::bind(&FoldElevatorDriver::joint_states_cb, this, _1),
    sub_options);

  get_joint_limits_cli_ = create_client<GetJointLimits>(
    "/fold_elevator/get_joint_limits", 
    rmw_qos_profile_services_default,
    srv_cli_cbg_);

  exec_joints_cli_ = create_client<ExecuteJoints>(
    "/fold_elevator/execute_joints", 
    rmw_qos_profile_services_default,
    srv_cli_cbg_);

  exec_pose_cli_ = create_client<ExecutePose>(
    "/fold_elevator/execute_pose", 
    rmw_qos_profile_services_default,
    srv_cli_cbg_);

  exec_wps_cli_ = create_client<ExecuteWaypoints>(
    "/fold_elevator/execute_waypoints", 
    rmw_qos_profile_services_default,
    srv_cli_cbg_);

  home_srv_ = create_service<Trigger>(
    "/fold_elevator/move_to_home_pose", 
    std::bind(&FoldElevatorDriver::move_to_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  rotate_srv_ = create_service<Rotate>(
    "/fold_elevator/rotate", 
    std::bind(&FoldElevatorDriver::rotate_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  elevate_srv_ = create_service<Elevate>(
    "/fold_elevator/elevate", 
    std::bind(&FoldElevatorDriver::elevate_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  rotate_actions = {
    { "abs_front", std::bind(&FoldElevatorDriver::rotate_to_abs_front, this) },
    { "abs_back", std::bind(&FoldElevatorDriver::rotate_to_abs_back, this) },
    { "abs_left", std::bind(&FoldElevatorDriver::rotate_to_abs_left, this) },
    { "abs_right", std::bind(&FoldElevatorDriver::rotate_to_abs_right, this) },

    { "relative_back", std::bind(&FoldElevatorDriver::rotate_to_relative_back, this) },
    { "relative_left", std::bind(&FoldElevatorDriver::rotate_to_relative_left, this) },
    { "relative_right", std::bind(&FoldElevatorDriver::rotate_to_relative_right, this) },
  };

  for (const auto& action : rotate_actions)
  {
    rotate_actions_srv_.emplace_back(std::move(
      create_service<Trigger>(
        "/fold_elevator/rotate_to_" + std::string(action.first), 
        std::bind(&FoldElevatorDriver::rotate_to_cb, this, _1, _2, action.first),
        rmw_qos_profile_services_default,
        srv_ser_cbg_)
      )
    );
  }

  for (const auto& k : keys)
  {
    joint_states_[k] = 0.0;
  }

  RCLCPP_INFO(get_logger(), "Workflow Planner is up.");
}

FoldElevatorDriver::~FoldElevatorDriver()
{

}

void FoldElevatorDriver::joint_states_cb(const JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(joint_states_mutex_);

  if (joint_index_.size() != msg->name.size()) 
  {
    joint_index_.clear();

    for (size_t i = 0; i < msg->name.size(); ++i) 
    {
      joint_index_[msg->name[i]] = i;
    }
  }

  for (const auto& kv : joint_states_) 
  {
    const std::string& name = kv.first;
    auto it = joint_index_.find(name);
    if (it != joint_index_.end()) 
    {
      size_t idx = it->second;
      joint_states_[name] = (idx < msg->position.size()) ? msg->position[idx] : 0.0;
    } 
    else 
    {
      joint_states_[name] = 0.0;   // joint not present in this message
    }
  }
}

void FoldElevatorDriver::move_to_cb(
  const std::shared_ptr<Trigger::Request> request, 
  std::shared_ptr<Trigger::Response> response)
{
  (void) request;
  
  auto req = std::make_shared<ExecuteJoints::Request>();
  req->joints.insert(req->joints.end(), home_joints_.begin(), home_joints_.end());

  ExecuteJoints::Response::SharedPtr res;
  if (!send_sync_req<ExecuteJoints>(exec_joints_cli_, std::move(req), res, __FUNCTION__)) 
  {
    RCLCPP_ERROR(get_logger(), "Sent ExecuteJoints request failed");
    return;
  }

  if (!res->success) 
  {
    RCLCPP_WARN(get_logger(), "ExecuteJoints rejected");
    return;
  }

  RCLCPP_INFO(get_logger(), "Successfully elevated fold elevator");
  response->success = true;
  return;
}

void FoldElevatorDriver::rotate_cb(
  const std::shared_ptr<Rotate::Request> request, 
  std::shared_ptr<Rotate::Response> response)
{
  if (rotate(request->yaw, &response->message))
  {
    response->success = true;
  }
}

void FoldElevatorDriver::elevate_cb(
  const std::shared_ptr<Elevate::Request> request, 
  std::shared_ptr<Elevate::Response> response)
{
  const double POSITION_THRESHOLD = 0.005;
  const double ROTATION_THRESHOLD = 0.0523598776; // 3 deg in rad
  
  RCLCPP_INFO(get_logger(), "Received elevate request - x: %.4f, z: %.4f, yaw: %.4f rad (%.4f deg)", 
    request->x, request->z, request->yaw, request->yaw * 180.0/M_PI);

  // Check if position values are too small
  if (std::abs(request->x) <= POSITION_THRESHOLD && std::abs(request->z) <= POSITION_THRESHOLD)
  {
    RCLCPP_WARN(get_logger(), "Position values below threshold - x: %.4f, z: %.4f (threshold: %.4f)", 
      request->x, request->z, POSITION_THRESHOLD);

    if (std::abs(request->yaw) >= ROTATION_THRESHOLD)
    {
      RCLCPP_INFO(get_logger(), "Attempting rotation - yaw: %.4f rad (%.1f°)", request->yaw, request->yaw * 180.0/M_PI);
      
      if (rotate(request->yaw))
      {
        RCLCPP_INFO(get_logger(), "Rotation completed successfully");
        response->success = true;
      }
      else
      {
        RCLCPP_ERROR(get_logger(), "Rotation failed for yaw: %.4f rad", request->yaw);
        response->message = "Failed to rotate";
        return;
      }
    }
    else if (std::abs(request->x) <= POSITION_THRESHOLD && std::abs(request->z) <= POSITION_THRESHOLD)
    {
      // Both position and rotation are too small
      RCLCPP_WARN(get_logger(), "Rejecting request - all values below thresholds (x: %.4f, z: %.4f, yaw: %.4f)", 
        request->x, request->z, request->yaw);
      response->message = "Both position (x,z) and rotation (yaw) values are below minimum thresholds";
      return;
    }
  }
  
  RCLCPP_INFO(get_logger(), "Attempting elevation - x: %.4f, z: %.4f, yaw: %.4f rad", request->x, request->z, request->yaw);
  if (elevate(request->x, request->z, request->yaw))
  {
    RCLCPP_INFO(get_logger(), "Elevation completed successfully");
    response->success = true;
    response->message = "Elevation completed successfully";
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "Elevation failed - x: %.4f, z: %.4f, yaw: %.4f rad", request->x, request->z, request->yaw);
    response->message = "Failed to elevate";
  }
}

void FoldElevatorDriver::rotate_to_cb(
  const std::shared_ptr<Trigger::Request> request, 
  std::shared_ptr<Trigger::Response> response,
  std::string_view pose)
{
  (void) request;

  if (auto it = rotate_actions.find(pose); it != rotate_actions.end()) 
  {
    if (it->second()) 
    {
      RCLCPP_INFO(get_logger(), "Successfully rotated to pose '%s'", pose.data());
      response->success = true;
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to rotate to pose '%s'", pose.data());
    }
  } 
}

bool FoldElevatorDriver::exec_wps(std::vector<Pose> wps)
{
  auto req = std::make_shared<ExecuteWaypoints::Request>();
  req->waypoints = std::move(wps);
  req->speed = 60;

  ExecuteWaypoints::Response::SharedPtr res;
  if (!send_sync_req<ExecuteWaypoints>(exec_wps_cli_, req, res, __FUNCTION__)) 
  {
    RCLCPP_ERROR(get_logger(), "Sent ExecuteWaypoints request failed");
    return false;
  }

  if (!res->success) 
  {
    RCLCPP_WARN(get_logger(), "ExecuteWaypoints rejected");
    return false;
  }

  RCLCPP_INFO(get_logger(), "Successfully elevated fold elevator");
  return true;
}

void FoldElevatorDriver::testing_cb()
{
  auto tf_stamped = get_tf(BASE_FOOTPRINT, ELEV_FLAT_LINK);

  if (!tf_stamped.has_value()) 
  {
    RCLCPP_WARN(get_logger(), "TF lookup failed: %s -> %s", BASE_FOOTPRINT.c_str(), ELEV_FLAT_LINK.c_str());
    return;
  }

  tf2::Transform g_b__elev_flat;
  tf2::fromMsg(tf_stamped.value().transform, g_b__elev_flat);

  RCLCPP_INFO(get_logger(), "x: %.8f'", g_b__elev_flat.getOrigin().getX());
  RCLCPP_INFO(get_logger(), "y: %.8f'", g_b__elev_flat.getOrigin().getY());
  RCLCPP_INFO(get_logger(), "z: %.8f'", g_b__elev_flat.getOrigin().getZ());
  RCLCPP_INFO(get_logger(), "qx: %.8f'", g_b__elev_flat.getRotation().getX());
  RCLCPP_INFO(get_logger(), "qy: %.8f'", g_b__elev_flat.getRotation().getY());
  RCLCPP_INFO(get_logger(), "qz: %.8f'", g_b__elev_flat.getRotation().getZ());
  RCLCPP_INFO(get_logger(), "qw: %.8f'", g_b__elev_flat.getRotation().getW());
}

