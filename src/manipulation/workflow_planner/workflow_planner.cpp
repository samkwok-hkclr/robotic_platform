#include "manipulation/workflow_planner/workflow_planner.hpp"

WorkflowPlanner::WorkflowPlanner(
  const rclcpp::NodeOptions& options)
: Node("workflow_planner", options)
{
  state_ = RobotStatus::IDLE;

  srv_cli_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  action_ser_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  vison_srv_cli_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  exec_timer_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  exec_wps_cli_ = create_client<ExecuteWaypoints>(
    "execute_waypoints", 
    rmw_qos_profile_services_default,
    srv_cli_cbg_);

  get_curr_pose_cli_ = create_client<GetCurrentPose>(
    "get_current_pose", 
    rmw_qos_profile_services_default,
    srv_cli_cbg_);

  get_slot_state_tri_cli_ = create_client<GetSlotStateTrigger>(
    "get_slot_state_trigger", 
    rmw_qos_profile_services_default,
    vison_srv_cli_cbg_);
  
  get_obj_pose_tri_cli_ = create_client<GetObjectPoseTrigger>(
    "get_object_pose_trigger", 
    rmw_qos_profile_services_default,
    vison_srv_cli_cbg_);

  scan_sku_action_ser_ = rclcpp_action::create_server<ScanSku>(
    this,
    "scan_sku",
    std::bind(&WorkflowPlanner::scan_sku_goal_cb, this, _1, _2),
    std::bind(&WorkflowPlanner::scan_sku_cancel_cb, this, _1),
    std::bind(&WorkflowPlanner::scan_sku_accepted, this, _1),
    rcl_action_server_get_default_options(),
    action_ser_cbg_);

  RCLCPP_INFO(get_logger(), "Workflow Planner is up.");
}

WorkflowPlanner::~WorkflowPlanner()
{

}

geometry_msgs::msg::Pose WorkflowPlanner::compose_pose_msg(const std::vector<double>& pose_vec)
{
  if (pose_vec.size() != 7) 
  {
    RCLCPP_ERROR(get_logger(), "Invalid dimensions for pose, vector size: %ld", pose_vec.size());
    return geometry_msgs::msg::Pose();
  }

  geometry_msgs::msg::Pose msg;

  msg.position.x = pose_vec[0];
  msg.position.y = pose_vec[1];
  msg.position.z = pose_vec[2];
  msg.orientation.x = pose_vec[3];
  msg.orientation.y = pose_vec[4];
  msg.orientation.z = pose_vec[5];
  msg.orientation.w = pose_vec[6];

  return msg; 
}

bool WorkflowPlanner::are_poses_equal(
  const geometry_msgs::msg::Pose& pose_1,
  const geometry_msgs::msg::Pose& pose_2, 
  double pos_thd,
  double ori_thd)
{
  Eigen::Vector3d pos1(pose_1.position.x, pose_1.position.y, pose_1.position.z);
  Eigen::Vector3d pos2(pose_2.position.x, pose_2.position.y, pose_2.position.z);

  if ((pos1 - pos2).norm() > pos_thd) 
    return false;
  
  // Quaternion check (normalized)
  Eigen::Quaterniond q1(
    pose_1.orientation.w,
    pose_1.orientation.x,
    pose_1.orientation.y,
    pose_1.orientation.z
  );
  Eigen::Quaterniond q2(
    pose_2.orientation.w,
    pose_2.orientation.x,
    pose_2.orientation.y,
    pose_2.orientation.z
  );

  // Check if q1 and q2 represent the same rotation (allowing for sign flips)
  return (q1.angularDistance(q2) < ori_thd);
}

template bool WorkflowPlanner::send_sync_req<robot_controller_msgs::srv::ExecuteWaypoints>(
  rclcpp::Client<robot_controller_msgs::srv::ExecuteWaypoints>::SharedPtr,
  const robot_controller_msgs::srv::ExecuteWaypoints::Request::SharedPtr,
  robot_controller_msgs::srv::ExecuteWaypoints::Response::SharedPtr&,
  const std::string) const;
template bool WorkflowPlanner::send_sync_req<robot_controller_msgs::srv::GetCurrentPose>(
  rclcpp::Client<robot_controller_msgs::srv::GetCurrentPose>::SharedPtr,
  const robot_controller_msgs::srv::GetCurrentPose::Request::SharedPtr,
  robot_controller_msgs::srv::GetCurrentPose::Response::SharedPtr&,
  const std::string) const;
template bool WorkflowPlanner::send_sync_req<robotic_platform_msgs::srv::GetSlotStateTrigger>(
  rclcpp::Client<robotic_platform_msgs::srv::GetSlotStateTrigger>::SharedPtr,
  const robotic_platform_msgs::srv::GetSlotStateTrigger::Request::SharedPtr,
  robotic_platform_msgs::srv::GetSlotStateTrigger::Response::SharedPtr&,
  const std::string) const;

template <typename T>
bool WorkflowPlanner::send_sync_req(
  typename rclcpp::Client<T>::SharedPtr cli, 
  const typename T::Request::SharedPtr request,
  typename T::Response::SharedPtr& response,
  const std::string srv_name) const
{
  if (!cli_wait_for_srv<T>(cli, srv_name))
  {
    RCLCPP_INFO(get_logger(), "Failed to wait service");
    return false;
  }

  auto future = cli->async_send_request(request);
  std::future_status status = future.wait_for(CLI_REQ_TIMEOUT);

  switch (status)
  {
  case std::future_status::ready:
    // Yech!!!
    break;
  case std::future_status::deferred:
    RCLCPP_INFO(get_logger(), "Failed to call service %s, status: %s", srv_name.c_str(), "deferred");
    return false;
  case std::future_status::timeout:
    RCLCPP_INFO(get_logger(), "Failed to call service %s, status: %s", srv_name.c_str(), "timeout");
    return false;
  }

  response = future.get();

  if (!response->success)
  {
    RCLCPP_INFO(get_logger(), "Service %s call failed with error: {%s}", srv_name.c_str(), response->message.c_str());
    return false;
  }

  return true;
}

template void WorkflowPlanner::reset_req_res<robot_controller_msgs::srv::ExecuteWaypoints>(
  robot_controller_msgs::srv::ExecuteWaypoints::Request::SharedPtr,
  robot_controller_msgs::srv::ExecuteWaypoints::Response::SharedPtr) const;

template <typename T>
void WorkflowPlanner::reset_req_res(
  typename T::Request::SharedPtr request,
  typename T::Response::SharedPtr response) const
{
  request.reset();
  response.reset();
}

template <typename T>
bool WorkflowPlanner::cli_wait_for_srv(
  typename rclcpp::Client<T>::SharedPtr cli, 
  const std::string srv_name) const
{
  uint8_t retry = 0;

  while (rclcpp::ok() && !cli->wait_for_service(std::chrono::milliseconds(100)))
  {
    if (retry >= SRV_CLI_MAX_RETIES)
    {
      RCLCPP_DEBUG(get_logger(), "Interrupted while waiting for the service. Exiting.");
      return false;
    }

    RCLCPP_DEBUG(get_logger(), "%s service not available, waiting again...", srv_name.c_str());
    retry++;
  }

  return true;
}