#include "node_base.hpp"

template <typename T>
bool NodeBase::send_sync_req(
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
  std::future_status status = future.wait_for(get_cli_req_timeout());

  switch (status)
  {
  case std::future_status::ready:
    RCLCPP_DEBUG(get_logger(), "call service %s successfully", srv_name.c_str());
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

template <typename T>
bool NodeBase::cli_wait_for_srv(
  typename rclcpp::Client<T>::SharedPtr cli, 
  const std::string srv_name) const
{
  uint8_t retry = 0;

  while (rclcpp::ok() && !cli->wait_for_service(WAIT_SER_TIMEOUT))
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

template <typename T>
void NodeBase::reset_req_res(
  typename T::Request::SharedPtr& request,
  typename T::Response::SharedPtr& response) const
{
  request.reset();
  response.reset();
}

// ===================== send_sync_req std_msgs =====================

template bool NodeBase::send_sync_req<std_srvs::srv::SetBool>(
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr,
  const std_srvs::srv::SetBool::Request::SharedPtr,
  std_srvs::srv::SetBool::Response::SharedPtr&,
  const std::string) const;

// ===================== send_sync_req robot_controller_msgs =====================

template bool NodeBase::send_sync_req<robot_controller_msgs::srv::RobotSpeed>(
  rclcpp::Client<robot_controller_msgs::srv::RobotSpeed>::SharedPtr,
  const robot_controller_msgs::srv::RobotSpeed::Request::SharedPtr,
  robot_controller_msgs::srv::RobotSpeed::Response::SharedPtr&,
  const std::string) const;
template bool NodeBase::send_sync_req<robot_controller_msgs::srv::ExecuteWaypoints>(
  rclcpp::Client<robot_controller_msgs::srv::ExecuteWaypoints>::SharedPtr,
  const robot_controller_msgs::srv::ExecuteWaypoints::Request::SharedPtr,
  robot_controller_msgs::srv::ExecuteWaypoints::Response::SharedPtr&,
  const std::string) const;
template bool NodeBase::send_sync_req<robot_controller_msgs::srv::AddCollisionObjects>(
  rclcpp::Client<robot_controller_msgs::srv::AddCollisionObjects>::SharedPtr,
  const robot_controller_msgs::srv::AddCollisionObjects::Request::SharedPtr,
  robot_controller_msgs::srv::AddCollisionObjects::Response::SharedPtr&,
  const std::string) const;
template bool NodeBase::send_sync_req<robot_controller_msgs::srv::RemoveCollisionObjects>(
  rclcpp::Client<robot_controller_msgs::srv::RemoveCollisionObjects>::SharedPtr,
  const robot_controller_msgs::srv::RemoveCollisionObjects::Request::SharedPtr,
  robot_controller_msgs::srv::RemoveCollisionObjects::Response::SharedPtr&,
  const std::string) const;
template bool NodeBase::send_sync_req<robot_controller_msgs::srv::ApplyAttachedCollisionObjects>(
  rclcpp::Client<robot_controller_msgs::srv::ApplyAttachedCollisionObjects>::SharedPtr,
  const robot_controller_msgs::srv::ApplyAttachedCollisionObjects::Request::SharedPtr,
  robot_controller_msgs::srv::ApplyAttachedCollisionObjects::Response::SharedPtr&,
  const std::string) const;
template bool NodeBase::send_sync_req<robot_controller_msgs::srv::MoveCollisionObjects>(
  rclcpp::Client<robot_controller_msgs::srv::MoveCollisionObjects>::SharedPtr,
  const robot_controller_msgs::srv::MoveCollisionObjects::Request::SharedPtr,
  robot_controller_msgs::srv::MoveCollisionObjects::Response::SharedPtr&,
  const std::string) const;
template bool NodeBase::send_sync_req<robot_controller_msgs::srv::GetCollisionObjectsFromScene>(
  rclcpp::Client<robot_controller_msgs::srv::GetCollisionObjectsFromScene>::SharedPtr,
  const robot_controller_msgs::srv::GetCollisionObjectsFromScene::Request::SharedPtr,
  robot_controller_msgs::srv::GetCollisionObjectsFromScene::Response::SharedPtr&,
  const std::string) const;
template bool NodeBase::send_sync_req<robot_controller_msgs::srv::GetCurrentPose>(
  rclcpp::Client<robot_controller_msgs::srv::GetCurrentPose>::SharedPtr,
  const robot_controller_msgs::srv::GetCurrentPose::Request::SharedPtr,
  robot_controller_msgs::srv::GetCurrentPose::Response::SharedPtr&,
  const std::string) const;

  // ===================== send_sync_req robotic_platform_msgs =====================

template bool NodeBase::send_sync_req<robotic_platform_msgs::srv::GetSlotStateTrigger>(
  rclcpp::Client<robotic_platform_msgs::srv::GetSlotStateTrigger>::SharedPtr,
  const robotic_platform_msgs::srv::GetSlotStateTrigger::Request::SharedPtr,
  robotic_platform_msgs::srv::GetSlotStateTrigger::Response::SharedPtr&,
  const std::string) const;
template bool NodeBase::send_sync_req<robotic_platform_msgs::srv::GetObjectPoseTrigger>(
  rclcpp::Client<robotic_platform_msgs::srv::GetObjectPoseTrigger>::SharedPtr,
  const robotic_platform_msgs::srv::GetObjectPoseTrigger::Request::SharedPtr,
  robotic_platform_msgs::srv::GetObjectPoseTrigger::Response::SharedPtr&,
  const std::string) const;
template bool NodeBase::send_sync_req<robotic_platform_msgs::srv::PickPlan>(
  rclcpp::Client<robotic_platform_msgs::srv::PickPlan>::SharedPtr,
  const robotic_platform_msgs::srv::PickPlan::Request::SharedPtr,
  robotic_platform_msgs::srv::PickPlan::Response::SharedPtr&,
  const std::string) const;
template bool NodeBase::send_sync_req<robotic_platform_msgs::srv::PlacePlan>(
  rclcpp::Client<robotic_platform_msgs::srv::PlacePlan>::SharedPtr,
  const robotic_platform_msgs::srv::PlacePlan::Request::SharedPtr,
  robotic_platform_msgs::srv::PlacePlan::Response::SharedPtr&,
  const std::string) const;

// ===================== reset_req_res =====================

template void NodeBase::reset_req_res<std_srvs::srv::SetBool>(
  std_srvs::srv::SetBool::Request::SharedPtr&,
  std_srvs::srv::SetBool::Response::SharedPtr&) const;

template void NodeBase::reset_req_res<robot_controller_msgs::srv::ExecuteWaypoints>(
  robot_controller_msgs::srv::ExecuteWaypoints::Request::SharedPtr&,
  robot_controller_msgs::srv::ExecuteWaypoints::Response::SharedPtr&) const;