#include "manipulation/workflow_planner/workflow_planner.hpp"

std::optional<std::vector<robotic_platform_msgs::msg::ObjectPose>> WorkflowPlanner::get_obj_poses(
  const int32_t sku_id,
  const uint8_t camera_id)
{
  auto request = std::make_shared<GetObjectPoseTrigger::Request>();

  request->target_object_id = sku_id;
  request->camera_id = camera_id;

  GetObjectPoseTrigger::Response::SharedPtr response;
  if (!send_sync_req<GetObjectPoseTrigger>(get_obj_pose_tri_cli_, std::move(request), response))
  {
    RCLCPP_ERROR(get_logger(), "Sent GetObjectPoseTrigger request failed");
    return std::nullopt;
  }

  if (!response->success)
    return std::nullopt;

  RCLCPP_WARN(get_logger(), "OK!");
  return std::make_optional(std::move(response->object_poses));
}

std::optional<robotic_platform_msgs::msg::PickPlanResult> WorkflowPlanner::get_pick_plan(
  const Pose& object_pose,
  const RackInfo& rack,
  const std::string& flat_frame)
{
  auto request = std::make_shared<PickPlan::Request>();

  request->object_pose = object_pose;
  request->rack = rack;
  request->flat_frame = flat_frame;

  PickPlan::Response::SharedPtr response;
  if (!send_sync_req<PickPlan>(pick_plan_cli_, std::move(request), response))
  {
    RCLCPP_ERROR(get_logger(), "Sent PickPlan request failed");
    return std::nullopt;
  }

  if (!response->success)
    return std::nullopt;

  return std::make_optional(std::move(response->result));
}

std::optional<robotic_platform_msgs::msg::PlacePlanResult> WorkflowPlanner::get_place_plan(const Pose& place_pose)
{
  auto request = std::make_shared<PlacePlan::Request>();

  request->place_pose = place_pose;

  PlacePlan::Response::SharedPtr response;
  if (!send_sync_req<PlacePlan>(place_plan_cli_, std::move(request), response))
  {
    RCLCPP_ERROR(get_logger(), "Sent PlacePlan request failed");
    return std::nullopt;
  }

  if (!response->success)
    return std::nullopt;

  return std::make_optional(std::move(response->result));
}

bool WorkflowPlanner::set_camera_lifecycle(RobotArm arm, bool activate)
{
  RobotArm arm_wo_action;

  if (arm == RobotArm::LEFT_ACTION)
    arm_wo_action = RobotArm::LEFT;
  else if (arm == RobotArm::RIGHT_ACTION)
    arm_wo_action = RobotArm::RIGHT;
  else
    arm_wo_action = arm;

  const std::string srv_name = "change_state";

  uint8_t retry = 0;
  const uint8_t SRV_CLI_MAX_RETIES = 5;

  while (rclcpp::ok() && !camera_cli_[arm_wo_action]->wait_for_service(std::chrono::milliseconds(100)))
  {
    if (retry >= SRV_CLI_MAX_RETIES)
    {
      RCLCPP_DEBUG(get_logger(), "Interrupted while waiting for the service. Exiting.");
      return false;
    }

    RCLCPP_DEBUG(get_logger(), "%s service not available, waiting again...", srv_name.c_str());
    retry++;
  }

  auto request = std::make_shared<ChangeState::Request>();
  
  if (activate)
    request->transition.id = Transition::TRANSITION_ACTIVATE;
  else
    request->transition.id = Transition::TRANSITION_DEACTIVATE;

  auto future = camera_cli_[arm_wo_action]->async_send_request(request);
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

  auto response = future.get();

  if (!response->success)
  {
    RCLCPP_INFO(get_logger(), "Service %s call failed", srv_name.c_str());
    return false;
  }

  return true;
}
// std::optional<geometry_msgs::msg::Pose> WorkflowPlanner::get_curr_pose(RobotArm arm, const std::string& joint_name)
// {
//   if (joint_name.empty()) 
//   {
//     RCLCPP_ERROR(get_logger(), "Joint name is empty!");
//     return std::nullopt;
//   } 
  
//   auto request = std::make_shared<GetPose::Request>();
//   request->joint_name = joint_name;
//   GetPose::Response::SharedPtr response;
//   if (!(send_sync_req<GetPose>(get_curr_pose_cli_[arm], std::move(request), response) && response))
//   {
//     RCLCPP_ERROR(get_logger(), "Sent GetPose request failed");
//     return std::nullopt;
//   }

//   if (!response->success)
//     return std::nullopt; 

//   return std::make_optional(std::move(response->pose));
// }

std::optional<geometry_msgs::msg::Pose> WorkflowPlanner::get_scan_pose(uint8_t rack_id, uint8_t shelf_level, uint8_t shelf_slot)
{
  const std::string frame = "rack_" + std::to_string(rack_id) + "_shelf_" + std::to_string(shelf_level) + "_slot_" + std::to_string(shelf_slot) + "_link";
  std::optional<TransformStamped> tf_stamped = get_tf(ARM_REF_FRAME, frame);

  RCLCPP_WARN(get_logger(), "Target scan slot: %s", frame.c_str());

  if (!tf_stamped.has_value())
  {
    RCLCPP_ERROR(get_logger(), "have no value");
    return std::nullopt;
  }
  
  tf2::Transform g_b__slot;
  tf2::fromMsg(tf_stamped.value().transform, g_b__slot);

  tf2::Transform g_slot__scan = get_g(0, 0, -scan_distance_, 0, 0, 0);
  tf2::Transform g_b__scan = g_b__slot * g_slot__scan;

  RCLCPP_ERROR(get_logger(), "Get scan pose okay");

  return std::make_optional(cvt_g_to_pose(g_b__scan)); 
}

void WorkflowPlanner::push_tf_buf(const std::tuple<Pose, std::string, std::string>& tf)
{
  std::lock_guard<std::mutex> lock(tf_mutex_);

  tf_buf_.emplace_back(tf);
}

void WorkflowPlanner::clear_tf_buf()
{
  std::lock_guard<std::mutex> lock(tf_mutex_);

  tf_buf_.clear();
}

std::string WorkflowPlanner::get_flat_link(uint8_t rack_id, uint8_t shelf_level) const
{
  return "rack_" + std::to_string(rack_id) + "_shelf_" + std::to_string(shelf_level) + "_flat_link";
}

std::string WorkflowPlanner::get_place_link(uint8_t table_id, uint8_t index) const
{
  return "table_" + std::to_string(table_id) + "_place_" + std::to_string(index) + "_link";
}

