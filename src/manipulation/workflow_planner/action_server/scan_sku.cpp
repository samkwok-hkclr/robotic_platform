#include "manipulation/workflow_planner/workflow_planner.hpp"

rclcpp_action::GoalResponse WorkflowPlanner::scan_sku_goal_cb(
  const rclcpp_action::GoalUUID & uuid, 
  std::shared_ptr<const ScanSku::Goal> goal)
{
  (void)uuid;
  std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);

  // set state
  if (false)
  {
    RCLCPP_INFO(get_logger(), "FIXME rclcpp_action::GoalResponse::REJECT");
    // set state
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(get_logger(), "Received goal request with order %u", goal->order_id);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse WorkflowPlanner::scan_sku_cancel_cb(
  const std::shared_ptr<GoalHandlerScanSku> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Received request to cancel goal");
  (void) goal_handle;

  if (false)
  {
    RCLCPP_INFO(get_logger(), "FIXME rclcpp_action::GoalResponse::REJECT");
    return rclcpp_action::CancelResponse::REJECT;
  }
  
  return rclcpp_action::CancelResponse::ACCEPT;
}

void WorkflowPlanner::scan_sku_accepted(const std::shared_ptr<GoalHandlerScanSku> goal_handle)
{
  std::thread{std::bind(&WorkflowPlanner::scan_sku_execution, this, _1), goal_handle}.detach();
}

void WorkflowPlanner::scan_sku_execution(const std::shared_ptr<GoalHandlerScanSku> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Executing ScanSku goal");

  const auto goal = goal_handle->get_goal();

  auto feedback = std::make_shared<ScanSku::Feedback>();
  auto& running = feedback->running;
  auto& state = feedback->state;
  
  auto result = std::make_shared<ScanSku::Result>();

  std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);

  std::vector<std::pair<int, std::vector<double>>> selling_slot_vec {
    {174,   { 0.22, -0.555, 0.81, 0.5, 0.5, -0.5, 0.5 } },     //"Nescafe Continental Coffee 250ml"
    {35,    { 0.145, -0.555, 0.815, 0.5, 0.5, -0.5, 0.5 } },   //"Coca-cola Coke Dragon 330ml"
    {10004, { 0.07, -0.53, 0.86, 0.5, 0.5, -0.5, 0.5 } },      //"Cool Water 750ml"
    {17,    { -0.005, -0.53, 0.85, 0.5, 0.5, -0.5, 0.5 } },    //"Coca-cola Coke 500ml"
    {234,   { -0.08, -0.535, 0.835, 0.5, 0.5, -0.5, 0.5 } },   //"ION Supply Drink Pocari Sweat Bottle 500ml"
    {166,   { -0.145, -0.535, 0.835, 0.5, 0.5, -0.5, 0.5 } },  //"Tsuji - Oolong Tea 500ml"
    {28,    { -0.20, -0.535, 0.835, 0.5, 0.5, -0.5, 0.5 } }    //"Lucozade 300ml"
  };

  const geometry_msgs::msg::Pose init_pose = compose_pose_msg( { 0.6, 0.0, 1.0, 0.0, 0.7853975, 0.0, 0.7853975} );
  const geometry_msgs::msg::Pose pre_scan_pose = compose_pose_msg( { 0.0, -0.45, 1.0, 0.5, 0.5, -0.5, 0.5 });
  const geometry_msgs::msg::Pose pre_init_pose = compose_pose_msg( { 0.25, -0.25, 1.0, 0.2705981, 0.6532815, -0.2705981, 0.6532815 });

  auto request = std::make_shared<ExecuteWaypoints::Request>();
  request->waypoints.push_back(init_pose);
  request->waypoints.push_back(compose_pose_msg(selling_slot_vec[0].second));
  request->speed = 50;

  ExecuteWaypoints::Response::SharedPtr response;
  if (!(send_sync_req<ExecuteWaypoints>(exec_wps_cli_, request, response, __FUNCTION__) && response))
  {
    RCLCPP_ERROR(get_logger(), "Sent ExecuteWaypoints request failed");
    result->info = {};
    goal_handle->succeed(result);
  }

  for (const auto &sku_pair : selling_slot_vec) 
  {
    RCLCPP_ERROR(get_logger(), "AAAAA");

    const uint8_t target_obj_id = sku_pair.first;
    reset_req_res<ExecuteWaypoints>(request, response);
    request = std::make_shared<ExecuteWaypoints::Request>();

    request->waypoints.push_back(compose_pose_msg(sku_pair.second));
    request->speed = 50;
    ExecuteWaypoints::Response::SharedPtr response;
    if (!(send_sync_req<ExecuteWaypoints>(exec_wps_cli_, request, response, __FUNCTION__) && response))
    {
      RCLCPP_ERROR(get_logger(), "Sent ExecuteWaypoints request failed");
      break;
    }

    auto get_pose_request = std::make_shared<GetCurrentPose::Request>();
    get_pose_request->joint_name = "tcp";
    GetCurrentPose::Response::SharedPtr get_pose_response;
    if (!(send_sync_req<GetCurrentPose>(get_curr_pose_cli_, get_pose_request, get_pose_response, __FUNCTION__) && get_pose_response))
    {
      RCLCPP_ERROR(get_logger(), "Sent GetCurrentPose request failed");
      break;
    }
    
    RCLCPP_ERROR(get_logger(), "BBBBB");
    if (are_poses_equal(get_pose_response->pose, compose_pose_msg(sku_pair.second)))
    {
      RCLCPP_ERROR(get_logger(), "It is close");
      RCLCPP_INFO(get_logger(), "scanning sku: [%d]", target_obj_id);

      auto get_slot_state_request = std::make_shared<GetSlotStateTrigger::Request>();
      get_slot_state_request->camera_id = 1;
      get_slot_state_request->target_object_id = target_obj_id;
      GetSlotStateTrigger::Response::SharedPtr get_slot_state_response;
      if (!(send_sync_req<GetSlotStateTrigger>(get_slot_state_tri_cli_, get_slot_state_request, get_slot_state_response, __FUNCTION__) && get_slot_state_response))
      {
        RCLCPP_ERROR(get_logger(), "Sent GetCurrentPose request failed");
        break;
      }
      RCLCPP_INFO(get_logger(), "%d remain_qty: [%d]", target_obj_id, get_slot_state_response->remain_qty);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "It is not close");
    }
  }

  reset_req_res<ExecuteWaypoints>(request, response);
  request = std::make_shared<ExecuteWaypoints::Request>();
  request->waypoints.push_back(pre_scan_pose);
  request->waypoints.push_back(pre_init_pose);
  request->waypoints.push_back(init_pose);
  if (!(send_sync_req<ExecuteWaypoints>(exec_wps_cli_, request, response, __FUNCTION__) && response))
  {
    RCLCPP_ERROR(get_logger(), "Sent ExecuteWaypoints request failed");
  }

  if (rclcpp::ok()) 
  {
    result->info = {};
    goal_handle->succeed(result);

    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}
