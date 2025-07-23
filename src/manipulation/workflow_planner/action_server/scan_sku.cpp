#include "manipulation/workflow_planner/workflow_planner.hpp"

rclcpp_action::GoalResponse WorkflowPlanner::scan_sku_goal_cb(
  const rclcpp_action::GoalUUID & uuid, 
  std::shared_ptr<const ScanSku::Goal> goal)
{
  (void)uuid;
  std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);

  if (state_ != RobotStatus::IDLE)
  {
    RCLCPP_INFO(get_logger(), "Robot is not idle");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (scan_poses_.size() == 0)
  {
    RCLCPP_INFO(get_logger(), "Scan poses is empty.");
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

  auto pub_fb = [this, &goal_handle, &feedback, &running, &state]() {
    if (rclcpp::ok())
    {
      state = 1;
      running = true;
    }
    RCLCPP_INFO(get_logger(), "Publish feedback!!!");
    goal_handle->publish_feedback(feedback);
  };
  
  rclcpp::TimerBase::SharedPtr pub_fb_timer = create_wall_timer(std::chrono::seconds(1), pub_fb, exec_timer_cbg_);

  if (!motion_planner_->move_to_pre_scan_pose(80.0))
  {
    goal_handle->abort(result);
    return;
  }

  for (const auto &sku_pair : scan_order_) 
  {
    const int target_sku_id = std::get<1>(sku_pair);
    RCLCPP_INFO(get_logger(), "target_sku_id: %d", target_sku_id);

    if (scan_poses_.find(target_sku_id) == scan_poses_.end()) 
    {
      RCLCPP_ERROR(get_logger(), "Pose of target_sku_id [%d] not found.", target_sku_id);
      continue;
    }

    Pose target_pose = scan_poses_[target_sku_id];
    if (!motion_planner_->move_to(target_pose, 80.0))
    {
      goal_handle->abort(result);
      return;
    }

    std::optional<Pose> curr_pose = get_curr_pose("tcp");
    if (!curr_pose.has_value())
    {
      goal_handle->abort(result);
      return;
    }

    if (are_poses_closed(*curr_pose, target_pose))
    {
      RCLCPP_INFO(get_logger(), "scanning sku: [%d]", target_sku_id);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));

      auto get_slot_state_request = std::make_shared<GetSlotStateTrigger::Request>();
      get_slot_state_request->camera_id = goal->camera_id;
      get_slot_state_request->target_object_id = target_sku_id;
      GetSlotStateTrigger::Response::SharedPtr get_slot_state_response;
      if (!(send_sync_req<GetSlotStateTrigger>(get_slot_state_tri_cli_, std::move(get_slot_state_request), get_slot_state_response) && get_slot_state_response))
      {
        RCLCPP_ERROR(get_logger(), "Sent GetCurrentPose request failed");
        break;
      }

      RCLCPP_INFO(get_logger(), "SKU ID: %d, remain_qty: [%d]", target_sku_id, get_slot_state_response->remain_qty);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "It is not close!!!");
    }
  }

  if (!motion_planner_->move_to_pre_scan_pose(70.0f))
  {
    goal_handle->abort(result);
    return;
  }

  if (!motion_planner_->move_to_home_pose(80.0f))
  {
    goal_handle->abort(result);
    return;
  }

  if (rclcpp::ok()) 
  {
    pub_fb_timer->cancel();
    result->info = {};
    goal_handle->succeed(result);

    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}
