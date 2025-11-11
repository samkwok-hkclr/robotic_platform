#include "manipulation/workflow_planner/workflow_planner.hpp"

rclcpp_action::GoalResponse WorkflowPlanner::pick_goal_cb(
  const rclcpp_action::GoalUUID & uuid, 
  std::shared_ptr<const Pick::Goal> goal)
{
  (void)uuid;

  if (state_ != RobotStatus::IDLE)
  {
    RCLCPP_INFO(get_logger(), "Robot is not idle");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->tasks.empty() || goal->tasks.size() > 2)
  {
    RCLCPP_INFO(get_logger(), "Invalid task size: %zu. Must be 1 or 2.", goal->tasks.size());
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal->tasks.size() == 2 && goal->tasks[0].arm_id == goal->tasks[1].arm_id)
  {
    RCLCPP_INFO(get_logger(), "Task arm_ids are not distinct!");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(get_logger(), "Received goal request with order: %u", goal->order_id);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse WorkflowPlanner::pick_cancel_cb(
  const std::shared_ptr<GoalHandlerPick> goal_handle)
{
  RCLCPP_DEBUG(get_logger(), "Received request to cancel goal");
  (void) goal_handle;

  if (false)
  {
    RCLCPP_INFO(get_logger(), "FIXME rclcpp_action::GoalResponse::REJECT");
    return rclcpp_action::CancelResponse::REJECT;
  }
  
  return rclcpp_action::CancelResponse::ACCEPT;
}

void WorkflowPlanner::pick_accepted(const std::shared_ptr<GoalHandlerPick> goal_handle)
{
  std::thread{std::bind(&WorkflowPlanner::pick_execution, this, _1), goal_handle}.detach();
}

void WorkflowPlanner::pick_execution(const std::shared_ptr<GoalHandlerPick> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Executing Pick goal");

  const auto goal = goal_handle->get_goal();

  auto feedback = std::make_shared<Pick::Feedback>();
  auto& running = feedback->running;
  auto& state = feedback->state;
  
  auto result = std::make_shared<Pick::Result>();

  auto pub_fb = [this, &goal_handle, &feedback, &running, &state]() {
    RCLCPP_DEBUG(get_logger(), "what should i publish?");
    goal_handle->publish_feedback(feedback);
  };

  rclcpp::TimerBase::SharedPtr pub_fb_timer = create_wall_timer(std::chrono::seconds(1), pub_fb, exec_timer_cbg_);
  clear_tf_buf();

  RCLCPP_INFO(get_logger(), "Starting Pick action sequence with %zu tasks", goal->tasks.size());

  for (const auto& [arm_id, camera_id, sku_id, rack, dimension] : goal->tasks) 
  {
    const RobotArm arm = static_cast<RobotArm>(arm_id);
    
    PickResult msg;
    msg.arm_id = arm_id;

    if (!set_camera_lifecycle(arm, true))
    {
      RCLCPP_ERROR(get_logger(), "Failed to activate camera");
      msg.success = false;
      msg.message = "Failed to activate camera";
      result->results.emplace_back(msg);
      continue;
    }

    // FIXME: wait for tf tree be stable
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    if (!optimal_pick_elvation(rack))
    {
      RCLCPP_ERROR(get_logger(), "Failed to elevate to rack id: [%d], shelf level: %d", rack.id, rack.shelf_level);
      msg.success = false;
      msg.message = "FIXME";
      result->results.emplace_back(msg);
      continue;
    }
    
    RCLCPP_INFO(get_logger(), "Attempting pick-up: arm=%s, sku=%d, rack=%d, shelf_level=%d, shelf_slot=%d", 
      arm_to_str.at(arm).c_str(),
      sku_id, 
      rack.id, 
      rack.shelf_level, 
      rack.shelf_slot);

    std::optional<double> height = try_to_pick_up(arm, sku_id, camera_id, rack);
 
    if (height.has_value())
    {
      clear_tf_buf();
      msg.success = true;
      msg.height = height.value();
      state += 1;
      RCLCPP_INFO(get_logger(), "Successfully picked up SKU %d using %s arm", sku_id, arm_to_str.at(arm).c_str());
    }
    else
    {
      msg.success = false;
      msg.message = "FIXME";
      RCLCPP_ERROR(get_logger(), "Failed to pick up SKU %d using %s arm", sku_id, arm_to_str.at(arm).c_str());
    }

    result->results.emplace_back(msg);

    if (height.has_value())
    {
      motion_planner_->move_to_holding_pose(arm, 100.0);
    }

    if (!set_camera_lifecycle(arm, false))
    {
      RCLCPP_ERROR(get_logger(), "Failed to deactivate camera");
      msg.success = false;
      msg.message = "Failed to deactivate camera";
      result->results.emplace_back(msg);
      continue;
    }

    RCLCPP_INFO(get_logger(), "SKU %d using %s arm", sku_id, arm_to_str.at(arm).c_str());
  }

  RCLCPP_INFO(get_logger(), "Pick action completed successful, final state=%d", state);

  if (rclcpp::ok()) 
  {
    RCLCPP_INFO(this->get_logger(), "Clear things...");
    clear_tf_buf();
    pub_fb_timer->cancel();
  
    goal_handle->succeed(result);

    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}
