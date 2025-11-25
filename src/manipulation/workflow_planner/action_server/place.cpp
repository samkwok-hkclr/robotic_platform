#include "manipulation/workflow_planner/workflow_planner.hpp"

rclcpp_action::GoalResponse WorkflowPlanner::place_goal_cb(
  const rclcpp_action::GoalUUID & uuid, 
  std::shared_ptr<const Place::Goal> goal)
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

rclcpp_action::CancelResponse WorkflowPlanner::place_cancel_cb(
  const std::shared_ptr<GoalHandlerPlace> goal_handle)
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

void WorkflowPlanner::place_accepted(const std::shared_ptr<GoalHandlerPlace> goal_handle)
{
  std::thread{std::bind(&WorkflowPlanner::place_execution, this, _1), goal_handle}.detach();
}

void WorkflowPlanner::place_execution(const std::shared_ptr<GoalHandlerPlace> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Executing Place goal");

  const auto goal = goal_handle->get_goal();

  auto feedback = std::make_shared<Place::Feedback>();
  auto& running = feedback->running;
  auto& state = feedback->state;
  
  auto result = std::make_shared<Place::Result>();

  auto pub_fb = [this, &goal_handle, &feedback, &running, &state]() {
    RCLCPP_DEBUG(get_logger(), "what should i publish?");
    goal_handle->publish_feedback(feedback);
  };
  
  rclcpp::TimerBase::SharedPtr pub_fb_timer = create_wall_timer(std::chrono::seconds(1), pub_fb, exec_timer_cbg_);
  clear_tf_buf();

  RCLCPP_INFO(get_logger(), "Starting Place action sequence with %zu tasks", goal->tasks.size());

  const double place_offset = get_parameter("place_offset").as_double();
  
  for (const auto& [arm_id, sku_id, height, table, dimension] : goal->tasks) 
  {
    const RobotArm arm = static_cast<RobotArm>(arm_id);
    PlaceResult msg;
    msg.arm_id = arm_id;

    if (height <= place_offset)
    {
      RCLCPP_WARN(get_logger(), "Height (%.4f) should not be less than or equal to place_offset (%.4f) in %s arm", 
        height, place_offset, arm_to_str.at(arm).c_str());
      msg.message = "Invalid height - below safety threshold";
      msg.success = false;
      result->results.emplace_back(msg);
      continue;
    }

    // FIXME: wait for tf tree be stable
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    if (!optimal_place_elvation(table))
    {
      RCLCPP_ERROR(get_logger(), "Failed to elevate to Table id: [%d], index: %d", table.id, table.index);
      msg.message = "Failed to elevate to Table";
      msg.success = false;
      result->results.emplace_back(msg);
      continue;
    }
    

    if (!motion_planner_->move_to_action_pose(arm, 100.0f)) 
    {
      RCLCPP_ERROR(get_logger(), "Failed to move to action pose");
      msg.success = false;
      msg.message = "Placement operation failed";
      result->results.emplace_back(msg);
      continue;
    }

    RCLCPP_INFO(get_logger(), "Attempting place-down: arm=%s, sku=%d, table=%d, table order=%d", 
      arm_to_str.at(arm).c_str(),
      sku_id, 
      table.id, 
      table.index);

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    bool success = try_to_place_down(arm, height + 0.04, table);
 
    clear_tf_buf();

    if (success)
    {
      if (!motion_planner_->move_to_action_pose(arm, 100.0f))
      {
        RCLCPP_ERROR(get_logger(), "Failed to move to action pose");
        msg.success = false;
        msg.message = "Placement operation failed";
        result->results.emplace_back(msg);
        continue;
      };

      msg.success = true;
      state += 1;
      RCLCPP_INFO(get_logger(), "Successfully placed down SKU %d using %s arm", sku_id, arm_to_str.at(arm).c_str());
    }
    else
    {
      msg.success = false;
      msg.message = "Placement operation failed";
      RCLCPP_WARN(get_logger(), "Failed to place down SKU %d using %s arm", sku_id, arm_to_str.at(arm).c_str());
    }

    result->results.emplace_back(msg);
  }

  RCLCPP_INFO(get_logger(), "Place action completed successful, final state=%d", state);

  if (rclcpp::ok()) 
  {
    RCLCPP_INFO(this->get_logger(), "Clear things...");
    clear_tf_buf();
    pub_fb_timer->cancel();
  
    goal_handle->succeed(result);

    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}