#include "manager/manager.hpp"

std::optional<std::map<uint8_t, double>> Manager::send_pick_goal(
  const std::vector<OrderItem>& order_items,
  const std::vector<size_t>& items_selected)
{
  while (rclcpp::ok() && !pick_cli_->wait_for_action_server()) 
  {
    RCLCPP_ERROR(get_logger(), "Pick Action server not available after waiting");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  auto goal_msg = Pick::Goal();
  for (const auto& i : items_selected)
  {
    PickTask task;
    
    task.arm_id = order_items[i].sku.is_suctionable ? LEFT_ARM : RIGHT_ARM;
    task.camera_id = order_items[i].sku.is_suctionable ? LEFT_ARM : RIGHT_ARM;
    task.sku_id = order_items[i].sku.id;
    task.rack.id = order_items[i].rack.id;
    task.rack.shelf_level = order_items[i].rack.shelf_level;
    task.rack.shelf_slot = order_items[i].rack.shelf_slot;
    
    goal_msg.tasks.push_back(task);
  }

  RCLCPP_INFO(this->get_logger(), "Sending action goal");

  auto goal_options = rclcpp_action::Client<Pick>::SendGoalOptions();
  goal_options.feedback_callback = std::bind(&Manager::pick_feedback_cb, this, _1, _2);
  
  std::shared_future<PickGoalHandle::SharedPtr> future_goal_handle = pick_cli_->async_send_goal(goal_msg, goal_options);
  std::future_status status = future_goal_handle.wait_for(ACTION_TIMEOUT);

  switch (status)
  {
  case std::future_status::ready:
    RCLCPP_DEBUG(get_logger(), "OK");
    break;
  case std::future_status::deferred:
    RCLCPP_INFO(get_logger(), "Failed to call action, status: %s", "deferred");
    return std::nullopt;
  case std::future_status::timeout:
    RCLCPP_INFO(get_logger(), "Failed to call action, status: %s", "timeout");
    return std::nullopt;
  }

  if (!future_goal_handle.valid()) 
  {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    return std::nullopt;
  } 
  
  RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");

  PickGoalHandle::SharedPtr pick_goal_handle = future_goal_handle.get();

  std::shared_future<PickGoalHandle::WrappedResult> future_wrapped_result = pick_cli_->async_get_result(pick_goal_handle);
  PickGoalHandle::WrappedResult wrapped_result = future_wrapped_result.get();

  bool result_code = false;
  switch (wrapped_result.code) 
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      result_code = true;
      RCLCPP_DEBUG(this->get_logger(), "A nice Goal. Happy day.");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      break;
  }

  RCLCPP_INFO(this->get_logger(), "Pick action completed");

  if (result_code)
  {
    const auto& pick_results = wrapped_result.result->results;
    RCLCPP_INFO(this->get_logger(), "pick_results size: %zu", pick_results.size());

    bool all_arm_success = std::all_of(pick_results.begin(), pick_results.end(), 
      [](const PickResult& result) { 
        return result.success; 
      });

    if (all_arm_success)
    {
      std::map<uint8_t, double> height;

      RCLCPP_INFO(get_logger(), "height map should contains %zu elements:", pick_results.size());
      for (const auto& pick_result : pick_results)
      {
        height[pick_result.arm_id] = pick_result.height;
        RCLCPP_INFO(get_logger(), "  arm_id: %u, height: %.3f", pick_result.arm_id, pick_result.height);
      }

      return std::make_optional(std::move(height));
    }
  }

  return std::nullopt;
}

void Manager::pick_feedback_cb(
  PickGoalHandle::SharedPtr,
  const std::shared_ptr<const Pick::Feedback> feedback)
{
  (void) feedback;
  
  RCLCPP_DEBUG(get_logger(), "pick feedback received");
}
