#include "manager/manager.hpp"

bool Manager::send_place_goal(
  const std::vector<OrderItem>& order_items,
  const std::vector<size_t>& items_selected,
  const uint8_t table_id,
  const std::map<uint8_t, double>& place_height)
{
  if (items_selected.size() != place_height.size())
    return false;

  RCLCPP_INFO(get_logger(), "place_height map contains %zu elements:", place_height.size());
  for (const auto& [arm_id, height] : place_height) 
  {
    RCLCPP_INFO(get_logger(), "  arm_id: %u, height: %.3f", arm_id, height);
  }

  while (rclcpp::ok() && !place_cli_->wait_for_action_server()) 
  {
    RCLCPP_ERROR(get_logger(), "place Action server not available after waiting");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  auto goal_msg = Place::Goal();
  for (const auto& i : items_selected)
  {
    PlaceTask task;
    
    task.arm_id = order_items[i].sku.is_suctionable ? LEFT_ARM : RIGHT_ARM;
    task.sku_id = order_items[i].sku.id;
    task.height = place_height.at(order_items[i].sku.is_suctionable ? LEFT_ARM : RIGHT_ARM); 
    task.table.id = table_id;
    task.table.index = i + 1; // FIXME!
    
    goal_msg.tasks.push_back(task);
  }

  RCLCPP_INFO(this->get_logger(), "Sending action goal");

  auto goal_options = rclcpp_action::Client<Place>::SendGoalOptions();
  goal_options.feedback_callback = std::bind(&Manager::place_feedback_cb, this, _1, _2);
  
  std::shared_future<PlaceGoalHandle::SharedPtr> future_goal_handle = place_cli_->async_send_goal(goal_msg, goal_options);
  std::future_status status = future_goal_handle.wait_for(ACTION_TIMEOUT);

  switch (status)
  {
  case std::future_status::ready:
    RCLCPP_DEBUG(get_logger(), "OK");
    break;
  case std::future_status::deferred:
    RCLCPP_INFO(get_logger(), "Failed to call action, status: %s", "deferred");
    return false;
  case std::future_status::timeout:
    RCLCPP_INFO(get_logger(), "Failed to call action, status: %s", "timeout");
    return false;
  }

  if (!future_goal_handle.valid()) 
  {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    return false;
  } 
  
  RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  
  PlaceGoalHandle::SharedPtr place_goal_handle = future_goal_handle.get();
  std::shared_future<PlaceGoalHandle::WrappedResult> future_wrapped_result = place_cli_->async_get_result(place_goal_handle);
  PlaceGoalHandle::WrappedResult wrapped_result = future_wrapped_result.get();

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

  RCLCPP_INFO(this->get_logger(), "Place action completed");

  if (result_code)
  {
    const auto& place_results = wrapped_result.result->results;
    RCLCPP_INFO(this->get_logger(), "place_results size: %zu", place_results.size());

    bool all_arm_success = std::all_of(place_results.begin(), place_results.end(), 
      [](const PlaceResult& result) { 
        return result.success; 
      });

    if (all_arm_success)
    {
      return true;
    }
  }

  return false;
}

void Manager::place_feedback_cb(
  PlaceGoalHandle::SharedPtr,
  const std::shared_ptr<const Place::Feedback> feedback)
{
  (void) feedback;
  
  RCLCPP_DEBUG(get_logger(), "place feedback received");
}
