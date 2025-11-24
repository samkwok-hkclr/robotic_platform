#include "manager/manager.hpp"

rclcpp_action::GoalResponse Manager::new_order_goal_cb(
  const rclcpp_action::GoalUUID & uuid, 
  std::shared_ptr<const NewOrderAction::Goal> goal)
{
  RCLCPP_INFO(get_logger(), "Received goal request with order");
  (void)uuid;
  
  const auto& order_id = goal->order.id;
  const auto& items = goal->order.order_items;
  const auto& port_id = goal->order.port_id;
  const auto& client_name = goal->client_name;

  size_t num_of_items = std::accumulate(items.begin(), items.end(), size_t{0}, 
    [](size_t sum, const auto& item) { return sum + item.qty; });

  if (num_of_items > MAX_ORDER_ITEMS)
  {
    RCLCPP_ERROR(get_logger(), "Order items size is larger than MAX_ORDER_ITEMS");
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(get_logger(), "Order items size: %zu", num_of_items);

  if (port_id == 0)
  {
    RCLCPP_ERROR(get_logger(), "Incorrect port id");
    return rclcpp_action::GoalResponse::REJECT;
  }

  std::string lower = client_name;
  std::transform(lower.begin(), lower.end(), lower.begin(),
    [](unsigned char c){ return std::tolower(c); });

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (lower == "test")
    {
      RCLCPP_WARN(get_logger(), "TESTING");
      robot_status_.store(RobotStatus::BUSY);
    }
    else if (client_name == client_name_)
    {
      RCLCPP_WARN(get_logger(), "client_name is matched!");
      robot_status_.store(RobotStatus::BUSY);
    }
    else
    {
      RCLCPP_WARN(get_logger(), "client_name does not match!");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  RCLCPP_INFO(get_logger(), "Processing new order ID: %d for port: %d", order_id, port_id);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Manager::new_order_cancel_cb(const std::shared_ptr<NewOrderGoalHandle> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Received request to cancel goal");
  (void)goal_handle;

  return rclcpp_action::CancelResponse::ACCEPT;
}

void Manager::new_order_accepted_cb(const std::shared_ptr<NewOrderGoalHandle> goal_handle)
{
  std::thread{std::bind(&Manager::order_execution, this, _1), goal_handle}.detach();
}

