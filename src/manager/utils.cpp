#include "manager/manager.hpp"

// version 1: select 1 suctionable and 1 gripperable item only
std::vector<size_t> Manager::select_next_items(
  const std::vector<OrderItem>& items, 
  const std::vector<bool>& items_completed)
{
  std::vector<size_t> items_selected;

  if (items.size() != items_completed.size())
  {
    RCLCPP_ERROR(get_logger(), "Items and completion vectors have different sizes: %zu vs %zu", 
      items.size(), items_completed.size());
    return items_selected;
  }

  bool suctionable_selected = false;
  bool grippable_selected = false;

  for (size_t i = 0; i < items.size(); i++)
  {
    if (items_completed[i])
      continue;

    if (items[i].sku.is_grippable && !grippable_selected)
    {
      grippable_selected = true;
      items_selected.push_back(i);
    }
    else if (items[i].sku.is_suctionable && !suctionable_selected)
    {
      suctionable_selected = true;
      items_selected.push_back(i);
    }
    else if (!items[i].sku.is_grippable && !items[i].sku.is_suctionable)
    {
      RCLCPP_ERROR(get_logger(), "FIXME: Item is ungrippable and suctionable");
    }

    if (suctionable_selected && grippable_selected)
      break;
  }

  if (!items_selected.empty()) 
  {
    std::ostringstream indices_ss;
    for (size_t i = 0; i < items_selected.size(); i++) 
    {
      indices_ss << items_selected[i];
      if (i < items_selected.size() - 1) 
      indices_ss << ", ";
    }

    RCLCPP_INFO(get_logger(), "Selected %zu items from %zu total items. Indices: [%s]", 
      items_selected.size(), 
      items.size(),
      indices_ss.str().c_str());
  } 
  else 
  {
    RCLCPP_INFO(get_logger(), "Selected 0 items from %zu total items", items.size());
  }

  return items_selected;
} 

bool Manager::rotate_to(std::string pose)
{
  if (auto it = fold_elev_rotate_cli_.find(pose); it == fold_elev_rotate_cli_.end())
    return false;

  auto request = std::make_shared<Trigger::Request>();

  Trigger::Response::SharedPtr response;
  if (!send_sync_req<Trigger>(fold_elev_rotate_cli_[pose], std::move(request), response))
  {
    RCLCPP_ERROR(get_logger(), "Sent Trigger request failed");
    return false;
  }

  if (!response->success)
    return false;

  return true;
} 

bool Manager::move_to_basic_pose(RobotArm arm, const std::string& pose)
{
  auto request = std::make_shared<Trigger::Request>();

  Trigger::Response::SharedPtr response;
  if (!send_sync_req<Trigger>(arm_basic_ctrl_cli_[arm][pose], std::move(request), response) )
  {
    RCLCPP_ERROR(get_logger(), "Sent Trigger request failed");
    return false;
  }

  if (!response->success)
  {
    RCLCPP_ERROR(get_logger(), "Unsuccessful: %s", response->message.c_str());
    return false;
  }
  
  return true;
}