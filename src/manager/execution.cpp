#include "manager/manager.hpp"

void Manager::order_execution(const std::shared_ptr<NewOrderGoalHandle> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Executing new order goal");
  const auto goal = goal_handle->get_goal();

  const auto& order_id = goal->order.id;
  const auto& port_id = goal->order.port_id;
  const auto& order_items = goal->order.order_items;

  auto feedback = std::make_shared<NewOrderAction::Feedback>();
  auto result = std::make_shared<NewOrderAction::Result>();

  auto pub_fb = [this, &goal_handle, &feedback]() {
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(get_logger(), "publish order feedback");
  };

  rclcpp::TimerBase::SharedPtr pub_fb_timer = create_wall_timer(std::chrono::seconds(1), pub_fb);

  // Split order items one by one
  std::vector<OrderItem> organized_items; 
  for (const auto& item : order_items)
  {
    if (item.qty > 1)
    {
      for (uint8_t i = 0; i < item.qty; i++)
      {
        OrderItem msg = item;
        msg.qty = 1;
        organized_items.emplace_back(msg);
        RCLCPP_INFO(get_logger(), "splitted a SKU [ID: %d]", item.sku.id);
      }
    }
    else
    {
      OrderItem msg = item;
      organized_items.emplace_back(msg);
    }
  }

  for (const auto& item : order_items)
  {
    ItemResult msg;
    msg.sku = item.sku;
    msg.rack.id = item.rack.id;
    msg.rack.shelf_level = item.rack.shelf_level;
    msg.rack.shelf_slot = item.rack.shelf_slot;
    RCLCPP_WARN(get_logger(), ">>>>>>>>>>>> id: %d, level: %d, slot: %d", msg.rack.id, msg.rack.shelf_level, msg.rack.shelf_slot);
    msg.qty = 0;
    msg.success = false;
    msg.message = "";
    
    result->results.emplace_back(msg);
  }
  // while (rclcpp::ok()) {std::this_thread::sleep_for(std::chrono::milliseconds(500));}

  std::vector<bool> items_completed(organized_items.size(), false);
  std::vector<double> items_pick_height(organized_items.size(), false);
  std::vector<bool> place_occupancy_map;
  place_occupancy_map.resize(MAX_ORDER_ITEMS, false);

  bool all_items_completed = false;

  while (rclcpp::ok() && !all_items_completed) 
  {
    // Step 1: Choose 1-2 not finished SKUs from list
    std::vector<size_t> selected_indices = select_next_items(organized_items, items_completed);
    
    if (selected_indices.empty()) 
    {
      RCLCPP_INFO(get_logger(), "All items completed for order %d", order_id);
      break;
    }

    feedback->status.clear();
    for (const auto& idx : selected_indices)
    {
      ItemStatus msg;
      msg.sku = organized_items[idx].sku;
      msg.rack.id = organized_items[idx].rack.id;
      msg.rack.shelf_level = organized_items[idx].rack.shelf_level;
      msg.rack.shelf_slot = organized_items[idx].rack.shelf_slot;

      auto it = std::find_if(result->results.begin(), result->results.end(), 
        [&msg](const auto& result_item) { 
          return result_item.sku.id == msg.sku.id && 
            result_item.rack.id == msg.rack.id && 
            result_item.rack.shelf_level == msg.rack.shelf_level && 
            result_item.rack.shelf_slot == msg.rack.shelf_slot;
        });

      if (it != result->results.end())
      {
        msg.curr_qty = it->qty + 1;
      }
      msg.state = ItemStatus::IN_PROGRESS;

      feedback->status.emplace_back(msg);
    }

    RCLCPP_INFO(get_logger(), "Selected %zu items to process", selected_indices.size());

    // Step 2: Navigate to rack
    // if (!navigate_to_rack(rack_info.id)) 
    // {
    //   RCLCPP_ERROR(get_logger(), "Failed to navigate to rack %d", rack_info.id);
    //   response->success = false;
    //   response->message = "Navigation to rack failed";
    //   return;
    // }

    RCLCPP_INFO(get_logger(), "Step 7: Rotate to front");
    if (!rotate_to("abs_front"))
    {
      RCLCPP_ERROR(get_logger(), "abs_front");
      break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    RCLCPP_INFO(get_logger(), "Step 4: Pick item");
    std::optional<std::map<uint8_t, double>> opt_results = send_pick_goal(organized_items, selected_indices);

    if (!opt_results.has_value()) 
    {
      RCLCPP_ERROR(get_logger(), "Failed to pick item(s)");
      break;
    }

    std::map<uint8_t, double>& place_height = opt_results.value();

    // Step 5: Navigate to table
    // if (!navigate_to_table(port_id)) 
    // {
    //   RCLCPP_ERROR(get_logger(), "Failed to navigate to table %d", port_id);
    //   response->success = false;
    //   response->message = "Navigation to table failed";
    //   return;
    // }

    RCLCPP_INFO(get_logger(), "Step 7: Rotate to relative back");
    if (!rotate_to("abs_back"))
    {
      RCLCPP_ERROR(get_logger(), "abs_back");
      break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Step 7: Place all carried items
    RCLCPP_INFO(get_logger(), "Step 7: Place all carried items");
    if (!send_place_goal(organized_items, selected_indices, port_id, place_height, place_occupancy_map)) 
    {
      RCLCPP_ERROR(get_logger(), "Failed to place item SKU");
      break;
    }

    for (auto& status : feedback->status)
    {
      status.state = ItemStatus::COMPLETED;
    }
      
    for (const auto& idx : selected_indices) 
    {
      // Mark item as completed after successful placement
      const auto& item = organized_items[idx];
    
      auto it = std::find_if(result->results.begin(), result->results.end(), 
        [&item](const auto& result_item) { 
          return result_item.sku.id == item.sku.id && 
                 result_item.rack.id == item.rack.id && 
                 result_item.rack.shelf_level == item.rack.shelf_level && 
                 result_item.rack.shelf_slot == item.rack.shelf_slot;
        });

      if (it != result->results.end())
      {
        it->qty++;
        RCLCPP_INFO(get_logger(), "Successfully placed SKU: %d, new quantity: %d", item.sku.id, it->qty);
      }
      else
      {
        RCLCPP_WARN(get_logger(), "SKU: %d not found in result, cannot update quantity", item.sku.id);
      }

      items_completed[idx] = true;
      RCLCPP_INFO(get_logger(), "Successfully placed SKU: %d", item.sku.id);
    }

    for (size_t idx : selected_indices) 
    {
      if (items_completed[idx] && organized_items[idx].sku.is_suctionable)
        move_to_basic_pose(RobotArm::LEFT_ACTION, "home");
      else if (items_completed[idx] && organized_items[idx].sku.is_grippable)
        move_to_basic_pose(RobotArm::RIGHT_ACTION, "home");
    }

    // Step 8: Check if all items are finished
    all_items_completed = std::all_of(items_completed.begin(), items_completed.end(), 
      [](bool completed) { return completed; });
  }

  if (!rotate_to("abs_front"))
  {
    RCLCPP_ERROR(get_logger(), "abs_front");
    return;
  }

  for (auto& sub_result : result->results)
  {
    auto it = std::find_if(order_items.begin(), order_items.end(), 
      [&sub_result](const auto& result_item) { 
        return result_item.sku.id == sub_result.sku.id && 
               result_item.rack.id == sub_result.rack.id && 
               result_item.rack.shelf_level == sub_result.rack.shelf_level && 
               result_item.rack.shelf_slot == sub_result.rack.shelf_slot;
      });

    if (it != order_items.end())
    {
      if (it->qty == sub_result.qty)
      {
        sub_result.success = true;
        RCLCPP_INFO(get_logger(), "Successfully SKU: %d, quantity: %d, success: %s",
          sub_result.sku.id, sub_result.qty, sub_result.success ? "true" : "false");
      }
      else
        RCLCPP_WARN(get_logger(), "Failed SKU: %d, quantity: %d", sub_result.sku.id, sub_result.qty);
    }
    else
    {
      RCLCPP_WARN(get_logger(), "SKU: %d not found in result, cannot update quantity", sub_result.sku.id);
    }
  }

  RCLCPP_INFO(get_logger(), "Order %d completed successfully", order_id);

  if (rclcpp::ok()) 
  {
    pub_fb_timer->cancel();
    goal_handle->succeed(result);

    robot_status_.store(RobotStatus::IDLE);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}