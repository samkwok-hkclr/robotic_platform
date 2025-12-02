#include "manager/manager.hpp"

void Manager::order_execution(const std::shared_ptr<NewOrderGoalHandle> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Executing new order goal");
  const auto goal = goal_handle->get_goal();

  const std::string& order_id = goal->order.id;
  const uint8_t& port_id = goal->order.port_id;
  const std::vector<OrderItem>& order_items = goal->order.order_items;

  std::mutex fb_mutex;
  std::vector<std::shared_ptr<ItemStatus>> feedbacks;
  auto result = std::make_shared<NewOrderAction::Result>();

  auto pub_fb = [this, &goal_handle, &fb_mutex, &feedbacks]() {
    std::lock_guard<std::mutex> lock(fb_mutex);

    for (const auto& feedback : feedbacks)
    {
      auto fb = std::make_shared<NewOrderAction::Feedback>();
      fb->status = *feedback;
      goal_handle->publish_feedback(fb);
    }
    RCLCPP_INFO(get_logger(), "publish order feedback");
  };

  rclcpp::TimerBase::SharedPtr pub_fb_timer = create_wall_timer(std::chrono::seconds(1), pub_fb);
  pub_fb_timer->cancel();

  // Split order items one by one
  std::vector<OrderItem> organized_items; 
  for (const OrderItem& item : order_items)
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

  for (const OrderItem& item : order_items)
  {
    ItemResult msg;
    msg.sku = item.sku;
    msg.rack = item.rack;
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

  pub_fb_timer->reset();

  while (rclcpp::ok() && !all_items_completed) 
  {
    // Step 1: Choose 1-2 not finished SKUs from list
    std::vector<size_t> selected_indices = select_next_items(organized_items, items_completed);
    
    if (selected_indices.empty()) 
    {
      RCLCPP_INFO(get_logger(), "All items completed for order %s", order_id.c_str());
      break;
    }

    if (rclcpp::ok())
    {
      std::lock_guard<std::mutex> lock(fb_mutex);

      for (auto& feedback : feedbacks)
      {
        feedback.reset();
      }

      feedbacks.clear();

      for (const size_t& idx : selected_indices)
      {
        auto msg = std::make_shared<ItemStatus>();
        msg->sku = organized_items[idx].sku;
        msg->rack = organized_items[idx].rack;
        msg->curr_qty = 0;
        msg->state = ItemStatus::IN_PROGRESS;

        feedbacks.emplace_back(std::move(msg));
      }
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

    if (rclcpp::ok())
    {
      std::lock_guard<std::mutex> lock(fb_mutex);

      for (auto& feedback : feedbacks)
      {
        feedback->curr_qty = 1;
        feedback->state = ItemStatus::COMPLETED;
      }
    }

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
      
    for (const size_t& idx : selected_indices) 
    {
      // Mark item as completed after successful placement
      const OrderItem& item = organized_items[idx];

      auto it = std::find_if(result->results.begin(), result->results.end(), 
        [this, &item](const ItemResult& result_item) { 
          return compare_same_item(result_item, item);
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
      [this, &sub_result](const auto& result_item) { 
        return compare_same_item(result_item, sub_result);
      });

    if (it != order_items.end())
    {
      RCLCPP_WARN(get_logger(), "SKU: %d not found in result, cannot update quantity", sub_result.sku.id);
    }

    if (it->qty == sub_result.qty)
    {
      sub_result.success = true;
      RCLCPP_INFO(get_logger(), "Successfully SKU: %d, quantity: %d, success: %s",
        sub_result.sku.id, sub_result.qty, sub_result.success ? "true" : "false");
    }
    else
      RCLCPP_WARN(get_logger(), "Failed SKU: %d, quantity: %d", sub_result.sku.id, sub_result.qty);
  }

  RCLCPP_INFO(get_logger(), "Order %s completed successfully", order_id.c_str());

  if (rclcpp::ok()) 
  {
    pub_fb_timer->cancel();
    goal_handle->succeed(result);

        robot_status_.store(RobotStatus::IDLE);
    RCLCPP_INFO(this->get_logger(), "Robot Status: %s", robot_status_.load() == RobotStatus::IDLE? "idle" : "busy");

    RCLCPP_INFO(this->get_logger(), "A simulation goal succeeded");
  }
}

void Manager::order_execution_sim(const std::shared_ptr<NewOrderGoalHandle> goal_handle)
{
  RCLCPP_WARN(get_logger(), "Manager simulation is on");

  const auto goal = goal_handle->get_goal();

  const std::string& order_id = goal->order.id;
  // const uint8_t& port_id = goal->order.port_id;
  const std::vector<OrderItem>& order_items = goal->order.order_items;

  std::mutex fb_mutex;
  std::vector<std::shared_ptr<ItemStatus>> feedbacks;
  auto result = std::make_shared<NewOrderAction::Result>();

  auto pub_fb = [this, &goal_handle, &fb_mutex, &feedbacks]() {
    std::lock_guard<std::mutex> lock(fb_mutex);

    for (const auto& feedback : feedbacks)
    {
      auto fb = std::make_shared<NewOrderAction::Feedback>();
      fb->status = *feedback;
      goal_handle->publish_feedback(fb);
    }
    RCLCPP_INFO(get_logger(), "publish order feedback");
  };

  rclcpp::TimerBase::SharedPtr pub_fb_timer = create_wall_timer(std::chrono::seconds(1), pub_fb);
  pub_fb_timer->cancel();

  std::vector<OrderItem> organized_items; 
  for (const OrderItem& item : order_items)
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

  for (const OrderItem& item : order_items)
  {
    ItemResult msg;
    msg.sku = item.sku;
    msg.rack = item.rack;
    msg.qty = 0;
    msg.success = false;
    msg.message = "";
    
    result->results.emplace_back(msg);
  }

  std::vector<bool> items_completed(organized_items.size(), false);
  std::vector<double> items_pick_height(organized_items.size(), false);
  std::vector<bool> place_occupancy_map;
  place_occupancy_map.resize(MAX_ORDER_ITEMS, false);

  bool all_items_completed = false;

  pub_fb_timer->reset();

  while (rclcpp::ok() && !all_items_completed) 
  {
    // Step 1: Choose 1-2 not finished SKUs from list
    std::vector<size_t> selected_indices = select_next_items(organized_items, items_completed);
    
    if (selected_indices.empty()) 
    {
      RCLCPP_INFO(get_logger(), "All items completed for order %s", order_id.c_str());
      break;
    }

    if (rclcpp::ok())
    {
      std::lock_guard<std::mutex> lock(fb_mutex);

      for (auto& feedback : feedbacks)
      {
        feedback.reset();
      }

      feedbacks.clear();

      for (const size_t& idx : selected_indices)
      {
        auto msg = std::make_shared<ItemStatus>();
        msg->sku = organized_items[idx].sku;
        msg->rack = organized_items[idx].rack;
        msg->curr_qty = 0;
        msg->state = ItemStatus::IN_PROGRESS;

        feedbacks.emplace_back(std::move(msg));
      }
    }

    RCLCPP_INFO(get_logger(), "Selected %zu items to process", selected_indices.size());

    std::this_thread::sleep_for(std::chrono::seconds(1));
  
    if (rclcpp::ok())
    {
      std::lock_guard<std::mutex> lock(fb_mutex);

      for (auto& status : feedbacks)
      {
        status->curr_qty = 1;
        status->state = ItemStatus::COMPLETED;
      }
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
      
    for (const size_t& idx : selected_indices) 
    {
      // Mark item as completed after successful placement
      const OrderItem& item = organized_items[idx];
    
      auto it = std::find_if(result->results.begin(), result->results.end(), 
        [this, &item](const ItemResult& result_item) { 
          return compare_same_item(result_item, item);
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

    // Step 8: Check if all items are finished
    all_items_completed = std::all_of(items_completed.begin(), items_completed.end(), 
      [](bool completed) { return completed; });
  }

  for (auto& sub_result : result->results)
  {
    auto it = std::find_if(order_items.begin(), order_items.end(), 
      [this, &sub_result](const auto& result_item) { 
        return compare_same_item(result_item, sub_result);
      });

    if (it == order_items.end())
    {
      RCLCPP_WARN(get_logger(), "SKU: %d not found in result, cannot update quantity", sub_result.sku.id);
      continue;
    }

    if (it->qty == sub_result.qty)
    {
      sub_result.success = true;
      RCLCPP_INFO(get_logger(), "Successfully SKU: %d, quantity: %d, success: %s",
        sub_result.sku.id, sub_result.qty, sub_result.success ? "true" : "false");
    }
    else
      RCLCPP_WARN(get_logger(), "Failed SKU: %d, quantity: %d", sub_result.sku.id, sub_result.qty);
  }

  if (rclcpp::ok()) 
  {
    pub_fb_timer->cancel();
    goal_handle->succeed(result);

    robot_status_.store(RobotStatus::IDLE);
    RCLCPP_INFO(this->get_logger(), "Robot Status: %s", robot_status_.load() == RobotStatus::IDLE? "idle" : "busy");

    RCLCPP_INFO(this->get_logger(), "A simulation goal succeeded");
  }
}

template<typename T1, typename T2>
bool Manager::compare_same_item(const T1& item1, const T2& item2)
{
  return item1.sku.id == item2.sku.id && 
         item1.rack.id == item2.rack.id && 
         item1.rack.shelf_level == item2.rack.shelf_level && 
         item1.rack.shelf_slot == item2.rack.shelf_slot;
}