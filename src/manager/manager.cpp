#include "manager/manager.hpp"

Manager::Manager(const rclcpp::NodeOptions& options)
: PlannerBase("manager", options)
{
  declare_parameter<std::vector<std::string>>("rotation_name", std::vector<std::string>{});
  declare_parameter<std::vector<std::string>>("arm_poses_name", std::vector<std::string>{});

  rotation_name_ = get_parameter("rotation_name").as_string_array();
  arm_poses_name_ = get_parameter("arm_poses_name").as_string_array();

  if (rotation_name_.empty() || arm_poses_name_.empty())
  {
    RCLCPP_INFO(get_logger(), "rotation name does not set");
    rclcpp::shutdown();
  }

  srv_ser_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_cli_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  action_cli_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  action_ser_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  robot_status_timer_ = create_wall_timer(
    std::chrono::seconds(1), 
    std::bind(&Manager::robot_status_cb, this));

  testing_timer_ = create_wall_timer(
    std::chrono::milliseconds(100), 
    std::bind(&Manager::testing_cb, this));
  testing_timer_->cancel();

  clear_occupy_timer_ = create_wall_timer(
    std::chrono::seconds(5), 
    std::bind(&Manager::clear_occupancy_cb, this));
  clear_occupy_timer_->cancel();

  testing_pub_ = create_publisher<Int32>("testing", 10);
  robot_status_pub_ = create_publisher<RobotStatus>("robot_status", 10);
  
  for (const auto& name : rotation_name_)
  {
    fold_elev_rotate_cli_[name] = create_client<Trigger>(
      "/fold_elevator/rotate_to_" + name, 
      rmw_qos_profile_services_default,
      srv_cli_cbg_);
  }

  for (RobotArm arm : all_arms)
  {
    std::unordered_map<std::string, rclcpp::Client<Trigger>::SharedPtr> arm_map;
    for (const auto& name : arm_poses_name_)
    {
      std::string srv_name = "/" + arm_to_str.at(arm) + "/move_to_" + name + "_pose";
      arm_map[name] = create_client<Trigger>(
        srv_name, 
        rmw_qos_profile_services_default,
        srv_cli_cbg_);
    }
    arm_basic_ctrl_cli_[arm] = std::move(arm_map);
  }

  get_robot_status_srv_ = create_service<GetRobotStatus>(
    "/get_robot_status", 
    std::bind(&Manager::get_robot_status_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  occupy_srv_ = create_service<Occupy>(
    "/occupy_robot", 
    std::bind(&Manager::occupy_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  new_order_srv_ = create_service<NewOrderSrv>(
    "/new_order", 
    std::bind(&Manager::new_order_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  pick_cli_ = rclcpp_action::create_client<Pick>(
    get_node_base_interface(),
    get_node_graph_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "/pick",
    action_cli_cbg_);

  place_cli_ = rclcpp_action::create_client<Place>(
    get_node_base_interface(),
    get_node_graph_interface(),
    get_node_logging_interface(),
    get_node_waitables_interface(),
    "/place",
    action_cli_cbg_);

  new_order_action_ser_ = rclcpp_action::create_server<NewOrderAction>(
    this,
    "/new_order",
    std::bind(&Manager::new_order_goal_cb, this, _1, _2),
    std::bind(&Manager::new_order_cancel_cb, this, _1),
    std::bind(&Manager::new_order_accepted_cb, this, _1),
    rcl_action_server_get_default_options(),
    action_ser_cbg_);

  robot_status_.store(RobotStatus::IDLE);
  RCLCPP_INFO(get_logger(), "Manager is up.");
}

void Manager::new_order_cb(
  const std::shared_ptr<NewOrderSrv::Request> request, 
  std::shared_ptr<NewOrderSrv::Response> response)
{
  const int order_id = request->order.id;
  const uint8_t port_id = request->order.port_id;
  auto order_items = request->order.order_items;
  
  if (order_items.size() > MAX_ORDER_ITEMS)
  {
    response->message = "Order items size is larger than MAX_ORDER_ITEMS";
    return;
  }

  if (port_id == 0)
  {
    response->message = "Incorrect table id";
    return;
  }

  RCLCPP_INFO(get_logger(), "Processing new order ID: %d for table: %d", order_id, port_id);
  std::vector<bool> items_completed(order_items.size(), false);
  std::vector<double> items_pick_height(order_items.size(), false);
  std::vector<bool> place_occupancy_map;
  place_occupancy_map.resize(6, false);

  bool all_items_completed = false;

  while (rclcpp::ok() && !all_items_completed) 
  {
    // Step 1: Choose 1-2 not finished SKUs from list
    std::vector<size_t> selected_indices = select_next_items(order_items, items_completed);
    
    if (selected_indices.empty()) 
    {
      RCLCPP_INFO(get_logger(), "All items completed for order %d", order_id);
      break;
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
      return;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    RCLCPP_INFO(get_logger(), "Step 4: Pick item");
    std::optional<std::map<uint8_t, double>> opt_results = send_pick_goal(order_items, selected_indices);

    if (!opt_results.has_value()) 
    {
      RCLCPP_ERROR(get_logger(), "Failed to pick item(s)");
      response->success = false;
      response->message = "Item picking failed";
      return;
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
      return;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Step 7: Place all carried items
    RCLCPP_INFO(get_logger(), "Step 7: Place all carried items");
    if (!send_place_goal(order_items, selected_indices, port_id, place_height, place_occupancy_map)) 
    {
      RCLCPP_ERROR(get_logger(), "Failed to place item SKU");
      response->success = false;
      response->message = "Item placement failed";
      return;
    }
      
    for (size_t idx : selected_indices) 
    {
      // Mark item as completed after successful placement
      const auto& item = order_items[idx];
      items_completed[idx] = true;
      RCLCPP_INFO(get_logger(), "Successfully placed SKU: %d", item.sku.id);
    }

    for (size_t idx : selected_indices) 
    {
      if (items_completed[idx] && order_items[idx].sku.is_suctionable)
        move_to_basic_pose(RobotArm::LEFT_ACTION, "home");
      else if (items_completed[idx] && order_items[idx].sku.is_grippable)
        move_to_basic_pose(RobotArm::RIGHT_ACTION, "home");
    }

    // Step 8: Check if all items are finished
    all_items_completed = std::all_of(items_completed.begin(), items_completed.end(), 
      [](bool completed) { 
        return completed; 
      });
  }

  if (!rotate_to("abs_front"))
  {
    RCLCPP_ERROR(get_logger(), "abs_front");
    return;
  }

  RCLCPP_INFO(get_logger(), "Order %d completed successfully", order_id);
  response->success = true;
  response->message = "Order processed successfully";
}

void Manager::get_robot_status_cb(
  const std::shared_ptr<GetRobotStatus::Request> request, 
  std::shared_ptr<GetRobotStatus::Response> response)
{
  (void) request;
  
  RobotStatus msg;
  msg.state = robot_status_.load();

  response->status = std::move(msg);
  response->success = true;
}

void Manager::occupy_cb(
  const std::shared_ptr<Occupy::Request> request, 
  std::shared_ptr<Occupy::Response> response)
{
  RobotStatus msg;
  msg.state = robot_status_.load();

  response->status = std::move(msg);

  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!client_name_.empty())
    {
      response->message = "Robot is occupied by " + client_name_;
      RCLCPP_WARN(get_logger(), "Robot is occupied by %s", client_name_.c_str());
      return;
    }

    client_name_ = request->client_name;
  }

  response->success = true;
  RCLCPP_WARN(get_logger(), "Occupy request is accepted [%s]", request->client_name.c_str());
  clear_occupy_timer_->reset();
}

void Manager::robot_status_cb(void)
{
  if (!robot_status_pub_ || robot_status_pub_->get_subscription_count() == 0)
    return;

  RobotStatus msg;
  msg.state = robot_status_.load();
  robot_status_pub_->publish(msg);
}

void Manager::clear_occupancy_cb(void)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (client_name_.empty())
      RCLCPP_WARN(get_logger(), "client_name is empty but clear occupancy timer triggered");
    else
      client_name_.clear();
  }

  clear_occupy_timer_->cancel();
}

void Manager::testing_cb(void)
{
  if (!testing_pub_ || testing_pub_->get_subscription_count() == 0)
    return;

  std::random_device rd;
  std::mt19937 gen(rd());

  std::uniform_int_distribution<int> dis(0, 100);

  Int32 msg;
  msg.data = dis(gen);

  testing_pub_->publish(msg);
  RCLCPP_INFO(get_logger(), "testing_cb: %d", msg.data);
}
