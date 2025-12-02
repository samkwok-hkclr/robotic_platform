#include "manager/manager.hpp"

Manager::Manager(const rclcpp::NodeOptions& options)
: PlannerBase("manager", options)
{
  declare_parameter<bool>("sim", false);
  declare_parameter<std::vector<std::string>>("rotation_name", std::vector<std::string>{});
  declare_parameter<std::vector<std::string>>("arm_poses_name", std::vector<std::string>{});

  get_parameter("sim", simulation_);
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
    if (name.empty())
      continue;

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
    "/occupy", 
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
  (void) request;
  response->message = "not supported";
  response->success = false;
  return;
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
  RCLCPP_WARN(get_logger(), "Occupy timer started for [%s]", request->client_name.c_str());
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
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (client_name_.empty())
    RCLCPP_WARN(get_logger(), "client_name is empty but clear occupancy timer triggered");
  else
  {
    client_name_.clear();
    RCLCPP_WARN(get_logger(), "Timeout! client_name will be clear");
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
