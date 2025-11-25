#include "manipulation/workflow_planner/workflow_planner.hpp"

WorkflowPlanner::WorkflowPlanner(
  const rclcpp::NodeOptions& options,
  std::shared_ptr<FoldElevatorDriver> fold_elev_driver,
  std::shared_ptr<MotionPlanner> motion_planner)
: PlannerBase("workflow_planner", options)
  , fold_elev_driver_(fold_elev_driver)
  , motion_planner_(motion_planner)
  , state_(RobotStatus::IDLE)
{
  declare_parameter<bool>("sim", false);
  declare_parameter<double>("valid_z_threshold", 0.01);
  declare_parameter<int>("max_pick_attempt", 0);
  declare_parameter<int>("max_scan_attempt", 0);
  declare_parameter<double>("re_scan_x_translation", 0.05);
  declare_parameter<double>("re_scan_y_translation", 0.1);
  declare_parameter<double>("place_offset", 0.02);
  declare_parameter<double>("scan_x_distance", 0.05);
  declare_parameter<double>("scan_z_distance", 0.2);
  declare_parameter<double>("optimal_arm_flat_height_distance", 0.2);
  declare_parameter<double>("optimal_arm_flat_distance", 0.2);
  declare_parameter<double>("optimal_arm_flat_front_distance", 0.5);
  declare_parameter<double>("table_front_offset", 0.5);
  declare_parameter<double>("table_height_offset", 0.0);
  declare_parameter<std::vector<double>>("tcp_to_left_camera", std::vector<double>{});
  declare_parameter<std::vector<double>>("tcp_to_right_camera", std::vector<double>{});

  get_parameter("sim", simulation_);
  get_parameter("valid_z_threshold", valid_z_threshold_);
  get_parameter("max_scan_attempt", max_scan_attempt_);
  get_parameter("optimal_arm_flat_height_distance", optimal_arm_flat_height_distance_);
  get_parameter("table_front_offset", table_front_offset_);
  get_parameter("table_height_offset", table_height_offset_);

  setup_camera_transform(RobotArm::LEFT, "left");
  setup_camera_transform(RobotArm::RIGHT, "right");

  RCLCPP_INFO(get_logger(), "Workflow Planner - initiated motion_planner");

  action_ser_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  vision_srv_cli_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  plan_srv_cli_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cam_srv_cli_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  exec_timer_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  tf_timer_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_ser_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  init_timer_ = create_wall_timer(
    std::chrono::seconds(5), 
    std::bind(&WorkflowPlanner::init_cb, this));

  if (simulation_)
    init_timer_->cancel();

  tf_pub_timer_ = create_wall_timer(
    std::chrono::milliseconds(20), 
    std::bind(&WorkflowPlanner::tf_pub_cb, this),
    tf_timer_cbg_);

  debug_sub_ = create_subscription<Float32>(
    "debug_function", 
    10, 
    std::bind(&WorkflowPlanner::debug_cb, this, _1));

  const std::string left_camera_prefix = "/left_camera/realsense";
  const std::string right_camera_prefix = "/right_camera/realsense";
  const std::vector<std::tuple<RobotArm, std::string, std::string>> camera_srv = {
    { RobotArm::LEFT, "/get_state", "/change_state" },
    { RobotArm::RIGHT, "/get_state", "/change_state" }
  };

  for (const auto& cam : camera_srv)
  {
    const std::string prefix = std::get<0>(cam) == RobotArm::LEFT ? left_camera_prefix : right_camera_prefix;

    get_camera_cli_[std::get<0>(cam)] = create_client<GetState>(
      prefix + std::get<1>(cam), 
      rmw_qos_profile_services_default,
      cam_srv_cli_cbg_);

    change_camera_cli_[std::get<0>(cam)] = create_client<ChangeState>(
      prefix + std::get<2>(cam), 
      rmw_qos_profile_services_default,
      cam_srv_cli_cbg_);

    RCLCPP_INFO(get_logger(), "Created a RobotArm %s camera lifecycle clients", arm_to_str.at(std::get<0>(cam)).c_str());
  }

  get_slot_state_tri_cli_ = create_client<GetSlotStateTrigger>(
    "get_slot_state_trigger", 
    rmw_qos_profile_services_default,
    vision_srv_cli_cbg_);
  
  get_obj_pose_tri_cli_ = create_client<GetObjectPoseTrigger>(
    "get_object_pose_trigger", 
    rmw_qos_profile_services_default,
    vision_srv_cli_cbg_);

  pick_plan_cli_ = create_client<PickPlan>(
    "pick_plan", 
    rmw_qos_profile_services_default,
    plan_srv_cli_cbg_);

  place_plan_cli_ = create_client<PlacePlan>(
    "place_plan", 
    rmw_qos_profile_services_default,
    plan_srv_cli_cbg_);

  RCLCPP_INFO(get_logger(), "Workflow Planner - initiated service clients");

  simple_pick_srv_ = create_service<SimplePick>(
    "simple_pick",
    std::bind(&WorkflowPlanner::simple_pick_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  simple_place_srv_ = create_service<SimplePlace>(
    "simple_place",
    std::bind(&WorkflowPlanner::simple_place_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  RCLCPP_INFO(get_logger(), "Workflow Planner - initiated service servers");

  pick_action_ser_ = rclcpp_action::create_server<Pick>(
    this,
    "pick",
    std::bind(&WorkflowPlanner::pick_goal_cb, this, _1, _2),
    std::bind(&WorkflowPlanner::pick_cancel_cb, this, _1),
    std::bind(&WorkflowPlanner::pick_accepted, this, _1),
    rcl_action_server_get_default_options(),
    action_ser_cbg_);

  place_action_ser_ = rclcpp_action::create_server<Place>(
    this,
    "place",
    std::bind(&WorkflowPlanner::place_goal_cb, this, _1, _2),
    std::bind(&WorkflowPlanner::place_cancel_cb, this, _1),
    std::bind(&WorkflowPlanner::place_accepted, this, _1),
    rcl_action_server_get_default_options(),
    action_ser_cbg_);

  scan_sku_action_ser_ = rclcpp_action::create_server<ScanSku>(
    this,
    "scan_sku",
    std::bind(&WorkflowPlanner::scan_sku_goal_cb, this, _1, _2),
    std::bind(&WorkflowPlanner::scan_sku_cancel_cb, this, _1),
    std::bind(&WorkflowPlanner::scan_sku_accepted, this, _1),
    rcl_action_server_get_default_options(),
    action_ser_cbg_);

  replenish_action_ser_ = rclcpp_action::create_server<Replenish>(
    this,
    "replenish",
    std::bind(&WorkflowPlanner::replenish_goal_cb, this, _1, _2),
    std::bind(&WorkflowPlanner::replenish_cancel_cb, this, _1),
    std::bind(&WorkflowPlanner::replenish_accepted, this, _1),
    rcl_action_server_get_default_options(),
    action_ser_cbg_);
    
  RCLCPP_INFO(get_logger(), "Workflow Planner - initiated action servers");

  RCLCPP_INFO(get_logger(), "Workflow Planner is up.");
}

WorkflowPlanner::~WorkflowPlanner()
{

}

void WorkflowPlanner::setup_camera_transform(RobotArm arm, const std::string& cam_name)
{
  std::string param_name = "tcp_to_" + cam_name + "_camera";
  auto tmp = get_parameter(param_name).as_double_array();
  
  if (tmp.size() == 7) 
  {
    g_tcp__cam_[arm] = get_g(tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5], tmp[6]);
    RCLCPP_WARN(get_logger(), "g_tcp__%s_cam_ got from config yaml", cam_name.c_str());
  } 
  else 
  {
    g_tcp__cam_[arm] = get_g(0, 0, 0, 0, 0, 0, 1);
    RCLCPP_WARN(get_logger(), "g_tcp__%s_cam_ not defined, using identity", cam_name.c_str());
  }
  
  print_g(g_tcp__cam_[arm]);
}

void WorkflowPlanner::tf_pub_cb(void)
{
  std::lock_guard<std::mutex> lock(tf_mutex_);

  for (const auto& tf : tf_buf_)
  {
    std::apply(std::bind(&WorkflowPlanner::send_transform, this, _1, _2, _3), tf);
  }
}

void WorkflowPlanner::debug_cb(const Float32::SharedPtr msg)
{
  (void) msg;
}

void WorkflowPlanner::init_cb(void)
{
  const std::vector<RobotArm> arms = { RobotArm::LEFT, RobotArm::RIGHT }; 
  RCLCPP_INFO(get_logger(), "Starting camera lifecycle initialization check");

  std::string srv = "/realsense/set_parameters";
  std::map<RobotArm, rclcpp::Client<SetParameters>::SharedPtr> set_cam_param_cli;
  
  for (const auto& arm : arms)
  {
    const std::string prefix = arm == RobotArm::LEFT ? "/left_camera" : "/right_camera";
    set_cam_param_cli[arm] = create_client<SetParameters>(
      prefix + srv, 
      rmw_qos_profile_services_default,
      cam_srv_cli_cbg_);

    RCLCPP_INFO(get_logger(), "Created a RobotArm %s camera set param client", arm_to_str.at(arm).c_str());
  }

  auto set_cam_param = [this, &set_cam_param_cli](RobotArm arm, bool enable_depth) -> bool {
    const std::string arm_name = (arm == RobotArm::LEFT) ? "LEFT" : "RIGHT";
    
    Parameter param;
    param.name = "enable_depth";
    param.value.type = 1;
    param.value.bool_value = enable_depth;

    std::vector<Parameter> params;
    params.emplace_back(std::move(param));

    bool success = set_camera_param(set_cam_param_cli[arm], std::move(params));
    
    if (success) 
    {
      RCLCPP_INFO(get_logger(), "Successfully set enable_depth to %s for %s arm", 
        enable_depth ? "true" : "false", arm_name.c_str());
    } 
    else 
    {
      RCLCPP_ERROR(get_logger(), "Failed to set enable_depth to %s for %s arm", 
        enable_depth ? "true" : "false", arm_name.c_str());
    }
    
    return success;
  };

  for (const auto& arm : arms)
  {
    const std::string arm_name = (arm == RobotArm::LEFT) ? "LEFT" : "RIGHT";
    RCLCPP_DEBUG(get_logger(), "Checking %s arm camera lifecycle", arm_name.c_str());

    std::optional<State> opt = get_camera_lifecycle_state(arm);
    
    if (!opt.has_value())
    {
      RCLCPP_ERROR(get_logger(), "Failed to get camera lifecycle state for %s arm - state unavailable", arm_name.c_str());
      return;
    }

    State current_state = opt.value();
    RCLCPP_INFO(get_logger(), "%s arm camera lifecycle state: %d", arm_name.c_str(), current_state.id);

    if (current_state.id == State::PRIMARY_STATE_ACTIVE)
    {
      continue;
    }

    if (current_state.id == State::PRIMARY_STATE_INACTIVE)
    {
      RCLCPP_INFO(get_logger(), "Camera for %s arm is in INACTIVE state (current state: %d)", arm_name.c_str(), current_state.id);
      
      if (!set_camera_lifecycle(arm, true))
      {
        RCLCPP_ERROR(get_logger(), "Failed to activate camera for %s arm", arm_name.c_str());
        return;
      }
      RCLCPP_INFO(get_logger(), "Successfully activated camera for %s arm", arm_name.c_str());
    }
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  for (const auto& arm : arms) 
  {
    set_cam_param(arm, false);
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  for (const auto& arm : arms) 
  {
    set_cam_param(arm, true);
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  for (const auto& arm : arms)
  {
    const std::string arm_name = (arm == RobotArm::LEFT) ? "LEFT" : "RIGHT";
    RCLCPP_INFO(get_logger(), "Deactivating camera for %s arm", arm_name.c_str());
    std::optional<State> opt = get_camera_lifecycle_state(arm);
    
    if (!opt.has_value())
    {
      RCLCPP_ERROR(get_logger(), "Failed to get camera lifecycle state for %s arm - state unavailable", arm_name.c_str());
      return;
    }

    State current_state = opt.value();
    RCLCPP_INFO(get_logger(), "%s arm camera lifecycle state: %d", arm_name.c_str(), current_state.id);

    if (current_state.id == State::PRIMARY_STATE_ACTIVE)
    {
      if (!set_camera_lifecycle(arm, false))
      {
        RCLCPP_ERROR(get_logger(), "Failed to deactivate camera for %s arm", arm_name.c_str());
        return;
      }
      RCLCPP_INFO(get_logger(), "Successfully deactivated camera for %s arm", arm_name.c_str());
    }
  }

  RCLCPP_INFO(get_logger(), "Camera lifecycle initialization completed successfully - all camera nodes are operational");
  init_timer_->cancel();
}