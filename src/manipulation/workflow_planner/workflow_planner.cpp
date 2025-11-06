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
  declare_parameter<double>("valid_z_threshold", 0.01);
  declare_parameter<int>("max_pick_attempt", 0);
  declare_parameter<int>("max_scan_attempt", 0);
  declare_parameter<double>("re_scan_translation", 0.05);
  declare_parameter<double>("place_offset", 0.02);
  declare_parameter<double>("scan_distance", 0.25);
  declare_parameter<double>("optimal_arm_flat_height_distance", 0.2);
  declare_parameter<double>("optimal_arm_flat_distance", 0.2);
  declare_parameter<double>("optimal_arm_flat_front_distance", 0.5);
  declare_parameter<double>("table_front_offset", 0.5);
  declare_parameter<double>("table_height_offset", 0.0);
  declare_parameter<std::string>("poses_file", "");
  declare_parameter<std::vector<double>>("tcp_to_left_camera", std::vector<double>{});
  declare_parameter<std::vector<double>>("tcp_to_right_camera", std::vector<double>{});

  std::string poses_file;
  get_parameter("valid_z_threshold", valid_z_threshold_);
  get_parameter("max_pick_attempt", max_pick_attempt_);
  get_parameter("max_scan_attempt", max_scan_attempt_);
  get_parameter("re_scan_translation", re_scan_translation_);
  get_parameter("place_offset", place_offset_);
  get_parameter("scan_distance", scan_distance_);
  get_parameter("optimal_arm_flat_height_distance", optimal_arm_flat_height_distance_);
  get_parameter("table_front_offset", table_front_offset_);
  get_parameter("table_height_offset", table_height_offset_);
  get_parameter("poses_file", poses_file);

  setup_camera_transform(RobotArm::LEFT, "left");
  setup_camera_transform(RobotArm::RIGHT, "right");

  RCLCPP_INFO(get_logger(), "Workflow Planner - initiated motion_planner");

  action_ser_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  vision_srv_cli_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  plan_srv_cli_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  exec_timer_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  tf_timer_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  tf_pub_timer = create_wall_timer(
    std::chrono::milliseconds(20), 
    std::bind(&WorkflowPlanner::tf_pub_cb, this),
    tf_timer_cbg_);

  debug_sub_ = create_subscription<Float32>(
    "debug_function", 
    10, 
    std::bind(&WorkflowPlanner::debug_cb, this, _1));

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
    g_tcp__cam_[arm] = get_g(0, 0, 0, 0, 0, 0, 0);
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