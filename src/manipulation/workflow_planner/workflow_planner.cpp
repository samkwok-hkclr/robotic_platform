#include "manipulation/workflow_planner/workflow_planner.hpp"

WorkflowPlanner::WorkflowPlanner(
  const rclcpp::NodeOptions& options,
  std::shared_ptr<MotionPlanner> motion_planner)
: PlannerBase("workflow_planner", options)
{
  declare_parameter<std::string>("poses_file", "");

  std::string poses_file;
  get_parameter("poses_file", poses_file);

  PosesLoader loader;
  std::optional<YAML::Node> config = loader.parse_yaml(poses_file);
  if (!config.has_value())
  {
    rclcpp::shutdown();
    return;
  }

  // std::vector<std::pair<int, int>> fixme = {}; // FIXME: how to avoid to create this vector?
  loader.load_ordered_poses_from_yaml(config.value(), "scan_poses", scan_poses_, scan_order_);
  // loader.load_poses_from_yaml(config.value(), "place_poses", place_poses_, fixme, false);
  loader.load_poses_from_yaml(config.value(), "place_poses", place_poses_);

  motion_planner_ = motion_planner;
  RCLCPP_INFO(get_logger(), "Workflow Planner - initiated motion_planner");

  RCLCPP_INFO(get_logger(), "Motion Planner - scan_poses:");
  for (const auto& pose : scan_poses_)
  {
    print_pose(pose.second);
  }

  RCLCPP_INFO(get_logger(), "Motion Planner - scan_order:");
  for (const auto& p : scan_order_)
  {
    RCLCPP_WARN(get_logger(), "order: %d, sku_id: %d", p.first, p.second);
  }

  RCLCPP_INFO(get_logger(), "Motion Planner - place_poses:");
  for (const auto& pose : place_poses_)
  {
    print_pose(pose.second);
  }

  state_ = RobotStatus::IDLE;

  action_ser_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  vision_srv_cli_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  plan_srv_cli_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  exec_timer_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  tf_timer_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  tf_pub_timer = create_wall_timer(
    std::chrono::seconds(1), 
    std::bind(&WorkflowPlanner::tf_pub_cb, this),
    tf_timer_cbg_);

  get_curr_pose_cli_ = create_client<GetCurrentPose>(
    "get_current_pose", 
    rmw_qos_profile_services_default,
    exec_srv_cli_cbg_);

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

void WorkflowPlanner::tf_pub_cb(void)
{
  std::lock_guard<std::mutex> lock(tf_mutex_);

  for (const auto& tf : tf_buf_)
  {
    std::apply(std::bind(&WorkflowPlanner::send_transform, this, _1, _2, _3), tf);
  }
}