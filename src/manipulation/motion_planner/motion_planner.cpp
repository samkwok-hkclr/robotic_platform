#include "manipulation/motion_planner/motion_planner.hpp"

MotionPlanner::MotionPlanner(
  const rclcpp::NodeOptions& options,
  std::shared_ptr<VacuumGripperCtlr> gripper_node)
: PlannerBase("motion_planner", options)
{
  declare_parameter<std::string>("poses_file", "");
  declare_parameter<std::vector<std::string>>("move_to_srv_names");

  std::string poses_file;
  get_parameter("poses_file", poses_file);
  move_to_srv_names_ = get_parameter("move_to_srv_names").as_string_array();

  PosesLoader loader;
  std::optional<YAML::Node> config = loader.parse_yaml(poses_file);
  if (!config.has_value())
  {
    rclcpp::shutdown();
    return;
  }

  pose_actions = {
    {"home_pose", std::bind(&MotionPlanner::move_to_home_pose, this, _1)},
    {"middle_pose", std::bind(&MotionPlanner::move_to_middle_pose, this, _1)},
    {"pre_pick_pose", std::bind(&MotionPlanner::move_to_pre_pick_pose, this, _1)},
    {"pre_place_pose", std::bind(&MotionPlanner::move_to_pre_place_pose, this, _1)}
  };

  loader.load_pose_from_yaml(config.value(), "home_pose", home_pose_);
  loader.load_pose_from_yaml(config.value(), "middle_pose", middle_pose_);
  loader.load_pose_from_yaml(config.value(), "pre_scan_pose", pre_scan_pose_);
  loader.load_pose_from_yaml(config.value(), "pre_pick_pose", pre_pick_pose_);
  loader.load_pose_from_yaml(config.value(), "pre_place_pose", pre_place_pose_);
  loader.load_waypoints_from_yaml(config.value(), "before_pick_waypoints", before_pick_waypoints_);
  loader.load_waypoints_from_yaml(config.value(), "lifted_pick_waypoints", lifted_pick_waypoints_);
  loader.load_waypoints_from_yaml(config.value(), "before_place_waypoints", before_place_waypoints_);
  loader.load_waypoints_from_yaml(config.value(), "lifted_place_waypoints", lifted_place_waypoints_);
  
  RCLCPP_INFO(get_logger(), "Motion Planner - home_pose:");
  print_pose(home_pose_);
  RCLCPP_INFO(get_logger(), "Motion Planner - middle_pose:");
  print_pose(middle_pose_);
  RCLCPP_INFO(get_logger(), "Motion Planner - pre_pick_pose:");
  print_pose(pre_pick_pose_);
  RCLCPP_INFO(get_logger(), "Motion Planner - pre_place_pose:");
  print_pose(pre_place_pose_);

  RCLCPP_INFO(get_logger(), "Motion Planner - before_pick_waypoints:");
  print_pose_arr(before_pick_waypoints_);

  RCLCPP_INFO(get_logger(), "Motion Planner - lifted_pick_waypoints:");
  print_pose_arr(lifted_pick_waypoints_);

  RCLCPP_INFO(get_logger(), "Motion Planner - All poses are loaded");

  gripper_ = gripper_node;
  
  RCLCPP_INFO(get_logger(), "Motion Planner - initiated gripper node");

  exec_srv_cli_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_ser_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  exec_wps_cli_ = create_client<ExecuteWaypoints>(
    "execute_waypoints", 
    rmw_qos_profile_services_default,
    exec_srv_cli_cbg_);

  for (const auto& srv_name : move_to_srv_names_)
  {
    move_to_srv_.push_back(
      create_service<Trigger>(
        "move_to_" + srv_name, 
        std::bind(&MotionPlanner::move_to_cb, this, _1, _2, srv_name),
        rmw_qos_profile_services_default,
        srv_ser_cbg_)
    );
  }

  RCLCPP_INFO(get_logger(), "Motion Planner - initiated service clients");

  RCLCPP_INFO(get_logger(), "Motion Planner is up.");
}

MotionPlanner::~MotionPlanner()
{

}

void MotionPlanner::move_to_cb(
  const std::shared_ptr<Trigger::Request> request, 
  std::shared_ptr<Trigger::Response> response,
  const std::string& pose)
{
  (void) request;

  if (auto it = pose_actions.find(pose); it != pose_actions.end()) 
  {
    if (it->second(40.0)) 
    {
      RCLCPP_INFO(get_logger(), "Successfully moved to pose '%s' (speed: %.1f)", pose.c_str(), 40.0);
      response->success = true;
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Failed to move to pose '%s' (speed: %.1f)", pose.c_str(), 40.0);
    }
  } 
  else 
  {
    std::string available_poses;
    for (const auto& pair : pose_actions) 
    {
      if (!available_poses.empty()) 
      {
        available_poses += ", ";
      }
      available_poses += "'" + pair.first + "'";
    }
    // can I use the it to get the pose name?
    RCLCPP_ERROR(get_logger(), "Unknown pose requested: '%s' - Available poses: [%s]", pose.c_str(), available_poses.c_str());
    response->success = false;
    response->message = "Unknown pose requested";
  }  
}