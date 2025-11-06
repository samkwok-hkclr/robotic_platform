#include "manipulation/motion_planner/motion_planner.hpp"

MotionPlanner::MotionPlanner(
  const rclcpp::NodeOptions& options,
  std::shared_ptr<VacuumGripper> vac_gripper)
: PlannerBase("motion_planner", options),
  vac_gripper_(vac_gripper)
{
  declare_parameter<bool>("sim", false);
  declare_parameter<bool>("enable_ultrasonic", false);

  declare_parameter<int>("max_pick_attempt", 0);
  declare_parameter<std::string>("poses_file", "");
  declare_parameter<std::vector<double>>("left_zero_joints", std::vector<double>{});
  declare_parameter<std::vector<double>>("left_home_joints", std::vector<double>{});
  declare_parameter<std::vector<double>>("left_holding_joints", std::vector<double>{});
  declare_parameter<std::vector<double>>("left_action_joints", std::vector<double>{});
  declare_parameter<std::vector<double>>("right_zero_joints", std::vector<double>{});
  declare_parameter<std::vector<double>>("right_home_joints", std::vector<double>{});
  declare_parameter<std::vector<double>>("right_holding_joints", std::vector<double>{});
  declare_parameter<std::vector<double>>("right_action_joints", std::vector<double>{});

  std::string poses_file;
  get_parameter("sim", simulation_);
  get_parameter("enable_ultrasonic", enable_ultrasonic_);
  get_parameter("max_pick_attempt", max_pick_attempt_);
  get_parameter("poses_file", poses_file);

  std::vector<double> left_zero_joints = get_parameter("left_zero_joints").as_double_array();
  std::vector<double> left_home_joints = get_parameter("left_home_joints").as_double_array();
  std::vector<double> left_holding_joints = get_parameter("left_holding_joints").as_double_array();
  std::vector<double> left_action_joints = get_parameter("left_action_joints").as_double_array();
  std::vector<double> right_zero_joints = get_parameter("right_zero_joints").as_double_array();
  std::vector<double> right_home_joints = get_parameter("right_home_joints").as_double_array();
  std::vector<double> right_holding_joints = get_parameter("right_holding_joints").as_double_array();
  std::vector<double> right_action_joints = get_parameter("right_action_joints").as_double_array();

  if (left_zero_joints.size() != 7 || right_zero_joints.size() != 7)
  {
    rclcpp::shutdown();
    return;
  }
  zero_joint_pose_[RobotArm::LEFT] = std::move(left_zero_joints);
  zero_joint_pose_[RobotArm::RIGHT] = std::move(right_zero_joints);

  if (left_home_joints.size() != 7 || right_home_joints.size() != 7)
  {
    rclcpp::shutdown();
    return;
  }
  home_joint_pose_[RobotArm::LEFT] = std::move(left_home_joints);
  home_joint_pose_[RobotArm::RIGHT] = std::move(right_home_joints);

  if (left_holding_joints.size() != 7 || right_holding_joints.size() != 7)
  {
    rclcpp::shutdown();
    return;
  }
  holding_joint_pose_[RobotArm::LEFT] = std::move(left_holding_joints);
  holding_joint_pose_[RobotArm::RIGHT] = std::move(right_holding_joints);

  if (left_action_joints.size() != 7 || right_action_joints.size() != 7)
  {
    rclcpp::shutdown();
    return;
  }
  action_joint_pose_[RobotArm::LEFT] = std::move(left_action_joints);
  action_joint_pose_[RobotArm::RIGHT] = std::move(right_action_joints);

  PosesLoader loader;
  std::optional<YAML::Node> config = loader.parse_yaml(poses_file);
  if (!config.has_value())
  {
    rclcpp::shutdown();
    return;
  }

  pose_actions = {
    { RobotArm::LEFT, 
      { 
        { "zero_pose", std::bind(&MotionPlanner::move_to_zero_pose, this, _1, _2) },
        { "home_pose", std::bind(&MotionPlanner::move_to_home_pose, this, _1, _2) },
        { "holding_pose", std::bind(&MotionPlanner::move_to_holding_pose, this, _1, _2) },
        { "action_pose", std::bind(&MotionPlanner::move_to_action_pose, this, _1, _2) },
      }
    },
    { RobotArm::RIGHT,
      {
        { "zero_pose", std::bind(&MotionPlanner::move_to_zero_pose, this, _1, _2) },
        { "home_pose", std::bind(&MotionPlanner::move_to_home_pose, this, _1, _2) },
        { "holding_pose", std::bind(&MotionPlanner::move_to_holding_pose, this, _1, _2) },
        { "action_pose", std::bind(&MotionPlanner::move_to_action_pose, this, _1, _2) },
      }
    }
};

  // loader.load_pose_from_yaml(config.value(), "home_pose", home_pose_);

  RCLCPP_INFO(get_logger(), "Motion Planner - home_pose:");
  // print_pose(home_pose_);

  RCLCPP_INFO(get_logger(), "Motion Planner - All poses are loaded");

  exec_srv_cli_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_ser_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  const std::array<RobotArm, 3> all_arms = {RobotArm::LEFT, RobotArm::RIGHT, RobotArm::WHOLE};

  for (RobotArm arm : all_arms)
  {
    exec_wps_cli_[arm] = create_client<ExecuteWaypoints>(
      "/" + arm_to_str.at(arm) + "/execute_waypoints", 
      rmw_qos_profile_services_default,
      exec_srv_cli_cbg_);

    exec_joints_cli_[arm] = create_client<ExecuteJoints>(
      "/" + arm_to_str.at(arm) + "/execute_joints", 
      rmw_qos_profile_services_default,
      exec_srv_cli_cbg_);

    exec_pose_cli_[arm] = create_client<ExecutePose>(
      "/" + arm_to_str.at(arm) + "/execute_pose", 
      rmw_qos_profile_services_default,
      exec_srv_cli_cbg_);
  }

  // move_to_srv_.reserve(pose_actions.size());
  for (const auto& [arm, actions] : pose_actions) 
  {
    for (const auto& [pose_name, func] : actions) 
    {
      std::string service_name = "/" + arm_to_str.at(arm) + "/move_to_" + std::string(pose_name);

      move_to_srv_.emplace_back(
        create_service<std_srvs::srv::Trigger>(
          service_name,
          std::bind(&MotionPlanner::move_to_cb, this, _1, _2, arm, pose_name),
          rmw_qos_profile_services_default,
          srv_ser_cbg_
        )
      );
        
      RCLCPP_INFO(this->get_logger(), "Created service: %s", service_name.c_str());
    }
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
  RobotArm arm,
  std::string_view pose)
{
  (void) request;
  const float default_speed = 100.0f;

  auto arm_it = pose_actions.find(arm);
  
  if (arm_it == pose_actions.end()) 
  {
    RCLCPP_ERROR(get_logger(), "Unknown arm requested");
    response->success = false;
    response->message = "Unknown arm requested";
    return;
  }

  auto& pose_map = arm_it->second;
  auto pose_it = pose_map.find(pose);

  if (pose_it != pose_map.end()) 
  {
    if (pose_it->second(arm, default_speed)) 
    {
      RCLCPP_INFO(get_logger(), "Successfully moved %s arm to pose '%s' (speed: %.1f)", 
        (arm == RobotArm::LEFT) ? "left" : "right", 
        pose.data(), default_speed);
      response->success = true;
      response->message = "Motion completed successfully";
    } 
    else 
    {
      RCLCPP_ERROR(get_logger(), "Failed to move %s arm to pose '%s' (speed: %.1f)", 
        (arm == RobotArm::LEFT) ? "left" : "right", 
        pose.data(), default_speed);
      response->success = false;
      response->message = "Motion execution failed";
    }
  }
  else 
  {
    std::string available_poses;
    for (const auto& pair : pose_map) 
    {
      if (!available_poses.empty()) 
        available_poses += ", ";
      
      available_poses += "'" + std::string(pair.first) + "'";
    }
    
    RCLCPP_ERROR(get_logger(), "Unknown pose '%s' for %s arm - Available poses: [%s]", 
      pose.data(), 
      (arm == RobotArm::LEFT) ? "left" : "right",
      available_poses.c_str());    
    response->success = false;
    response->message = "Unknown pose requested for this arm";
  }  

  // if (auto it = pose_actions.find(pose); it != pose_actions.end()) 
  // {
  //   if (it->second(arm, default_speed)) 
  //   {
  //     RCLCPP_INFO(get_logger(), "Successfully moved to pose '%s' (speed: %.1f)", pose.data(), default_speed);
  //     response->success = true;
  //   }
  //   else
  //   {
  //     RCLCPP_ERROR(get_logger(), "Failed to move to pose '%s' (speed: %.1f)", pose.data(), default_speed);
  //   }
  // } 
  // else 
  // {
  //   std::string available_poses;
  //   for (const auto& pair : pose_actions) 
  //   {
  //     if (!available_poses.empty()) 
  //       available_poses += ", ";
      
  //     available_poses += "'" + std::string(pair.first) + "'";
  //   }
    
  //   RCLCPP_ERROR(get_logger(), "Unknown pose requested: '%s' - Available poses: [%s]", pose.data(), available_poses.c_str());
  //   response->success = false;
  //   response->message = "Unknown pose requested";
  // }  
}