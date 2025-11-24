#include "manipulation/action_planner/action_planner.hpp"

ActionPlanner::ActionPlanner(
  const rclcpp::NodeOptions& options)
: PlannerBase("action_planner", options)
{
  declare_parameter<std::vector<double>>("eef_offset", std::vector<double>{});
  declare_parameter<double>("obj_pose_offset", 0.0);
  declare_parameter<double>("pre_obj_pose_shift", 0.0);
  declare_parameter<double>("pre_place_pose_shift", 0.0);
  declare_parameter<double>("post_pick_lift_offset", 0.0);
  declare_parameter<double>("post_pick_back_offset", 0.0);
  declare_parameter<double>("post_place_down_offset", 0.0);

  eef_offset_ = get_parameter("eef_offset").as_double_array();

  tf_pub_timer = create_wall_timer(
    std::chrono::milliseconds(100), 
    std::bind(&ActionPlanner::tf_pub_cb, this));

  // no longer used
  tf_pub_timer->cancel();

  pick_plan_srv_ = create_service<PickPlan>(
    "pick_plan", 
    std::bind(&ActionPlanner::pick_plan_cb, this, _1, _2),
    rmw_qos_profile_services_default);

  place_plan_srv_ = create_service<PlacePlan>(
    "place_plan", 
    std::bind(&ActionPlanner::place_plan_cb, this, _1, _2),
    rmw_qos_profile_services_default);

  RCLCPP_INFO(get_logger(), "Action Planner is up.");
}

ActionPlanner::~ActionPlanner()
{

}

void ActionPlanner::tf_pub_cb(void)
{
  std::lock_guard<std::mutex> lock(tf_mutex_);

  for (const auto& tf : tf_buf_)
  {
    std::apply(std::bind(&ActionPlanner::send_transform, this, _1, _2, _3), tf);
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto options = rclcpp::NodeOptions();
  auto action_planner = std::make_shared<ActionPlanner>(options);

  exec->add_node(action_planner->get_node_base_interface());
  exec->spin();

  rclcpp::shutdown();
}