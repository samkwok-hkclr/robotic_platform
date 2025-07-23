#include "manipulation/action_planner/action_planner.hpp"

ActionPlanner::ActionPlanner(
  const rclcpp::NodeOptions& options)
: PlannerBase("action_planner", options)
{
  declare_parameter<std::vector<double>>("eef_offset", std::vector<double>{});
  declare_parameter<double>("pre_obj_pose_shift", 0.0);
  declare_parameter<double>("pre_place_pose_shift", 0.0);
  declare_parameter<double>("post_pick_lift_offset", 0.0);
  declare_parameter<double>("post_pick_back_offset", 0.0);
  declare_parameter<double>("post_place_down_offset", 0.0);

  eef_offset_ = get_parameter("eef_offset").as_double_array();
  get_parameter("pre_obj_pose_shift", pre_obj_pose_shift_);
  get_parameter("pre_place_pose_shift", pre_place_pose_shift_);
  get_parameter("post_pick_lift_offset", post_pick_lift_offset_);
  get_parameter("post_pick_back_offset", post_pick_back_offset_);
  get_parameter("post_place_down_offset", post_place_down_offset_);

  srv_ser_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  pick_plan_srv_ = create_service<PickPlan>(
    "pick_plan", 
    std::bind(&ActionPlanner::pick_plan_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  place_plan_srv_ = create_service<PlacePlan>(
    "place_plan", 
    std::bind(&ActionPlanner::place_plan_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_ser_cbg_);

  RCLCPP_INFO(get_logger(), "Action Planner is up.");
}

ActionPlanner::~ActionPlanner()
{

}
