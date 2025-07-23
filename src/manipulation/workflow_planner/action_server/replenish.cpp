#include "manipulation/workflow_planner/workflow_planner.hpp"

rclcpp_action::GoalResponse WorkflowPlanner::replenish_goal_cb(
  const rclcpp_action::GoalUUID & uuid, 
  std::shared_ptr<const Replenish::Goal> goal)
{
  (void)uuid;
  std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);

  if (state_ != RobotStatus::IDLE)
  {
    RCLCPP_INFO(get_logger(), "Robot is not idle");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (place_poses_.find(goal->sku_id) == place_poses_.end()) 
  {
    RCLCPP_ERROR(get_logger(), "Pose of sku_id [%d] not found.", goal->sku_id);
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(get_logger(), "Received goal request with order %u", goal->order_id);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse WorkflowPlanner::replenish_cancel_cb(
  const std::shared_ptr<GoalHandlerReplenish> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Received request to cancel goal");
  (void) goal_handle;

  if (false)
  {
    RCLCPP_INFO(get_logger(), "FIXME rclcpp_action::GoalResponse::REJECT");
    return rclcpp_action::CancelResponse::REJECT;
  }
  
  return rclcpp_action::CancelResponse::ACCEPT;
}

void WorkflowPlanner::replenish_accepted(const std::shared_ptr<GoalHandlerReplenish> goal_handle)
{
  std::thread{std::bind(&WorkflowPlanner::replenish_execution, this, _1), goal_handle}.detach();
}

void WorkflowPlanner::replenish_execution(const std::shared_ptr<GoalHandlerReplenish> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Executing replenish goal");

  const auto goal = goal_handle->get_goal();

  auto feedback = std::make_shared<Replenish::Feedback>();
//   auto& running = feedback->running;
//   auto& state = feedback->state;
  
  auto result = std::make_shared<Replenish::Result>();

  push_tf_buf(std::make_tuple(goal->object_pose, "base_footprint", "object_pose"));

  std::optional<PickPlanResult> pick_plan_result = get_pick_plan(goal->object_pose);
  if (!pick_plan_result.has_value())
  {
    goal_handle->abort(result);
    return;
  }

  push_tf_buf(std::make_tuple(pick_plan_result.value().pre_pick_pose, "base_footprint", "pre_pick_pose"));
  auto& pick_poses = pick_plan_result.value().pick_poses;
  std::for_each(pick_poses.begin(), pick_poses.end(), [this, i = 0](auto& pose) mutable {
      push_tf_buf(std::make_tuple(pose, "base_footprint", "pick_poses_" + std::to_string(i++)));
    });

  if (!motion_planner_->pick(pick_plan_result.value(), 90.0))
  {
    goal_handle->abort(result);
    return;
  }

  if (!motion_planner_->move_from_pick_to_place(15.0))
  {
    goal_handle->abort(result);
    return;
  }

  std::optional<PlacePlanResult> place_plan_result = get_place_plan(place_poses_[goal->sku_id]);
  if (!place_plan_result.has_value())
  {
    goal_handle->abort(result);
    return;
  }

  if (!motion_planner_->place(place_plan_result.value(), 15.0))
  {
    goal_handle->abort(result);
    return;
  }

  if (!motion_planner_->move_to_home_pose(90.0))
  {
    goal_handle->abort(result);
    return;
  }
  RCLCPP_WARN(get_logger(), "Moved to home pose");

  if (rclcpp::ok()) 
  {
    clear_tf_buf();
    result->info = {};
    goal_handle->succeed(result);

    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}