#include "manipulation/workflow_planner/workflow_planner.hpp"

rclcpp_action::GoalResponse WorkflowPlanner::pick_up_goal_cb(
  const rclcpp_action::GoalUUID & uuid, 
  std::shared_ptr<const PickUp::Goal> goal)
{
  (void)uuid;

  if (state_ != RobotStatus::IDLE)
  {
    RCLCPP_INFO(get_logger(), "Robot is not idle");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(get_logger(), "Received goal request with order: %u [sku_id: %d]", goal->order_id, goal->sku_id);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse WorkflowPlanner::pick_up_cancel_cb(
  const std::shared_ptr<GoalHandlerPickUp> goal_handle)
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

void WorkflowPlanner::pick_up_accepted(const std::shared_ptr<GoalHandlerPickUp> goal_handle)
{
  std::thread{std::bind(&WorkflowPlanner::pick_up_execution, this, _1), goal_handle}.detach();
}

void WorkflowPlanner::pick_up_execution(const std::shared_ptr<GoalHandlerPickUp> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Executing PickUp goal");

  const auto goal = goal_handle->get_goal();

  auto feedback = std::make_shared<PickUp::Feedback>();
  auto& running = feedback->running;
  auto& state = feedback->state;
  
  auto result = std::make_shared<PickUp::Result>();

  auto pub_fb = [this, &goal_handle, &feedback, &running, &state]() {
    if (rclcpp::ok())
    {
      state = 1;
      running = true;
    }
    RCLCPP_INFO(get_logger(), "Publish feedback!!!");
    goal_handle->publish_feedback(feedback);
  };
  
  rclcpp::TimerBase::SharedPtr pub_fb_timer = create_wall_timer(std::chrono::seconds(1), pub_fb, exec_timer_cbg_);
  clear_tf_buf();

  RCLCPP_WARN(get_logger(), "start the pick up action <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");

  if (!try_to_pick_up(goal->sku_id, goal->camera_id))
  {
    goal_handle->abort(result);
    return;
  }
  RCLCPP_WARN(get_logger(), "picked up a sku");

  const std::string flat_frame = get_flat_link(goal->rack.id, goal->rack.shelf_level);

  std::optional<TransformStamped> g_obj_flat = get_tf(flat_frame, object_pose_);
  if (!g_obj_flat.has_value())
  {
    goal_handle->abort(result);
    RCLCPP_INFO(get_logger(), "have no value");
    return;
  }

  const float height = std::abs(g_obj_flat.value().transform.translation.z);
  RCLCPP_WARN(get_logger(), "height: %.6f", height);

  if (!motion_planner_->move_from_pick_to_place(20.0))
  {
    goal_handle->abort(result);
    return;
  }

  const std::string place_frame = get_place_link(goal->table.id, goal->table.order);
  RCLCPP_WARN(get_logger(), "place frame: %s", place_frame.c_str());
  std::optional<TransformStamped> tf_stamped = get_tf("base_footprint", place_frame);
  if (!tf_stamped.has_value())
  {
    goal_handle->abort(result);
    RCLCPP_INFO(get_logger(), "have no value");
    return;
  }
  
  tf2::Transform g_base__place_flat;
  tf2::fromMsg(tf_stamped.value().transform, g_base__place_flat);
  tf2::Transform g_place_flat__place = get_g(0, 0, height + place_offset_, 0, M_PI/2.0f, M_PI/2.0f);
  tf2::Transform g_base_place = g_base__place_flat * g_place_flat__place;

  const Pose place_pose = cvt_g_to_pose(g_base_place);
  push_tf_buf(std::make_tuple(place_pose, "base_footprint", "place_pose"));

  std::optional<PlacePlanResult> place_plan_result = get_place_plan(place_pose);
  if (!place_plan_result.has_value())
  {
    goal_handle->abort(result);
    return;
  }

  if (!motion_planner_->place(place_plan_result.value(), 10.0))
  {
    goal_handle->abort(result);
    return;
  }
  RCLCPP_WARN(get_logger(), "Placed a sku");

  if (!motion_planner_->move_to_home_pose(25.0))
  {
    goal_handle->abort(result);
    return;
  }
  RCLCPP_WARN(get_logger(), "Moved to home pose");

  if (rclcpp::ok()) 
  {
    clear_tf_buf();
    pub_fb_timer->cancel();
    // result->XXX = {};
    goal_handle->succeed(result);

    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}