#include "manipulation/workflow_planner/workflow_planner.hpp"

rclcpp_action::GoalResponse WorkflowPlanner::scan_sku_goal_cb(
  const rclcpp_action::GoalUUID & uuid, 
  std::shared_ptr<const ScanSku::Goal> goal)
{
  (void)uuid;

  if (state_ != RobotStatus::IDLE)
  {
    RCLCPP_INFO(get_logger(), "Robot is not idle");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (scan_poses_.size() == 0)
  {
    RCLCPP_INFO(get_logger(), "Scan poses is empty.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(get_logger(), "Received goal request with order %u", goal->order_id);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse WorkflowPlanner::scan_sku_cancel_cb(
  const std::shared_ptr<GoalHandlerScanSku> goal_handle)
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

void WorkflowPlanner::scan_sku_accepted(const std::shared_ptr<GoalHandlerScanSku> goal_handle)
{
  std::thread{std::bind(&WorkflowPlanner::scan_sku_execution, this, _1), goal_handle}.detach();
}

void WorkflowPlanner::scan_sku_execution(const std::shared_ptr<GoalHandlerScanSku> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Executing ScanSku goal");

  const auto goal = goal_handle->get_goal();

  auto feedback = std::make_shared<ScanSku::Feedback>();
  auto& running = feedback->running;
  auto& state = feedback->state;
  
  auto result = std::make_shared<ScanSku::Result>();

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

  if (!motion_planner_->move_to_scan_pose(25.0))
  {
    goal_handle->abort(result);
    return;
  }

  RCLCPP_WARN(get_logger(), "start to scan <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
  std::optional<std::vector<ObjectPose>> obj_poses = try_to_scan(goal->sku_id, goal->camera_id);
  if (!obj_poses.has_value())
  {
    goal_handle->abort(result);
    RCLCPP_INFO(get_logger(), "have no value");
    return;
  }

  std::vector<ObjectPose>& poses_in_camera = obj_poses.value();
  RCLCPP_INFO(get_logger(), "pose length: %ld", poses_in_camera.size());

  std::optional<Pose> obj_pose = extract_object_pose(poses_in_camera);
  if (!obj_pose.has_value())
  {
    goal_handle->abort(result);
    RCLCPP_INFO(get_logger(), "obj_pose have no value");
    return;
  }

  if (rclcpp::ok()) 
  {
    clear_tf_buf();
    result->success = true;
    pub_fb_timer->cancel();
    goal_handle->succeed(result);

    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}
