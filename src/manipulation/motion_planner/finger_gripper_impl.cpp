#include "manipulation/motion_planner/motion_planner.hpp"

bool MotionPlanner::try_to_pick_by_finger(
  RobotArm arm,
  const Pose& pre_pick_pose, 
  const std::vector<Pose>& pick_poses, 
  const std::chrono::milliseconds holding_check_duration)
{
  (void) pre_pick_pose;
  std::atomic<bool> start_drop_valid{false};
  std::atomic<bool> obj_drop_detected{false};

  auto drop_cb = [this, &start_drop_valid, &obj_drop_detected](const Empty::SharedPtr msg) {
    (void) msg;

    if (simulation_)
      return;

    if (!start_drop_valid.load())
      return;

    RCLCPP_WARN(get_logger(), "Obejct dropped detected during pick operation!");
    obj_drop_detected.store(true);
  };

  auto obj_drop_sub = create_subscription<Empty>("object_dropped_warning", 10, drop_cb);

  for (uint8_t attempt = 1; attempt <= max_pick_attempt_; ++attempt)
  {
    if (!gripper_action(arm, false))
      return false;

    RCLCPP_INFO(get_logger(), "Pick attempt %d/%d", attempt, max_pick_attempt_);

    if (!move_to(arm, pick_poses, 50.0)) 
    {
      RCLCPP_ERROR(get_logger(), "Movement failed on attempt %d", attempt);
      return false;
    }

    if (!gripper_action(arm, true))
      return false;

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    RCLCPP_DEBUG(get_logger(), "Checking for object dropped...");
    const auto start = std::chrono::steady_clock::now();
    start_drop_valid.store(true);

    while (!obj_drop_detected.load() && (std::chrono::steady_clock::now() - start) < holding_check_duration) 
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (obj_drop_detected.load()) 
    {
      if (!gripper_action(arm, false))
        return false;

      RCLCPP_ERROR(get_logger(), "Object dropped, retrying...");
      start_drop_valid.store(false);
      obj_drop_detected.store(false); // Reset for next attempt

      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      continue;
    }

    RCLCPP_INFO(get_logger(), "Pick attempt %d successful", attempt);
    return true;
  }

  return false;
}

bool MotionPlanner::try_to_place_by_finger( 
  RobotArm arm,
  const Pose& pre_place_pose, 
  const std::vector<Pose>& place_poses,
  const uint8_t max_retries)
{
  (void) max_retries;

  if (!move_to(arm, pre_place_pose, 75.0))
    return false;
  
  if (!move_to(arm, place_poses, 50.0))
    return false;

  if (!gripper_action(arm, false))
    return false;

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  RCLCPP_WARN(get_logger(), "Moved the place poses");
  return true;
}
