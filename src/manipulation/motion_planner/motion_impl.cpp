#include "manipulation/motion_planner/motion_planner.hpp"

bool MotionPlanner::try_to_pick(
  const Pose& pre_pick_pose, 
  const std::vector<Pose>& pick_poses, 
  const std::chrono::milliseconds leak_check_duration)
{
  (void) pre_pick_pose;
  std::atomic<bool> start_leak_valid{false};
  std::atomic<bool> leak_detected{false};

  auto leak_cb = [this, &start_leak_valid, &leak_detected](const Empty::SharedPtr msg) {
    (void) msg;
    if (!start_leak_valid)
      return;

    RCLCPP_WARN(get_logger(), "Leak detected during pick operation!");
    leak_detected.store(true);
  };

  auto leak_sub = create_subscription<Empty>("leak_warning", 10, leak_cb);

  for (uint8_t attempt = 1; attempt <= max_pick_attempt_; ++attempt)
  {
    RCLCPP_INFO(get_logger(), "Pick attempt %d/%d", attempt, max_pick_attempt_);

    if (!gripper_action(true))
      return false;
    
    if (!move_to(pick_poses, 3.0)) 
    {
      RCLCPP_ERROR(get_logger(), "Movement failed on attempt %d", attempt);
      return false;
    }

    RCLCPP_DEBUG(get_logger(), "Checking for leaks...");
    auto start = std::chrono::steady_clock::now();
    start_leak_valid.store(true);

    while (!leak_detected && (std::chrono::steady_clock::now() - start) < leak_check_duration) 
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (leak_detected) 
    {
      if (!gripper_action(false))
        return false;

      RCLCPP_ERROR(get_logger(), "Leak detected, retrying...");
      start_leak_valid.store(false);
      leak_detected.store(false); // Reset for next attempt

      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      continue;
    }

    RCLCPP_INFO(get_logger(), "Pick attempt %d successful", attempt);
    return true;
  }

  return false;
}

bool MotionPlanner::try_to_place(
  const Pose& pre_place_pose, 
  const std::vector<Pose>& place_poses,
  const uint8_t max_retries)
{
  (void) pre_place_pose;
  (void) max_retries;

  std::atomic<bool> start_to_valid{false};
  std::atomic<bool> is_first_message{true};
  std::atomic<bool> mono_increasing{true};
  std::atomic<float> last_valid_range{0.0};
  std::atomic<uint32_t> consecutive_failures{0};
  const uint32_t max_consecutive_failures = 3;
  const float tolerance = 0.01;

  // how to pass start_to_valid, is_first_message, ... to range_cb?
  auto range_cb = [&](const Range::SharedPtr msg) {
    if (!start_to_valid.load()) 
      return;

    if (!mono_increasing.load())
      return;

    float curr_range = msg->range;
    
    if (is_first_message.load()) 
    {
      // Initialize with first value
      last_valid_range.store(curr_range);
      is_first_message.store(false);
      consecutive_failures.store(0);
      return;
    } 
  
    float last_range = last_valid_range.load();
    if (curr_range >= (last_range - tolerance)) 
    {
      if (curr_range > last_range) 
      {
        consecutive_failures.store(0);
        last_valid_range.store(curr_range);
      } 
      else 
      {
        consecutive_failures.fetch_add(1);
      }
    } 
    else 
    {
      uint32_t failures = consecutive_failures.fetch_add(1) + 1;
      if (failures > max_consecutive_failures) 
      {
        mono_increasing.store(false);
        RCLCPP_WARN(get_logger(), "Range validation failed! Current: %.4f, Last valid: %.4f, Failures: %d", 
          curr_range, last_range, failures);
      } 
      else 
      {
        RCLCPP_WARN(this->get_logger(), "Temporary range decrease (failure %d/%d): %.4f < %.4f (with tolerance)", 
          failures, max_consecutive_failures, curr_range, last_valid_range.load());
      }
    }
  };

  auto range_sub = create_subscription<Range>("ultrasonic_range", 10, range_cb);
  
  if (!move_to(place_poses, 10.0))
    return false;

  if (!gripper_action(false))
    return false;

  start_to_valid.store(true);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  if (mono_increasing.load()) 
  {
    RCLCPP_WARN(get_logger(), "Object is moving far away from expected range.");
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "Range validation failed!");
  }

  RCLCPP_WARN(get_logger(), "Moved the place poses");
  return true;
}

bool MotionPlanner::move_to(const Pose& waypoint, const float speed)
{
  std::vector<Pose> poses{ waypoint };

  return move_to(poses, speed);
}

bool MotionPlanner::move_to(const std::vector<Pose>& waypoints, const float speed)
{
  auto request = std::make_shared<ExecuteWaypoints::Request>();
  request->waypoints.insert(request->waypoints.end(), waypoints.begin(), waypoints.end());
  request->speed = speed;

  ExecuteWaypoints::Response::SharedPtr response;
  if (!(send_sync_req<ExecuteWaypoints>(exec_wps_cli_, std::move(request), response) && response))
  {
    RCLCPP_ERROR(get_logger(), "Sent ExecuteWaypoints request failed");
    return false;
  }

  if (!response->success)
  {
    RCLCPP_ERROR(get_logger(), "Unsuccessful: %s", response->message.c_str());
    return false;
  }

  return true;
}