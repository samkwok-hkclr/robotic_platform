#include "manipulation/end_effector_ctrl/finger_gripper.hpp"

FingerGripper::FingerGripper(
  const rclcpp::NodeOptions& options)
: NodeBase("finger_gripper", options)
{
  srv_cli_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  obj_dropped_timer = create_wall_timer(
    std::chrono::milliseconds(100), 
    std::bind(&FingerGripper::obj_state_cb, this));

  obj_dropped_pub_ = create_publisher<Empty>("object_dropped_warning", 10);

  gripper_status_sub_ = create_subscription<GripperStatus>(
    "gripper_status", 
    10, 
    std::bind(&FingerGripper::gripper_status_cb, this, _1));

  gripper_ctrl_cli_ = create_client<ControlGripper>(
    "gripper_control", 
    rmw_qos_profile_services_default,
    srv_cli_cbg_);

  RCLCPP_INFO(get_logger(), "Finger Gripper Controller is up.");
}

void FingerGripper::obj_state_cb(void)
{
  if (obj_state_.load() == OBJECT_STATE::DROPPED) 
  {
    RCLCPP_WARN(get_logger(), "Finger gripper object drop detected!");

    if (!obj_dropped_pub_ || obj_dropped_pub_->get_subscription_count() == 0)
      return;

    Empty msg;
    obj_dropped_pub_->publish(msg);
  }
}

bool FingerGripper::gripper_action(const uint8_t force, const float position)
{
  if (!is_initialized_.load())
  {
    RCLCPP_ERROR(get_logger(), "FingerGripper not initialized.");
    return false;
  }

  auto request = std::make_shared<ControlGripper::Request>();
  request->force = force;
  request->position = position;

  ControlGripper::Response::SharedPtr response;
  if (!send_sync_req<ControlGripper>(gripper_ctrl_cli_, std::move(request), response, __FUNCTION__))
  {
    RCLCPP_ERROR(get_logger(), "Sent ControlGripper request failed");
    return false;
  }

  if (!response->success)
    return false;

  RCLCPP_INFO(get_logger(), "Gripper action successful (force: %u, position: %.3f)", force, position);
  return true;
}

void FingerGripper::gripper_status_cb(const GripperStatus::SharedPtr msg)
{
  is_initialized_.store(msg->is_initialized);

  if (msg->state == 2 || msg->state == 3)
  {
    obj_state_.store(static_cast<OBJECT_STATE>(msg->state));
  }
  else
  {
    obj_state_.store(OBJECT_STATE::NONE);
  }

  OBJECT_STATE curr_state = obj_state_.load();
  const char* state_str = "";
  
  switch (curr_state) {
    case OBJECT_STATE::NONE:
      state_str = "NONE";
      break;
    case static_cast<OBJECT_STATE>(2):
      state_str = "GRIPPED";  // Assuming state 2 means gripped
      break;
    case static_cast<OBJECT_STATE>(3):
      state_str = "DROPPED";  // Assuming state 3 means dropped
      break;
    default:
      state_str = "UNKNOWN";
      break;
  }
  
  RCLCPP_DEBUG(get_logger(), "Finger Gripper state: %s (raw: %d)", state_str, msg->state);
}
