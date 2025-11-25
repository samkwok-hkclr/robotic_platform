#include "manipulation/end_effector_ctrl/vacuum_gripper.hpp"

VacuumGripper::VacuumGripper(
  const rclcpp::NodeOptions& options)
: NodeBase("vacuum_gripper", options)
{
  declare_parameter<bool>("enable_ultrasonic", false);
  declare_parameter<double>("leak_threshold", 0.0);
  get_parameter("enable_ultrasonic", enable_ultrasonic_);
  get_parameter("leak_threshold", leak_threshold_);

  if (std::abs(leak_threshold_) <= 0.01)
  {
    RCLCPP_INFO(get_logger(), "Leak threshold does not set.");
    rclcpp::shutdown();
    return;
  }

  srv_cli_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  leak_valid_timer = create_wall_timer(
    std::chrono::milliseconds(100), 
    std::bind(&VacuumGripper::leak_validation_cb, this));

  leak_pub_ = create_publisher<Empty>("leak_warning", 10);

  pump_ctrl_cli_ = create_client<SetBool>(
    "pump_control", 
    rmw_qos_profile_services_default,
    srv_cli_cbg_);

  if (enable_ultrasonic_)
  {
    state_sub_ = create_subscription<Bool>(
      "vacuum_gripper_status", 
      10, 
      std::bind(&VacuumGripper::state_cb, this, _1));

    range_sub_ = create_subscription<Range>(
      "ultrasonic_range", 
      10, 
      std::bind(&VacuumGripper::range_cb, this, _1));

    temp_sub_ = create_subscription<Temperature>(
      "ultrasonic_sensor_internal_temperature", 
      10, 
      std::bind(&VacuumGripper::temp_cb, this, _1));
  }

  pressure_sub_ = create_subscription<FluidPressure>(
    "vaccum_pressure", 
    10, 
    std::bind(&VacuumGripper::pressure_cb, this, _1));

  RCLCPP_INFO(get_logger(), "Vacuum Gripper Controller is up.");
}

void VacuumGripper::leak_validation_cb(void)
{
  if (!state_.load())
    return;

  const float curr_pressure = pressure_.load();

  if (curr_pressure > (leak_threshold_ / 1000.0)) 
  {
    RCLCPP_WARN(get_logger(), "Vacuum leak detected! Pressure: %.4f Pa [leak threshold: %.4f Pa]", curr_pressure, leak_threshold_ / 1000.0);

    if (!leak_pub_ || leak_pub_->get_subscription_count() == 0)
      return;

    Empty msg;
    leak_pub_->publish(msg);
  }
}

bool VacuumGripper::gripper_action(const bool cmd)
{
  auto request = std::make_shared<SetBool::Request>();
  request->data = cmd;

  SetBool::Response::SharedPtr response;
  if (!send_sync_req<SetBool>(pump_ctrl_cli_, std::move(request), response, __FUNCTION__))
  {
    RCLCPP_ERROR(get_logger(), "Sent SetBool request failed");
    return false;
  }

  if (!response->success)
    return false;

  return true;
}

void VacuumGripper::state_cb(const Bool::SharedPtr msg)
{
  state_.store(msg->data);
  RCLCPP_DEBUG(get_logger(), "Gripper state: %s", msg->data ? "ON" : "OFF");
}

void VacuumGripper::range_cb(const Range::SharedPtr msg)
{
  distance_.store(msg->range);
  RCLCPP_DEBUG(get_logger(), "Gripper ultra distance: %.4f", msg->range);
}

void VacuumGripper::temp_cb(const Temperature::SharedPtr msg)
{
  temperature_.store(msg->temperature);
  RCLCPP_DEBUG(get_logger(), "Gripper ultra distance: %.1f", msg->temperature);
}

void VacuumGripper::pressure_cb(const FluidPressure::SharedPtr msg)
{
  pressure_.store(msg->fluid_pressure);
  RCLCPP_DEBUG(get_logger(), "Vacuum gripper pressure: %.f", msg->fluid_pressure);
}