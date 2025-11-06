#include "manipulation/end_effector_ctrl/vacuum_gripper.hpp"

VacuumGripper::VacuumGripper(
  const rclcpp::NodeOptions& options)
: NodeBase("vacuum_gripper", options)
{
  declare_parameter<double>("leak_threshold", 0.0);
  get_parameter("leak_threshold", leak_threshold_);

  if (std::abs(leak_threshold_) <= 0.001)
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

  pressure_sub_ = create_subscription<FluidPressure>(
    "vaccum_pressure", 
    10, 
    std::bind(&VacuumGripper::pressure_cb, this, _1));

  RCLCPP_INFO(get_logger(), "Vacuum Gripper Controller is up.");
}

void VacuumGripper::leak_validation_cb(void)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!status_.get_state())
    return;

  const float curr_pressure = status_.get_pressure();

  if (curr_pressure > (leak_threshold_ / 1000.0)) 
  {
    Empty msg;
    leak_pub_->publish(msg);
    RCLCPP_WARN(get_logger(), "Vacuum leak detected! Pressure: %.4f Pa [leak threshold: %.4f Pa]", curr_pressure, leak_threshold_);
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
  std::lock_guard<std::mutex> lock(mutex_);

  status_.set_state(msg->data);
  RCLCPP_DEBUG(get_logger(), "Gripper state: %s", msg->data ? "ON" : "OFF");
}

void VacuumGripper::range_cb(const Range::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  status_.set_distance(msg->range);
  RCLCPP_DEBUG(get_logger(), "Gripper ultra distance: %.4f", msg->range);
}

void VacuumGripper::temp_cb(const Temperature::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  status_.set_temperature(msg->temperature);
  RCLCPP_DEBUG(get_logger(), "Gripper ultra distance: %.1f", msg->temperature);
}

void VacuumGripper::pressure_cb(const FluidPressure::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  status_.set_pressure(msg->fluid_pressure);
  RCLCPP_DEBUG(get_logger(), "Vacuum gripper pressure: %.f", msg->fluid_pressure);
}