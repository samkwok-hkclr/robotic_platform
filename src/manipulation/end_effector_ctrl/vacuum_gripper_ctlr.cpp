#include "manipulation/end_effector_ctrl/vacuum_gripper_ctlr.hpp"

VacuumGripperCtlr::VacuumGripperCtlr(
  const rclcpp::NodeOptions& options)
: NodeBase("vacuum_gripper_ctlr", options)
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
    std::bind(&VacuumGripperCtlr::leak_validation_cb, this));

  leak_pub_ = create_publisher<Empty>("leak_warning", 10);

  valve_cli_ = create_client<SetBool>(
    "valve_control", 
    rmw_qos_profile_services_default,
    srv_cli_cbg_);

  state_sub_ = create_subscription<Bool>(
    "gripper_status", 
    10, 
    std::bind(&VacuumGripperCtlr::state_cb, this, _1));

  range_sub_ = create_subscription<Range>(
    "ultrasonic_range", 
    10, 
    std::bind(&VacuumGripperCtlr::range_cb, this, _1));

  temp_sub_ = create_subscription<Temperature>(
    "ultrasonic_sensor_internal_temperature", 
    10, 
    std::bind(&VacuumGripperCtlr::temp_cb, this, _1));

  pressure_sub_ = create_subscription<FluidPressure>(
    "vaccum_pressure", 
    10, 
    std::bind(&VacuumGripperCtlr::pressure_cb, this, _1));

  RCLCPP_INFO(get_logger(), "Vacuum Gripper Controller is up.");
}

void VacuumGripperCtlr::leak_validation_cb(void)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!status_.get_state())
    return;

  const float curr_pressure = status_.get_pressure();

  if (curr_pressure > leak_threshold_) 
  {
    Empty msg;
    leak_pub_->publish(std::move(msg));
    RCLCPP_DEBUG(get_logger(), "Vacuum leak detected! Pressure: %.1f kpa", curr_pressure / 1000.0);
  }
}

bool VacuumGripperCtlr::gripper_action(const bool cmd)
{
  auto request = std::make_shared<SetBool::Request>();
  request->data = cmd;

  SetBool::Response::SharedPtr response;
  if (!(send_sync_req<SetBool>(valve_cli_, std::move(request), response, __FUNCTION__) && response))
  {
    RCLCPP_ERROR(get_logger(), "Sent SetBool request failed");
    return false;
  }

  if (!response->success)
    return false;

  return true;
}

void VacuumGripperCtlr::state_cb(const Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  status_.set_state(msg->data);

  RCLCPP_DEBUG(get_logger(), "Gripper state: %s", msg->data ? "on" : "off");
}

void VacuumGripperCtlr::range_cb(const Range::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  status_.set_distance(msg->range);

  RCLCPP_DEBUG(get_logger(), "Gripper ultra distance: %.4f", msg->range);
}

void VacuumGripperCtlr::temp_cb(const Temperature::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  status_.set_temperature(msg->temperature);

  RCLCPP_DEBUG(get_logger(), "Gripper ultra distance: %.1f", msg->temperature);
}

void VacuumGripperCtlr::pressure_cb(const FluidPressure::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  status_.set_pressure(msg->fluid_pressure);

  RCLCPP_DEBUG(get_logger(), "Gripper ultra distance: %.f", msg->fluid_pressure);
}