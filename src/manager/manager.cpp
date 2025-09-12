#include "manager/manager.hpp"

Manager::Manager(
  std::string node_name, 
  const rclcpp::NodeOptions& options)
: NodeBase(node_name, options)
{
  RCLCPP_INFO(get_logger(), "Manager is up.");
}