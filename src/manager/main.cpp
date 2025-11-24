#include "manager/manager.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto options = rclcpp::NodeOptions();
  auto manager = std::make_shared<Manager>(options);

  exec->add_node(manager->get_node_base_interface());
  exec->spin();

  rclcpp::shutdown();
}