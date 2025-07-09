#include "manipulation/collision_planner/collision_planner.hpp"
#include "manipulation/workflow_planner/workflow_planner.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto options = rclcpp::NodeOptions();

  auto collision_planner_node = std::make_shared<CollisionPlanner>(options);
  auto workflow_planner_node = std::make_shared<WorkflowPlanner>(options);

  exec->add_node(collision_planner_node->get_node_base_interface());
  exec->add_node(workflow_planner_node->get_node_base_interface());
  
  exec->spin();

  rclcpp::shutdown();
}