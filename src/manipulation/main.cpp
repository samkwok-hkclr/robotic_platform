
#include "manipulation/end_effector_ctrl/vacuum_gripper_ctlr.hpp"

#include "manipulation/collision_planner/collision_planner.hpp"
#include "manipulation/workflow_planner/workflow_planner.hpp"
#include "manipulation/action_planner/action_planner.hpp"
#include "manipulation/motion_planner/motion_planner.hpp"


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto options = rclcpp::NodeOptions();

  auto gripper = std::make_shared<VacuumGripperCtlr>(options);
  auto collision_planner = std::make_shared<CollisionPlanner>(options);
  auto action_planner = std::make_shared<ActionPlanner>(options);
  auto motion_planner = std::make_shared<MotionPlanner>(options, gripper);
  // FIXME: decouple motion_planner and workflow_planner later
  auto workflow_planner = std::make_shared<WorkflowPlanner>(options, motion_planner);
  
  exec->add_node(gripper->get_node_base_interface());
  exec->add_node(collision_planner->get_node_base_interface());
  exec->add_node(action_planner->get_node_base_interface());
  exec->add_node(motion_planner->get_node_base_interface());
  exec->add_node(workflow_planner->get_node_base_interface());
  
  exec->spin();

  rclcpp::shutdown();
}