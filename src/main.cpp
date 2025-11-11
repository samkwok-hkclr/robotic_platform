
#include "tf_broadcaster.hpp"

#include "manager/manager.hpp"

#include "manipulation/end_effector_ctrl/vacuum_gripper.hpp"
#include "manipulation/collision_planner/collision_planner.hpp"
#include "manipulation/workflow_planner/workflow_planner.hpp"
#include "manipulation/action_planner/action_planner.hpp"
#include "manipulation/motion_planner/motion_planner.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto options = rclcpp::NodeOptions();

  auto tf_broadcaster = std::make_shared<TfBroadcaster>("tf_broadcaster", options);

  auto vac_gripper = std::make_shared<VacuumGripper>(options);
  auto finger_gripper = std::make_shared<FingerGripper>(options);
  auto fold_elev_driver = std::make_shared<FoldElevatorDriver>(options);

  // auto collision_planner = std::make_shared<CollisionPlanner>(options);
  auto action_planner = std::make_shared<ActionPlanner>(options);
  auto motion_planner = std::make_shared<MotionPlanner>(options, vac_gripper, finger_gripper);
  auto workflow_planner = std::make_shared<WorkflowPlanner>(options, fold_elev_driver, motion_planner);

  auto manager = std::make_shared<Manager>("manager", options);
  
  exec->add_node(tf_broadcaster->get_node_base_interface());
  
  exec->add_node(vac_gripper->get_node_base_interface());
  exec->add_node(finger_gripper->get_node_base_interface());
  exec->add_node(fold_elev_driver->get_node_base_interface());

  // exec->add_node(collision_planner->get_node_base_interface());
  exec->add_node(action_planner->get_node_base_interface());
  exec->add_node(motion_planner->get_node_base_interface());
  exec->add_node(workflow_planner->get_node_base_interface());

  exec->add_node(manager->get_node_base_interface());
  
  exec->spin();

  rclcpp::shutdown();
}