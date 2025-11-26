#include "manipulation/workflow_planner/workflow_planner.hpp"

void WorkflowPlanner::simple_pick_cb(
  const std::shared_ptr<SimplePick::Request> request, 
  std::shared_ptr<SimplePick::Response> response)
{
  if (request->enable_left)
  {
    push_tf_buf(std::make_tuple(request->object_pose_left, ARM_REF_FRAME, "object_pose_left"));
  }

  if (request->enable_right)
  {
    push_tf_buf(std::make_tuple(request->object_pose_right, ARM_REF_FRAME, "object_pose_right"));
  }

  fold_elev_driver_->move_to_home_joint();
  fold_elev_driver_->elevate(0.05, -0.05, 0);

  if (request->enable_left)
  {
    Pose obj_pose = request->object_pose_left;
    Pose pre_obj_pose = cvt_g_to_pose(get_g(obj_pose) * get_g(0, 0, -0.04, 0, 0, 0));
    Pose holding_pose = cvt_g_to_pose(get_g(0.0, 0.2, 0.66, M_PI, 0, 0));

    motion_planner_->move_to_action_pose(RobotArm::LEFT, 100);

    motion_planner_->move_to(RobotArm::LEFT_ACTION, pre_obj_pose, 75);
    motion_planner_->gripper_action(RobotArm::LEFT_ACTION, true);
    motion_planner_->move_to(RobotArm::LEFT_ACTION, obj_pose, 50);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    motion_planner_->move_to(RobotArm::LEFT_ACTION, pre_obj_pose, 100);
    motion_planner_->move_to(RobotArm::LEFT_ACTION, holding_pose, 100);
  }

  if (request->enable_right)
  {
    // op: object pose
    // pop: pre object pose
    // lop: lifted object pose
    tf2::Transform g_op__pop = get_g(0, 0, -0.15, 0, 0, 0);
    tf2::Transform g_op__lop = get_g(0.02, 0, 0, 0, 0, 0);
    motion_planner_->move_to_action_pose(RobotArm::RIGHT, 100);
    Pose pre_obj_pose = cvt_g_to_pose(get_g(request->object_pose_right) * g_op__pop);
    Pose lifted_obj_pose = cvt_g_to_pose(get_g(request->object_pose_right) * g_op__lop);
    Pose lifted_back_obj_pose = cvt_g_to_pose(get_g(lifted_obj_pose) * g_op__pop);

    motion_planner_->move_to(RobotArm::RIGHT_ACTION, pre_obj_pose, 100);
    motion_planner_->move_to(RobotArm::RIGHT_ACTION, request->object_pose_right, 50);
    motion_planner_->gripper_action(RobotArm::RIGHT_ACTION, true);
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    motion_planner_->move_to(RobotArm::RIGHT_ACTION, lifted_obj_pose, 75);
    motion_planner_->move_to(RobotArm::RIGHT_ACTION, lifted_back_obj_pose, 100);

    motion_planner_->move_to_holding_pose(RobotArm::RIGHT_ACTION, 100);
  }

  fold_elev_driver_->move_to_home_joint();
  fold_elev_driver_->rotate_to_abs_front();

  clear_tf_buf();
  response->success = true;
}

void WorkflowPlanner::simple_place_cb(
  const std::shared_ptr<SimplePlace::Request> request, 
  std::shared_ptr<SimplePlace::Response> response)
{
  if (request->enable_left)
  {
    push_tf_buf(std::make_tuple(request->place_pose_left, ARM_REF_FRAME, "place_pose_left"));
  }

  if (request->enable_right)
  {
    push_tf_buf(std::make_tuple(request->place_pose_right, ARM_REF_FRAME, "place_pose_right"));
  }

  fold_elev_driver_->move_to_home_joint();
  fold_elev_driver_->elevate(0.05, -0.05, 0);

  if (request->enable_left)
  {
    Pose place_pose = request->place_pose_left;

    Pose pre_place_pose = cvt_g_to_pose(get_g(place_pose) * get_g(0, 0, -0.1, 0, 0, 0));

    motion_planner_->move_to(RobotArm::LEFT_ACTION, pre_place_pose, 75);
    motion_planner_->move_to(RobotArm::LEFT_ACTION, place_pose, 50);
    // motion_planner_->move_to(RobotArm::LEFT_ACTION, place_down_pose, 50);

    motion_planner_->gripper_action(RobotArm::LEFT_ACTION, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    motion_planner_->move_to(RobotArm::LEFT_ACTION, pre_place_pose, 75);
    
    motion_planner_->move_to_home_pose(RobotArm::LEFT_ACTION, 100);
  }

  if (request->enable_right)
  {
    // pp: place pose
    // ppp: pre place pose
    // pdp: place down pose
    // lpp: lifted place pose
    tf2::Transform g_pp__ppp = get_g(0, 0, -0.1, 0, 0, 0);
    tf2::Transform g_pp__pdp = get_g(-0.01, 0, 0, 0, 0, 0);
    Pose pre_place_pose = cvt_g_to_pose(get_g(request->place_pose_right) * g_pp__ppp);
    Pose place_down_pose = cvt_g_to_pose(get_g(request->place_pose_right) * g_pp__pdp);
    Pose lifted_place_pose = cvt_g_to_pose(get_g(request->place_pose_right) * g_pp__ppp * g_pp__pdp);

    motion_planner_->move_to(RobotArm::RIGHT_ACTION, pre_place_pose, 100);
    motion_planner_->move_to(RobotArm::RIGHT_ACTION, request->place_pose_right, 100);
    motion_planner_->move_to(RobotArm::RIGHT_ACTION, place_down_pose, 50);

    motion_planner_->gripper_action(RobotArm::RIGHT_ACTION, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    motion_planner_->move_to(RobotArm::RIGHT_ACTION, lifted_place_pose, 100);

    motion_planner_->move_to_home_pose(RobotArm::RIGHT_ACTION, 100);
  }

  fold_elev_driver_->move_to_home_joint();
  fold_elev_driver_->rotate_to_abs_front();

  clear_tf_buf();
  response->success = true;
}