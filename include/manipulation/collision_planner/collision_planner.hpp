#ifndef COLLISION_PLANNER_HPP__
#define COLLISION_PLANNER_HPP__

#pragma once

#include <functional>
#include <future>
#include <memory>
#include <chrono>
#include <algorithm>

#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "boost/variant/get.hpp"

#include "geometric_shapes/shapes.h"
#include "geometric_shapes/mesh_operations.h"
#include "geometric_shapes/shape_operations.h"

#include "geometry_msgs/msg/pose.hpp"
#include "moveit_msgs/msg/collision_object.hpp"

#include "robot_controller_msgs/srv/add_collision_objects.hpp"
#include "robot_controller_msgs/srv/apply_attached_collision_objects.hpp"
#include "robot_controller_msgs/srv/get_collision_objects_from_scene.hpp"
#include "robot_controller_msgs/srv/move_collision_objects.hpp"
#include "robot_controller_msgs/srv/remove_collision_objects.hpp"

#include "manipulation/planner_base.hpp"

struct CollisionObject 
{
  int id;
  int type;
  std::string name;
  std::vector<float> dimension = {0.0f, 0.0f, 0.0f};
  geometry_msgs::msg::Pose pose;
  std::string mesh_pkg_name;
  std::string mesh_folder;
  std::string mesh_file;
  bool visible = true;
  bool dynamic;
};

using std::placeholders::_1;
using std::placeholders::_2;

class CollisionPlanner : public PlannerBase
{
  using AddCollisionObjects = robot_controller_msgs::srv::AddCollisionObjects;
  using RemoveCollisionObjects = robot_controller_msgs::srv::RemoveCollisionObjects;
  using ApplyAttachedCollisionObjects = robot_controller_msgs::srv::ApplyAttachedCollisionObjects;
  using MoveCollisionObjects = robot_controller_msgs::srv::MoveCollisionObjects;
  using GetCollisionObjectsFromScene = robot_controller_msgs::srv::GetCollisionObjectsFromScene;

public:
  explicit CollisionPlanner(const rclcpp::NodeOptions& options);
  ~CollisionPlanner();

  bool get_col_obj_form_file(std::string file);

  bool get_col_obj_from_scene(std::vector<std::string>& object_ids);
  bool add_col_obj(const std::vector<std::string>& existed_object_ids);
  bool add_col_obj(const moveit_msgs::msg::CollisionObject::SharedPtr object);
  bool remove_col_obj(const std::vector<std::string>& existed_object_ids);

  void init_cb(void);
  void status_cb(void);
  void col_obj_sync_cb(void);

  bool cvt_moveit_col_obj(
    moveit_msgs::msg::CollisionObject::SharedPtr msg,
    const CollisionObject object);

  bool compare_id(const CollisionObject &a, const CollisionObject &b);
  void sort_collision_objects(void);

private:
  std::mutex mutex_;

  bool sim_;

  std::vector<CollisionObject> collision_objects_;

  rclcpp::CallbackGroup::SharedPtr cli_cbg_;
  rclcpp::CallbackGroup::SharedPtr timer_cbg_;

  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr col_obj_sync_timer_;

  rclcpp::Client<AddCollisionObjects>::SharedPtr add_col_obj_cli_;
  rclcpp::Client<RemoveCollisionObjects>::SharedPtr remove_col_obj_cli_;
  rclcpp::Client<ApplyAttachedCollisionObjects>::SharedPtr apply_attach_col_obj_cli_;
  rclcpp::Client<MoveCollisionObjects>::SharedPtr move_col_obj_cli_;
  rclcpp::Client<GetCollisionObjectsFromScene>::SharedPtr get_col_obj_from_secne_cli_;

};


#endif // COLLISION_PLANNER_HPP__