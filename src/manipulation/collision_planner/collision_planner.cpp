#include "manipulation/collision_planner/collision_planner.hpp"

CollisionPlanner::CollisionPlanner(
  const rclcpp::NodeOptions& options)
: Node("collision_planner", options)
{
  declare_parameter<bool>("sim", true);
  declare_parameter<std::string>("collision_objects_file", "");

  std::string file;
  get_parameter("sim", sim_);
  get_parameter("collision_objects_file", file);

  if (!get_col_obj_form_file(file))
  {
    rclcpp::shutdown();
    return;
  }

  cli_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  init_timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&CollisionPlanner::init_cb, this), timer_cbg_);
  status_timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&CollisionPlanner::status_cb, this), timer_cbg_);
  col_obj_sync_timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&CollisionPlanner::col_obj_sync_cb, this), timer_cbg_);

  add_col_obj_cli_ = create_client<AddCollisionObjects>(
    "add_collision_objects",
    rmw_qos_profile_services_default,
    cli_cbg_);

  remove_col_obj_cli_ = create_client<RemoveCollisionObjects>(
    "remove_collision_objects",
    rmw_qos_profile_services_default,
    cli_cbg_);

  apply_attach_col_obj_cli_ = create_client<ApplyAttachedCollisionObjects>(
    "apply_attached_collision_objects",
    rmw_qos_profile_services_default,
    cli_cbg_);

  move_col_obj_cli_ = create_client<MoveCollisionObjects>(
    "move_collision_objects",
    rmw_qos_profile_services_default,
    cli_cbg_);

  get_col_obj_from_secne_cli_ = create_client<GetCollisionObjectsFromScene>(
    "get_collision_objects_from_sence",
    rmw_qos_profile_services_default,
    cli_cbg_);

  if (sim_)
    RCLCPP_ERROR(get_logger(), "Started Simluation Mode");

  RCLCPP_INFO(get_logger(), "Collision Planner is up.");
}

CollisionPlanner::~CollisionPlanner()
{

}

bool CollisionPlanner::get_col_obj_form_file(std::string file)
{
  if (file.empty())
  {
    RCLCPP_ERROR(get_logger(), "Collision object file does not exist");
    return false;
  }
  
  RCLCPP_WARN(get_logger(), "Path: %s", file.c_str());

  std::lock_guard<std::mutex> lock(mutex_);
  try 
  {
    YAML::Node objects = YAML::LoadFile(file);
    if (!objects["collision_objects"])
    {
      RCLCPP_ERROR(get_logger(), "YAML file missing 'collision_objects' key");
      return false;
    }

    for (const auto& object : objects["collision_objects"]) 
    {
      CollisionObject obj;

      obj.id = object["id"].as<int>();
      obj.type = object["type"].as<int>();
      obj.name = object["name"].as<std::string>();
      obj.dimension = object["dimension"].as<std::vector<float>>();

      std::vector<float> tmp = object["pose"].as<std::vector<float>>();
      if (tmp.size() != 7) 
      {
        RCLCPP_ERROR(get_logger(), "Invalid pose format for object %s", obj.name.c_str());
        return false;
      }

      geometry_msgs::msg::Pose pose;
      pose.position.x = tmp[0];
      pose.position.y = tmp[1];
      pose.position.z = tmp[2];
      pose.orientation.x = tmp[3];
      pose.orientation.y = tmp[4];
      pose.orientation.z = tmp[5];
      pose.orientation.w = tmp[6];
      obj.pose = pose;

      obj.mesh_pkg_name = object["mesh_pkg_name"].as<std::string>();
      obj.mesh_folder = object["mesh_folder"].as<std::string>();
      obj.mesh_file = object["mesh_file"].as<std::string>();
      obj.visible = object["visible"].as<bool>();
      obj.dynamic = object["dynamic"].as<bool>();

      collision_objects_.push_back(obj);
    }
  } 
  catch (const std::exception& e) 
  {
    RCLCPP_ERROR(get_logger(), "Error reading collision objects: %s", e.what());
    return false;
  }

  sort_collision_objects();

  for (const auto& object : collision_objects_)
  {
    RCLCPP_INFO(get_logger(), "Loaded collision object: %s", object.name.c_str());
  }

  return !collision_objects_.empty();
}

void CollisionPlanner::init_cb(void)
{
  if (!cli_wait_for_srv<AddCollisionObjects>(add_col_obj_cli_, ""))
    return;
  if (!cli_wait_for_srv<RemoveCollisionObjects>(remove_col_obj_cli_, ""))
    return;
  if (!cli_wait_for_srv<ApplyAttachedCollisionObjects>(apply_attach_col_obj_cli_, ""))
    return;
  if (!cli_wait_for_srv<MoveCollisionObjects>(move_col_obj_cli_, ""))
    return;
  if (!cli_wait_for_srv<GetCollisionObjectsFromScene>(get_col_obj_from_secne_cli_, ""))
    return;
  
  RCLCPP_INFO(get_logger(), "All collision services are ready");
  init_timer_->cancel();
}

void CollisionPlanner::col_obj_sync_cb(void)
{
  std::vector<std::string> existing_object_ids{};

  if (!get_col_obj_from_scene(existing_object_ids))
  {
    RCLCPP_INFO(get_logger(), "Failed to get_col_obj_from_scene");
    return;
  }

  add_col_obj(existing_object_ids);
  // remove_col_obj
}

void CollisionPlanner::status_cb(void)
{
  RCLCPP_DEBUG(get_logger(), "Collision Planner is alive.");
}

bool CollisionPlanner::get_col_obj_from_scene(std::vector<std::string>& object_ids)
{
  auto request = std::make_shared<GetCollisionObjectsFromScene::Request>();

  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& object : collision_objects_)
      request->object_ids.push_back(std::to_string(object.id));
  }
  
  GetCollisionObjectsFromScene::Response::SharedPtr response;
  if (!(send_req<GetCollisionObjectsFromScene>(get_col_obj_from_secne_cli_, request, response, __FUNCTION__) && response))
  {
    RCLCPP_ERROR(get_logger(), "Sent GetCollisionObjectsFromScene request failed");
    return false;
  }
  
  if (response->id_map.size() == 0)
    RCLCPP_INFO(get_logger(), "id_map size: %ld", response->id_map.size());
  else
  {
    for (const auto& instance : response->id_map)
      object_ids.push_back(instance.object_id);
  }

  return true;
}

bool CollisionPlanner::add_col_obj(const std::vector<std::string>& existing_object_ids)
{
  uint16_t no_of_added = 0;
  uint16_t no_of_skipped = 0;

  auto request = std::make_shared<AddCollisionObjects::Request>();

  std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);
  lock.lock();

  for (auto& object : collision_objects_)
  {
    if (std::find(existing_object_ids.begin(), existing_object_ids.end(), std::to_string(object.id)) != existing_object_ids.end())
    {
      no_of_skipped++;
      continue;
    }

    auto msg = std::make_shared<moveit_msgs::msg::CollisionObject>();
    if (cvt_moveit_col_obj(msg, object))
      request->collision_objects.emplace_back(std::move(*msg));

    no_of_added++;
  }

  lock.unlock();

  if (no_of_added == 0) 
  {
    RCLCPP_DEBUG(get_logger(), "No new collision objects to add (skipped %d existing)", no_of_skipped);
    return true;
  }

  AddCollisionObjects::Response::SharedPtr response;
  if (!(send_req<AddCollisionObjects>(add_col_obj_cli_, request, response, __FUNCTION__) && response))
  {
    RCLCPP_ERROR(get_logger(), "Sent AddCollisionObjects request failed");
    return false;
  }

  RCLCPP_INFO(get_logger(), "Added %d collision objects", no_of_added);
  return true;
}

bool CollisionPlanner::add_col_obj(const moveit_msgs::msg::CollisionObject::SharedPtr object)
{
  std::vector<std::string> existing_object_ids{};

  if (!get_col_obj_from_scene(existing_object_ids))
  {
    RCLCPP_INFO(get_logger(), "Failed to get_col_obj_from_scene");
    return false;
  }

  if (std::find(existing_object_ids.begin(), existing_object_ids.end(), object->id) != existing_object_ids.end())
  {
    RCLCPP_INFO(get_logger(), "The object id is existed in scene. id: %s",  object->id.c_str());
    return false;
  }

  auto request = std::make_shared<AddCollisionObjects::Request>();
  request->collision_objects.push_back(std::move(*object));

  AddCollisionObjects::Response::SharedPtr response;
  if (!(send_req<AddCollisionObjects>(add_col_obj_cli_, request, response, __FUNCTION__) && response))
  {
    RCLCPP_ERROR(get_logger(), "Sent AddCollisionObjects request failed");
    return false;
  }

  RCLCPP_INFO(get_logger(), "Added %d collision object", 1);
  return true;
}

bool CollisionPlanner::remove_col_obj(const std::vector<std::string>& existing_object_ids)
{
  (void) existing_object_ids;

  // TBD

  return true;
}

bool CollisionPlanner::cvt_moveit_col_obj(
  const moveit_msgs::msg::CollisionObject::SharedPtr msg,
  const CollisionObject object)
{
  msg->id = std::to_string(object.id);

  if (!object.visible)
  {
    RCLCPP_WARN(get_logger(), "Created a invisible object: %d", object.id);
  }

  if(object.mesh_file.empty()) 
  {
    shape_msgs::msg::SolidPrimitive primitive;

    primitive.type = object.type;
    if (object.dimension.size() < 3) 
    {
      RCLCPP_ERROR(get_logger(), "Invalid dimensions for object %d", object.id);
      return false;
    }
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = object.dimension[0];
    primitive.dimensions[1] = object.dimension[1];
    primitive.dimensions[2] = object.dimension[2];

    msg->primitives.push_back(primitive);
    msg->primitive_poses.push_back(object.pose);
  } 
  else 
  {
    // load mesh
    std::string pkg_share_dir = ament_index_cpp::get_package_share_directory(object.mesh_pkg_name);

    if (pkg_share_dir.empty() || object.mesh_folder.empty() || object.mesh_file.empty())
    {
      RCLCPP_ERROR(get_logger(), "Invalid mesh path for object %d: %s/%s/%s", 
        object.id, 
        object.mesh_pkg_name.c_str(),
        object.mesh_folder.c_str(),
        object.mesh_file.c_str());
      return false;
    }
    
    std::string filepath = pkg_share_dir + "/" + object.mesh_folder + "/" + object.mesh_file;
    shapes::Mesh *mesh = shapes::createMeshFromResource(filepath);

    if(!mesh) 
    {
      RCLCPP_ERROR(get_logger(), "Failed to load mesh for object %d: %s", object.id, filepath.c_str());
      return false;
    } 
    
    shapes::ShapeMsg shape_msg;
    if (!shapes::constructMsgFromShape(mesh, shape_msg))
    {
      RCLCPP_ERROR(get_logger(), "Failed to convert mesh to message for object %d", object.id);
      return false;
    }
    msg->meshes.push_back(boost::get<shape_msgs::msg::Mesh>(shape_msg));
    msg->mesh_poses.push_back(object.pose);

    RCLCPP_WARN(get_logger(), "Loaded mesh file: %s pose:[%.2f %.2f %.2f %.2f %.2f %.2f %.2f]", filepath.c_str(), 
      object.pose.position.x, 
      object.pose.position.y, 
      object.pose.position.z, 
      object.pose.orientation.x, 
      object.pose.orientation.y, 
      object.pose.orientation.z, 
      object.pose.orientation.w);
  }

  msg->operation = moveit_msgs::msg::CollisionObject::ADD;

  return true;
}

void CollisionPlanner::sort_collision_objects(void)
{
  std::sort(collision_objects_.begin(), collision_objects_.end(), std::bind(&CollisionPlanner::compare_id, this, _1, _2));
}

bool CollisionPlanner::compare_id(const CollisionObject &a, const CollisionObject &b)
{
  return a.id < b.id;
}

template <typename T>
bool CollisionPlanner::send_req(
  typename rclcpp::Client<T>::SharedPtr cli, 
  const typename T::Request::SharedPtr request,
  typename T::Response::SharedPtr& response,
  const std::string srv_name) const
{
  if (!cli_wait_for_srv<T>(cli, srv_name))
  {
    RCLCPP_INFO(get_logger(), "Failed to wait service");
    return false;
  }

  auto future = cli->async_send_request(request);
  std::future_status status = future.wait_for(CLI_REQ_TIMEOUT);

  switch (status)
  {
  case std::future_status::ready:
    // Yech!!!
    break;
  case std::future_status::deferred:
    RCLCPP_INFO(get_logger(), "Failed to call service %s, status: %s", srv_name.c_str(), "deferred");
    return false;
  case std::future_status::timeout:
    RCLCPP_INFO(get_logger(), "Failed to call service %s, status: %s", srv_name.c_str(), "timeout");
    return false;
  }

  response = future.get();

  if (!response->success)
  {
    RCLCPP_INFO(get_logger(), "Service %s call failed with error: {%s}", srv_name.c_str(), response->message.c_str());
    return false;
  }

  return true;
}

template <typename T>
bool CollisionPlanner::cli_wait_for_srv(
  typename rclcpp::Client<T>::SharedPtr cli, 
  const std::string srv_name) const
{
  uint8_t retry = 0;

  while (rclcpp::ok() && !cli->wait_for_service(std::chrono::milliseconds(100)))
  {
    if (retry >= SRV_CLI_MAX_RETIES)
    {
      RCLCPP_DEBUG(get_logger(), "Interrupted while waiting for the service. Exiting.");
      return false;
    }

    RCLCPP_DEBUG(get_logger(), "%s service not available, waiting again...", srv_name.c_str());
    retry++;
  }

  return true;
}