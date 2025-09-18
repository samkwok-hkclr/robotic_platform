#include "tf_broadcaster.hpp"

TfBroadcaster::TfBroadcaster(
	std::string node_name,
  const rclcpp::NodeOptions& options)
: PlannerBase(node_name, options)
{
  // FIXME: the frames should be provided by AGV
	push_tf_buf(std::make_tuple(cvt_g_to_pose(get_g(0.9, 1.35, 0, 0, 0, M_PI/2)), "map", BASE_FOOTPRINT));

  setup_static_tf();

	tf_timer_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

	tf_pub_timer = create_wall_timer(
    std::chrono::milliseconds(20), 
    std::bind(&TfBroadcaster::tf_pub_cb, this),
    tf_timer_cbg_);

	RCLCPP_INFO(get_logger(), "TF Broadcaster is up.");
}

void TfBroadcaster::setup_static_tf()
{
  struct ShelfDefinition 
  {
    std::string rack;
    std::string shelf;
    std::vector<double> slot_distances;
  };

  std::vector<ShelfDefinition> all_shelves = {
    {"rack_1", "shelf_1", { }},
    {"rack_1", "shelf_2", { 0.22, 0.37, 0.90 }},
    {"rack_1", "shelf_3", { 0.22, 0.37, 0.61, 0.79, 0.92, 1.05 }},
    {"rack_1", "shelf_4", { 0.24, 0.37, 0.61, 0.79, 0.92, 1.05 }},
    // {"rack_2", "shelf_2", {0.1, 0.15, 0.25, 0.4}}, // Irregular spacing
  };

  for (const auto& shelf_def : all_shelves) 
  {
    for (size_t slot_idx = 0; slot_idx < shelf_def.slot_distances.size(); ++slot_idx) 
    {
      std::string base_frame = shelf_def.rack + "_" + shelf_def.shelf + "_flat_link";
      std::string target_frame = shelf_def.rack + "_" + shelf_def.shelf + "_slot_" + std::to_string(slot_idx + 1) + "_link";
      
      push_static_tf(std::make_tuple(
        cvt_g_to_pose(get_g(shelf_def.slot_distances[slot_idx], 0, 0, 0, -M_PI/2.0, -M_PI/2.0)),
        base_frame,
        target_frame
      ));
    }
  }
}

void TfBroadcaster::push_static_tf(const std::tuple<Pose, std::string, std::string>& tf)
{
  std::lock_guard<std::mutex> lock(tf_mutex_);

  std::apply(std::bind(&TfBroadcaster::send_static_transform, this, _1, _2, _3), tf);
}

void TfBroadcaster::push_tf_buf(const std::tuple<Pose, std::string, std::string>& tf)
{
  std::lock_guard<std::mutex> lock(tf_mutex_);

  tf_buf_.emplace_back(tf);
}

void TfBroadcaster::tf_pub_cb(void)
{
	std::lock_guard<std::mutex> lock(tf_mutex_);

  for (const auto& tf : tf_buf_)
  {
    std::apply(std::bind(&TfBroadcaster::send_transform, this, _1, _2, _3), tf);
  }
}

