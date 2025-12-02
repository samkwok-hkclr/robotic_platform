#include "tf_broadcaster.hpp"

TfBroadcaster::TfBroadcaster(
	std::string node_name,
  const rclcpp::NodeOptions& options)
: PlannerBase(node_name, options)
{
  declare_parameter<bool>("sim", false);
  declare_parameter<std::vector<double>>("rack_1_shelf_1", std::vector<double>{});
  declare_parameter<std::vector<double>>("rack_1_shelf_2", std::vector<double>{});
  declare_parameter<std::vector<double>>("rack_1_shelf_3", std::vector<double>{});
  declare_parameter<std::vector<double>>("rack_1_shelf_4", std::vector<double>{});
  declare_parameter<std::vector<double>>("left_tcp_to_camera", std::vector<double>{});
  declare_parameter<std::vector<double>>("right_tcp_to_camera", std::vector<double>{});

  get_parameter("sim", simulation_);

  // FIXME: the frames should be provided by AGV
	// push_tf_buf(std::make_tuple(cvt_g_to_pose(get_g(0.8, 1.35, 0, 0, 0, M_PI/2)), "map", BASE_FOOTPRINT));
	// push_tf_buf(std::make_tuple(cvt_g_to_pose(get_g(1.0, 1.0, 0, 0, 0, 0)), "map", BASE_FOOTPRINT));
	push_tf_buf(std::make_tuple(cvt_g_to_pose(get_g(0, 0, 0, 0, 0, 0)), "map", BASE_FOOTPRINT));

  setup_tcp_tf();
  // setup_static_tf();

	tf_timer_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

	tf_pub_timer = create_wall_timer(
    std::chrono::milliseconds(50), 
    std::bind(&TfBroadcaster::tf_pub_cb, this),
    tf_timer_cbg_);

	RCLCPP_INFO(get_logger(), "TF Broadcaster is up.");
}

void TfBroadcaster::setup_tcp_tf(void)
{
  if (simulation_)
  {

  }
  else
  {
    std::vector<double> right = get_parameter("right_tcp_to_camera").as_double_array();

    if (right.size() != 7)
    {
      RCLCPP_ERROR(get_logger(), "right tcp size does not match!");
      rclcpp::shutdown();
    }

    tf2::Transform g_cam_tcp = get_g(right[0], right[1], right[2], right[3], right[4], right[5], right[6]);
    Pose p = cvt_g_to_pose(g_cam_tcp.inverse());
    push_static_tf(std::make_tuple(p, "right_camera_color_optical_frame", "right_tcp"));
  } 

}

void TfBroadcaster::setup_static_tf(void)
{
  struct ShelfDefinition 
  {
    std::string rack;
    std::string shelf;
    std::vector<double> slot_distances;
  };

  std::vector<ShelfDefinition> all_shelves = {
    {"rack_1", "shelf_1", get_parameter("rack_1_shelf_1").as_double_array()},
    {"rack_1", "shelf_2", get_parameter("rack_1_shelf_2").as_double_array()},
    {"rack_1", "shelf_3", get_parameter("rack_1_shelf_3").as_double_array()},
    {"rack_1", "shelf_4", get_parameter("rack_1_shelf_4").as_double_array()},
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

  if (tf_buf_.empty())
    return;
  
  for (const auto& tf : tf_buf_)
  {
    std::apply(std::bind(&TfBroadcaster::send_transform, this, _1, _2, _3), tf);
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto options = rclcpp::NodeOptions();

  auto tf_broadcaster = std::make_shared<TfBroadcaster>("tf_broadcaster", options);
  exec->add_node(tf_broadcaster->get_node_base_interface());
  
  exec->spin();

  rclcpp::shutdown();
}
