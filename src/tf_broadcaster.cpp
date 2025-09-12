#include "tf_broadcaster.hpp"

TfBroadcaster::TfBroadcaster(
	std::string node_name,
  const rclcpp::NodeOptions& options)
: PlannerBase(node_name, options)
{
  // FIXME: those two frames should be provided by AGV
	push_tf_buf(std::make_tuple(Pose(), "map", "odom"));
	push_tf_buf(std::make_tuple(cvt_g_to_pose(get_g(0.8, 0.7, 0, 0, 0, M_PI)), "odom", "base_footprint"));

	tf_timer_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

	tf_pub_timer = create_wall_timer(
    std::chrono::milliseconds(20), 
    std::bind(&TfBroadcaster::tf_pub_cb, this),
    tf_timer_cbg_);

	RCLCPP_INFO(get_logger(), "TF Broadcaster is up.");
}

void TfBroadcaster::push_tf_static_buf(const std::tuple<Pose, std::string, std::string>& tf)
{
  std::lock_guard<std::mutex> lock(tf_mutex_);

  std::apply(std::bind(&TfBroadcaster::send_transform, this, _1, _2, _3), tf);
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

