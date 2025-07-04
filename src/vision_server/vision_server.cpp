#include "vision_server/vision_server.hpp"

VisionServer::VisionServer(
  const rclcpp::NodeOptions& options)
: Node("vision_server", options)
{
  declare_parameter<int>("history_max_size", 100);
  declare_parameter<std::string>("image_left_topic", "/robostore/left_hand/color/image_raw");
  declare_parameter<std::string>("image_right_topic", "/robostore/right_hand/color/image_raw");
  declare_parameter<std::string>("pointcloud_left_topic", "/robostore/left_hand/depth/color/points");
  declare_parameter<std::string>("pointcloud_right_topic", "/robostore/right_hand/depth/color/points");

  get_parameter("history_max_size", history_max_size_);
  image_topic_[CameraId::LEFT] = get_parameter("image_left_topic").as_string();
  image_topic_[CameraId::RIGHT] = get_parameter("image_right_topic").as_string();
  pc_topic_[CameraId::LEFT] = get_parameter("pointcloud_left_topic").as_string();
  pc_topic_[CameraId::RIGHT] = get_parameter("pointcloud_right_topic").as_string();

  image_sub_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  pc_sub_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions image_sub_options;
  image_sub_options.callback_group = image_sub_cbg_;
  rclcpp::SubscriptionOptions pc_sub_options;
  pc_sub_options.callback_group = pc_sub_cbg_;

  image_cb_[CameraId::LEFT] = std::bind(&VisionServer::image_recv_cb, this, std::placeholders::_1, CameraId::LEFT);
  image_cb_[CameraId::RIGHT] = std::bind(&VisionServer::image_recv_cb, this, std::placeholders::_1, CameraId::RIGHT);
  pc_cb_[CameraId::LEFT] = std::bind(&VisionServer::pc_recv_cb, this, std::placeholders::_1, CameraId::LEFT);
  pc_cb_[CameraId::RIGHT] = std::bind(&VisionServer::pc_recv_cb, this, std::placeholders::_1, CameraId::RIGHT);

  image_sub_[CameraId::LEFT] = create_subscription<Image>(
    image_topic_[CameraId::LEFT], 
    1000, 
    image_cb_[CameraId::LEFT],
    image_sub_options);
  
  image_sub_[CameraId::RIGHT] = create_subscription<Image>(
    image_topic_[CameraId::RIGHT], 
    1000, 
    image_cb_[CameraId::RIGHT],
    image_sub_options);
  
  pc_sub_[CameraId::LEFT] = create_subscription<PointCloud2>(
    pc_topic_[CameraId::LEFT], 
    1000, 
    pc_cb_[CameraId::LEFT],
    pc_sub_options);

  pc_sub_[CameraId::RIGHT] = create_subscription<PointCloud2>(
    pc_topic_[CameraId::RIGHT], 
    1000, 
    pc_cb_[CameraId::RIGHT],
    pc_sub_options);

  RCLCPP_INFO(get_logger(), "Vision Server is up.");
}

VisionServer::~VisionServer()
{

}

void VisionServer::image_recv_cb(const Image::SharedPtr msg, const CameraId id)
{
  std::lock_guard<std::mutex> lock(image_mutexes_[id]);  

  image_history_[id].emplace_back(std::move(*msg));

  if (history_max_size_ > 0 && image_history_[id].size() > history_max_size_) 
  {
    image_history_[id].pop_front();
  }

  RCLCPP_INFO(get_logger(), "Image received! %s <Size: %ld>", 
    id == CameraId::LEFT ? "left" : "right", 
    image_history_[id].size());
}

void VisionServer::pc_recv_cb(const PointCloud2::SharedPtr msg, const CameraId id)
{
  std::lock_guard<std::mutex> lock(pc_mutexes_[id]);  

  pc_history_[id].emplace_back(std::move(*msg));

  if (history_max_size_ > 0 && pc_history_[id].size() > history_max_size_) 
  {
    pc_history_[id].pop_front();
  }

  RCLCPP_INFO(get_logger(), "Pointcloud received! %s <Size: %ld>", 
    id == CameraId::LEFT ? "left" : "right", 
    pc_history_[id].size());
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<VisionServer>(options);

  exec->add_node(node->get_node_base_interface());
  exec->spin();

  rclcpp::shutdown();
}