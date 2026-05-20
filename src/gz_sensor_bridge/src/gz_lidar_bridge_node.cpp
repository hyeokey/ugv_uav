#include <algorithm>
#include <cstdint>
#include <string>

#include <gz/msgs/header.pb.h>
#include <gz/msgs/laserscan.pb.h>
#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace
{

rclcpp::Time stampFromHeader(const gz::msgs::Header & header, rclcpp::Clock & fallback_clock)
{
  if (!header.has_stamp()) {
    return fallback_clock.now();
  }

  const auto & stamp = header.stamp();
  return rclcpp::Time(stamp.sec(), static_cast<uint32_t>(stamp.nsec()), RCL_ROS_TIME);
}

std::string frameIdFromHeader(const gz::msgs::Header & header, const std::string & fallback)
{
  for (int i = 0; i < header.data_size(); ++i) {
    const auto & entry = header.data(i);
    if ((entry.key() == "frame_id" || entry.key() == "frame") && entry.value_size() > 0) {
      return entry.value(0);
    }
  }

  return fallback;
}

class GzLidarBridgeNode : public rclcpp::Node
{
public:
  GzLidarBridgeNode()
  : Node("gz_lidar_bridge_node")
  {
    const auto gz_lidar_topic = declare_parameter<std::string>(
      "gz_lidar_topic",
      "/world/baylands/model/explorer_rover_1/link/lidar_sensor_link/sensor/lidar/scan");
    const auto ros_lidar_topic = declare_parameter<std::string>(
      "ros_lidar_topic", "/ugv/lidar/scan");
    frame_id_ = declare_parameter<std::string>("frame_id", "ugv_lidar");
    scan_time_ = declare_parameter<double>("scan_time", 0.05);
    stamp_from_gz_header_ = declare_parameter<bool>("stamp_from_gz_header", false);

    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      ros_lidar_topic, rclcpp::SensorDataQoS());

    if (!gz_node_.Subscribe(gz_lidar_topic, &GzLidarBridgeNode::onScan, this)) {
      throw std::runtime_error("failed to subscribe Gazebo lidar topic: " + gz_lidar_topic);
    }

    RCLCPP_INFO(get_logger(), "Gazebo lidar: %s -> ROS: %s",
      gz_lidar_topic.c_str(), ros_lidar_topic.c_str());
  }

private:
  void onScan(const gz::msgs::LaserScan & msg)
  {
    sensor_msgs::msg::LaserScan ros_msg;
    ros_msg.header.stamp =
      (stamp_from_gz_header_ && msg.has_header()) ? stampFromHeader(msg.header(), *get_clock()) : now();
    ros_msg.header.frame_id = msg.has_header() ? frameIdFromHeader(msg.header(), frame_id_) : frame_id_;
    ros_msg.angle_min = static_cast<float>(msg.angle_min());
    ros_msg.angle_max = static_cast<float>(msg.angle_max());
    ros_msg.angle_increment = static_cast<float>(msg.angle_step());
    ros_msg.range_min = static_cast<float>(msg.range_min());
    ros_msg.range_max = static_cast<float>(msg.range_max());
    ros_msg.scan_time = static_cast<float>(scan_time_);

    const int range_count = msg.ranges_size();
    if (range_count > 1 && scan_time_ > 0.0) {
      ros_msg.time_increment = static_cast<float>(scan_time_ / static_cast<double>(range_count - 1));
    }

    ros_msg.ranges.reserve(static_cast<size_t>(range_count));
    for (int i = 0; i < range_count; ++i) {
      ros_msg.ranges.push_back(static_cast<float>(msg.ranges(i)));
    }

    const int intensity_count = msg.intensities_size();
    ros_msg.intensities.reserve(static_cast<size_t>(intensity_count));
    for (int i = 0; i < intensity_count; ++i) {
      ros_msg.intensities.push_back(static_cast<float>(msg.intensities(i)));
    }

    scan_pub_->publish(std::move(ros_msg));
  }

  gz::transport::Node gz_node_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  std::string frame_id_;
  double scan_time_;
  bool stamp_from_gz_header_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GzLidarBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
