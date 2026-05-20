#include <algorithm>
#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include <gz/msgs/camera_info.pb.h>
#include <gz/msgs/header.pb.h>
#include <gz/msgs/image.pb.h>
#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

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

std::string rosEncodingFromGz(const gz::msgs::PixelFormatType format)
{
  switch (format) {
    case gz::msgs::L_INT8:
      return sensor_msgs::image_encodings::MONO8;
    case gz::msgs::L_INT16:
      return sensor_msgs::image_encodings::MONO16;
    case gz::msgs::RGB_INT8:
      return sensor_msgs::image_encodings::RGB8;
    case gz::msgs::RGBA_INT8:
      return sensor_msgs::image_encodings::RGBA8;
    case gz::msgs::BGRA_INT8:
      return sensor_msgs::image_encodings::BGRA8;
    case gz::msgs::BGR_INT8:
      return sensor_msgs::image_encodings::BGR8;
    case gz::msgs::BAYER_RGGB8:
      return sensor_msgs::image_encodings::BAYER_RGGB8;
    case gz::msgs::BAYER_BGGR8:
      return sensor_msgs::image_encodings::BAYER_BGGR8;
    case gz::msgs::BAYER_GBRG8:
      return sensor_msgs::image_encodings::BAYER_GBRG8;
    case gz::msgs::BAYER_GRBG8:
      return sensor_msgs::image_encodings::BAYER_GRBG8;
    default:
      return "passthrough";
  }
}

std::string distortionModelFromGz(const gz::msgs::CameraInfo_Distortion_DistortionModelType model)
{
  switch (model) {
    case gz::msgs::CameraInfo_Distortion_DistortionModelType_RATIONAL_POLYNOMIAL:
      return "rational_polynomial";
    case gz::msgs::CameraInfo_Distortion_DistortionModelType_EQUIDISTANT:
      return "equidistant";
    case gz::msgs::CameraInfo_Distortion_DistortionModelType_PLUMB_BOB:
    default:
      return "plumb_bob";
  }
}

template<typename RepeatedField>
void copyRepeatedToArray(const RepeatedField & source, double * destination, const size_t destination_size)
{
  const auto count = std::min(destination_size, static_cast<size_t>(source.size()));
  for (size_t i = 0; i < count; ++i) {
    destination[i] = source.Get(static_cast<int>(i));
  }
}

class GzCameraBridgeNode : public rclcpp::Node
{
public:
  GzCameraBridgeNode()
  : Node("gz_camera_bridge_node")
  {
    const auto gz_image_topic = declare_parameter<std::string>(
      "gz_image_topic",
      "/world/baylands/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image");
    const auto gz_camera_info_topic = declare_parameter<std::string>(
      "gz_camera_info_topic",
      "/world/baylands/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info");
    const auto ros_image_topic = declare_parameter<std::string>(
      "ros_image_topic", "/drone/down_camera/image_raw");
    const auto ros_camera_info_topic = declare_parameter<std::string>(
      "ros_camera_info_topic", "/drone/down_camera/camera_info");
    frame_id_ = declare_parameter<std::string>("frame_id", "drone_down_camera");

    image_pub_ = create_publisher<sensor_msgs::msg::Image>(ros_image_topic, rclcpp::SensorDataQoS());
    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      ros_camera_info_topic, rclcpp::SensorDataQoS());

    if (!gz_node_.Subscribe(gz_image_topic, &GzCameraBridgeNode::onImage, this)) {
      throw std::runtime_error("failed to subscribe Gazebo image topic: " + gz_image_topic);
    }

    if (!gz_node_.Subscribe(gz_camera_info_topic, &GzCameraBridgeNode::onCameraInfo, this)) {
      throw std::runtime_error("failed to subscribe Gazebo camera_info topic: " + gz_camera_info_topic);
    }

    RCLCPP_INFO(get_logger(), "Gazebo image: %s -> ROS: %s",
      gz_image_topic.c_str(), ros_image_topic.c_str());
    RCLCPP_INFO(get_logger(), "Gazebo camera_info: %s -> ROS: %s",
      gz_camera_info_topic.c_str(), ros_camera_info_topic.c_str());
  }

private:
  void onImage(const gz::msgs::Image & msg)
  {
    sensor_msgs::msg::Image ros_msg;
    ros_msg.header.stamp = msg.has_header() ? stampFromHeader(msg.header(), *get_clock()) : now();
    ros_msg.header.frame_id = msg.has_header() ? frameIdFromHeader(msg.header(), frame_id_) : frame_id_;
    ros_msg.height = msg.height();
    ros_msg.width = msg.width();
    ros_msg.encoding = rosEncodingFromGz(msg.pixel_format_type());
    ros_msg.is_bigendian = false;
    ros_msg.step = msg.step();

    if (ros_msg.step == 0 && ros_msg.height > 0) {
      ros_msg.step = static_cast<uint32_t>(msg.data().size() / ros_msg.height);
    }

    const auto & data = msg.data();
    ros_msg.data.assign(data.begin(), data.end());
    image_pub_->publish(std::move(ros_msg));
  }

  void onCameraInfo(const gz::msgs::CameraInfo & msg)
  {
    sensor_msgs::msg::CameraInfo ros_msg;
    ros_msg.header.stamp = msg.has_header() ? stampFromHeader(msg.header(), *get_clock()) : now();
    ros_msg.header.frame_id = msg.has_header() ? frameIdFromHeader(msg.header(), frame_id_) : frame_id_;
    ros_msg.height = msg.height();
    ros_msg.width = msg.width();

    ros_msg.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

    if (msg.has_distortion()) {
      ros_msg.distortion_model = distortionModelFromGz(msg.distortion().model());
      ros_msg.d.assign(msg.distortion().k().begin(), msg.distortion().k().end());
    } else {
      ros_msg.distortion_model = "plumb_bob";
    }

    if (msg.has_intrinsics()) {
      copyRepeatedToArray(msg.intrinsics().k(), ros_msg.k.data(), ros_msg.k.size());
    }

    if (msg.rectification_matrix_size() > 0) {
      copyRepeatedToArray(msg.rectification_matrix(), ros_msg.r.data(), ros_msg.r.size());
    }

    if (msg.has_projection()) {
      copyRepeatedToArray(msg.projection().p(), ros_msg.p.data(), ros_msg.p.size());
    }

    camera_info_pub_->publish(std::move(ros_msg));
  }

  gz::transport::Node gz_node_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  std::string frame_id_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GzCameraBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
