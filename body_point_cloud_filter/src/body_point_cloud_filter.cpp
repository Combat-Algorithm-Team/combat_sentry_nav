// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "body_point_cloud_filter/body_point_cloud_filter.hpp"

#include <algorithm>
#include <cstring>
#include <functional>
#include <stdexcept>
#include <vector>

#include "rclcpp_components/register_node_macro.hpp"

namespace body_point_cloud_filter
{

BodyPointCloudFilterNode::BodyPointCloudFilterNode(const rclcpp::NodeOptions & options)
: Node("body_point_cloud_filter", options)
{
  this->declare_parameter<std::string>("input_topic", "livox/lidar");
  this->declare_parameter<std::string>("output_topic", "livox/lidar_filtered");
  this->declare_parameter<double>("min_x", -0.5);
  this->declare_parameter<double>("max_x", 0.5);
  this->declare_parameter<double>("min_y", -0.5);
  this->declare_parameter<double>("max_y", 0.5);
  this->declare_parameter<double>("min_z", -0.5);
  this->declare_parameter<double>("max_z", 0.5);
  this->declare_parameter<bool>("use_sim_time", false);

  this->get_parameter("input_topic", input_topic_);
  this->get_parameter("output_topic", output_topic_);
  this->get_parameter("min_x", min_x_);
  this->get_parameter("max_x", max_x_);
  this->get_parameter("min_y", min_y_);
  this->get_parameter("max_y", max_y_);
  this->get_parameter("min_z", min_z_);
  this->get_parameter("max_z", max_z_);

  if (min_x_ > max_x_ || min_y_ > max_y_ || min_z_ > max_z_) {
    RCLCPP_FATAL(
      this->get_logger(),
      "Invalid box filter bounds: x[%f, %f], y[%f, %f], z[%f, %f]. Each min value must be "
      "less than or equal to its max value.",
      min_x_, max_x_, min_y_, max_y_, min_z_, max_z_);
    throw std::invalid_argument("body_point_cloud_filter received invalid min/max bounds");
  }

  const auto qos = rclcpp::SensorDataQoS();
  point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_, qos,
    std::bind(&BodyPointCloudFilterNode::pointCloudCallback, this, std::placeholders::_1));
  point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);

  RCLCPP_INFO(
    this->get_logger(),
    "Filtering point cloud from '%s' to '%s' by removing points inside x[%f, %f], y[%f, %f], "
    "z[%f, %f].",
    input_topic_.c_str(), output_topic_.c_str(), min_x_, max_x_, min_y_, max_y_, min_z_, max_z_);
}

void BodyPointCloudFilterNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (msg->point_step == 0U) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000, "Received PointCloud2 with point_step == 0.");
    return;
  }

  const std::size_t minimum_row_step = static_cast<std::size_t>(msg->width) * msg->point_step;
  if (msg->row_step < minimum_row_step) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Received malformed PointCloud2: row_step %u is smaller than width * point_step (%zu).",
      static_cast<unsigned int>(msg->row_step), minimum_row_step);
    return;
  }

  const std::size_t expected_size = static_cast<std::size_t>(msg->row_step) * msg->height;
  if (msg->data.size() < expected_size) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Received malformed PointCloud2: data size %zu is smaller than row_step * height (%zu).",
      msg->data.size(), expected_size);
    return;
  }

  sensor_msgs::msg::PointField x_field;
  sensor_msgs::msg::PointField y_field;
  sensor_msgs::msg::PointField z_field;
  if (!findField(*msg, "x", x_field) || !findField(*msg, "y", y_field) || !findField(*msg, "z", z_field)) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Dropping point cloud because x/y/z fields are missing.");
    return;
  }

  if (
    !validateFieldLayout(*msg, x_field, "x") || !validateFieldLayout(*msg, y_field, "y") ||
    !validateFieldLayout(*msg, z_field, "z"))
  {
    return;
  }

  std::vector<uint8_t> filtered_data;
  filtered_data.reserve(static_cast<std::size_t>(msg->width) * msg->height * msg->point_step);

  std::size_t kept_points = 0U;
  for (std::size_t row = 0; row < msg->height; ++row) {
    const std::size_t row_offset = row * msg->row_step;
    for (std::size_t col = 0; col < msg->width; ++col) {
      const std::size_t point_offset = row_offset + col * msg->point_step;
      const uint8_t * point_data = msg->data.data() + point_offset;
      if (shouldRemovePoint(point_data, x_field, y_field, z_field, msg->is_bigendian)) {
        continue;
      }

      filtered_data.insert(
        filtered_data.end(), point_data, point_data + msg->point_step);
      ++kept_points;
    }
  }

  sensor_msgs::msg::PointCloud2 filtered_msg;
  filtered_msg.header = msg->header;
  filtered_msg.height = 1U;
  filtered_msg.width = static_cast<uint32_t>(kept_points);
  filtered_msg.fields = msg->fields;
  filtered_msg.is_bigendian = msg->is_bigendian;
  filtered_msg.point_step = msg->point_step;
  filtered_msg.row_step = static_cast<uint32_t>(kept_points * msg->point_step);
  filtered_msg.data = std::move(filtered_data);
  filtered_msg.is_dense = msg->is_dense;

  point_cloud_pub_->publish(std::move(filtered_msg));
}

bool BodyPointCloudFilterNode::findField(
  const sensor_msgs::msg::PointCloud2 & msg, const std::string & field_name,
  sensor_msgs::msg::PointField & field) const
{
  auto it = std::find_if(
    msg.fields.begin(), msg.fields.end(),
    [&field_name](const sensor_msgs::msg::PointField & candidate) {
      return candidate.name == field_name;
    });

  if (it == msg.fields.end()) {
    return false;
  }

  field = *it;
  return true;
}

bool BodyPointCloudFilterNode::validateFieldLayout(
  const sensor_msgs::msg::PointCloud2 & msg, const sensor_msgs::msg::PointField & field,
  const char * field_name)
{
  std::size_t field_size = 0U;
  switch (field.datatype) {
    case sensor_msgs::msg::PointField::FLOAT32:
      field_size = sizeof(float);
      break;
    case sensor_msgs::msg::PointField::FLOAT64:
      field_size = sizeof(double);
      break;
    default:
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Dropping point cloud because field '%s' has unsupported datatype %u.",
        field_name, static_cast<unsigned int>(field.datatype));
      return false;
  }

  if (field.count == 0U) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Dropping point cloud because field '%s' has count == 0.", field_name);
    return false;
  }

  const std::size_t field_end = static_cast<std::size_t>(field.offset) + field_size;
  if (field_end > msg.point_step) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Dropping point cloud because field '%s' exceeds point_step.", field_name);
    return false;
  }

  return true;
}

bool BodyPointCloudFilterNode::readCoordinate(
  const uint8_t * point_data, const sensor_msgs::msg::PointField & field, bool is_bigendian,
  double & value) const
{
  const uint8_t * field_data = point_data + field.offset;
  switch (field.datatype) {
    case sensor_msgs::msg::PointField::FLOAT32:
      value = static_cast<double>(readScalar<float>(field_data, is_bigendian));
      return true;
    case sensor_msgs::msg::PointField::FLOAT64:
      value = readScalar<double>(field_data, is_bigendian);
      return true;
    default:
      return false;
  }
}

bool BodyPointCloudFilterNode::shouldRemovePoint(
  const uint8_t * point_data, const sensor_msgs::msg::PointField & x_field,
  const sensor_msgs::msg::PointField & y_field, const sensor_msgs::msg::PointField & z_field,
  bool is_bigendian) const
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  const bool read_success =
    readCoordinate(point_data, x_field, is_bigendian, x) &&
    readCoordinate(point_data, y_field, is_bigendian, y) &&
    readCoordinate(point_data, z_field, is_bigendian, z);

  if (!read_success) {
    return false;
  }

  return
    x >= min_x_ && x <= max_x_ &&
    y >= min_y_ && y <= max_y_ &&
    z >= min_z_ && z <= max_z_;
}

bool BodyPointCloudFilterNode::isHostBigEndian()
{
  const std::uint16_t value = 0x0102;
  const auto * bytes = reinterpret_cast<const std::uint8_t *>(&value);
  return bytes[0] == 0x01;
}

template<typename T>
T BodyPointCloudFilterNode::readScalar(const uint8_t * data, bool data_is_bigendian)
{
  T value;
  auto * value_bytes = reinterpret_cast<uint8_t *>(&value);
  if (data_is_bigendian == isHostBigEndian()) {
    std::memcpy(value_bytes, data, sizeof(T));
  } else {
    std::reverse_copy(data, data + sizeof(T), value_bytes);
  }
  return value;
}

}  // namespace body_point_cloud_filter

RCLCPP_COMPONENTS_REGISTER_NODE(body_point_cloud_filter::BodyPointCloudFilterNode)
