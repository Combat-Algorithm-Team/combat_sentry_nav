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

#ifndef BODY_POINT_CLOUD_FILTER__BODY_POINT_CLOUD_FILTER_HPP_
#define BODY_POINT_CLOUD_FILTER__BODY_POINT_CLOUD_FILTER_HPP_

#include <cstddef>
#include <cstdint>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace body_point_cloud_filter
{

class BodyPointCloudFilterNode : public rclcpp::Node
{
public:
  explicit BodyPointCloudFilterNode(const rclcpp::NodeOptions & options);

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  bool findField(
    const sensor_msgs::msg::PointCloud2 & msg, const std::string & field_name,
    sensor_msgs::msg::PointField & field) const;

  bool validateFieldLayout(
    const sensor_msgs::msg::PointCloud2 & msg, const sensor_msgs::msg::PointField & field,
    const char * field_name);

  bool readCoordinate(
    const uint8_t * point_data, const sensor_msgs::msg::PointField & field, bool is_bigendian,
    double & value) const;

  bool shouldRemovePoint(
    const uint8_t * point_data, const sensor_msgs::msg::PointField & x_field,
    const sensor_msgs::msg::PointField & y_field, const sensor_msgs::msg::PointField & z_field,
    bool is_bigendian) const;

  static bool isHostBigEndian();

  template<typename T>
  static T readScalar(const uint8_t * data, bool data_is_bigendian);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;

  std::string input_topic_;
  std::string output_topic_;

  double min_x_ = -0.5;
  double max_x_ = 0.5;
  double min_y_ = -0.5;
  double max_y_ = 0.5;
  double min_z_ = -0.5;
  double max_z_ = 0.5;
};

}  // namespace body_point_cloud_filter

#endif  // BODY_POINT_CLOUD_FILTER__BODY_POINT_CLOUD_FILTER_HPP_
