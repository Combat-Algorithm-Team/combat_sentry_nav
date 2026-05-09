// Copyright 2026 Jieliang Li
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

#ifndef POINT_CLOUD_DESKEW__POINT_CLOUD_DESKEW_HPP_
#define POINT_CLOUD_DESKEW__POINT_CLOUD_DESKEW_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace point_cloud_deskew
{

class PointCloudDeskewNode : public rclcpp::Node
{
public:
  explicit PointCloudDeskewNode(const rclcpp::NodeOptions & options);

private:
  struct PoseSample
  {
    int64_t stamp_ns = 0;
    tf2::Transform odom_to_base;
  };

  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  bool deskewCloud(
    const sensor_msgs::msg::PointCloud2 & input, sensor_msgs::msg::PointCloud2 & output);
  bool lookupBaseToLidar(
    const sensor_msgs::msg::PointCloud2 & cloud, tf2::Transform & base_to_lidar,
    std::string & lidar_frame);

  bool interpolatePose(int64_t stamp_ns, std::size_t & cursor, tf2::Transform & odom_to_base) const;
  bool interpolateBetween(
    const PoseSample & before, const PoseSample & after, int64_t stamp_ns,
    tf2::Transform & odom_to_base) const;
  tf2::Quaternion interpolateQuaternion(
    tf2::Quaternion before, tf2::Quaternion after, double ratio) const;

  const std::uint8_t * pointData(
    const sensor_msgs::msg::PointCloud2 & cloud, std::size_t index) const;
  std::uint8_t * pointData(sensor_msgs::msg::PointCloud2 & cloud, std::size_t index) const;
  int64_t readPointTimeNs(const std::uint8_t * point_data) const;

  bool validateCloudLayout(const sensor_msgs::msg::PointCloud2 & cloud);

  int64_t messageTimeToNanoseconds(const builtin_interfaces::msg::Time & stamp) const;
  builtin_interfaces::msg::Time nanosecondsToMessageTime(int64_t stamp_ns) const;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::deque<PoseSample> odom_cache_;
  tf2::Transform base_to_lidar_;
  std::string resolved_lidar_frame_;

  std::string input_cloud_topic_;
  std::string output_cloud_topic_;
  std::string odom_topic_;
  std::string base_frame_;
  std::string lidar_frame_;
  std::string output_frame_;

  bool base_to_lidar_ready_ = false;
  int64_t time_offset_ns_ = 0;
  int64_t odom_cache_duration_ns_ = 0;
  int64_t max_odom_gap_ns_ = 0;
  int64_t max_extrapolation_ns_ = 0;
  double tf_lookup_timeout_sec_ = 0.05;
};

}  // namespace point_cloud_deskew

#endif  // POINT_CLOUD_DESKEW__POINT_CLOUD_DESKEW_HPP_
