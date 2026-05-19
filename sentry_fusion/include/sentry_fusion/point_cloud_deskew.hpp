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

#ifndef SENTRY_FUSION__POINT_CLOUD_DESKEW_HPP_
#define SENTRY_FUSION__POINT_CLOUD_DESKEW_HPP_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace sentry_fusion
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

  struct CloudSample
  {
    int64_t stamp_ns = 0;
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
  };

  struct DeskewedCloudSample
  {
    int64_t stamp_ns = 0;
    sensor_msgs::msg::PointCloud2::SharedPtr cloud;
    tf2::Transform target_odom_to_base;
  };

  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void odinCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  bool deskewCloud(
    const sensor_msgs::msg::PointCloud2 & input, sensor_msgs::msg::PointCloud2 & output,
    tf2::Transform & target_odom_to_base);
  bool lookupBaseToLidar(
    const sensor_msgs::msg::PointCloud2 & cloud, tf2::Transform & base_to_lidar);

  void storeDeskewedCloud(
    sensor_msgs::msg::PointCloud2 && cloud, const tf2::Transform & target_odom_to_base);
  void tryPublishFusedClouds();
  sensor_msgs::msg::PointCloud2::ConstSharedPtr findClosestOdinCloud(
    int64_t stamp_ns, int64_t & stamp_delta_ns) const;
  void pruneCloudCaches(int64_t newest_stamp_ns);

  bool fuseClouds(
    const sensor_msgs::msg::PointCloud2 & livox_cloud,
    const sensor_msgs::msg::PointCloud2 & odin_cloud, const tf2::Transform & target_odom_to_base,
    sensor_msgs::msg::PointCloud2 & fused_cloud);
  bool initializeFusionTransforms(const rclcpp::Time & stamp);
  bool resolveOdinRawCloudTransform(
    const sensor_msgs::msg::PointCloud2 & odin_cloud, tf2::Transform & cloud_to_fused_output);
  bool appendLivoxCloudAsXyzi(
    const sensor_msgs::msg::PointCloud2 & msg, const tf2::Transform & transform,
    pcl::PointCloud<pcl::PointXYZI> & cloud);
  bool appendOdinCloudAsXyzi(
    const sensor_msgs::msg::PointCloud2 & msg, const tf2::Transform & transform,
    pcl::PointCloud<pcl::PointXYZI> & cloud);
  bool shouldDropOdinSectorPoint(const tf2::Vector3 & point_in_base_frame) const;

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
  bool validateOdinCloudLayout(const sensor_msgs::msg::PointCloud2 & cloud);

  int64_t messageTimeToNanoseconds(const builtin_interfaces::msg::Time & stamp) const;
  builtin_interfaces::msg::Time nanosecondsToMessageTime(int64_t stamp_ns) const;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr odin_cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr deskewed_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fused_cloud_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::deque<PoseSample> odom_cache_;
  std::deque<CloudSample> odin_cloud_cache_;
  std::deque<DeskewedCloudSample> deskewed_cloud_cache_;
  tf2::Transform base_to_lidar_;
  std::string resolved_lidar_frame_;

  std::string input_cloud_topic_;
  std::string output_cloud_topic_;
  std::string odin_cloud_topic_;
  std::string fused_cloud_topic_;
  std::string odom_topic_;
  std::string base_frame_;
  std::string lidar_frame_;
  std::string fusion_base_frame_;
  std::string fused_output_frame_;

  bool base_to_lidar_ready_ = false;
  bool enable_fusion_ = true;
  bool publish_deskewed_cloud_ = false;
  bool enable_odin_sector_filter_ = false;
  bool fusion_transforms_initialized_ = false;
  int64_t time_offset_ns_ = 0;
  int64_t odom_cache_duration_ns_ = 0;
  int64_t fusion_cache_duration_ns_ = 0;
  int64_t sync_tolerance_ns_ = 0;
  int64_t max_odom_gap_ns_ = 0;
  int64_t max_extrapolation_ns_ = 0;
  int64_t last_fused_livox_stamp_ns_ = std::numeric_limits<int64_t>::min();
  double tf_lookup_timeout_sec_ = 0.05;
  double odin_sector_filter_radius_min_ = 0.0;
  double odin_sector_filter_radius_ = 0.0;
  double odin_sector_filter_angle_center_rad_ = 0.0;
  double odin_sector_filter_angle_half_width_rad_ = 0.0;
  tf2::Transform tf_fusion_base_to_odin_odom_;
};

}  // namespace sentry_fusion

#endif  // SENTRY_FUSION__POINT_CLOUD_DESKEW_HPP_
