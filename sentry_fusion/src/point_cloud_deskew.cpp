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

#include "sentry_fusion/point_cloud_deskew.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <utility>

#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace sentry_fusion
{

namespace
{

constexpr int64_t K_NANOSECONDS_PER_SECOND = 1000000000LL;
constexpr std::uint32_t K_LIVOX_X_OFFSET = 0U;
constexpr std::uint32_t K_LIVOX_Y_OFFSET = 4U;
constexpr std::uint32_t K_LIVOX_Z_OFFSET = 8U;
constexpr std::uint32_t K_LIVOX_TIMESTAMP_OFFSET = 18U;
constexpr std::uint32_t K_LIVOX_MIN_POINT_STEP =
  K_LIVOX_TIMESTAMP_OFFSET + static_cast<std::uint32_t>(sizeof(double));

template <typename T>
T readScalar(const std::uint8_t * data, std::uint32_t offset)
{
  T value;
  std::memcpy(&value, data + offset, sizeof(T));
  return value;
}

template <typename T>
void writeScalar(std::uint8_t * data, std::uint32_t offset, const T & value)
{
  std::memcpy(data + offset, &value, sizeof(T));
}

int64_t secondsToNanoseconds(double seconds)
{
  return static_cast<int64_t>(std::llround(seconds * K_NANOSECONDS_PER_SECOND));
}

int64_t absDeltaNs(int64_t lhs, int64_t rhs) { return lhs > rhs ? lhs - rhs : rhs - lhs; }

}  // namespace

PointCloudDeskewNode::PointCloudDeskewNode(const rclcpp::NodeOptions & options)
: Node("point_cloud_deskew", options)
{
  this->declare_parameter<std::string>("input_cloud_topic", "livox/lidar/pointcloud");
  this->declare_parameter<std::string>("output_cloud_topic", "livox/lidar_deskewed");
  this->declare_parameter<bool>("publish_deskewed_cloud", false);
  this->declare_parameter<bool>("enable_fusion", true);
  this->declare_parameter<std::string>("odin_cloud_topic", "odin1/cloud_slam");
  this->declare_parameter<std::string>("fused_cloud_topic", "registered_scan");
  this->declare_parameter<std::string>("odom_topic", "odin1/odometry_highfreq");
  this->declare_parameter<std::string>("base_frame", "odin1_base_link");
  this->declare_parameter<std::string>("lidar_frame", "front_mid360");
  this->declare_parameter<std::string>("output_frame", "front_mid360");
  this->declare_parameter<std::string>("fusion_base_frame", "base_footprint");
  this->declare_parameter<std::string>("odin_frame", "odin1_base_link");
  this->declare_parameter<std::string>("livox_frame", "front_mid360");
  this->declare_parameter<std::string>("fused_output_frame", "odom");
  this->declare_parameter<double>("time_offset_sec", 0.0);
  this->declare_parameter<double>("odom_cache_duration_sec", 2.0);
  this->declare_parameter<double>("fusion_cache_duration_sec", 1.0);
  this->declare_parameter<double>("sync_tolerance_sec", 0.05);
  this->declare_parameter<double>("max_odom_gap_sec", 0.10);
  this->declare_parameter<double>("max_extrapolation_sec", 0.05);
  this->declare_parameter<double>("tf_lookup_timeout_sec", 0.05);

  double time_offset_sec = 0.0;
  double odom_cache_duration_sec = 2.0;
  double fusion_cache_duration_sec = 1.0;
  double sync_tolerance_sec = 0.05;
  double max_odom_gap_sec = 0.10;
  double max_extrapolation_sec = 0.05;

  this->get_parameter("input_cloud_topic", input_cloud_topic_);
  this->get_parameter("output_cloud_topic", output_cloud_topic_);
  this->get_parameter("publish_deskewed_cloud", publish_deskewed_cloud_);
  this->get_parameter("enable_fusion", enable_fusion_);
  this->get_parameter("odin_cloud_topic", odin_cloud_topic_);
  this->get_parameter("fused_cloud_topic", fused_cloud_topic_);
  this->get_parameter("odom_topic", odom_topic_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("output_frame", output_frame_);
  this->get_parameter("fusion_base_frame", fusion_base_frame_);
  this->get_parameter("odin_frame", odin_frame_);
  this->get_parameter("livox_frame", livox_frame_);
  this->get_parameter("fused_output_frame", fused_output_frame_);
  this->get_parameter("time_offset_sec", time_offset_sec);
  this->get_parameter("odom_cache_duration_sec", odom_cache_duration_sec);
  this->get_parameter("fusion_cache_duration_sec", fusion_cache_duration_sec);
  this->get_parameter("sync_tolerance_sec", sync_tolerance_sec);
  this->get_parameter("max_odom_gap_sec", max_odom_gap_sec);
  this->get_parameter("max_extrapolation_sec", max_extrapolation_sec);
  this->get_parameter("tf_lookup_timeout_sec", tf_lookup_timeout_sec_);

  if (livox_frame_.empty()) {
    livox_frame_ = output_frame_.empty() ? lidar_frame_ : output_frame_;
  }

  time_offset_ns_ = secondsToNanoseconds(time_offset_sec);
  odom_cache_duration_ns_ = secondsToNanoseconds(odom_cache_duration_sec);
  sync_tolerance_ns_ = secondsToNanoseconds(sync_tolerance_sec);
  fusion_cache_duration_ns_ =
    std::max(secondsToNanoseconds(fusion_cache_duration_sec), sync_tolerance_ns_);
  max_odom_gap_ns_ = secondsToNanoseconds(max_odom_gap_sec);
  max_extrapolation_ns_ = secondsToNanoseconds(max_extrapolation_sec);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, rclcpp::SensorDataQoS(),
    std::bind(&PointCloudDeskewNode::pointCloudCallback, this, std::placeholders::_1));

  auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(200));
  odom_qos.best_effort();
  odom_qos.durability_volatile();
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, odom_qos,
    std::bind(&PointCloudDeskewNode::odometryCallback, this, std::placeholders::_1));

  if (enable_fusion_) {
    odin_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      odin_cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PointCloudDeskewNode::odinCloudCallback, this, std::placeholders::_1));
    fused_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      fused_cloud_topic_, rclcpp::SensorDataQoS());
  }

  if (publish_deskewed_cloud_ || !enable_fusion_) {
    auto output_qos = rclcpp::QoS(rclcpp::KeepLast(10));
    output_qos.reliable();
    output_qos.durability_volatile();
    deskewed_cloud_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, output_qos);
  }

  if (enable_fusion_) {
    RCLCPP_INFO(
      this->get_logger(),
      "Deskew and fuse Livox '%s' with Odin cloud '%s' into '%s'. Odometry: '%s'.",
      input_cloud_topic_.c_str(), odin_cloud_topic_.c_str(), fused_cloud_topic_.c_str(),
      odom_topic_.c_str());
  } else {
    RCLCPP_INFO(
      this->get_logger(),
      "Deskew '%s' -> '%s' with odometry '%s'. LiDAR frame: '%s', base frame: '%s'.",
      input_cloud_topic_.c_str(), output_cloud_topic_.c_str(), odom_topic_.c_str(),
      lidar_frame_.c_str(), base_frame_.c_str());
  }
}

void PointCloudDeskewNode::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  const int64_t stamp_ns = messageTimeToNanoseconds(msg->header.stamp);
  if (stamp_ns <= 0) {
    return;
  }

  tf2::Quaternion orientation(
    msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  if (orientation.length2() < std::numeric_limits<double>::epsilon()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Ignoring odometry with invalid zero-length orientation.");
    return;
  }
  orientation.normalize();

  PoseSample sample;
  sample.stamp_ns = stamp_ns;
  sample.odom_to_base.setOrigin(
    tf2::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
  sample.odom_to_base.setRotation(orientation);

  if (!odom_cache_.empty() && stamp_ns <= odom_cache_.back().stamp_ns) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "Ignoring non-increasing odometry timestamp.");
    return;
  }
  odom_cache_.push_back(sample);

  const int64_t keep_after_ns = odom_cache_.back().stamp_ns - odom_cache_duration_ns_;
  while (!odom_cache_.empty() && odom_cache_.front().stamp_ns < keep_after_ns) {
    odom_cache_.pop_front();
  }
}

void PointCloudDeskewNode::odinCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  const int64_t stamp_ns = messageTimeToNanoseconds(msg->header.stamp);
  if (stamp_ns <= 0) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "Ignoring Odin cloud with invalid timestamp.");
    return;
  }

  odin_cloud_cache_.push_back(CloudSample{stamp_ns, msg});
  pruneCloudCaches(stamp_ns);
  tryPublishFusedClouds();
}

void PointCloudDeskewNode::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  sensor_msgs::msg::PointCloud2 deskewed_cloud;
  tf2::Transform target_odom_to_base;
  if (!deskewCloud(*msg, deskewed_cloud, target_odom_to_base)) {
    return;
  }

  if (deskewed_cloud_pub_) {
    deskewed_cloud_pub_->publish(deskewed_cloud);
  }

  if (enable_fusion_) {
    storeDeskewedCloud(std::move(deskewed_cloud), target_odom_to_base);
  }
}

bool PointCloudDeskewNode::deskewCloud(
  const sensor_msgs::msg::PointCloud2 & input, sensor_msgs::msg::PointCloud2 & output,
  tf2::Transform & target_odom_to_base)
{
  if (!validateCloudLayout(input)) {
    return false;
  }

  tf2::Transform base_to_lidar;
  std::string lidar_frame;
  if (!lookupBaseToLidar(input, base_to_lidar, lidar_frame)) {
    return false;
  }

  const std::size_t point_count = static_cast<std::size_t>(input.width) * input.height;
  if (point_count == 0U) {
    output = input;
    target_odom_to_base.setIdentity();
    return true;
  }

  int64_t target_stamp_ns = std::numeric_limits<int64_t>::min();
  for (std::size_t i = 0; i < point_count; ++i) {
    target_stamp_ns = std::max(target_stamp_ns, readPointTimeNs(pointData(input, i)));
  }
  if (target_stamp_ns <= 0) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Drop cloud: no valid point timestamp was found.");
    return false;
  }

  std::size_t target_cursor = 1U;
  if (!interpolatePose(target_stamp_ns, target_cursor, target_odom_to_base)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Drop cloud: scan end time %.6f is outside the odometry cache.",
      static_cast<double>(target_stamp_ns) / K_NANOSECONDS_PER_SECOND);
    return false;
  }

  const tf2::Transform target_lidar_to_odom = (target_odom_to_base * base_to_lidar).inverse();

  output = input;
  output.header.stamp = nanosecondsToMessageTime(target_stamp_ns);
  output.header.frame_id = output_frame_.empty() ? lidar_frame : output_frame_;

  std::size_t odom_cursor = 1U;
  for (std::size_t i = 0; i < point_count; ++i) {
    std::uint8_t * point = pointData(output, i);
    const int64_t point_stamp_ns = readPointTimeNs(point);

    tf2::Transform point_odom_to_base;
    if (!interpolatePose(point_stamp_ns, odom_cursor, point_odom_to_base)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Drop cloud: point time %.6f is outside the odometry cache.",
        static_cast<double>(point_stamp_ns) / K_NANOSECONDS_PER_SECOND);
      return false;
    }

    const float x = readScalar<float>(point, K_LIVOX_X_OFFSET);
    const float y = readScalar<float>(point, K_LIVOX_Y_OFFSET);
    const float z = readScalar<float>(point, K_LIVOX_Z_OFFSET);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }

    const tf2::Transform point_odom_to_lidar = point_odom_to_base * base_to_lidar;
    const tf2::Vector3 deskewed =
      target_lidar_to_odom * (point_odom_to_lidar * tf2::Vector3(x, y, z));

    writeScalar<float>(point, K_LIVOX_X_OFFSET, static_cast<float>(deskewed.x()));
    writeScalar<float>(point, K_LIVOX_Y_OFFSET, static_cast<float>(deskewed.y()));
    writeScalar<float>(point, K_LIVOX_Z_OFFSET, static_cast<float>(deskewed.z()));
  }

  return true;
}

void PointCloudDeskewNode::storeDeskewedCloud(
  sensor_msgs::msg::PointCloud2 && cloud, const tf2::Transform & target_odom_to_base)
{
  const int64_t stamp_ns = messageTimeToNanoseconds(cloud.header.stamp);
  if (stamp_ns <= 0) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Skip fusion for deskewed Livox cloud with invalid timestamp.");
    return;
  }

  auto cloud_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(std::move(cloud));
  deskewed_cloud_cache_.push_back(DeskewedCloudSample{stamp_ns, cloud_ptr, target_odom_to_base});
  pruneCloudCaches(stamp_ns);
  tryPublishFusedClouds();
}

void PointCloudDeskewNode::tryPublishFusedClouds()
{
  if (
    !enable_fusion_ || !fused_cloud_pub_ || deskewed_cloud_cache_.empty() ||
    odin_cloud_cache_.empty()) {
    return;
  }

  for (const auto & deskewed_sample : deskewed_cloud_cache_) {
    if (deskewed_sample.stamp_ns <= last_fused_livox_stamp_ns_) {
      continue;
    }

    int64_t stamp_delta_ns = std::numeric_limits<int64_t>::max();
    const auto odin_cloud = findClosestOdinCloud(deskewed_sample.stamp_ns, stamp_delta_ns);
    if (!odin_cloud || stamp_delta_ns > sync_tolerance_ns_) {
      continue;
    }

    sensor_msgs::msg::PointCloud2 fused_cloud;
    if (!fuseClouds(
          *deskewed_sample.cloud, *odin_cloud, deskewed_sample.target_odom_to_base, fused_cloud)) {
      continue;
    }

    fused_cloud_pub_->publish(fused_cloud);
    last_fused_livox_stamp_ns_ = deskewed_sample.stamp_ns;
  }
}

sensor_msgs::msg::PointCloud2::ConstSharedPtr PointCloudDeskewNode::findClosestOdinCloud(
  int64_t stamp_ns, int64_t & stamp_delta_ns) const
{
  sensor_msgs::msg::PointCloud2::ConstSharedPtr closest_cloud;
  stamp_delta_ns = std::numeric_limits<int64_t>::max();

  for (const auto & sample : odin_cloud_cache_) {
    const int64_t delta_ns = absDeltaNs(sample.stamp_ns, stamp_ns);
    if (delta_ns < stamp_delta_ns) {
      stamp_delta_ns = delta_ns;
      closest_cloud = sample.cloud;
    }
  }

  return closest_cloud;
}

void PointCloudDeskewNode::pruneCloudCaches(int64_t newest_stamp_ns)
{
  const int64_t keep_after_ns = newest_stamp_ns - fusion_cache_duration_ns_;
  while (!odin_cloud_cache_.empty() && odin_cloud_cache_.front().stamp_ns < keep_after_ns) {
    odin_cloud_cache_.pop_front();
  }
  while (!deskewed_cloud_cache_.empty() && deskewed_cloud_cache_.front().stamp_ns < keep_after_ns) {
    deskewed_cloud_cache_.pop_front();
  }
}

bool PointCloudDeskewNode::fuseClouds(
  const sensor_msgs::msg::PointCloud2 & livox_cloud,
  const sensor_msgs::msg::PointCloud2 & odin_cloud, const tf2::Transform & target_odom_to_base,
  sensor_msgs::msg::PointCloud2 & fused_cloud)
{
  const rclcpp::Time stamp(livox_cloud.header.stamp);
  if (!initializeFusionTransforms(stamp)) {
    return false;
  }

  const tf2::Transform tf_fusion_base_to_livox =
    tf_fusion_base_to_odin_odom_ * target_odom_to_base * tf_odin_to_livox_;

  pcl::PointCloud<pcl::PointXYZI> fused_pcl;
  fused_pcl.points.reserve(
    static_cast<std::size_t>(livox_cloud.width) * livox_cloud.height +
    static_cast<std::size_t>(odin_cloud.width) * odin_cloud.height);

  const bool has_livox = appendCloudAsXyzi(livox_cloud, tf_fusion_base_to_livox, fused_pcl);
  const bool has_odin = appendCloudAsXyzi(odin_cloud, tf_fusion_base_to_odin_odom_, fused_pcl);
  if (!has_livox && !has_odin) {
    return false;
  }

  fused_pcl.width = static_cast<std::uint32_t>(fused_pcl.points.size());
  fused_pcl.height = 1U;
  fused_pcl.is_dense = false;

  pcl::toROSMsg(fused_pcl, fused_cloud);
  fused_cloud.header.stamp = livox_cloud.header.stamp;
  fused_cloud.header.frame_id = fused_output_frame_;
  return true;
}

bool PointCloudDeskewNode::initializeFusionTransforms(const rclcpp::Time & stamp)
{
  if (fusion_transforms_initialized_) {
    return true;
  }

  try {
    auto fusion_base_to_odin_odom = tf_buffer_->lookupTransform(
      fusion_base_frame_, odin_frame_, stamp,
      rclcpp::Duration::from_seconds(tf_lookup_timeout_sec_));
    auto odin_to_livox = tf_buffer_->lookupTransform(
      odin_frame_, livox_frame_, stamp, rclcpp::Duration::from_seconds(tf_lookup_timeout_sec_));
    tf2::fromMsg(fusion_base_to_odin_odom.transform, tf_fusion_base_to_odin_odom_);
    tf2::fromMsg(odin_to_livox.transform, tf_odin_to_livox_);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "Waiting point cloud fusion TFs: %s",
      ex.what());
    return false;
  }

  fusion_transforms_initialized_ = true;
  return true;
}

bool PointCloudDeskewNode::appendCloudAsXyzi(
  const sensor_msgs::msg::PointCloud2 & msg, const tf2::Transform & transform,
  pcl::PointCloud<pcl::PointXYZI> & cloud)
{
  if (!hasFloat32Field(msg, "x") || !hasFloat32Field(msg, "y") || !hasFloat32Field(msg, "z")) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Skip cloud from '%s': x/y/z fields must be float32.", msg.header.frame_id.c_str());
    return false;
  }

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");
  const bool has_intensity = hasFloat32Field(msg, "intensity");
  const std::size_t point_count =
    static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(msg.height);

  if (has_intensity) {
    sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(msg, "intensity");
    for (std::size_t i = 0; i < point_count; ++i) {
      const tf2::Vector3 point = transform * tf2::Vector3(*iter_x, *iter_y, *iter_z);
      if (std::isfinite(point.x()) && std::isfinite(point.y()) && std::isfinite(point.z())) {
        pcl::PointXYZI output_point;
        output_point.x = static_cast<float>(point.x());
        output_point.y = static_cast<float>(point.y());
        output_point.z = static_cast<float>(point.z());
        output_point.intensity = std::isfinite(*iter_intensity) ? *iter_intensity : 0.0F;
        cloud.points.push_back(output_point);
      }

      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_intensity;
    }
    return true;
  }

  for (std::size_t i = 0; i < point_count; ++i) {
    const tf2::Vector3 point = transform * tf2::Vector3(*iter_x, *iter_y, *iter_z);
    if (std::isfinite(point.x()) && std::isfinite(point.y()) && std::isfinite(point.z())) {
      pcl::PointXYZI output_point;
      output_point.x = static_cast<float>(point.x());
      output_point.y = static_cast<float>(point.y());
      output_point.z = static_cast<float>(point.z());
      output_point.intensity = 0.0F;
      cloud.points.push_back(output_point);
    }

    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  return true;
}

bool PointCloudDeskewNode::hasFloat32Field(
  const sensor_msgs::msg::PointCloud2 & msg, const std::string & name) const
{
  for (const auto & field : msg.fields) {
    if (field.name == name && field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
      return true;
    }
  }
  return false;
}

bool PointCloudDeskewNode::lookupBaseToLidar(
  const sensor_msgs::msg::PointCloud2 & cloud, tf2::Transform & base_to_lidar,
  std::string & lidar_frame)
{
  if (base_to_lidar_ready_) {
    base_to_lidar = base_to_lidar_;
    lidar_frame = resolved_lidar_frame_;
    return true;
  }

  lidar_frame = lidar_frame_.empty() ? cloud.header.frame_id : lidar_frame_;
  if (lidar_frame.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Drop cloud: lidar_frame is empty and the input cloud has no frame_id.");
    return false;
  }

  if (base_frame_ == lidar_frame) {
    base_to_lidar_.setIdentity();
    base_to_lidar = base_to_lidar_;
    resolved_lidar_frame_ = lidar_frame;
    base_to_lidar_ready_ = true;
    return true;
  }

  try {
    auto transform = tf_buffer_->lookupTransform(
      base_frame_, lidar_frame, rclcpp::Time(0),
      rclcpp::Duration::from_seconds(tf_lookup_timeout_sec_));
    tf2::fromMsg(transform.transform, base_to_lidar_);
    base_to_lidar = base_to_lidar_;
    resolved_lidar_frame_ = lidar_frame;
    base_to_lidar_ready_ = true;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "Drop cloud: missing TF %s <- %s: %s",
      base_frame_.c_str(), lidar_frame.c_str(), ex.what());
    return false;
  }
}

bool PointCloudDeskewNode::interpolatePose(
  int64_t stamp_ns, std::size_t & cursor, tf2::Transform & odom_to_base) const
{
  if (odom_cache_.empty()) {
    return false;
  }

  if (stamp_ns <= odom_cache_.front().stamp_ns) {
    if (odom_cache_.front().stamp_ns - stamp_ns > max_extrapolation_ns_) {
      return false;
    }
    odom_to_base = odom_cache_.front().odom_to_base;
    return true;
  }

  if (stamp_ns >= odom_cache_.back().stamp_ns) {
    if (stamp_ns - odom_cache_.back().stamp_ns > max_extrapolation_ns_) {
      return false;
    }
    odom_to_base = odom_cache_.back().odom_to_base;
    return true;
  }

  if (
    cursor == 0U || cursor >= odom_cache_.size() || stamp_ns < odom_cache_[cursor - 1U].stamp_ns) {
    cursor = static_cast<std::size_t>(
      std::lower_bound(
        odom_cache_.begin(), odom_cache_.end(), stamp_ns,
        [](const PoseSample & pose, int64_t stamp) { return pose.stamp_ns < stamp; }) -
      odom_cache_.begin());
  }

  while (cursor < odom_cache_.size() && odom_cache_[cursor].stamp_ns < stamp_ns) {
    ++cursor;
  }

  if (cursor == 0U || cursor >= odom_cache_.size()) {
    return false;
  }
  return interpolateBetween(odom_cache_[cursor - 1U], odom_cache_[cursor], stamp_ns, odom_to_base);
}

bool PointCloudDeskewNode::interpolateBetween(
  const PoseSample & before, const PoseSample & after, int64_t stamp_ns,
  tf2::Transform & odom_to_base) const
{
  const int64_t gap_ns = after.stamp_ns - before.stamp_ns;
  if (gap_ns <= 0 || gap_ns > max_odom_gap_ns_) {
    return false;
  }

  const double ratio =
    static_cast<double>(stamp_ns - before.stamp_ns) / static_cast<double>(gap_ns);
  const tf2::Vector3 origin =
    before.odom_to_base.getOrigin() +
    (after.odom_to_base.getOrigin() - before.odom_to_base.getOrigin()) * ratio;
  const tf2::Quaternion rotation = interpolateQuaternion(
    before.odom_to_base.getRotation(), after.odom_to_base.getRotation(), ratio);

  odom_to_base.setOrigin(origin);
  odom_to_base.setRotation(rotation);
  return true;
}

tf2::Quaternion PointCloudDeskewNode::interpolateQuaternion(
  tf2::Quaternion before, tf2::Quaternion after, double ratio) const
{
  before.normalize();
  after.normalize();

  double dot = before.x() * after.x() + before.y() * after.y() + before.z() * after.z() +
               before.w() * after.w();
  if (dot < 0.0) {
    after = tf2::Quaternion(-after.x(), -after.y(), -after.z(), -after.w());
    dot = -dot;
  }

  double before_scale = 1.0 - ratio;
  double after_scale = ratio;
  if (dot < 0.9995) {
    dot = std::min(dot, 1.0);
    const double theta = std::acos(dot);
    const double sin_theta = std::sin(theta);
    before_scale = std::sin((1.0 - ratio) * theta) / sin_theta;
    after_scale = std::sin(ratio * theta) / sin_theta;
  }

  tf2::Quaternion result(
    before.x() * before_scale + after.x() * after_scale,
    before.y() * before_scale + after.y() * after_scale,
    before.z() * before_scale + after.z() * after_scale,
    before.w() * before_scale + after.w() * after_scale);
  result.normalize();
  return result;
}

const std::uint8_t * PointCloudDeskewNode::pointData(
  const sensor_msgs::msg::PointCloud2 & cloud, std::size_t index) const
{
  const std::size_t row = index / cloud.width;
  const std::size_t col = index % cloud.width;
  return cloud.data.data() + row * cloud.row_step + col * cloud.point_step;
}

std::uint8_t * PointCloudDeskewNode::pointData(
  sensor_msgs::msg::PointCloud2 & cloud, std::size_t index) const
{
  const std::size_t row = index / cloud.width;
  const std::size_t col = index % cloud.width;
  return cloud.data.data() + row * cloud.row_step + col * cloud.point_step;
}

int64_t PointCloudDeskewNode::readPointTimeNs(const std::uint8_t * point_data) const
{
  const double stamp_ns = readScalar<double>(point_data, K_LIVOX_TIMESTAMP_OFFSET);
  return static_cast<int64_t>(std::llround(stamp_ns)) + time_offset_ns_;
}

bool PointCloudDeskewNode::validateCloudLayout(const sensor_msgs::msg::PointCloud2 & cloud)
{
  if (cloud.is_bigendian) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Drop cloud: big-endian PointCloud2 is not supported.");
    return false;
  }
  if (cloud.width == 0U || cloud.height == 0U) {
    return true;
  }
  if (cloud.point_step < K_LIVOX_MIN_POINT_STEP) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Drop cloud: expected Livox PointCloud2 layout with timestamp at offset 18.");
    return false;
  }
  const std::size_t minimum_row_step = static_cast<std::size_t>(cloud.width) * cloud.point_step;
  const std::size_t expected_size = static_cast<std::size_t>(cloud.row_step) * cloud.height;
  if (cloud.row_step < minimum_row_step || cloud.data.size() < expected_size) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000, "Drop cloud: malformed PointCloud2 layout.");
    return false;
  }
  return true;
}

int64_t PointCloudDeskewNode::messageTimeToNanoseconds(
  const builtin_interfaces::msg::Time & stamp) const
{
  return static_cast<int64_t>(stamp.sec) * K_NANOSECONDS_PER_SECOND +
         static_cast<int64_t>(stamp.nanosec);
}

builtin_interfaces::msg::Time PointCloudDeskewNode::nanosecondsToMessageTime(int64_t stamp_ns) const
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(stamp_ns / K_NANOSECONDS_PER_SECOND);
  stamp.nanosec = static_cast<std::uint32_t>(stamp_ns % K_NANOSECONDS_PER_SECOND);
  return stamp;
}

}  // namespace sentry_fusion

RCLCPP_COMPONENTS_REGISTER_NODE(sentry_fusion::PointCloudDeskewNode)
