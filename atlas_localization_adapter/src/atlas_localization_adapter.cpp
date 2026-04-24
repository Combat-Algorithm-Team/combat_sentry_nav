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

#include "atlas_localization_adapter/atlas_localization_adapter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>

#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace atlas_localization_adapter
{

AtlasLocalizationAdapterNode::AtlasLocalizationAdapterNode(const rclcpp::NodeOptions & options)
: Node("atlas_localization_adapter", options)
{
  this->declare_parameter<std::string>("input_odometry_topic", "aft_mapped_to_init");
  this->declare_parameter<std::string>("input_cloud_topic", "cloud_registered");
  this->declare_parameter<std::string>("input_perception_cloud_topic", "cloud_registered");
  this->declare_parameter<std::string>("registered_scan_topic", "registered_scan");
  this->declare_parameter<std::string>("lidar_odometry_topic", "lidar_odometry");
  this->declare_parameter<std::string>("robot_base_odometry_topic", "odometry");
  this->declare_parameter<std::string>("base_yaw_joint_topic", "base_yaw_joint_publisher");
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("base_frame", "base_footprint");
  this->declare_parameter<std::string>("robot_base_frame", "base_yaw");
  this->declare_parameter<std::string>("robot_base_odom_frame", "base_yaw_odom");
  this->declare_parameter<std::string>("lidar_frame", "front_mid360");
  this->declare_parameter<std::string>("perception_lidar_frame", "front_mid360");
  this->declare_parameter<bool>("perception_enable", false);
  this->declare_parameter<double>("perception_lidar_fusion_time_tolerance_sec", 0.15);
  this->declare_parameter<double>("perception_lidar_fallback_timeout_sec", 0.2);
  this->declare_parameter<double>("tf_lookup_timeout_sec", 0.05);

  this->get_parameter("input_odometry_topic", input_odom_topic_);
  this->get_parameter("input_cloud_topic", input_cloud_topic_);
  this->get_parameter("input_perception_cloud_topic", input_perception_cloud_topic_);
  this->get_parameter("registered_scan_topic", registered_scan_topic_);
  this->get_parameter("lidar_odometry_topic", lidar_odom_topic_);
  this->get_parameter("robot_base_odometry_topic", robot_base_odom_topic_);
  this->get_parameter("base_yaw_joint_topic", base_yaw_joint_topic_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("robot_base_odom_frame", robot_base_odom_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("perception_lidar_frame", perception_lidar_frame_);
  this->get_parameter("perception_enable", perception_enable_);
  this->get_parameter(
    "perception_lidar_fusion_time_tolerance_sec",
    perception_cloud_fusion_time_tolerance_sec_);
  this->get_parameter(
    "perception_lidar_fallback_timeout_sec",
    perception_cloud_fallback_timeout_sec_);
  this->get_parameter("tf_lookup_timeout_sec", tf_lookup_timeout_sec_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  pcd_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>(registered_scan_topic_, 5);
  lidar_odom_pub_ =
    this->create_publisher<nav_msgs::msg::Odometry>(lidar_odom_topic_, 5);
  robot_base_odom_pub_ =
    this->create_publisher<nav_msgs::msg::Odometry>(robot_base_odom_topic_, 2);
  base_yaw_joint_pub_ =
    this->create_publisher<sensor_msgs::msg::JointState>(base_yaw_joint_topic_, 2);

  const auto point_cloud_qos = rclcpp::QoS(rclcpp::KeepLast(5));
  RCLCPP_INFO(
    this->get_logger(), "Subscribing to registered scan topic: %s", input_cloud_topic_.c_str());
  cloud_sub_.subscribe(this, input_cloud_topic_, point_cloud_qos.get_rmw_qos_profile());
  cloud_sub_.registerCallback(
    std::bind(&AtlasLocalizationAdapterNode::pointCloudCallback, this, std::placeholders::_1));

  if (perception_enable_) {
    RCLCPP_INFO(
      this->get_logger(),
      "Perception LiDAR fusion enabled, subscribing to: %s, sync tolerance: %.3f s, fallback timeout: %.3f s",
      input_perception_cloud_topic_.c_str(), perception_cloud_fusion_time_tolerance_sec_,
      perception_cloud_fallback_timeout_sec_);

    perception_cloud_sub_.subscribe(
      this, input_perception_cloud_topic_, point_cloud_qos.get_rmw_qos_profile());
    perception_cloud_sub_.registerCallback(
      std::bind(
        &AtlasLocalizationAdapterNode::perceptionCloudCallback, this, std::placeholders::_1));

    point_cloud_sync_ = std::make_shared<message_filters::Synchronizer<PointCloudSyncPolicy>>(
      PointCloudSyncPolicy(10), cloud_sub_, perception_cloud_sub_);
    point_cloud_sync_->setMaxIntervalDuration(
      rclcpp::Duration::from_seconds(perception_cloud_fusion_time_tolerance_sec_));
    point_cloud_sync_->registerCallback(
      std::bind(
        &AtlasLocalizationAdapterNode::synchronizedPointCloudCallback, this,
        std::placeholders::_1, std::placeholders::_2));

    const double timer_period_sec = std::max(
      0.01, std::min(0.05, perception_cloud_fallback_timeout_sec_ / 2.0));
    fusion_fallback_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(timer_period_sec),
      std::bind(&AtlasLocalizationAdapterNode::fusionFallbackTimerCallback, this));
  }

  RCLCPP_INFO(
    this->get_logger(), "Primary odometry topic: %s", input_odom_topic_.c_str());
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    input_odom_topic_, 5,
    std::bind(&AtlasLocalizationAdapterNode::odometryCallback, this, std::placeholders::_1));
}

bool AtlasLocalizationAdapterNode::initializeBaseToLidarTransform(
  const rclcpp::Time & stamp)
{
  if (base_to_lidar_initialized_) {
    return true;
  }

  if (!getTransform(base_frame_, lidar_frame_, stamp, tf_odom_to_lidar_odom_)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Drop frame while waiting Atlas odometry alignment TF at %.6f: %s <- %s.",
      stamp.seconds(), base_frame_.c_str(), lidar_frame_.c_str());
    return false;
  }

  if (
    perception_enable_ &&
    !getTransform(lidar_frame_, perception_lidar_frame_, stamp, tf_lidar_to_perception_lidar_))
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Drop frame while waiting Atlas perception TF at %.6f: %s <- %s.", stamp.seconds(),
      lidar_frame_.c_str(), perception_lidar_frame_.c_str());
    return false;
  }

  base_to_lidar_initialized_ = true;
  return true;
}

bool AtlasLocalizationAdapterNode::initializeRobotBaseTransforms(const rclcpp::Time & stamp)
{
  if (robot_base_transforms_initialized_) {
    return true;
  }

  if (!getTransform(lidar_frame_, robot_base_frame_, stamp, tf_lidar_robot_base_)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Skip Atlas nav outputs while waiting TF from pb robot description at %.6f: %s <- %s.",
      stamp.seconds(), robot_base_frame_.c_str(), lidar_frame_.c_str());
    return false;
  }

  if (!getTransform(robot_base_odom_frame_, base_frame_, stamp, tf_robot_base_odom_to_base_)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Skip Atlas nav outputs while waiting TF from pb robot description at %.6f: %s <- %s.",
      stamp.seconds(), base_frame_.c_str(), robot_base_odom_frame_.c_str());
    return false;
  }

  robot_base_transforms_initialized_ = true;
  return true;
}

bool AtlasLocalizationAdapterNode::getTransform(
  const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time,
  tf2::Transform & transform)
{
  try {
    auto transform_stamped = tf_buffer_->lookupTransform(
      target_frame, source_frame, time, rclcpp::Duration::from_seconds(tf_lookup_timeout_sec_));
    tf2::fromMsg(transform_stamped.transform, transform);
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "TF lookup failed for %s <- %s at %.6f: %s", target_frame.c_str(), source_frame.c_str(),
      time.seconds(), ex.what());
    return false;
  }
}

void AtlasLocalizationAdapterNode::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  sensor_msgs::msg::PointCloud2 primary_cloud;
  if (!transformCloudToOdom(msg, primary_cloud)) {
    return;
  }

  if (!perception_enable_) {
    pcd_pub_->publish(primary_cloud);
    return;
  }

  const auto stamp_key = stampKey(primary_cloud);
  const auto received_time = this->get_clock()->now();

  std::lock_guard<std::mutex> lock(cloud_mutex_);
  if (wasHandledRecently(handled_registered_stamp_keys_, stamp_key)) {
    return;
  }

  enqueuePendingCloud(pending_registered_clouds_, primary_cloud, received_time);
}

void AtlasLocalizationAdapterNode::perceptionCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  sensor_msgs::msg::PointCloud2 perception_cloud;
  if (!transformPerceptionCloudToOdom(msg, perception_cloud)) {
    return;
  }

  const auto stamp_key = stampKey(perception_cloud);
  const auto received_time = this->get_clock()->now();

  std::lock_guard<std::mutex> lock(cloud_mutex_);
  if (wasHandledRecently(handled_perception_stamp_keys_, stamp_key)) {
    return;
  }

  enqueuePendingCloud(pending_perception_clouds_, perception_cloud, received_time);
}

void AtlasLocalizationAdapterNode::synchronizedPointCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr primary_msg,
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr perception_msg)
{
  sensor_msgs::msg::PointCloud2 primary_cloud;
  sensor_msgs::msg::PointCloud2 perception_cloud;
  if (!transformCloudToOdom(primary_msg, primary_cloud)) {
    return;
  }
  if (!transformPerceptionCloudToOdom(perception_msg, perception_cloud)) {
    return;
  }

  const auto primary_stamp_key = stampKey(primary_cloud);
  const auto perception_stamp_key = stampKey(perception_cloud);

  bool should_fuse = false;
  bool publish_registered_only = false;
  bool publish_perception_only = false;

  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    const bool registered_handled =
      wasHandledRecently(handled_registered_stamp_keys_, primary_stamp_key);
    const bool perception_handled =
      wasHandledRecently(handled_perception_stamp_keys_, perception_stamp_key);

    if (!registered_handled) {
      removePendingCloudByStamp(pending_registered_clouds_, primary_stamp_key);
    }
    if (!perception_handled) {
      removePendingCloudByStamp(pending_perception_clouds_, perception_stamp_key);
    }

    if (!registered_handled && !perception_handled) {
      rememberHandledStamp(handled_registered_stamp_keys_, primary_stamp_key);
      rememberHandledStamp(handled_perception_stamp_keys_, perception_stamp_key);
      should_fuse = true;
    } else {
      publish_registered_only = !registered_handled;
      publish_perception_only = !perception_handled;

      if (publish_registered_only) {
        rememberHandledStamp(handled_registered_stamp_keys_, primary_stamp_key);
      }
      if (publish_perception_only) {
        rememberHandledStamp(handled_perception_stamp_keys_, perception_stamp_key);
      }
    }
  }

  if (!should_fuse) {
    if (publish_registered_only) {
      pcd_pub_->publish(primary_cloud);
    }
    if (publish_perception_only) {
      pcd_pub_->publish(perception_cloud);
    }
    return;
  }

  sensor_msgs::msg::PointCloud2 fused_cloud;
  if (!mergeClouds(primary_cloud, perception_cloud, fused_cloud)) {
    pcd_pub_->publish(primary_cloud);
    return;
  }

  pcd_pub_->publish(fused_cloud);
}

void AtlasLocalizationAdapterNode::fusionFallbackTimerCallback()
{
  if (!perception_enable_) {
    return;
  }

  std::vector<PendingCloud> timed_out_clouds;
  const auto now = this->get_clock()->now();

  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    collectTimedOutPendingClouds(
      pending_registered_clouds_, handled_registered_stamp_keys_, now, timed_out_clouds);
    collectTimedOutPendingClouds(
      pending_perception_clouds_, handled_perception_stamp_keys_, now, timed_out_clouds);
  }

  std::sort(
    timed_out_clouds.begin(), timed_out_clouds.end(),
    [](const PendingCloud & lhs, const PendingCloud & rhs) {
      return lhs.received_time.nanoseconds() < rhs.received_time.nanoseconds();
    });

  for (const auto & pending_cloud : timed_out_clouds) {
    if (pending_cloud.cloud) {
      pcd_pub_->publish(*pending_cloud.cloud);
    }
  }
}

bool AtlasLocalizationAdapterNode::transformCloudToOdom(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg,
  sensor_msgs::msg::PointCloud2 & out)
{
  if (!base_to_lidar_initialized_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Drop Atlas primary cloud while waiting odometry TF initialization.");
    return false;
  }

  pcl_ros::transformPointCloud(odom_frame_, tf_odom_to_lidar_odom_, *msg, out);
  out.header.stamp = msg->header.stamp;
  out.header.frame_id = odom_frame_;
  return true;
}

bool AtlasLocalizationAdapterNode::transformPerceptionCloudToOdom(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg,
  sensor_msgs::msg::PointCloud2 & out)
{
  if (!base_to_lidar_initialized_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Drop Atlas perception cloud while waiting odometry TF initialization.");
    return false;
  }

  const tf2::Transform tf_odom_to_perception_lidar_odom =
    tf_odom_to_lidar_odom_ * tf_lidar_to_perception_lidar_;
  pcl_ros::transformPointCloud(odom_frame_, tf_odom_to_perception_lidar_odom, *msg, out);
  out.header.stamp = msg->header.stamp;
  out.header.frame_id = odom_frame_;
  return true;
}

bool AtlasLocalizationAdapterNode::mergeClouds(
  const sensor_msgs::msg::PointCloud2 & primary_cloud,
  const sensor_msgs::msg::PointCloud2 & perception_cloud,
  sensor_msgs::msg::PointCloud2 & fused_cloud)
{
  pcl::PointCloud<pcl::PointXYZI> fused_pcl;
  fused_pcl.points.reserve(
    static_cast<std::size_t>(primary_cloud.width) * primary_cloud.height +
    static_cast<std::size_t>(perception_cloud.width) * perception_cloud.height);

  const bool has_primary = appendCloudAsXyzi(primary_cloud, fused_pcl);
  const bool has_perception = appendCloudAsXyzi(perception_cloud, fused_pcl);
  if (!has_primary && !has_perception) {
    return false;
  }

  fused_pcl.width = static_cast<std::uint32_t>(fused_pcl.points.size());
  fused_pcl.height = 1;
  fused_pcl.is_dense = false;

  pcl::toROSMsg(fused_pcl, fused_cloud);
  fused_cloud.header.stamp = primary_cloud.header.stamp;
  fused_cloud.header.frame_id = odom_frame_;
  return true;
}

bool AtlasLocalizationAdapterNode::appendCloudAsXyzi(
  const sensor_msgs::msg::PointCloud2 & msg, pcl::PointCloud<pcl::PointXYZI> & cloud)
{
  if (!hasFloat32Field(msg, "x") || !hasFloat32Field(msg, "y") || !hasFloat32Field(msg, "z")) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Skip cloud from %s because it does not expose float32 x/y/z fields.",
      msg.header.frame_id.c_str());
    return false;
  }

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");
  const std::size_t point_count =
    static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(msg.height);

  for (std::size_t i = 0; i < point_count; ++i, ++iter_x, ++iter_y, ++iter_z) {
    if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
      continue;
    }

    pcl::PointXYZI point;
    point.x = *iter_x;
    point.y = *iter_y;
    point.z = *iter_z;
    point.intensity = 0.0F;
    cloud.points.push_back(point);
  }

  return true;
}

bool AtlasLocalizationAdapterNode::hasFloat32Field(
  const sensor_msgs::msg::PointCloud2 & msg, const std::string & name) const
{
  for (const auto & field : msg.fields) {
    if (field.name == name && field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
      return true;
    }
  }
  return false;
}

void AtlasLocalizationAdapterNode::enqueuePendingCloud(
  std::deque<PendingCloud> & pending_clouds, const sensor_msgs::msg::PointCloud2 & cloud,
  const rclcpp::Time & received_time)
{
  const auto stamp_key = stampKey(cloud);
  for (auto & pending_cloud : pending_clouds) {
    if (pending_cloud.stamp_key == stamp_key) {
      pending_cloud.cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(cloud);
      pending_cloud.received_time = received_time;
      return;
    }
  }

  PendingCloud pending_cloud;
  pending_cloud.cloud = std::make_shared<sensor_msgs::msg::PointCloud2>(cloud);
  pending_cloud.received_time = received_time;
  pending_cloud.stamp_key = stamp_key;
  pending_clouds.push_back(pending_cloud);
}

bool AtlasLocalizationAdapterNode::removePendingCloudByStamp(
  std::deque<PendingCloud> & pending_clouds, const std::int64_t stamp_key)
{
  for (auto it = pending_clouds.begin(); it != pending_clouds.end(); ++it) {
    if (it->stamp_key == stamp_key) {
      pending_clouds.erase(it);
      return true;
    }
  }
  return false;
}

void AtlasLocalizationAdapterNode::collectTimedOutPendingClouds(
  std::deque<PendingCloud> & pending_clouds, std::deque<std::int64_t> & handled_stamp_keys,
  const rclcpp::Time & now, std::vector<PendingCloud> & timed_out_clouds)
{
  while (!pending_clouds.empty()) {
    const auto & pending_cloud = pending_clouds.front();
    if (!isPendingCloudTimedOut(pending_cloud, now)) {
      break;
    }

    if (pending_cloud.cloud && !wasHandledRecently(handled_stamp_keys, pending_cloud.stamp_key)) {
      rememberHandledStamp(handled_stamp_keys, pending_cloud.stamp_key);
      timed_out_clouds.push_back(pending_cloud);
    }

    pending_clouds.pop_front();
  }
}

bool AtlasLocalizationAdapterNode::isPendingCloudTimedOut(
  const PendingCloud & pending_cloud, const rclcpp::Time & now) const
{
  if (!pending_cloud.cloud) {
    return true;
  }

  return (now - pending_cloud.received_time).seconds() >= perception_cloud_fallback_timeout_sec_;
}

void AtlasLocalizationAdapterNode::rememberHandledStamp(
  std::deque<std::int64_t> & handled_stamp_keys, const std::int64_t stamp_key)
{
  handled_stamp_keys.push_back(stamp_key);
  while (handled_stamp_keys.size() > kHandledStampHistorySize) {
    handled_stamp_keys.pop_front();
  }
}

bool AtlasLocalizationAdapterNode::wasHandledRecently(
  const std::deque<std::int64_t> & handled_stamp_keys, const std::int64_t stamp_key) const
{
  for (const auto handled_stamp_key : handled_stamp_keys) {
    if (handled_stamp_key == stamp_key) {
      return true;
    }
  }
  return false;
}

std::int64_t AtlasLocalizationAdapterNode::stampKey(
  const sensor_msgs::msg::PointCloud2 & cloud) const
{
  return rclcpp::Time(cloud.header.stamp).nanoseconds();
}

void AtlasLocalizationAdapterNode::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  const rclcpp::Time stamp = msg->header.stamp;
  if (
    !initializeBaseToLidarTransform(stamp) ||
    !initializeRobotBaseTransforms(stamp))
  {
    return;
  }

  tf2::Transform tf_lidar_odom_to_lidar;
  tf2::fromMsg(msg->pose.pose, tf_lidar_odom_to_lidar);

  const tf2::Transform tf_odom_to_lidar =
    tf_odom_to_lidar_odom_ * tf_lidar_odom_to_lidar;
  const tf2::Transform tf_odom_to_robot_base = tf_odom_to_lidar * tf_lidar_robot_base_;

  const double robot_base_yaw = tf2::getYaw(tf_odom_to_robot_base.getRotation());
  publishRobotBaseJoint(robot_base_yaw, stamp);

  const tf2::Transform tf_robot_base_odom_robot_base(
    tf2::Quaternion(tf2::Vector3(0, 0, 1), robot_base_yaw), tf2::Vector3(0, 0, 0));
  const tf2::Transform tf_odom_to_base =
    (tf_odom_to_robot_base * tf_robot_base_odom_robot_base.inverse()) *
    tf_robot_base_odom_to_base_;
  publishTransform(tf_odom_to_base, odom_frame_, base_frame_, stamp);

  geometry_msgs::msg::Twist lidar_twist;
  transformTwist(msg->twist.twist, tf_odom_to_lidar_odom_, lidar_twist);
  publishOdometry(
    tf_odom_to_lidar, lidar_twist, odom_frame_, lidar_frame_, stamp, lidar_odom_pub_);

  geometry_msgs::msg::Twist robot_base_twist;
  transformTwist(lidar_twist, tf_lidar_robot_base_, robot_base_twist);
  publishOdometry(
    tf_odom_to_robot_base, robot_base_twist, odom_frame_, robot_base_frame_, stamp,
    robot_base_odom_pub_);
}

void AtlasLocalizationAdapterNode::transformTwist(
  const geometry_msgs::msg::Twist & twist_in, const tf2::Transform & tf_in_to_out,
  geometry_msgs::msg::Twist & twist_out) const
{
  const tf2::Vector3 linear_velocity_in(
    twist_in.linear.x, twist_in.linear.y, twist_in.linear.z);
  const tf2::Vector3 angular_velocity_in(
    twist_in.angular.x, twist_in.angular.y, twist_in.angular.z);
  const tf2::Vector3 linear_velocity_out_origin_in =
    linear_velocity_in + angular_velocity_in.cross(tf_in_to_out.getOrigin());
  const auto rotation_in_to_out = tf_in_to_out.getRotation();
  const tf2::Vector3 linear_velocity_out = tf2::quatRotate(
    rotation_in_to_out.inverse(), linear_velocity_out_origin_in);
  const tf2::Vector3 angular_velocity_out = tf2::quatRotate(
    rotation_in_to_out.inverse(), angular_velocity_in);

  twist_out.linear.x = linear_velocity_out.x();
  twist_out.linear.y = linear_velocity_out.y();
  twist_out.linear.z = linear_velocity_out.z();
  twist_out.angular.x = angular_velocity_out.x();
  twist_out.angular.y = angular_velocity_out.y();
  twist_out.angular.z = angular_velocity_out.z();
}

void AtlasLocalizationAdapterNode::publishOdometry(
  const tf2::Transform & transform, const geometry_msgs::msg::Twist & twist,
  const std::string & parent_frame, const std::string & child_frame, const rclcpp::Time & stamp,
  const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & publisher) const
{
  nav_msgs::msg::Odometry out;
  out.header.stamp = stamp;
  out.header.frame_id = parent_frame;
  out.child_frame_id = child_frame;

  const auto & origin = transform.getOrigin();
  out.pose.pose.position.x = origin.x();
  out.pose.pose.position.y = origin.y();
  out.pose.pose.position.z = origin.z();
  out.pose.pose.orientation = tf2::toMsg(transform.getRotation());
  out.twist.twist = twist;

  publisher->publish(out);
}

void AtlasLocalizationAdapterNode::publishTransform(
  const tf2::Transform & transform, const std::string & parent_frame,
  const std::string & child_frame, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header.stamp = stamp;
  transform_msg.header.frame_id = parent_frame;
  transform_msg.child_frame_id = child_frame;
  transform_msg.transform = tf2::toMsg(transform);
  tf_broadcaster_->sendTransform(transform_msg);
}

void AtlasLocalizationAdapterNode::publishRobotBaseJoint(
  const double robot_base_yaw, const rclcpp::Time & stamp)
{
  sensor_msgs::msg::JointState joint_state;
  joint_state.header.stamp = stamp;
  joint_state.name.push_back("base_yaw_joint");
  joint_state.position.push_back(robot_base_yaw);
  base_yaw_joint_pub_->publish(joint_state);
}

}  // namespace atlas_localization_adapter

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(atlas_localization_adapter::AtlasLocalizationAdapterNode)
