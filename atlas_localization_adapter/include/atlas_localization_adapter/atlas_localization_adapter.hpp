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

#ifndef ATLAS_LOCALIZATION_ADAPTER__ATLAS_LOCALIZATION_ADAPTER_HPP_
#define ATLAS_LOCALIZATION_ADAPTER__ATLAS_LOCALIZATION_ADAPTER_HPP_

#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace atlas_localization_adapter
{

class AtlasLocalizationAdapterNode : public rclcpp::Node
{
public:
  explicit AtlasLocalizationAdapterNode(const rclcpp::NodeOptions & options);

private:
  using PointCloudSyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2,
    sensor_msgs::msg::PointCloud2>;

  struct PendingCloud
  {
    sensor_msgs::msg::PointCloud2::SharedPtr cloud;
    rclcpp::Time received_time{0, 0, RCL_ROS_TIME};
    std::int64_t stamp_key = 0;
  };

  static constexpr std::size_t kHandledStampHistorySize = 64U;

  bool initializeBaseToLidarTransform(
    const rclcpp::Time & stamp);

  bool initializeRobotBaseTransforms(const rclcpp::Time & stamp);

  bool getTransform(
    const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time,
    tf2::Transform & transform);

  void transformTwist(
    const geometry_msgs::msg::Twist & twist_in, const tf2::Transform & tf_in_to_out,
    geometry_msgs::msg::Twist & twist_out) const;

  void publishOdometry(
    const tf2::Transform & transform, const geometry_msgs::msg::Twist & twist,
    const std::string & parent_frame, const std::string & child_frame, const rclcpp::Time & stamp,
    const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & publisher) const;

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  void perceptionCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  void synchronizedPointCloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr primary_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr perception_msg);

  void fusionFallbackTimerCallback();

  bool transformCloudToOdom(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg,
    sensor_msgs::msg::PointCloud2 & out);

  bool transformPerceptionCloudToOdom(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg,
    sensor_msgs::msg::PointCloud2 & out);

  bool mergeClouds(
    const sensor_msgs::msg::PointCloud2 & primary_cloud,
    const sensor_msgs::msg::PointCloud2 & perception_cloud,
    sensor_msgs::msg::PointCloud2 & fused_cloud);

  bool appendCloudAsXyzi(
    const sensor_msgs::msg::PointCloud2 & msg, pcl::PointCloud<pcl::PointXYZI> & cloud);

  bool ensureIntensityField(sensor_msgs::msg::PointCloud2 & cloud);

  bool hasFloat32Field(const sensor_msgs::msg::PointCloud2 & msg, const std::string & name) const;

  void enqueuePendingCloud(
    std::deque<PendingCloud> & pending_clouds, const sensor_msgs::msg::PointCloud2 & cloud,
    const rclcpp::Time & received_time);

  bool removePendingCloudByStamp(
    std::deque<PendingCloud> & pending_clouds, std::int64_t stamp_key);

  void collectTimedOutPendingClouds(
    std::deque<PendingCloud> & pending_clouds, std::deque<std::int64_t> & handled_stamp_keys,
    const rclcpp::Time & now, std::vector<PendingCloud> & timed_out_clouds);

  bool isPendingCloudTimedOut(const PendingCloud & pending_cloud, const rclcpp::Time & now) const;

  void rememberHandledStamp(
    std::deque<std::int64_t> & handled_stamp_keys, std::int64_t stamp_key);

  bool wasHandledRecently(
    const std::deque<std::int64_t> & handled_stamp_keys, std::int64_t stamp_key) const;

  std::int64_t stampKey(const sensor_msgs::msg::PointCloud2 & cloud) const;

  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  void publishTransform(
    const tf2::Transform & transform, const std::string & parent_frame,
    const std::string & child_frame, const rclcpp::Time & stamp);

  void publishRobotBaseJoint(const double robot_base_yaw, const rclcpp::Time & stamp);

  std::string input_odom_topic_;
  std::string input_cloud_topic_;
  std::string input_perception_cloud_topic_;
  std::string registered_scan_topic_;
  std::string lidar_odom_topic_;
  std::string robot_base_odom_topic_;
  std::string base_yaw_joint_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string robot_base_frame_;
  std::string robot_base_odom_frame_;
  std::string lidar_frame_;
  std::string perception_lidar_frame_;
  double tf_lookup_timeout_sec_ = 0.05;
  double perception_cloud_fusion_time_tolerance_sec_ = 0.15;
  double perception_cloud_fallback_timeout_sec_ = 0.2;

  bool perception_enable_ = false;
  bool base_to_lidar_initialized_ = false;
  bool robot_base_transforms_initialized_ = false;

  mutable std::mutex cloud_mutex_;
  std::deque<PendingCloud> pending_registered_clouds_;
  std::deque<PendingCloud> pending_perception_clouds_;
  std::deque<std::int64_t> handled_registered_stamp_keys_;
  std::deque<std::int64_t> handled_perception_stamp_keys_;

  tf2::Transform tf_odom_to_lidar_odom_;
  tf2::Transform tf_lidar_to_perception_lidar_;
  tf2::Transform tf_lidar_to_robot_base_;
  tf2::Transform tf_robot_base_odom_to_base_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr lidar_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr robot_base_odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr base_yaw_joint_pub_;

  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> perception_cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::shared_ptr<message_filters::Synchronizer<PointCloudSyncPolicy>> point_cloud_sync_;
  rclcpp::TimerBase::SharedPtr fusion_fallback_timer_;
};

}  // namespace atlas_localization_adapter

#endif  // ATLAS_LOCALIZATION_ADAPTER__ATLAS_LOCALIZATION_ADAPTER_HPP_
