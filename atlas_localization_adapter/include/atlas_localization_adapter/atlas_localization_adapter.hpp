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

#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
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
  bool initializeBaseToLidarTransform(const rclcpp::Time & stamp);

  bool initializeRobotBaseTransforms(const rclcpp::Time & stamp);

  bool getTransform(
    const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time,
    tf2::Transform & transform);

  void transformTwist(
    const tf2::Vector3 & linear_velocity_in, const tf2::Vector3 & angular_velocity_in,
    const tf2::Transform & tf_in_to_out, tf2::Vector3 & linear_velocity_out,
    tf2::Vector3 & angular_velocity_out) const;

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  void publishOdometry(
    const tf2::Transform & transform, const tf2::Vector3 & linear_velocity,
    const tf2::Vector3 & angular_velocity, const std::string & parent_frame,
    const std::string & child_frame, const rclcpp::Time & stamp,
    const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & publisher) const;

  void publishTransform(
    const tf2::Transform & transform, const std::string & parent_frame,
    const std::string & child_frame, const rclcpp::Time & stamp);

  void publishRobotBaseJoint(const double robot_base_yaw, const rclcpp::Time & stamp);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr lidar_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr robot_base_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr base_odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr base_yaw_joint_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string input_odom_topic_;
  std::string input_cloud_topic_;
  std::string registered_scan_topic_;
  std::string lidar_odom_topic_;
  std::string robot_base_odom_topic_;
  std::string base_odom_topic_;
  std::string base_yaw_joint_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string robot_base_frame_;
  std::string robot_base_odom_frame_;
  std::string lidar_frame_;
  double tf_lookup_timeout_sec_ = 0.05;

  bool base_to_lidar_initialized_ = false;
  bool robot_base_transforms_initialized_ = false;

  tf2::Transform tf_base_to_lidar_;
  tf2::Transform tf_lidar_robot_base_;
  tf2::Transform tf_robot_base_odom_to_base_;
};

}  // namespace atlas_localization_adapter

#endif  // ATLAS_LOCALIZATION_ADAPTER__ATLAS_LOCALIZATION_ADAPTER_HPP_
