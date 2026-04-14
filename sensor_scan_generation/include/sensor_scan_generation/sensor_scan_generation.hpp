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

#ifndef SENSOR_SCAN_GENERATION__SENSOR_SCAN_GENERATION_HPP_
#define SENSOR_SCAN_GENERATION__SENSOR_SCAN_GENERATION_HPP_

#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace sensor_scan_generation
{

class SensorScanGenerationNode : public rclcpp::Node
{
public:
  explicit SensorScanGenerationNode(const rclcpp::NodeOptions & options);

private:
  void odometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr & odometry);

  bool getTransform(
    const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time,
    tf2::Transform & transform);

  void publishTransform(
    const tf2::Transform & transform, const std::string & parent_frame,
    const std::string & child_frame, const rclcpp::Time & stamp);

  void publishOdometry(
    const tf2::Transform & transform, const std::string & parent_frame,
    const std::string & child_frame, const rclcpp::Time & stamp);

  void publishRobotBaseJoint(const double robot_base_yaw, const rclcpp::Time & stamp);

  std::string lidar_frame_;
  std::string base_frame_;
  std::string robot_base_frame_;
  std::string robot_base_odom_frame_;
  double tf_lookup_timeout_sec_ = 0.05;

  std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_chassis_odometry_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_base_yaw_joint_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

  tf2::Transform tf_lidar_to_robot_base_;
  tf2::Transform tf_robot_base_odom_to_chassis_;
  tf2::Transform previous_odom_transform_ = tf2::Transform::getIdentity();
  rclcpp::Time previous_odom_stamp_;
  bool has_previous_odom_sample_ = false;

  bool initialized_ = false;
};

}  // namespace sensor_scan_generation

#endif  // SENSOR_SCAN_GENERATION__SENSOR_SCAN_GENERATION_HPP_
