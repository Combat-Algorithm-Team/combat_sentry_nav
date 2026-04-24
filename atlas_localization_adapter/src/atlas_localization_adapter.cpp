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

#include "pcl_ros/transforms.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace atlas_localization_adapter
{

AtlasLocalizationAdapterNode::AtlasLocalizationAdapterNode(const rclcpp::NodeOptions & options)
: Node("atlas_localization_adapter", options)
{
  this->declare_parameter<std::string>("input_odometry_topic", "aft_mapped_to_init");
  this->declare_parameter<std::string>("input_registered_scan_topic", "cloud_registered");
  this->declare_parameter<std::string>("input_perception_lidar_topic", "cloud_registered");
  this->declare_parameter<std::string>("registered_scan_topic", "registered_scan");
  this->declare_parameter<std::string>("lidar_odometry_topic", "lidar_odometry");
  this->declare_parameter<std::string>("robot_base_odometry_topic", "odometry");
  this->declare_parameter<std::string>("base_frame_odometry_topic", "base_frame_odometry");
  this->declare_parameter<std::string>("base_yaw_joint_topic", "base_yaw_joint_publisher");
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("base_frame", "base_footprint");
  this->declare_parameter<std::string>("robot_base_frame", "base_yaw");
  this->declare_parameter<std::string>("robot_base_odom_frame", "base_yaw_odom");
  this->declare_parameter<std::string>("lidar_frame", "front_mid360");
  this->declare_parameter<std::string>("perception_lidar_frame", "front_mid360");
  this->declare_parameter<bool>("perception_lidar_enabled", false);
  this->declare_parameter<double>("tf_lookup_timeout_sec", 0.05);

  this->get_parameter("input_odometry_topic", input_odom_topic_);
  this->get_parameter("input_registered_scan_topic", input_cloud_topic_);
  this->get_parameter("input_perception_lidar_topic", input_perception_lidar_topic_);
  this->get_parameter("registered_scan_topic", registered_scan_topic_);
  this->get_parameter("lidar_odometry_topic", lidar_odom_topic_);
  this->get_parameter("robot_base_odometry_topic", robot_base_odom_topic_);
  this->get_parameter("base_frame_odometry_topic", base_odom_topic_);
  this->get_parameter("base_yaw_joint_topic", base_yaw_joint_topic_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("robot_base_odom_frame", robot_base_odom_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("perception_lidar_frame", perception_lidar_frame_);
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
  base_odom_pub_ =
    this->create_publisher<nav_msgs::msg::Odometry>(base_odom_topic_, 2);
  base_yaw_joint_pub_ =
    this->create_publisher<sensor_msgs::msg::JointState>(base_yaw_joint_topic_, 2);
  
  bool perception_lidar_enabled = false;
  this->get_parameter("perception_lidar_enabled", perception_lidar_enabled);
  if (perception_lidar_enabled) {
    RCLCPP_INFO(this->get_logger(), "Subscribing to perception LiDAR topic: %s", input_perception_lidar_topic_.c_str());
    pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_perception_lidar_topic_, 5,
      std::bind(&AtlasLocalizationAdapterNode::pointCloudCallback, this, std::placeholders::_1));
  } else {
    RCLCPP_INFO(this->get_logger(), "Subscribing to registered scan topic: %s", input_cloud_topic_.c_str());
    
  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, 5,
    std::bind(&AtlasLocalizationAdapterNode::pointCloudCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    input_odom_topic_, 5,
    std::bind(&AtlasLocalizationAdapterNode::odometryCallback, this, std::placeholders::_1));
}

bool AtlasLocalizationAdapterNode::initializeBaseToLidarTransform(const rclcpp::Time & stamp)
{
  if (base_to_lidar_initialized_) {
    return true;
  }

  try {
    auto tf_stamped = tf_buffer_->lookupTransform(
      base_frame_, lidar_frame_, stamp, rclcpp::Duration::from_seconds(0.5));
    tf2::fromMsg(tf_stamped.transform, tf_base_to_lidar_);
    base_to_lidar_initialized_ = true;
    return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Drop frame while waiting Atlas alignment TF from pb robot description at %.6f: %s <- %s. %s",
      stamp.seconds(), base_frame_.c_str(), lidar_frame_.c_str(), ex.what());
    return false;
  }
}

bool AtlasLocalizationAdapterNode::initializeRobotBaseTransforms(const rclcpp::Time & stamp)
{
  if (robot_base_transforms_initialized_) {
    return true;
  }

  tf2::Transform tf_lidar_robot_base;
  tf2::Transform tf_robot_base_odom_to_base;
  const bool has_lidar_robot_base =
    getTransform(lidar_frame_, robot_base_frame_, stamp, tf_lidar_robot_base);
  const bool has_robot_base_odom_to_base = getTransform(
    robot_base_odom_frame_, base_frame_, stamp, tf_robot_base_odom_to_base);

  if (!has_lidar_robot_base || !has_robot_base_odom_to_base) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Skip Atlas nav outputs while waiting TFs from pb robot description at %.6f: %s <- %s, %s <- %s.",
      stamp.seconds(), robot_base_frame_.c_str(), lidar_frame_.c_str(),
      base_frame_.c_str(), robot_base_odom_frame_.c_str());
    return false;
  }

  tf_lidar_robot_base_ = tf_lidar_robot_base;
  tf_robot_base_odom_to_base_ = tf_robot_base_odom_to_base;
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

void AtlasLocalizationAdapterNode::transformTwist(
  const tf2::Vector3 & linear_velocity_in, const tf2::Vector3 & angular_velocity_in,
  const tf2::Transform & tf_in_to_out, tf2::Vector3 & linear_velocity_out,
  tf2::Vector3 & angular_velocity_out) const
{
  const tf2::Vector3 linear_velocity_out_origin_in =
    linear_velocity_in + angular_velocity_in.cross(tf_in_to_out.getOrigin());
  const auto rotation_in_to_out = tf_in_to_out.getRotation();
  linear_velocity_out = tf2::quatRotate(
    rotation_in_to_out.inverse(), linear_velocity_out_origin_in);
  angular_velocity_out = tf2::quatRotate(rotation_in_to_out.inverse(), angular_velocity_in);
}

void AtlasLocalizationAdapterNode::publishOdometry(
  const tf2::Transform & transform, const tf2::Vector3 & linear_velocity,
  const tf2::Vector3 & angular_velocity, const std::string & parent_frame,
  const std::string & child_frame, const rclcpp::Time & stamp,
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

  out.twist.twist.linear.x = linear_velocity.x();
  out.twist.twist.linear.y = linear_velocity.y();
  out.twist.twist.linear.z = linear_velocity.z();
  out.twist.twist.angular.x = angular_velocity.x();
  out.twist.twist.angular.y = angular_velocity.y();
  out.twist.twist.angular.z = angular_velocity.z();

  publisher->publish(out);
}

void AtlasLocalizationAdapterNode::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (!base_to_lidar_initialized_ && !initializeBaseToLidarTransform(msg->header.stamp)) {
    return;
  }

  sensor_msgs::msg::PointCloud2 out;
  pcl_ros::transformPointCloud(odom_frame_, tf_base_to_lidar_, *msg, out);
  pcd_pub_->publish(out);
}

void AtlasLocalizationAdapterNode::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  const rclcpp::Time stamp = msg->header.stamp;

  if (!initializeBaseToLidarTransform(stamp)) {
    return;
  }

  tf2::Transform tf_input_to_base;
  tf2::fromMsg(msg->pose.pose, tf_input_to_base);
  const tf2::Transform tf_odom_to_lidar = tf_base_to_lidar_ * tf_input_to_base;

  const tf2::Vector3 linear_velocity_base(
    msg->twist.twist.linear.x,
    msg->twist.twist.linear.y,
    msg->twist.twist.linear.z);
  const tf2::Vector3 angular_velocity_base(
    msg->twist.twist.angular.x,
    msg->twist.twist.angular.y,
    msg->twist.twist.angular.z);

  tf2::Vector3 linear_velocity_lidar;
  tf2::Vector3 angular_velocity_lidar;
  transformTwist(
    linear_velocity_base, angular_velocity_base, tf_base_to_lidar_, linear_velocity_lidar,
    angular_velocity_lidar);
  publishOdometry(
    tf_odom_to_lidar, linear_velocity_lidar, angular_velocity_lidar, odom_frame_, lidar_frame_,
    stamp, lidar_odom_pub_);

  if (!initializeRobotBaseTransforms(stamp)) {
    return;
  }

  tf2::Transform tf_odom_to_robot_base;
  tf2::Transform tf_odom_to_robot_base_odom;
  tf2::Transform tf_robot_base_odom_robot_base;
  tf2::Transform tf_odom_to_base;
  tf2::Transform tf_lidar_to_base;

  tf_odom_to_robot_base = tf_odom_to_lidar * tf_lidar_robot_base_;

  const double robot_base_yaw = tf2::getYaw(tf_odom_to_robot_base.getRotation());
  publishRobotBaseJoint(robot_base_yaw, stamp);

  tf_robot_base_odom_robot_base = tf2::Transform(
    tf2::Quaternion(tf2::Vector3(0, 0, 1), robot_base_yaw), tf2::Vector3(0, 0, 0));
  tf_odom_to_robot_base_odom =
    tf_odom_to_robot_base * tf_robot_base_odom_robot_base.inverse();
  tf_odom_to_base = tf_odom_to_robot_base_odom * tf_robot_base_odom_to_base_;
  tf_lidar_to_base = tf_odom_to_lidar.inverse() * tf_odom_to_base;

  publishTransform(tf_odom_to_base, odom_frame_, base_frame_, stamp);

  tf2::Vector3 linear_velocity_robot_base;
  tf2::Vector3 angular_velocity_robot_base;
  transformTwist(
    linear_velocity_lidar, angular_velocity_lidar, tf_lidar_robot_base_,
    linear_velocity_robot_base, angular_velocity_robot_base);
  publishOdometry(
    tf_odom_to_robot_base, linear_velocity_robot_base, angular_velocity_robot_base, odom_frame_,
    robot_base_frame_, stamp, robot_base_odom_pub_);

  tf2::Vector3 linear_velocity_base_frame;
  tf2::Vector3 angular_velocity_base_frame;
  transformTwist(
    linear_velocity_lidar, angular_velocity_lidar, tf_lidar_to_base, linear_velocity_base_frame,
    angular_velocity_base_frame);
  publishOdometry(
    tf_odom_to_base, linear_velocity_base_frame, angular_velocity_base_frame, odom_frame_,
    base_frame_, stamp, base_odom_pub_);
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
