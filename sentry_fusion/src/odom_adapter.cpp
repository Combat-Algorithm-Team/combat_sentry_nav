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

#include "sentry_fusion/odom_adapter.hpp"

#include <functional>

#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace sentry_fusion
{

OdomAdapterNode::OdomAdapterNode(const rclcpp::NodeOptions & options)
: Node("odom_adapter", options)
{
  this->declare_parameter<std::string>("input_odometry_topic", "odin1/odometry_highfreq");
  this->declare_parameter<std::string>("lidar_odometry_topic", "lidar_odometry");
  this->declare_parameter<std::string>("robot_base_odometry_topic", "odometry");
  this->declare_parameter<std::string>("base_yaw_joint_topic", "base_yaw_joint_publisher");
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("base_frame", "base_footprint");
  this->declare_parameter<std::string>("robot_base_frame", "base_yaw");
  this->declare_parameter<std::string>("robot_base_odom_frame", "base_yaw_odom");
  this->declare_parameter<std::string>("lidar_frame", "odin1_base_link");
  this->declare_parameter<double>("tf_lookup_timeout_sec", 0.05);
  this->declare_parameter<bool>("use_sim_time", false);

  this->get_parameter("input_odometry_topic", input_odometry_topic_);
  this->get_parameter("lidar_odometry_topic", lidar_odometry_topic_);
  this->get_parameter("robot_base_odometry_topic", robot_base_odometry_topic_);
  this->get_parameter("base_yaw_joint_topic", base_yaw_joint_topic_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("robot_base_odom_frame", robot_base_odom_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("tf_lookup_timeout_sec", tf_lookup_timeout_sec_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  lidar_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(lidar_odometry_topic_, 5);
  robot_base_odom_pub_ =
    this->create_publisher<nav_msgs::msg::Odometry>(robot_base_odometry_topic_, 2);
  base_yaw_joint_pub_ =
    this->create_publisher<sensor_msgs::msg::JointState>(base_yaw_joint_topic_, 2);

  auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(200));
  odom_qos.best_effort();
  odom_qos.durability_volatile();
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    input_odometry_topic_, odom_qos,
    std::bind(&OdomAdapterNode::odometryCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    this->get_logger(), "Convert odometry '%s' into '%s' and '%s', publishing %s -> %s TF.",
    input_odometry_topic_.c_str(), lidar_odometry_topic_.c_str(),
    robot_base_odometry_topic_.c_str(), odom_frame_.c_str(), base_frame_.c_str());
}

bool OdomAdapterNode::initializeBaseToLidarTransform(const rclcpp::Time & stamp)
{
  if (base_to_lidar_initialized_) {
    return true;
  }

  if (!getTransform(base_frame_, lidar_frame_, stamp, tf_odom_to_lidar_odom_)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Drop odometry while waiting alignment TF at %.6f: %s <- %s.", stamp.seconds(),
      base_frame_.c_str(), lidar_frame_.c_str());
    return false;
  }

  base_to_lidar_initialized_ = true;
  return true;
}

bool OdomAdapterNode::initializeRobotBaseTransforms(const rclcpp::Time & stamp)
{
  if (robot_base_transforms_initialized_) {
    return true;
  }

  if (!getTransform(lidar_frame_, robot_base_frame_, stamp, tf_lidar_to_robot_base_)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Skip odometry outputs while waiting TF at %.6f: %s <- %s.", stamp.seconds(),
      lidar_frame_.c_str(), robot_base_frame_.c_str());
    return false;
  }

  if (!getTransform(robot_base_odom_frame_, base_frame_, stamp, tf_robot_base_odom_to_base_)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Skip odometry outputs while waiting TF at %.6f: %s <- %s.", stamp.seconds(),
      robot_base_odom_frame_.c_str(), base_frame_.c_str());
    return false;
  }

  robot_base_transforms_initialized_ = true;
  return true;
}

bool OdomAdapterNode::getTransform(
  const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time,
  tf2::Transform & transform)
{
  try {
    auto transform_stamped = tf_buffer_->lookupTransform(
      target_frame, source_frame, time, rclcpp::Duration::from_seconds(tf_lookup_timeout_sec_));
    tf2::fromMsg(transform_stamped.transform, transform);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000, "TF lookup failed for %s <- %s at %.6f: %s",
      target_frame.c_str(), source_frame.c_str(), time.seconds(), ex.what());
    return false;
  }
}

void OdomAdapterNode::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  const rclcpp::Time stamp = msg->header.stamp;
  if (!initializeBaseToLidarTransform(stamp) || !initializeRobotBaseTransforms(stamp)) {
    return;
  }

  tf2::Transform tf_lidar_odom_to_lidar;
  tf2::fromMsg(msg->pose.pose, tf_lidar_odom_to_lidar);

  const tf2::Transform tf_odom_to_lidar = tf_odom_to_lidar_odom_ * tf_lidar_odom_to_lidar;
  const tf2::Transform tf_odom_to_robot_base = tf_odom_to_lidar * tf_lidar_to_robot_base_;

  const double robot_base_yaw = tf2::getYaw(tf_odom_to_robot_base.getRotation());
  publishRobotBaseJoint(robot_base_yaw, stamp);

  const tf2::Transform tf_robot_base_to_robot_base_odom(
    tf2::Quaternion(tf2::Vector3(0, 0, 1), -robot_base_yaw), tf2::Vector3(0, 0, 0));
  const tf2::Transform tf_odom_to_base =
    tf_odom_to_robot_base * tf_robot_base_to_robot_base_odom * tf_robot_base_odom_to_base_;
  publishTransform(tf_odom_to_base, odom_frame_, base_frame_, stamp);

  geometry_msgs::msg::Twist lidar_twist;
  transformTwist(msg->twist.twist, tf_odom_to_lidar_odom_, lidar_twist);
  publishOdometry(tf_odom_to_lidar, lidar_twist, odom_frame_, lidar_frame_, stamp, lidar_odom_pub_);

  geometry_msgs::msg::Twist robot_base_twist;
  transformTwist(lidar_twist, tf_lidar_to_robot_base_, robot_base_twist);
  publishOdometry(
    tf_odom_to_robot_base, robot_base_twist, odom_frame_, robot_base_frame_, stamp,
    robot_base_odom_pub_);
}

void OdomAdapterNode::transformTwist(
  const geometry_msgs::msg::Twist & twist_in, const tf2::Transform & tf_in_to_out,
  geometry_msgs::msg::Twist & twist_out) const
{
  const tf2::Vector3 linear_velocity_in(twist_in.linear.x, twist_in.linear.y, twist_in.linear.z);
  const tf2::Vector3 angular_velocity_in(
    twist_in.angular.x, twist_in.angular.y, twist_in.angular.z);
  const tf2::Vector3 linear_velocity_out_origin_in =
    linear_velocity_in + angular_velocity_in.cross(tf_in_to_out.getOrigin());
  const auto rotation_in_to_out = tf_in_to_out.getRotation();
  const tf2::Vector3 linear_velocity_out =
    tf2::quatRotate(rotation_in_to_out.inverse(), linear_velocity_out_origin_in);
  const tf2::Vector3 angular_velocity_out =
    tf2::quatRotate(rotation_in_to_out.inverse(), angular_velocity_in);

  twist_out.linear.x = linear_velocity_out.x();
  twist_out.linear.y = linear_velocity_out.y();
  twist_out.linear.z = linear_velocity_out.z();
  twist_out.angular.x = angular_velocity_out.x();
  twist_out.angular.y = angular_velocity_out.y();
  twist_out.angular.z = angular_velocity_out.z();
}

void OdomAdapterNode::publishOdometry(
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

void OdomAdapterNode::publishTransform(
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

void OdomAdapterNode::publishRobotBaseJoint(const double robot_base_yaw, const rclcpp::Time & stamp)
{
  sensor_msgs::msg::JointState joint_state;
  joint_state.header.stamp = stamp;
  joint_state.name.push_back("base_yaw_joint");
  joint_state.position.push_back(robot_base_yaw);
  base_yaw_joint_pub_->publish(joint_state);
}

}  // namespace sentry_fusion

RCLCPP_COMPONENTS_REGISTER_NODE(sentry_fusion::OdomAdapterNode)
