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

#include "sensor_scan_generation/sensor_scan_generation.hpp"

#include <cmath>

#include "tf2/utils.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace sensor_scan_generation
{
namespace
{
double normalizeAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}
}  // namespace

SensorScanGenerationNode::SensorScanGenerationNode(const rclcpp::NodeOptions & options)
: Node("sensor_scan_generation", options)
{
  this->declare_parameter<std::string>("lidar_frame", "");
  this->declare_parameter<std::string>("base_frame", "");
  this->declare_parameter<std::string>("robot_base_frame", "");
  this->declare_parameter<std::string>("robot_base_odom_frame", "");
  this->declare_parameter<double>("tf_lookup_timeout_sec", 0.05);

  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("robot_base_odom_frame", robot_base_odom_frame_);
  this->get_parameter("tf_lookup_timeout_sec", tf_lookup_timeout_sec_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  pub_chassis_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("odometry", 2);
  pub_base_yaw_joint_ = this->create_publisher<sensor_msgs::msg::JointState>("base_yaw_joint", 2);

  auto odom_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  odom_qos.best_effort();
  odom_qos.durability_volatile();
  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "lidar_odometry", odom_qos,
    std::bind(&SensorScanGenerationNode::odometryHandler, this, std::placeholders::_1));
}

void SensorScanGenerationNode::odometryHandler(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odometry_msg)
{
  const rclcpp::Time reference_stamp = odometry_msg->header.stamp;

  if (!initialized_) {
    tf2::Transform tf_lidar_to_robot_base;
    tf2::Transform tf_robot_base_odom_to_chassis;
    const bool has_lidar_to_robot_base =
      getTransform(lidar_frame_, robot_base_frame_, reference_stamp, tf_lidar_to_robot_base);
    const bool has_robot_base_odom_to_chassis = getTransform(
      robot_base_odom_frame_, base_frame_, reference_stamp, tf_robot_base_odom_to_chassis);
    if (!has_lidar_to_robot_base || !has_robot_base_odom_to_chassis) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Drop frame while waiting required TFs at %.6f: %s<- %s, %s <- %s.",
        reference_stamp.seconds(), robot_base_frame_.c_str(), lidar_frame_.c_str(),
        base_frame_.c_str(), robot_base_odom_frame_.c_str());
      return;
    }
    tf_lidar_to_robot_base_ = tf_lidar_to_robot_base;
    tf_robot_base_odom_to_chassis_ = tf_robot_base_odom_to_chassis;
    initialized_ = true;
  }

  tf2::Transform tf_odom_to_chassis;
  tf2::Transform tf_odom_to_robot_base;
  tf2::Transform tf_odom_to_lidar;
  tf2::Transform tf_odom_to_robot_base_odom;
  tf2::Transform tf_robot_base_odom_to_robot_base;

  tf2::fromMsg(odometry_msg->pose.pose, tf_odom_to_lidar);
  tf_odom_to_robot_base = tf_odom_to_lidar * tf_lidar_to_robot_base_;
  
  const double robot_base_yaw = tf2::getYaw(tf_odom_to_robot_base.getRotation());
  publishRobotBaseJoint(robot_base_yaw, reference_stamp);

  tf_robot_base_odom_to_robot_base = tf2::Transform(
    tf2::Quaternion(tf2::Vector3(0, 0, 1), robot_base_yaw), tf2::Vector3(0, 0, 0));
  tf_odom_to_robot_base_odom = tf_odom_to_robot_base * tf_robot_base_odom_to_robot_base.inverse();
  tf_odom_to_chassis = tf_odom_to_robot_base_odom * tf_robot_base_odom_to_chassis_;
  
  publishTransform(tf_odom_to_chassis, odometry_msg->header.frame_id, base_frame_, reference_stamp);

  publishOdometry(
    tf_odom_to_robot_base, odometry_msg->header.frame_id, robot_base_frame_, reference_stamp);
}

bool SensorScanGenerationNode::getTransform(
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

void SensorScanGenerationNode::publishTransform(
  const tf2::Transform & transform, const std::string & parent_frame,
  const std::string & child_frame, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header.stamp = stamp;
  transform_msg.header.frame_id = parent_frame;
  transform_msg.child_frame_id = child_frame;
  transform_msg.transform = tf2::toMsg(transform);
  br_->sendTransform(transform_msg);
}

void SensorScanGenerationNode::publishOdometry(
  const tf2::Transform & transform, const std::string & parent_frame, const std::string & child_frame,
  const rclcpp::Time & stamp)
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

  if (has_previous_odom_sample_) {
    const double dt = (stamp - previous_odom_stamp_).seconds();
    if (dt > 1e-6) {
      const tf2::Vector3 linear_velocity_parent =
        (transform.getOrigin() - previous_odom_transform_.getOrigin()) / dt;
      const tf2::Vector3 linear_velocity_child =
        tf2::quatRotate(transform.getRotation().inverse(), linear_velocity_parent);

      const double previous_yaw = tf2::getYaw(previous_odom_transform_.getRotation());
      const double current_yaw = tf2::getYaw(transform.getRotation());
      const double yaw_rate = normalizeAngle(current_yaw - previous_yaw) / dt;

      out.twist.twist.linear.x = linear_velocity_child.x();
      out.twist.twist.linear.y = linear_velocity_child.y();
      out.twist.twist.linear.z = linear_velocity_child.z();
      out.twist.twist.angular.x = 0.0;
      out.twist.twist.angular.y = 0.0;
      out.twist.twist.angular.z = yaw_rate;
    }
  }

  previous_odom_transform_ = transform;
  previous_odom_stamp_ = stamp;
  has_previous_odom_sample_ = true;

  pub_chassis_odometry_->publish(out);
}

void SensorScanGenerationNode::publishRobotBaseJoint(
  const double robot_base_yaw, const rclcpp::Time & stamp)
{
  sensor_msgs::msg::JointState joint_state;
  joint_state.header.stamp = stamp;
  joint_state.name.push_back("base_yaw_joint");
  joint_state.position.push_back(robot_base_yaw);
  pub_base_yaw_joint_->publish(joint_state);
}

}  // namespace sensor_scan_generation

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(sensor_scan_generation::SensorScanGenerationNode)
