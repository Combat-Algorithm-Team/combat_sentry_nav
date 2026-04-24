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

#include "cmd_vel_transform/cmd_vel_transform.hpp"

#include <cmath>

#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace cmd_vel_transform
{

constexpr double EPSILON = 1e-5;
constexpr double CONTROLLER_TIMEOUT = 0.5;
constexpr double SYNC_YAW_TIMEOUT = 0.2;

CmdVelTransform::CmdVelTransform(const rclcpp::NodeOptions & options)
: Node("cmd_vel_transform", options)
{
  RCLCPP_INFO(get_logger(), "Start CmdVelTransform!");

  this->declare_parameter<std::string>("robot_base_frame", "gimbal_link");
  this->declare_parameter<std::string>("odom_topic", "odom");
  this->declare_parameter<std::string>("local_plan_topic", "local_plan");
  this->declare_parameter<std::string>("input_cmd_vel_topic", "");
  this->declare_parameter<std::string>("output_cmd_vel_topic", "");

  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("odom_topic", odom_topic_);
  this->get_parameter("local_plan_topic", local_plan_topic_);
  this->get_parameter("input_cmd_vel_topic", input_cmd_vel_topic_);
  this->get_parameter("output_cmd_vel_topic", output_cmd_vel_topic_);

  cmd_vel_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, 1);

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    input_cmd_vel_topic_, 10,
    std::bind(&CmdVelTransform::cmdVelCallback, this, std::placeholders::_1));

  odom_sub_filter_.subscribe(this, odom_topic_);
  local_plan_sub_filter_.subscribe(this, local_plan_topic_);
  odom_sub_filter_.registerCallback(
    std::bind(&CmdVelTransform::odometryCallback, this, std::placeholders::_1));
  local_plan_sub_filter_.registerCallback(
    std::bind(&CmdVelTransform::localPlanCallback, this, std::placeholders::_1));

  // In Navigation2 Humble release, cmd_vel is geometry_msgs/Twist without timestamp.
  // Use local_plan only as a controller timing hint; velocity output must not depend on it.
  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(100), odom_sub_filter_, local_plan_sub_filter_);
  sync_->registerCallback(
    std::bind(&CmdVelTransform::syncCallback, this, std::placeholders::_1, std::placeholders::_2));
}

void CmdVelTransform::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (isZeroVelocity(*msg)) {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    return;
  }

  double yaw_diff = 0.0;
  const rclcpp::Time now = this->get_clock()->now();
  {
    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    if (!getYawForCurrentCommand(now, yaw_diff)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Drop non-zero cmd_vel because no odometry has been received on '%s'.",
        odom_topic_.c_str());
      return;
    }
  }

  auto aft_tf_vel = transformVelocity(*msg, yaw_diff);
  cmd_vel_pub_->publish(aft_tf_vel);
}

void CmdVelTransform::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  updateOdometryState(msg, this->get_clock()->now());
}

void CmdVelTransform::localPlanCallback(const nav_msgs::msg::Path::ConstSharedPtr & /*msg*/)
{
  // Keep local_plan as a controller activity hint only. Backup/recovery has no local_plan.
  last_controller_activate_time_ = this->get_clock()->now();
  has_controller_activation_time_ = true;
}

void CmdVelTransform::syncCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg,
  const nav_msgs::msg::Path::ConstSharedPtr & /*local_plan_msg*/)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  const rclcpp::Time receive_time = this->get_clock()->now();
  if (!has_latest_odom_) {
    updateOdometryState(odom_msg, receive_time);
  }

  latest_sync_odom_stamp_ = odom_msg->header.stamp;
  latest_sync_receive_time_ = receive_time;
  latest_sync_yaw_ = tf2::getYaw(odom_msg->pose.pose.orientation);
  has_latest_sync_yaw_ = true;
}

void CmdVelTransform::updateOdometryState(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom, const rclcpp::Time & receive_time)
{
  latest_odom_stamp_ = odom->header.stamp;
  latest_odom_receive_time_ = receive_time;
  latest_odom_yaw_ = tf2::getYaw(odom->pose.pose.orientation);
  has_latest_odom_ = true;
}

bool CmdVelTransform::getYawForCurrentCommand(const rclcpp::Time & now, double & yaw) const
{
  if (!has_latest_odom_) {
    return false;
  }

  yaw = latest_odom_yaw_;

  const bool controller_active_recently =
    has_controller_activation_time_ &&
    (now - last_controller_activate_time_).seconds() <= CONTROLLER_TIMEOUT;
  const bool sync_yaw_recent =
    has_latest_sync_yaw_ && (now - latest_sync_receive_time_).seconds() <= SYNC_YAW_TIMEOUT;
  const bool sync_yaw_not_older_than_latest_odom =
    has_latest_sync_yaw_ &&
    latest_sync_odom_stamp_.nanoseconds() >= latest_odom_stamp_.nanoseconds();

  if (controller_active_recently && sync_yaw_recent && sync_yaw_not_older_than_latest_odom) {
    yaw = latest_sync_yaw_;
  }

  return true;
}

bool CmdVelTransform::isZeroVelocity(const geometry_msgs::msg::Twist & twist) const
{
  return std::abs(twist.linear.x) < EPSILON && std::abs(twist.linear.y) < EPSILON &&
         std::abs(twist.linear.z) < EPSILON && std::abs(twist.angular.x) < EPSILON &&
         std::abs(twist.angular.y) < EPSILON && std::abs(twist.angular.z) < EPSILON;
}

geometry_msgs::msg::Twist CmdVelTransform::transformVelocity(
  const geometry_msgs::msg::Twist & twist, double yaw_diff) const
{
  geometry_msgs::msg::Twist aft_tf_vel;
  aft_tf_vel.angular = twist.angular;
  aft_tf_vel.linear.z = twist.linear.z;
  aft_tf_vel.linear.x =
    twist.linear.x * std::cos(yaw_diff) + twist.linear.y * std::sin(yaw_diff);
  aft_tf_vel.linear.y =
    -twist.linear.x * std::sin(yaw_diff) + twist.linear.y * std::cos(yaw_diff);
  return aft_tf_vel;
}

}  // namespace cmd_vel_transform

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(cmd_vel_transform::CmdVelTransform)
