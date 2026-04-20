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
  local_plan_sub_filter_.registerCallback(
    std::bind(&CmdVelTransform::localPlanCallback, this, std::placeholders::_1));

  // In Navigation2 Humble release, cmd_vel is geometry_msgs/Twist without timestamp.
  // Use local_plan timestamp as cmd_vel time proxy, then sync with odometry.
  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(100), odom_sub_filter_, local_plan_sub_filter_);
  sync_->registerCallback(
    std::bind(&CmdVelTransform::syncCallback, this, std::placeholders::_1, std::placeholders::_2));
}

void CmdVelTransform::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);

  const bool is_zero_vel = std::abs(msg->linear.x) < EPSILON && std::abs(msg->linear.y) < EPSILON &&
                           std::abs(msg->angular.z) < EPSILON;
  if (
    is_zero_vel ||
    (this->get_clock()->now() - last_controller_activate_time_).seconds() > CONTROLLER_TIMEOUT) {
    // If current cmd_vel cannot be synchronized, use latest known yaw.
    auto aft_tf_vel = transformVelocity(msg, current_robot_base_angle_);
    cmd_vel_pub_->publish(aft_tf_vel);
  } else {
    latest_cmd_vel_ = msg;
  }
}

void CmdVelTransform::localPlanCallback(const nav_msgs::msg::Path::ConstSharedPtr & /*msg*/)
{
  // Consider nav2_controller_server is activated when receiving local_plan
  last_controller_activate_time_ = this->get_clock()->now();
}

void CmdVelTransform::syncCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg,
  const nav_msgs::msg::Path::ConstSharedPtr & /*local_plan_msg*/)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  geometry_msgs::msg::Twist::SharedPtr current_cmd_vel;
  {
    if (!latest_cmd_vel_) {
      return;
    }
    current_cmd_vel = latest_cmd_vel_;
  }

  current_robot_base_angle_ = tf2::getYaw(odom_msg->pose.pose.orientation);
  float yaw_diff = current_robot_base_angle_;
  auto aft_tf_vel = transformVelocity(current_cmd_vel, yaw_diff);

  cmd_vel_pub_->publish(aft_tf_vel);
}

geometry_msgs::msg::Twist CmdVelTransform::transformVelocity(
  const geometry_msgs::msg::Twist::SharedPtr & twist, float yaw_diff)
{
  geometry_msgs::msg::Twist aft_tf_vel;
  aft_tf_vel.angular.z = twist->angular.z;
  aft_tf_vel.linear.x = twist->linear.x * cos(yaw_diff) + twist->linear.y * sin(yaw_diff);
  aft_tf_vel.linear.y = -twist->linear.x * sin(yaw_diff) + twist->linear.y * cos(yaw_diff);
  return aft_tf_vel;
}

}  // namespace cmd_vel_transform

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(cmd_vel_transform::CmdVelTransform)
