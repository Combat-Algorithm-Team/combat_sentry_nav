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

#ifndef CMD_VEL_TRANSFORM__CMD_VEL_TRANSFORM_HPP_
#define CMD_VEL_TRANSFORM__CMD_VEL_TRANSFORM_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cmd_vel_transform
{
class CmdVelTransform : public rclcpp::Node
{
public:
  explicit CmdVelTransform(const rclcpp::NodeOptions & options);

private:
  void syncCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom,
    const nav_msgs::msg::Path::ConstSharedPtr & local_plan);
  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void localPlanCallback(const nav_msgs::msg::Path::ConstSharedPtr & msg);
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  void updateOdometryState(
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom, const rclcpp::Time & receive_time);
  bool getYawForCurrentCommand(const rclcpp::Time & now, double & yaw) const;
  bool isZeroVelocity(const geometry_msgs::msg::Twist & twist) const;
  geometry_msgs::msg::Twist transformVelocity(
    const geometry_msgs::msg::Twist & twist, double yaw_diff) const;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_filter_;
  message_filters::Subscriber<nav_msgs::msg::Path> local_plan_sub_filter_;
  using SyncPolicy =
    message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, nav_msgs::msg::Path>;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  std::string robot_base_frame_;
  std::string odom_topic_;
  std::string local_plan_topic_;
  std::string input_cmd_vel_topic_;
  std::string output_cmd_vel_topic_;

  std::mutex cmd_vel_mutex_;
  rclcpp::Time latest_odom_stamp_;
  rclcpp::Time latest_odom_receive_time_;
  double latest_odom_yaw_{0.0};
  bool has_latest_odom_{false};

  rclcpp::Time latest_sync_odom_stamp_;
  rclcpp::Time latest_sync_receive_time_;
  double latest_sync_yaw_{0.0};
  bool has_latest_sync_yaw_{false};

  rclcpp::Time last_controller_activate_time_;
  bool has_controller_activation_time_{false};
};

}  // namespace cmd_vel_transform

#endif  // CMD_VEL_TRANSFORM__CMD_VEL_TRANSFORM_HPP_
