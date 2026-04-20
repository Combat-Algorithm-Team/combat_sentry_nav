#include "fake_vel_transform/fake_vel_transform.hpp"

#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace fake_vel_transform
{

constexpr double EPSILON = 1e-5;
constexpr double CONTROLLER_TIMEOUT = 0.5;

FakeVelTransform::FakeVelTransform(const rclcpp::NodeOptions & options)
: Node("fake_vel_transform", options)
{
  RCLCPP_INFO(get_logger(), "Start FakeVelTransform!");

  this->declare_parameter<std::string>("robot_base_frame", "gimbal_link");
  this->declare_parameter<std::string>("fake_robot_base_frame", "gimbal_link_fake");
  this->declare_parameter<std::string>("odom_topic", "odom");
  this->declare_parameter<std::string>("local_plan_topic", "local_plan");
  this->declare_parameter<std::string>("input_cmd_vel_topic", "");
  this->declare_parameter<std::string>("output_cmd_vel_topic", "");
  this->declare_parameter<std::string>("actual_vel_topic", "actual_cmd_vel");

  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("fake_robot_base_frame", fake_robot_base_frame_);
  this->get_parameter("odom_topic", odom_topic_);
  this->get_parameter("local_plan_topic", local_plan_topic_);
  this->get_parameter("input_cmd_vel_topic", input_cmd_vel_topic_);
  this->get_parameter("output_cmd_vel_topic", output_cmd_vel_topic_);
  this->get_parameter("actual_vel_topic", actual_vel_topic_);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  cmd_vel_chassis_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, 1);

  actual_vel_pub_ =
    this->create_publisher<geometry_msgs::msg::TwistStamped>(actual_vel_topic_, 1);

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    input_cmd_vel_topic_, 10,
    std::bind(&FakeVelTransform::cmdVelCallback, this, std::placeholders::_1));

  odom_sub_filter_.subscribe(this, odom_topic_);
  local_plan_sub_filter_.subscribe(this, local_plan_topic_);
  odom_sub_filter_.registerCallback(
    std::bind(&FakeVelTransform::odometryCallback, this, std::placeholders::_1));
  local_plan_sub_filter_.registerCallback(
    std::bind(&FakeVelTransform::localPlanCallback, this, std::placeholders::_1));

  // In Navigation2 Humble release, the velocity is published by the controller without timestamped.
  // We consider the velocity is published at the same time as local_plan.
  // Therefore, we use ApproximateTime policy to synchronize `cmd_vel` and `odometry`.
  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(100), odom_sub_filter_, local_plan_sub_filter_);
  sync_->registerCallback(
    std::bind(&FakeVelTransform::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

  // 50Hz Timer to send transform from `robot_base_frame` to `fake_robot_base_frame`
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20), std::bind(&FakeVelTransform::publishTransform, this));
}

void FakeVelTransform::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  // NOTE: Haven't synced with local_plan
  const bool controller_active_recently =
    has_controller_activation_time_ &&
    (this->get_clock()->now() - last_controller_activate_time_).seconds() <= CONTROLLER_TIMEOUT;
  if (!controller_active_recently) {
    current_robot_base_angle_ = tf2::getYaw(msg->pose.pose.orientation);
  }
}

void FakeVelTransform::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  const bool is_zero_vel = std::abs(msg->linear.x) < EPSILON && std::abs(msg->linear.y) < EPSILON &&
                           std::abs(msg->angular.z) < EPSILON;
  const bool controller_active_recently =
    has_controller_activation_time_ &&
    (this->get_clock()->now() - last_controller_activate_time_).seconds() <= CONTROLLER_TIMEOUT;
  if (
    is_zero_vel ||
    !controller_active_recently) {
    // If received velocity cannot be synchronized, publish it directly
    auto aft_tf_vel = transformVelocity(msg, current_robot_base_angle_);
    cmd_vel_chassis_pub_->publish(aft_tf_vel);
  } else {
    latest_cmd_vel_ = msg;
  }
}

void FakeVelTransform::localPlanCallback(const nav_msgs::msg::Path::ConstSharedPtr & /*msg*/)
{
  // Consider nav2_controller_server is activated when receiving local_plan
  last_controller_activate_time_ = this->get_clock()->now();
  has_controller_activation_time_ = true;
}

void FakeVelTransform::syncCallback(
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

  cmd_vel_chassis_pub_->publish(aft_tf_vel);
  publishActualVel(odom_msg);
  
}

void FakeVelTransform::publishTransform()
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = robot_base_frame_;
  t.child_frame_id = fake_robot_base_frame_;
  tf2::Quaternion q;
  q.setRPY(0, 0, -current_robot_base_angle_);
  t.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);
}

geometry_msgs::msg::Twist FakeVelTransform::transformVelocity(
  const geometry_msgs::msg::Twist::SharedPtr & twist, float yaw_diff)
{
  geometry_msgs::msg::Twist aft_tf_vel;
  aft_tf_vel.angular.z = twist->angular.z;
  aft_tf_vel.linear.x = twist->linear.x * cos(yaw_diff) + twist->linear.y * sin(yaw_diff);
  aft_tf_vel.linear.y = -twist->linear.x * sin(yaw_diff) + twist->linear.y * cos(yaw_diff);
  return aft_tf_vel;
}

void FakeVelTransform::publishActualVel(const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg)
{
  rclcpp::Time current_time = odom_msg->header.stamp;

  if (!has_last_time_) {
    last_odom_time_ = current_time;
    has_last_time_ = true;
    return;
  }

  double dt = (current_time - last_odom_time_).seconds();
  if (dt <= 1e-4) return; 

  geometry_msgs::msg::TransformStamped transform;
  try {
    // 目标坐标系：当前的 fake_robot_base_frame_
    // 目标时间：current_time
    // 源坐标系：过去的 fake_robot_base_frame_
    // 源时间：last_odom_time_
    // 固定的参考坐标系：odom_topic_ 
    transform = tf_buffer_->lookupTransform(
      fake_robot_base_frame_, current_time,
      fake_robot_base_frame_, last_odom_time_,
      "camera_init", 
      tf2::durationFromSec(0.05));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "TF Lookup failed: %s", ex.what());
    last_odom_time_ = current_time;
    return;
  }

  double v_x = transform.transform.translation.x / dt;
  double v_y = transform.transform.translation.y / dt;
  
  double roll, pitch, yaw;
  tf2::Quaternion q(
    transform.transform.rotation.x, transform.transform.rotation.y,
    transform.transform.rotation.z, transform.transform.rotation.w);
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  double w_z = yaw / dt;

  // 打包发布
  geometry_msgs::msg::TwistStamped actual_vel_msg;
  actual_vel_msg.header.stamp = current_time;
  actual_vel_msg.header.frame_id = fake_robot_base_frame_; 
  actual_vel_msg.twist.linear.x = v_x;
  actual_vel_msg.twist.linear.y = v_y;
  actual_vel_msg.twist.angular.z = w_z;
  
  actual_vel_pub_->publish(actual_vel_msg);

  // 更新时间
  last_odom_time_ = current_time;
}

}  // namespace fake_vel_transform

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::FakeVelTransform)
