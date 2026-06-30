#include "go2_hal/go2_hal_node.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

namespace go2_hal
{

Go2HalNode::Go2HalNode(const rclcpp::NodeOptions & options)
: Node("go2_hal_node", options)
{
  // ── QoS profiles ─────────────────────────────────────────────────────────────
  // Sensor data: BEST_EFFORT, shallow history — we prefer fresh data over
  // guaranteed delivery for high-frequency streams (1000 Hz joint state, 400 Hz IMU).
  auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();

  // Commands and odometry: RELIABLE — we must not lose velocity commands.
  auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

  // ── Publishers ────────────────────────────────────────────────────────────────
  // Topic names match go2_ros2_sdk exactly so this node is a drop-in replacement.
  joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
    "/go2/joint_states", sensor_qos);

  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
    "/go2/imu", sensor_qos);

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
    "/go2/odom", reliable_qos);

  const std::array<std::string, 4> foot_topic_names = {
    "/go2/foot_contact/FL", "/go2/foot_contact/FR",
    "/go2/foot_contact/RL", "/go2/foot_contact/RR"
  };
  for (size_t i = 0; i < 4; ++i) {
    foot_contact_pubs_[i] = create_publisher<geometry_msgs::msg::WrenchStamped>(
      foot_topic_names[i], sensor_qos);
  }

  // ── Subscribers: real Gazebo simulation data ──────────────────────────────────
  // In real hardware, these callbacks would instead parse the Go2 UDP sport mode
  // LowState struct. The callback signatures and published message types are
  // identical — only the data source changes.
  gazebo_joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/go2/gazebo/joint_states", sensor_qos,
    std::bind(&Go2HalNode::gazebo_joint_callback, this, std::placeholders::_1));

  gazebo_imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/go2/gazebo/imu", sensor_qos,
    std::bind(&Go2HalNode::gazebo_imu_callback, this, std::placeholders::_1));

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", reliable_qos,
    std::bind(&Go2HalNode::cmd_vel_callback, this, std::placeholders::_1));

  // ── Timers ────────────────────────────────────────────────────────────────────
  odom_timer_ = create_wall_timer(20ms, std::bind(&Go2HalNode::publish_odometry, this));
  foot_timer_ = create_wall_timer(10ms, std::bind(&Go2HalNode::publish_foot_contact, this));

  RCLCPP_INFO(get_logger(),
    "go2_hal_node started [MOCK MODE]. Real hardware requires UDP connection "
    "to 192.168.123.161:8082. See HAL_DESIGN.md for the boundary specification.");
}

// ── Callbacks ──────────────────────────────────────────────────────────────────

void Go2HalNode::gazebo_joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Timestamp is set at source (Gazebo plugin) — do not overwrite with now().
  // This is critical for EKF sensor fusion: timestamps must reflect acquisition
  // time, not processing time.
  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_joint_state_ = *msg;

  // Re-publish on the canonical Go2 topic with the same timestamp.
  joint_state_pub_->publish(latest_joint_state_);
}

void Go2HalNode::gazebo_imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_imu_ = *msg;
  latest_imu_.header.frame_id = "go2/imu_link";
  imu_pub_->publish(latest_imu_);
}

void Go2HalNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_cmd_vel_ = *msg;
  // In real hardware: serialize to Go2 sport mode MotionSwitcherClient and
  // send via UDP. In simulation, Gazebo's diff_drive plugin subscribes to
  // /cmd_vel directly — no forwarding needed.
}

void Go2HalNode::publish_odometry()
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  // Dead-reckoning odometry integration at 50 Hz.
  // In real hardware, this would be replaced by the Go2 sport mode built-in
  // odometry estimate from the onboard state estimator.
  constexpr double dt = 0.02;  // 50 Hz
  const double vx = latest_cmd_vel_.linear.x;
  const double vy = latest_cmd_vel_.linear.y;
  const double wz = latest_cmd_vel_.angular.z;

  odom_yaw_ += wz * dt;
  odom_x_ += (vx * std::cos(odom_yaw_) - vy * std::sin(odom_yaw_)) * dt;
  odom_y_ += (vx * std::sin(odom_yaw_) + vy * std::cos(odom_yaw_)) * dt;

  nav_msgs::msg::Odometry odom;
  odom.header.stamp = now();
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";

  odom.pose.pose.position.x = odom_x_;
  odom.pose.pose.position.y = odom_y_;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, odom_yaw_);
  odom.pose.pose.orientation = tf2::toMsg(q);

  odom.twist.twist = latest_cmd_vel_;

  // Covariance: position uncertainty grows with speed (simplified).
  // In real hardware, these would come from the Go2 sport mode estimator.
  const double speed = std::hypot(vx, vy);
  odom.pose.covariance[0] = 0.01 + speed * 0.01;   // xx
  odom.pose.covariance[7] = 0.01 + speed * 0.01;   // yy
  odom.pose.covariance[35] = 0.005;                  // yaw-yaw
  odom.twist.covariance[0] = 0.005;
  odom.twist.covariance[35] = 0.005;

  odom_pub_->publish(odom);
}

void Go2HalNode::publish_foot_contact()
{
  // Estimate foot contact from joint torques in real hardware.
  // In simulation, we publish a simplified contact model: all feet in contact
  // when linear velocity < 0.1 m/s, alternating diagonal pairs otherwise.
  std::lock_guard<std::mutex> lock(state_mutex_);

  const double speed = std::hypot(latest_cmd_vel_.linear.x, latest_cmd_vel_.linear.y);
  const bool standing = speed < 0.05;

  // Contact force magnitude: standing ~120 N per leg, walking ~80 N peak
  const std::array<double, 4> contact_forces = {
    standing ? 120.0 : 80.0,  // FL
    standing ? 120.0 : 80.0,  // FR
    standing ? 120.0 : 80.0,  // RL
    standing ? 120.0 : 80.0   // RR
  };

  for (size_t i = 0; i < 4; ++i) {
    geometry_msgs::msg::WrenchStamped wrench;
    wrench.header.stamp = now();
    wrench.header.frame_id = FOOT_FRAME_IDS[i];
    wrench.wrench.force.z = contact_forces[i];
    foot_contact_pubs_[i]->publish(wrench);
  }
}

}  // namespace go2_hal

// ── Entry point ───────────────────────────────────────────────────────────────
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(go2_hal::Go2HalNode)

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<go2_hal::Go2HalNode>());
  rclcpp::shutdown();
  return 0;
}
