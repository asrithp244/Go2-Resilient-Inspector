#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <mutex>

namespace go2_hal
{

/**
 * @brief Hardware Abstraction Layer for the Unitree Go2 quadruped.
 *
 * This node replicates the exact topic names, message types, and QoS
 * profiles used by the real go2_ros2_sdk UDP interface. The only delta
 * between this mock and real hardware is the transport layer:
 *   - Real: UDP socket to the Go2 sport mode controller (192.168.123.161:8082)
 *   - Mock: Gazebo JointState + IMU plugin callbacks, simulated foot contact
 *
 * See HAL_DESIGN.md for the full boundary specification.
 */
class Go2HalNode : public rclcpp::Node
{
public:
  explicit Go2HalNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ── Publishers (mirroring go2_ros2_sdk output topics) ──────────────────────
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  // Four foot contact forces, one per leg (FL, FR, RL, RR)
  std::array<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr, 4> foot_contact_pubs_;

  // ── Subscribers ─────────────────────────────────────────────────────────────
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  // ── Gazebo bridge subscribers (simulation input) ────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr gazebo_joint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr gazebo_imu_sub_;

  // ── Timers ──────────────────────────────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr odom_timer_;   // 50 Hz odometry integration
  rclcpp::TimerBase::SharedPtr foot_timer_;   // 100 Hz foot contact estimation

  // ── Internal state ───────────────────────────────────────────────────────────
  std::mutex state_mutex_;
  sensor_msgs::msg::JointState latest_joint_state_;
  sensor_msgs::msg::Imu latest_imu_;
  geometry_msgs::msg::Twist latest_cmd_vel_;

  // Simple forward odometry integration (replaces real Go2 sport mode estimate)
  double odom_x_{0.0};
  double odom_y_{0.0};
  double odom_yaw_{0.0};

  // ── Callbacks ────────────────────────────────────────────────────────────────
  void gazebo_joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void gazebo_imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void publish_odometry();
  void publish_foot_contact();

  // ── Helpers ──────────────────────────────────────────────────────────────────
  static constexpr std::array<const char*, 12> JOINT_NAMES = {
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"
  };

  static constexpr std::array<const char*, 4> FOOT_FRAME_IDS = {
    "FL_foot", "FR_foot", "RL_foot", "RR_foot"
  };
};

}  // namespace go2_hal
