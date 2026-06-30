/**
 * joy_teleop_node.cpp
 * ====================
 * PS5 DualSense controller teleoperation for the Go2.
 *
 * Axis mapping (DualSense in XInput mode via jstest-gtk):
 *   Left  stick Y  (axis 1) → linear.x  (forward/back)
 *   Left  stick X  (axis 0) → linear.y  (strafe — if Go2 supports it)
 *   Right stick X  (axis 2) → angular.z (yaw)
 *   Cross  button  (btn 0)  → toggle autonomous mode
 *   Circle button  (btn 1)  → emergency stop (halt all motion)
 *   PS    button   (btn 12) → arm/disarm (publish to /go2/arm topic)
 *
 * Priority: joy_teleop publishes to /cmd_vel with HIGH priority. Mission BT also
 * publishes to /cmd_vel. This is resolved by latching: the last message wins.
 * The toggle callback sets a ROS2 parameter on mission_bt_node to pause ticking.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>

#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

// ── Axis / button indices for Sony DualSense in Linux jstest layout ───────────
constexpr int AXIS_LEFT_Y   = 1;   // forward/back
constexpr int AXIS_LEFT_X   = 0;   // strafe
constexpr int AXIS_RIGHT_X  = 3;   // yaw
constexpr int BTN_CROSS     = 0;   // toggle autonomous
constexpr int BTN_CIRCLE    = 1;   // e-stop
constexpr int BTN_PS        = 12;  // arm/disarm

class JoyTeleopNode : public rclcpp::Node
{
public:
  JoyTeleopNode()
  : Node("joy_teleop_node")
  {
    // ── Parameters ────────────────────────────────────────────────────────
    this->declare_parameter("max_linear_speed",  0.8);   // m/s — Go2 walk limit
    this->declare_parameter("max_angular_speed", 1.0);   // rad/s
    this->declare_parameter("deadband",          0.05);  // ignore stick noise below this
    this->declare_parameter("autonomous_mode",   false); // start in teleop mode

    max_linear_  = this->get_parameter("max_linear_speed").as_double();
    max_angular_ = this->get_parameter("max_angular_speed").as_double();
    deadband_    = this->get_parameter("deadband").as_double();
    autonomous_  = this->get_parameter("autonomous_mode").as_bool();

    // ── QoS ──────────────────────────────────────────────────────────────
    auto reliable_qos = rclcpp::QoS(10).reliable();
    auto sensor_qos   = rclcpp::QoS(5).best_effort();

    // ── Subscribers ───────────────────────────────────────────────────────
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", sensor_qos,
      std::bind(&JoyTeleopNode::joy_callback, this, std::placeholders::_1));

    // ── Publishers ────────────────────────────────────────────────────────
    cmd_vel_pub_    = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", reliable_qos);
    auto_mode_pub_  = create_publisher<std_msgs::msg::Bool>("/mission/autonomous_mode", reliable_qos);

    // ── Watchdog: zero velocity if no joy input for 0.5s (controller dropout) ─
    watchdog_timer_ = create_wall_timer(
      500ms, std::bind(&JoyTeleopNode::watchdog_tick, this));

    RCLCPP_INFO(get_logger(),
      "joy_teleop_node started. Mode: %s. "
      "Cross: toggle auto. Circle: e-stop.",
      autonomous_ ? "AUTONOMOUS" : "TELEOP");
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    last_joy_time_ = now();

    // ── Button handling ───────────────────────────────────────────────────
    // Toggle autonomous mode (Cross button — rising edge only)
    if (msg->buttons.size() > static_cast<size_t>(BTN_CROSS)) {
      const bool cross_pressed = msg->buttons[BTN_CROSS];
      if (cross_pressed && !prev_cross_) {
        autonomous_ = !autonomous_;
        this->set_parameter(rclcpp::Parameter("autonomous_mode", autonomous_));
        std_msgs::msg::Bool mode_msg;
        mode_msg.data = autonomous_;
        auto_mode_pub_->publish(mode_msg);
        RCLCPP_INFO(get_logger(), "Mode toggled: %s",
          autonomous_ ? "AUTONOMOUS" : "TELEOP");
      }
      prev_cross_ = cross_pressed;
    }

    // E-stop (Circle button — immediate zero velocity, stays latched until released)
    if (msg->buttons.size() > static_cast<size_t>(BTN_CIRCLE)) {
      e_stop_ = static_cast<bool>(msg->buttons[BTN_CIRCLE]);
    }

    if (e_stop_) {
      publish_zero_vel();
      return;
    }

    // In autonomous mode, don't publish cmd_vel (let mission BT drive)
    if (autonomous_) {return;}

    // ── Axis reading with deadband ────────────────────────────────────────
    auto apply_deadband = [this](double val) -> double {
      return std::abs(val) < deadband_ ? 0.0 : val;
    };

    double linear_x  = 0.0;
    double linear_y  = 0.0;
    double angular_z = 0.0;

    if (msg->axes.size() > static_cast<size_t>(std::max(AXIS_LEFT_Y, AXIS_RIGHT_X))) {
      // Note: joy axes are typically inverted on the Y axis (push forward = negative)
      linear_x  = apply_deadband(-msg->axes[AXIS_LEFT_Y])  * max_linear_;
      linear_y  = apply_deadband(-msg->axes[AXIS_LEFT_X])  * max_linear_;
      angular_z = apply_deadband(-msg->axes[AXIS_RIGHT_X]) * max_angular_;
    }

    geometry_msgs::msg::Twist twist;
    twist.linear.x  = linear_x;
    twist.linear.y  = linear_y;
    twist.angular.z = angular_z;
    cmd_vel_pub_->publish(twist);
  }

  void watchdog_tick()
  {
    if (!autonomous_ && !e_stop_) {
      const auto dt = (now() - last_joy_time_).seconds();
      if (dt > 0.5) {
        // Controller silent for >500ms — zero velocity (safety)
        publish_zero_vel();
      }
    }
  }

  void publish_zero_vel()
  {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr auto_mode_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  double max_linear_;
  double max_angular_;
  double deadband_;
  bool autonomous_;
  bool e_stop_{false};
  bool prev_cross_{false};
  rclcpp::Time last_joy_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoyTeleopNode>());
  rclcpp::shutdown();
  return 0;
}
