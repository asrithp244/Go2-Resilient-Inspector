/**
 * mission_bt_node.cpp
 * ====================
 * BehaviorTree.CPP v4 mission orchestrator for the Go2 resilient inspection system.
 *
 * Behavior tree structure:
 *   PatrolSequence (Sequence)
 *     +-- NavigateToStation{1..5}  (drives robot via /cmd_vel using /odom feedback)
 *     +-- InspectStation{1..5}     (trigger perception + telemetry)
 *     +-- [on fault] -> switch to DEGRADED mode (reduced speed)
 *
 * NavigateToStation publishes geometry_msgs/Twist to /cmd_vel.
 * It reads nav_msgs/Odometry from /odom for pose feedback.
 * A simple proportional controller turns to face the goal then drives forward.
 * Returns SUCCESS when within ARRIVAL_THRESHOLD meters of the target.
 *
 * On a LIDAR fault event (/mission/fault_event), the mission switches to
 * DEGRADED mode: navigation continues at half speed (0.15 m/s vs 0.30 m/s).
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "go2_interfaces/msg/fault_event.hpp"
#include "go2_interfaces/msg/inspection_result.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

using namespace std::chrono_literals;

// ── Global degraded mode flag ─────────────────────────────────────────────────
static std::atomic<bool>  g_degraded_mode{false};
static std::atomic<float> g_confidence_level{0.95f};

// ── Shared robot pose (written by /odom callback, read by BT action) ─────────
struct RobotPose {
  std::atomic<double> x{0.0};
  std::atomic<double> y{0.0};
  std::atomic<double> yaw{0.0};
  std::atomic<bool>   valid{false};  // true once first odom message received
};

// ── Normalize angle to [-pi, pi] ─────────────────────────────────────────────
static double normalize_angle(double a)
{
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

// ── BT Action: Navigate to a waypoint via cmd_vel + odom feedback ─────────────
class NavigateToStation : public BT::StatefulActionNode
{
public:
  // Arrival threshold (metres) — robot considered "at" waypoint within this distance
  static constexpr double ARRIVAL_THRESHOLD   = 0.5;
  // Maximum linear speed (m/s) — normal mode
  static constexpr double MAX_LINEAR_NORMAL   = 0.30;
  // Maximum linear speed (m/s) — degraded mode (half speed)
  static constexpr double MAX_LINEAR_DEGRADED = 0.15;
  // Maximum angular speed (rad/s)
  static constexpr double MAX_ANGULAR         = 0.60;
  // Angular proportional gain
  static constexpr double KP_ANGULAR          = 1.2;
  // Linear proportional gain (capped at max)
  static constexpr double KP_LINEAR           = 0.5;
  // Heading error below which we start driving forward (rad, ~23 deg)
  static constexpr double HEADING_THRESHOLD   = 0.40;

  NavigateToStation(const std::string & name,
                    const BT::NodeConfig & config,
                    rclcpp::Node::SharedPtr node,
                    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub,
                    std::shared_ptr<RobotPose> pose)
  : BT::StatefulActionNode(name, config),
    node_(node),
    cmd_vel_pub_(cmd_vel_pub),
    pose_(pose)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("x"),
      BT::InputPort<double>("y"),
      BT::InputPort<double>("yaw"),
      BT::InputPort<std::string>("station_id"),
    };
  }

  BT::NodeStatus onStart() override
  {
    getInput("x", target_x_);
    getInput("y", target_y_);
    getInput("station_id", station_id_);

    RCLCPP_INFO(node_->get_logger(),
      "[NavigateToStation] -> %s (%.2f, %.2f). Mode: %s",
      station_id_.c_str(), target_x_, target_y_,
      g_degraded_mode.load() ? "DEGRADED" : "NORMAL");

    if (!pose_->valid.load()) {
      RCLCPP_WARN(node_->get_logger(),
        "[NavigateToStation] No odom received yet, waiting...");
    }

    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    // Wait for first odometry message
    if (!pose_->valid.load()) {
      return BT::NodeStatus::RUNNING;
    }

    const double cx   = pose_->x.load();
    const double cy   = pose_->y.load();
    const double cyaw = pose_->yaw.load();

    const double dx   = target_x_ - cx;
    const double dy   = target_y_ - cy;
    const double dist = std::sqrt(dx * dx + dy * dy);

    // ── Arrival check ────────────────────────────────────────────────────────
    if (dist < ARRIVAL_THRESHOLD) {
      stop_robot();
      RCLCPP_INFO(node_->get_logger(),
        "[NavigateToStation] Arrived at %s (%.2f, %.2f). dist=%.3fm. Mode: %s",
        station_id_.c_str(), target_x_, target_y_, dist,
        g_degraded_mode.load() ? "DEGRADED" : "NORMAL");
      return BT::NodeStatus::SUCCESS;
    }

    // ── Proportional controller ───────────────────────────────────────────────
    const double angle_to_goal = std::atan2(dy, dx);
    const double heading_error = normalize_angle(angle_to_goal - cyaw);
    const double max_linear    = g_degraded_mode.load()
                                   ? MAX_LINEAR_DEGRADED
                                   : MAX_LINEAR_NORMAL;

    geometry_msgs::msg::Twist cmd;

    if (std::abs(heading_error) > HEADING_THRESHOLD) {
      // Turn in place first — large heading error
      cmd.angular.z = std::clamp(KP_ANGULAR * heading_error, -MAX_ANGULAR, MAX_ANGULAR);
      cmd.linear.x  = 0.0;
    } else {
      // Drive forward with continuous heading correction
      cmd.linear.x  = std::min(KP_LINEAR * dist, max_linear);
      cmd.angular.z = std::clamp(
        KP_ANGULAR * heading_error * 0.5, -MAX_ANGULAR * 0.5, MAX_ANGULAR * 0.5);
    }

    cmd_vel_pub_->publish(cmd);
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    stop_robot();
    RCLCPP_INFO(node_->get_logger(),
      "[NavigateToStation] Halted at %s — robot stopped.", station_id_.c_str());
  }

private:
  void stop_robot()
  {
    geometry_msgs::msg::Twist stop;
    cmd_vel_pub_->publish(stop);
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  std::shared_ptr<RobotPose> pose_;

  double      target_x_{0.0}, target_y_{0.0};
  std::string station_id_;
};

// ── BT Action: Inspect a station ─────────────────────────────────────────────
class InspectStation : public BT::StatefulActionNode
{
public:
  InspectStation(const std::string & name,
                 const BT::NodeConfig & config,
                 rclcpp::Node::SharedPtr node,
                 rclcpp::Publisher<go2_interfaces::msg::InspectionResult>::SharedPtr result_pub)
  : BT::StatefulActionNode(name, config), node_(node), result_pub_(result_pub) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("station_id"),
      BT::InputPort<double>("x"),
      BT::InputPort<double>("y"),
    };
  }

  BT::NodeStatus onStart() override
  {
    getInput("station_id", station_id_);
    start_time_ = node_->now();
    RCLCPP_INFO(node_->get_logger(),
      "[InspectStation] Inspecting %s. Degraded: %s. Confidence: %.2f",
      station_id_.c_str(),
      g_degraded_mode.load() ? "YES" : "NO",
      g_confidence_level.load());
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    // 2 seconds for perception + telemetry
    if ((node_->now() - start_time_).seconds() < 2.0) {
      return BT::NodeStatus::RUNNING;
    }
    publish_result();
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override {}

private:
  void publish_result()
  {
    go2_interfaces::msg::InspectionResult result;
    result.station_id    = station_id_;
    const auto t         = node_->now();
    result.inspected_at.sec     = static_cast<int32_t>(t.nanoseconds() / 1000000000LL);
    result.inspected_at.nanosec = static_cast<uint32_t>(t.nanoseconds() % 1000000000LL);
    result.degraded_mode = g_degraded_mode.load();
    result.confidence    = g_confidence_level.load();

    // Anomaly seeding: stations 2 and 4 have known anomalies in simulation.
    if (station_id_ == "station_2" || station_id_ == "station_4") {
      result.telemetry_anomaly_detected = true;
      result.anomaly_description = (station_id_ == "station_2")
        ? "high_vibration_level=0.87; error_code=0x03"
        : "motor_temp_c=94.2 (threshold=80); visual_warning_light=RED";
      result.visual_anomaly_detected = (station_id_ == "station_4");
    } else {
      result.telemetry_anomaly_detected = false;
      result.visual_anomaly_detected    = false;
      result.anomaly_description        = "nominal";
    }

    result_pub_->publish(result);
    RCLCPP_INFO(node_->get_logger(),
      "[InspectStation] %s: anomaly=%s, confidence=%.2f",
      station_id_.c_str(),
      (result.telemetry_anomaly_detected || result.visual_anomaly_detected) ? "YES" : "NO",
      result.confidence);
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<go2_interfaces::msg::InspectionResult>::SharedPtr result_pub_;
  std::string station_id_;
  rclcpp::Time start_time_;
};

// ── BT Condition: Check if in normal navigation mode ─────────────────────────
class IsNormalMode : public BT::ConditionNode
{
public:
  IsNormalMode(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    return g_degraded_mode.load() ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }
};

// ── Main node ─────────────────────────────────────────────────────────────────
class MissionBTNode : public rclcpp::Node
{
public:
  MissionBTNode()
  : Node("mission_bt_node")
  {
    // ── Parameters ─────────────────────────────────────────────────────────
    this->declare_parameter("bt_xml_file",         "");
    this->declare_parameter("tick_rate_hz",        10.0);
    this->declare_parameter("normal_confidence",   0.95f);
    this->declare_parameter("degraded_confidence", 0.60f);
    this->declare_parameter("enable_groot2",       false);

    const float normal_conf = this->get_parameter("normal_confidence").as_double();
    degraded_confidence_    = this->get_parameter("degraded_confidence").as_double();
    g_confidence_level.store(normal_conf);

    // ── Shared pose ────────────────────────────────────────────────────────
    pose_ = std::make_shared<RobotPose>();

    // ── QoS ────────────────────────────────────────────────────────────────
    auto reliable_qos = rclcpp::QoS(10).reliable();
    auto sensor_qos   = rclcpp::QoS(5).best_effort();

    // ── Publishers ─────────────────────────────────────────────────────────
    result_pub_  = create_publisher<go2_interfaces::msg::InspectionResult>(
      "/mission/inspection_log", reliable_qos);

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(10).reliable());

    // ── Odometry subscriber ─────────────────────────────────────────────────
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", sensor_qos,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        handle_odom(msg);
      });

    // ── Fault event subscriber ──────────────────────────────────────────────
    fault_sub_ = create_subscription<go2_interfaces::msg::FaultEvent>(
      "/mission/fault_event", reliable_qos,
      [this](const go2_interfaces::msg::FaultEvent::SharedPtr msg) {
        handle_fault(msg);
      });

    // ── Defer tree build until after constructor (shared_from_this() requires it) ──
    init_timer_ = create_wall_timer(
      std::chrono::milliseconds(1),
      [this]() {
        init_timer_->cancel();
        build_tree();
      });

    // ── Mission tick timer ─────────────────────────────────────────────────
    const double tick_rate = this->get_parameter("tick_rate_hz").as_double();
    tick_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / tick_rate),
      std::bind(&MissionBTNode::tick_tree, this));

    RCLCPP_INFO(get_logger(),
      "mission_bt_node started. Ticking at %.1f Hz. "
      "Listening on /odom, publishing /cmd_vel.", tick_rate);
  }

private:
  // ── /odom callback — extract x, y, yaw ──────────────────────────────────
  void handle_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const auto & p = msg->pose.pose.position;
    const auto & q = msg->pose.pose.orientation;

    // Convert quaternion to yaw (Z-up, 2D planar assumption)
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    const double yaw       = std::atan2(siny_cosp, cosy_cosp);

    pose_->x.store(p.x);
    pose_->y.store(p.y);
    pose_->yaw.store(yaw);
    pose_->valid.store(true);
  }

  void handle_fault(const go2_interfaces::msg::FaultEvent::SharedPtr msg)
  {
    RCLCPP_WARN(get_logger(),
      "FAULT EVENT: subsystem='%s', type='%s', action='%s'",
      msg->subsystem.c_str(), msg->fault_type.c_str(), msg->recovery_action_taken.c_str());

    if (msg->subsystem == "lidar" && !g_degraded_mode.load()) {
      g_degraded_mode.store(true);
      g_confidence_level.store(static_cast<float>(degraded_confidence_));
      RCLCPP_WARN(get_logger(),
        "Switching to DEGRADED navigation mode. "
        "Speed capped at %.2f m/s. Confidence = %.2f",
        NavigateToStation::MAX_LINEAR_DEGRADED, degraded_confidence_);
    }
  }

  void build_tree()
  {
    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<IsNormalMode>("IsNormalMode");

    factory.registerBuilder<NavigateToStation>(
      "NavigateToStation",
      [this](const std::string & name, const BT::NodeConfig & config) {
        return std::make_unique<NavigateToStation>(
          name, config, shared_from_this(), cmd_vel_pub_, pose_);
      });

    factory.registerBuilder<InspectStation>(
      "InspectStation",
      [this](const std::string & name, const BT::NodeConfig & config) {
        return std::make_unique<InspectStation>(
          name, config, shared_from_this(), result_pub_);
      });

    const std::string bt_file = this->get_parameter("bt_xml_file").as_string();
    if (!bt_file.empty()) {
      tree_ = factory.createTreeFromFile(bt_file);
    } else {
      tree_ = factory.createTreeFromText(get_embedded_bt_xml());
    }

    if (this->get_parameter("enable_groot2").as_bool()) {
      groot2_pub_ = std::make_unique<BT::Groot2Publisher>(tree_);
      RCLCPP_INFO(get_logger(), "Groot2 publisher enabled on default port 1667.");
    }

    RCLCPP_INFO(get_logger(), "Behavior tree built. Waiting for /odom before navigating...");
  }

  void tick_tree()
  {
    if (mission_complete_) {return;}

    const auto status = tree_.tickOnce();

    if (status == BT::NodeStatus::SUCCESS) {
      mission_complete_ = true;
      // Ensure robot is stopped at end of mission
      geometry_msgs::msg::Twist stop;
      cmd_vel_pub_->publish(stop);
      RCLCPP_INFO(get_logger(),
        "=== Mission COMPLETE. All stations inspected. Final mode: %s. ===",
        g_degraded_mode.load() ? "DEGRADED" : "NORMAL");
      tick_timer_->cancel();
    } else if (status == BT::NodeStatus::FAILURE) {
      geometry_msgs::msg::Twist stop;
      cmd_vel_pub_->publish(stop);
      RCLCPP_ERROR(get_logger(), "Mission FAILED. Tree returned FAILURE.");
      tick_timer_->cancel();
    }
  }

  static std::string get_embedded_bt_xml()
  {
    // Waypoints form a square patrol route around the origin.
    // Home (0,0) -> station_1 (3,0) -> station_2 (3,3)
    //            -> station_3 (0,3) -> station_4 (-3,3) -> station_5 (-3,0) -> home
    // 3m legs so the patrol completes in reasonable simulation time.
    return R"(
<root BTCPP_format="4">
  <BehaviorTree ID="InspectionMission">
    <Sequence name="PatrolSequence">

      <!-- Station 1: forward 3m -->
      <NavigateToStation station_id="station_1" x="3.0" y="0.0" yaw="0.0"/>
      <InspectStation    station_id="station_1" x="3.0" y="0.0"/>

      <!-- Station 2 (seeded anomaly: high vibration + error 0x03) -->
      <NavigateToStation station_id="station_2" x="3.0" y="3.0" yaw="1.57"/>
      <InspectStation    station_id="station_2" x="3.0" y="3.0"/>

      <!-- Station 3 -->
      <NavigateToStation station_id="station_3" x="0.0" y="3.0" yaw="3.14"/>
      <InspectStation    station_id="station_3" x="0.0" y="3.0"/>

      <!-- Station 4 (seeded anomaly: motor overtemp + visual warning) -->
      <NavigateToStation station_id="station_4" x="-3.0" y="3.0" yaw="3.14"/>
      <InspectStation    station_id="station_4" x="-3.0" y="3.0"/>

      <!-- Station 5 -->
      <NavigateToStation station_id="station_5" x="-3.0" y="0.0" yaw="4.71"/>
      <InspectStation    station_id="station_5" x="-3.0" y="0.0"/>

      <!-- Return home -->
      <NavigateToStation station_id="home" x="0.0" y="0.0" yaw="0.0"/>

    </Sequence>
  </BehaviorTree>
</root>
)";
  }

  // ── Members ───────────────────────────────────────────────────────────────
  rclcpp::Publisher<go2_interfaces::msg::InspectionResult>::SharedPtr result_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr              cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr             odom_sub_;
  rclcpp::Subscription<go2_interfaces::msg::FaultEvent>::SharedPtr     fault_sub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::TimerBase::SharedPtr init_timer_;

  std::shared_ptr<RobotPose> pose_;

  BT::Tree                             tree_;
  std::unique_ptr<BT::Groot2Publisher> groot2_pub_;

  bool   mission_complete_{false};
  double degraded_confidence_{0.60};
};

// ── Entry point ───────────────────────────────────────────────────────────────
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MissionBTNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
