#include "ur20_display/ur20_display_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fmt/format.h>
#include <fmt/ranges.h>
#include <memory>
#include <numbers>
#include <optional>
#include <random>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "std_msgs/msg/color_rgba.hpp"

namespace {
constexpr auto kTimerPeriod = std::chrono::milliseconds{100};
constexpr auto kValidationThrottleMs = 3000;
constexpr auto kAxisLength = 0.20;
constexpr auto kAxisScale = 0.01;
constexpr auto kLabelHeight = 0.10;
constexpr auto kLabelScale = 0.07;
constexpr auto kVerificationTolerance = 1e-6;

constexpr std::string_view kLogPrefix = "[UR20_DISPLAY]";
constexpr std::string_view kLogBlue = "\033[34m";
constexpr std::string_view kLogReset = "\033[0m";

[[nodiscard]] std::string make_log_line(const std::string_view message)
{
  return fmt::format("{}{}{} {}", kLogBlue, kLogPrefix, kLogReset, message);
}

[[nodiscard]] geometry_msgs::msg::Point make_point(const Eigen::Vector3d& point)
{
  geometry_msgs::msg::Point msg;
  msg.x = point.x();
  msg.y = point.y();
  msg.z = point.z();
  return msg;
}

[[nodiscard]] std_msgs::msg::ColorRGBA make_color(const float red, const float green,
                                                  const float blue, const float alpha = 1.0f)
{
  std_msgs::msg::ColorRGBA color;
  color.r = red;
  color.g = green;
  color.b = blue;
  color.a = alpha;
  return color;
}

struct AxisVisual {
  Eigen::Vector3d tip;
  std_msgs::msg::ColorRGBA color;
};

[[nodiscard]] bool is_verified(const UR20DisplayNode::VerificationResult& result)
{
  return result.position_error < kVerificationTolerance &&
         result.rotation_error < kVerificationTolerance;
}
} // namespace

UR20DisplayNode::UR20DisplayNode() : Node{"ur20_display_node"}
{
  init_publishers();
  init_tf();
  declare_parameters();
  load_parameters();
  log_configuration();
  initialize_animation();

  m_timer = create_wall_timer(kTimerPeriod, [this] { timer_callback(); });
}

void UR20DisplayNode::init_publishers()
{
  m_joint_pub = create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  m_marker_pub =
      create_publisher<visualization_msgs::msg::MarkerArray>("/ur20_display_markers", 10);
}

void UR20DisplayNode::init_tf()
{
  m_tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
  m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
}

void UR20DisplayNode::declare_parameters()
{
  declare_parameter<std::vector<std::string>>(
      "joint_names", {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint",
                      "wrist_2_joint", "wrist_3_joint"});

  declare_parameter<std::vector<double>>("joint_positions", {0.0, -1.57, 1.57, -1.57, 1.57, 0.0});

  declare_parameter<std::string>("world_frame", "world");
  declare_parameter<std::string>("elbow_frame", "forearm_link");
  declare_parameter<std::string>("gripper_frame", "gripper_link");

  declare_parameter<bool>("enable_animation", true);
  declare_parameter<double>("trajectory_periods", 1.5);
  declare_parameter<int>("trajectory_points_per_period", 100);
}

void UR20DisplayNode::load_parameters()
{
  m_joint_names = get_parameter("joint_names").as_string_array();
  m_joint_positions = get_parameter("joint_positions").as_double_array();
  m_world_frame = get_parameter("world_frame").as_string();
  m_elbow_frame = get_parameter("elbow_frame").as_string();
  m_gripper_frame = get_parameter("gripper_frame").as_string();

  m_enable_animation = get_parameter("enable_animation").as_bool();
  m_trajectory_periods = get_parameter("trajectory_periods").as_double();
  m_trajectory_points_per_period =
      static_cast<int>(get_parameter("trajectory_points_per_period").as_int());
}

void UR20DisplayNode::log(const std::string_view msg, const LogLevel level) const
{
  const std::string line = make_log_line(msg);

  switch (level) {
  case LogLevel::Info:
    RCLCPP_INFO(get_logger(), "%s", line.c_str());
    break;

  case LogLevel::Warn:
    RCLCPP_WARN(get_logger(), "%s", line.c_str());
    break;

  case LogLevel::Error:
    RCLCPP_ERROR(get_logger(), "%s", line.c_str());
    break;

  case LogLevel::Debug:
    RCLCPP_DEBUG(get_logger(), "%s", line.c_str());
    break;
  }
}

void UR20DisplayNode::log_throttle(const std::string_view msg, const LogLevel level,
                                   const int64_t duration_ms)
{
  const std::string line = make_log_line(msg);

  switch (level) {
  case LogLevel::Info:
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), duration_ms, "%s", line.c_str());
    break;

  case LogLevel::Warn:
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), duration_ms, "%s", line.c_str());
    break;

  case LogLevel::Error:
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), duration_ms, "%s", line.c_str());
    break;

  case LogLevel::Debug:
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), duration_ms, "%s", line.c_str());
    break;
  }
}

bool UR20DisplayNode::validate_joint_parameters() const
{
  bool valid = true;

  if (m_joint_names.empty()) {
    log("Parameter 'joint_names' cannot be empty.", LogLevel::Error);
    valid = false;
  } else if (m_joint_names.size() != m_joint_positions.size()) {
    log(fmt::format("Parameter size mismatch: joint_names={}, joint_positions={}",
                    m_joint_names.size(), m_joint_positions.size()),
        LogLevel::Error);
    valid = false;
  }

  return valid;
}

bool UR20DisplayNode::validate_animation_parameters() const
{
  bool valid = true;

  if (m_trajectory_periods <= 0.0) {
    log("Parameter 'trajectory_periods' must be positive.", LogLevel::Error);
    valid = false;
  }

  if (m_trajectory_points_per_period <= 0) {
    log("Parameter 'trajectory_points_per_period' must be positive.", LogLevel::Error);
    valid = false;
  }

  return valid;
}

void UR20DisplayNode::log_configuration() const
{
  log(fmt::format("UR20 Display Node initialized: joints={}, world='{}', elbow='{}', gripper='{}', "
                  "animation={}, trajectory_periods={:.2f}, points_per_period={}",
                  m_joint_names.size(), m_world_frame, m_elbow_frame, m_gripper_frame,
                  m_enable_animation, m_trajectory_periods, m_trajectory_points_per_period),
      LogLevel::Info);
}

void UR20DisplayNode::publish_joint_state()
{
  sensor_msgs::msg::JointState joint_msg;
  joint_msg.header.stamp = now();
  joint_msg.name = m_joint_names;
  joint_msg.position = m_joint_positions;

  m_joint_pub->publish(joint_msg);
}

std::optional<UR20DisplayNode::TransformChain> UR20DisplayNode::lookup_transforms() const
{
  std::optional<TransformChain> chain;

  try {
    chain.emplace(TransformChain{
        .tf_elbow_gripper =
            m_tf_buffer->lookupTransform(m_elbow_frame, m_gripper_frame, rclcpp::Time{0}),

        .tf_world_elbow =
            m_tf_buffer->lookupTransform(m_world_frame, m_elbow_frame, rclcpp::Time{0}),

        .tf_world_gripper =
            m_tf_buffer->lookupTransform(m_world_frame, m_gripper_frame, rclcpp::Time{0})});
  } catch (const tf2::TransformException& ex) {
    log(fmt::format("Transform lookup failed: {}", ex.what()), LogLevel::Debug);
    chain.reset();
  }

  return chain;
}

Eigen::Isometry3d UR20DisplayNode::to_isometry(const geometry_msgs::msg::Transform& transform_msg)
{
  return tf2::transformToEigen(transform_msg);
}

UR20DisplayNode::VerificationResult
UR20DisplayNode::verify_chain(const TransformChain& chain,
                              Eigen::Isometry3d& t_world_gripper_computed) const
{
  const auto t_elbow_gripper = to_isometry(chain.tf_elbow_gripper.transform);
  const auto t_world_elbow = to_isometry(chain.tf_world_elbow.transform);
  const auto t_world_gripper_direct = to_isometry(chain.tf_world_gripper.transform);

  t_world_gripper_computed = t_world_elbow * t_elbow_gripper;

  return VerificationResult{
      .position_error =
          (t_world_gripper_direct.translation() - t_world_gripper_computed.translation()).norm(),
      .rotation_error =
          (t_world_gripper_direct.linear() - t_world_gripper_computed.linear()).norm()};
}

void UR20DisplayNode::publish_frame_and_label(const Eigen::Isometry3d& t_world_gripper_computed)
{
  visualization_msgs::msg::MarkerArray marker_array;

  const Eigen::Vector3d origin = t_world_gripper_computed.translation();
  const Eigen::Matrix3d rotation = t_world_gripper_computed.linear();
  const auto timestamp = now();

  visualization_msgs::msg::Marker axes;
  axes.header.frame_id = m_world_frame;
  axes.header.stamp = timestamp;
  axes.ns = "tf_world_gripper";
  axes.id = 0;
  axes.type = visualization_msgs::msg::Marker::LINE_LIST;
  axes.action = visualization_msgs::msg::Marker::ADD;
  axes.scale.x = kAxisScale;
  axes.color.a = 1.0;

  const std::array<AxisVisual, 3> axis_data{
      AxisVisual{.tip = origin + rotation.col(0) * kAxisLength,
                 .color = make_color(1.0f, 0.0f, 0.0f)},
      AxisVisual{.tip = origin + rotation.col(1) * kAxisLength,
                 .color = make_color(0.0f, 1.0f, 0.0f)},
      AxisVisual{.tip = origin + rotation.col(2) * kAxisLength,
                 .color = make_color(0.0f, 0.0f, 1.0f)}};

  axes.points.reserve(axis_data.size() * 2);
  axes.colors.reserve(axis_data.size() * 2);

  for (const auto& axis : axis_data) {
    axes.points.push_back(make_point(origin));
    axes.points.push_back(make_point(axis.tip));
    axes.colors.push_back(axis.color);
    axes.colors.push_back(axis.color);
  }

  visualization_msgs::msg::Marker label;
  label.header.frame_id = m_world_frame;
  label.header.stamp = timestamp;
  label.ns = "tf_world_gripper";
  label.id = 1;
  label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  label.action = visualization_msgs::msg::Marker::ADD;
  label.pose.position = make_point(origin + Eigen::Vector3d{0.0, 0.0, kLabelHeight});
  label.pose.orientation.w = 1.0;
  label.scale.z = kLabelScale;
  label.color = make_color(1.0f, 1.0f, 1.0f);
  label.text = "Tf_elbow_gripper";

  marker_array.markers.reserve(2);
  marker_array.markers.push_back(std::move(axes));
  marker_array.markers.push_back(std::move(label));

  m_marker_pub->publish(marker_array);
}

void UR20DisplayNode::log_verification(const VerificationResult& result) const
{
  const bool verified = is_verified(result);

  if (verified) {
    log(fmt::format("Transform chain verified. Position error={:.3e}, rotation error={:.3e}",
                    result.position_error, result.rotation_error),
        LogLevel::Debug);
  } else {
    log(fmt::format("Transform chain mismatch. Position error={:.6f}, rotation error={:.6f}",
                    result.position_error, result.rotation_error),
        LogLevel::Warn);
  }
}

std::optional<std::vector<double>> UR20DisplayNode::generate_random_goal_configuration() const
{
  constexpr std::array<std::pair<double, double>, 6> joint_limits{{
      {-std::numbers::pi, std::numbers::pi},
      {-std::numbers::pi, 0.0},
      {-std::numbers::pi, std::numbers::pi},
      {-std::numbers::pi, std::numbers::pi},
      {-std::numbers::pi, std::numbers::pi},
      {-std::numbers::pi, std::numbers::pi},
  }};

  std::optional<std::vector<double>> goal;

  if (m_joint_names.size() == joint_limits.size()) {
    static thread_local std::mt19937 generator{std::random_device{}()};

    goal.emplace();
    goal->reserve(joint_limits.size());

    for (const auto& [min_value, max_value] : joint_limits) {
      std::uniform_real_distribution<double> distribution{min_value, max_value};
      goal->push_back(distribution(generator));
    }

    log(fmt::format("Generated random goal configuration: [{}]", fmt::join(*goal, ", ")),
        LogLevel::Info);
  } else {
    log(fmt::format("Cannot generate UR20 goal configuration: expected 6 joints, got {}.",
                    m_joint_names.size()),
        LogLevel::Error);
  }

  return goal;
}

std::optional<std::vector<std::vector<double>>> UR20DisplayNode::generate_periodic_trajectory(
    const std::vector<double>& start_config, const std::vector<double>& goal_config,
    const double periods, const int points_per_period) const
{
  std::optional<std::vector<std::vector<double>>> trajectory;
  const bool compatible_sizes = start_config.size() == goal_config.size();
  const bool valid_parameters = periods > 0.0 && points_per_period > 0;

  if (compatible_sizes && valid_parameters) {
    const auto total_segments = static_cast<std::size_t>(
        std::max(1, static_cast<int>(std::round(periods * points_per_period))));
    const auto total_points = total_segments + 1U;

    trajectory.emplace();
    trajectory->reserve(total_points);

    // This blend creates a periodic start-goal-start-goal style motion.
    // With 1.5 periods, the robot starts at q0, reaches q_goal, returns to q0,
    // and finishes at q_goal again.
    for (std::size_t i = 0; i < total_points; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(total_segments);
      const double blend = 0.5 * (1.0 - std::cos(2.0 * std::numbers::pi * periods * t));

      std::vector<double> point;
      point.reserve(start_config.size());

      for (std::size_t joint_index = 0; joint_index < start_config.size(); ++joint_index) {
        point.push_back(start_config[joint_index] +
                        blend * (goal_config[joint_index] - start_config[joint_index]));
      }

      trajectory->push_back(std::move(point));
    }

    log(fmt::format(
            "Generated periodic trajectory: {} points, periods={:.2f}, points_per_period={}",
            trajectory->size(), periods, points_per_period),
        LogLevel::Info);
  } else {
    log(fmt::format("Cannot generate trajectory: start_size={}, goal_size={}, periods={:.2f}, "
                    "points_per_period={}",
                    start_config.size(), goal_config.size(), periods, points_per_period),
        LogLevel::Error);
  }

  return trajectory;
}

void UR20DisplayNode::initialize_animation()
{
  m_animation_initialized = true;
  m_animation_finished = false;
  m_trajectory_index = 0;
  m_trajectory.clear();
  m_goal_configuration.clear();
  m_initial_joint_positions = m_joint_positions;

  if (m_enable_animation) {
    const bool can_generate_animation =
        validate_joint_parameters() && validate_animation_parameters();

    if (can_generate_animation) {
      const auto goal_configuration = generate_random_goal_configuration();

      if (goal_configuration.has_value()) {
        m_goal_configuration = *goal_configuration;

        const auto trajectory =
            generate_periodic_trajectory(m_initial_joint_positions, m_goal_configuration,
                                         m_trajectory_periods, m_trajectory_points_per_period);

        if (trajectory.has_value()) {
          m_trajectory = *trajectory;
          log(fmt::format("Trajectory animation initialized with {} points. Timer period is {} ms.",
                          m_trajectory.size(), kTimerPeriod.count()),
              LogLevel::Info);
        } else {
          m_enable_animation = false;
        }
      } else {
        m_enable_animation = false;
      }
    } else {
      m_enable_animation = false;
    }
  }

  if (!m_enable_animation) {
    log("Trajectory animation is disabled; publishing the configured static joint state.",
        LogLevel::Info);
  }
}

void UR20DisplayNode::publish_next_trajectory_point()
{
  if (!m_animation_initialized) {
    initialize_animation();
  }

  if (m_enable_animation && !m_trajectory.empty() && !m_animation_finished) {
    if (m_trajectory_index < m_trajectory.size()) {
      m_joint_positions = m_trajectory[m_trajectory_index];
      publish_joint_state();

      if (m_trajectory_index % 10U == 0U || m_trajectory_index + 1U == m_trajectory.size()) {
        const double progress = 100.0 * static_cast<double>(m_trajectory_index) /
                                static_cast<double>(m_trajectory.size() - 1U);

        log(fmt::format("Animation progress: {:6.2f}% ({}/{})", progress, m_trajectory_index + 1U,
                        m_trajectory.size()),
            LogLevel::Info);
      }

      ++m_trajectory_index;
    } else {
      m_animation_finished = true;
      log("Trajectory animation complete.", LogLevel::Info);
      publish_joint_state();
    }
  } else {
    publish_joint_state();
  }
}

void UR20DisplayNode::timer_callback()
{
  const bool valid_params = validate_joint_parameters();

  if (valid_params) {
    publish_next_trajectory_point();

    const std::optional<TransformChain> chain = lookup_transforms();

    if (chain.has_value()) {
      Eigen::Isometry3d t_world_gripper_computed = Eigen::Isometry3d::Identity();
      const VerificationResult result = verify_chain(*chain, t_world_gripper_computed);

      log_verification(result);
      publish_frame_and_label(t_world_gripper_computed);
    }
  } else {
    log_throttle("Joint parameter validation failed. Skipping publish cycle.", LogLevel::Error,
                 kValidationThrottleMs);
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  const auto node = std::make_shared<UR20DisplayNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
