#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

class UR20DisplayNode : public rclcpp::Node {
public:
  enum class LogLevel { Info, Warn, Error, Debug };

  struct TransformChain {
    geometry_msgs::msg::TransformStamped tf_elbow_gripper;
    geometry_msgs::msg::TransformStamped tf_world_elbow;
    geometry_msgs::msg::TransformStamped tf_world_gripper;
  };

  struct VerificationResult {
    double position_error{0.0};
    double rotation_error{0.0};
  };

  UR20DisplayNode();

  void log(std::string_view msg, LogLevel level = LogLevel::Info) const;

  void log_throttle(std::string_view msg, LogLevel level = LogLevel::Info,
                    int64_t duration_ms = 3000);

private:
  void init_publishers();
  void init_tf();
  void declare_parameters();
  void load_parameters();

  [[nodiscard]] bool validate_joint_parameters() const;
  [[nodiscard]] bool validate_animation_parameters() const;
  void log_configuration() const;

  void publish_joint_state();
  [[nodiscard]] std::optional<TransformChain> lookup_transforms() const;

  static Eigen::Isometry3d to_isometry(const geometry_msgs::msg::Transform& transform_msg);

  [[nodiscard]] VerificationResult verify_chain(const TransformChain& chain,
                                                Eigen::Isometry3d& t_world_gripper_computed) const;

  void publish_frame_and_label(const Eigen::Isometry3d& t_world_gripper_computed);
  void log_verification(const VerificationResult& result) const;

  [[nodiscard]] std::optional<std::vector<double>> generate_random_goal_configuration() const;

  [[nodiscard]] std::optional<std::vector<std::vector<double>>>
  generate_periodic_trajectory(const std::vector<double>& start_config,
                               const std::vector<double>& goal_config, double periods,
                               int points_per_period) const;

  void initialize_animation();
  void publish_next_trajectory_point();

  void timer_callback();

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_joint_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_marker_pub;
  rclcpp::TimerBase::SharedPtr m_timer;

  std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;

  std::vector<std::string> m_joint_names;
  std::vector<double> m_joint_positions;

  std::string m_world_frame;
  std::string m_elbow_frame;
  std::string m_gripper_frame;

  bool m_enable_animation{false};
  bool m_animation_initialized{false};
  bool m_animation_finished{false};

  double m_trajectory_periods{1.5};
  int m_trajectory_points_per_period{100};

  std::vector<double> m_initial_joint_positions;
  std::vector<double> m_goal_configuration;
  std::vector<std::vector<double>> m_trajectory;
  std::size_t m_trajectory_index{0};
};
