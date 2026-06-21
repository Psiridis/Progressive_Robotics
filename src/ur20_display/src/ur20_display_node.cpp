#include "ur20_display/ur20_display_node.hpp"

#include <string>
#include <utility>

#include "std_msgs/msg/color_rgba.hpp"

UR20DisplayNode::UR20DisplayNode() : Node("ur20_display_node")
{
  init_publishers();
  init_tf();
  declare_parameters();
  load_parameters();
  log_configuration();

  m_timer = create_wall_timer(std::chrono::milliseconds(100),
                              std::bind(&UR20DisplayNode::timer_callback, this));
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
      "joint_names",
      std::vector<std::string>{"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                               "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"});

  declare_parameter<std::vector<double>>("joint_positions",
                                         std::vector<double>{0.0, -1.57, 1.57, -1.57, 1.57, 0.0});

  declare_parameter<std::string>("world_frame", "world");
  declare_parameter<std::string>("elbow_frame", "forearm_link");
  declare_parameter<std::string>("gripper_frame", "gripper_link");
}

void UR20DisplayNode::load_parameters()
{
  m_joint_names = get_parameter("joint_names").as_string_array();
  m_joint_positions = get_parameter("joint_positions").as_double_array();
  m_world_frame = get_parameter("world_frame").as_string();
  m_elbow_frame = get_parameter("elbow_frame").as_string();
  m_gripper_frame = get_parameter("gripper_frame").as_string();
}

bool UR20DisplayNode::validate_joint_parameters() const
{
  if (m_joint_names.empty()) {
    RCLCPP_ERROR(get_logger(), "Parameter 'joint_names' cannot be empty.");
    return false;
  }

  if (m_joint_names.size() != m_joint_positions.size()) {
    RCLCPP_ERROR(get_logger(), "Parameter size mismatch: joint_names=%zu, joint_positions=%zu",
                 m_joint_names.size(), m_joint_positions.size());
    return false;
  }

  return true;
}

void UR20DisplayNode::log_configuration() const
{
  RCLCPP_INFO(get_logger(),
              "UR20 Display Node initialized: joints=%zu, world='%s', elbow='%s', gripper='%s'",
              m_joint_names.size(), m_world_frame.c_str(), m_elbow_frame.c_str(),
              m_gripper_frame.c_str());
}

void UR20DisplayNode::publish_joint_state()
{
  sensor_msgs::msg::JointState joint_msg;
  joint_msg.header.stamp = now();
  joint_msg.name = m_joint_names;
  joint_msg.position = m_joint_positions;
  m_joint_pub->publish(joint_msg);
}

bool UR20DisplayNode::lookup_transforms(TransformChain& chain) const
{
  try {
    chain.tf_elbow_gripper =
        m_tf_buffer->lookupTransform(m_elbow_frame, m_gripper_frame, rclcpp::Time(0));
    chain.tf_world_elbow =
        m_tf_buffer->lookupTransform(m_world_frame, m_elbow_frame, rclcpp::Time(0));
    chain.tf_world_gripper =
        m_tf_buffer->lookupTransform(m_world_frame, m_gripper_frame, rclcpp::Time(0));
    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_DEBUG(get_logger(), "Transform lookup failed: %s", ex.what());
    return false;
  }
}

Eigen::Isometry3d UR20DisplayNode::to_isometry(const geometry_msgs::msg::Transform& transform_msg)
{
  return tf2::transformToEigen(transform_msg);
}

UR20DisplayNode::VerificationResult
UR20DisplayNode::verify_chain(const TransformChain& chain,
                              Eigen::Isometry3d& t_world_gripper_computed) const
{
  const Eigen::Isometry3d t_elbow_gripper = to_isometry(chain.tf_elbow_gripper.transform);
  const Eigen::Isometry3d t_world_elbow = to_isometry(chain.tf_world_elbow.transform);
  const Eigen::Isometry3d t_world_gripper_direct = to_isometry(chain.tf_world_gripper.transform);

  t_world_gripper_computed = t_world_elbow * t_elbow_gripper;

  VerificationResult result;
  result.position_error =
      (t_world_gripper_direct.translation() - t_world_gripper_computed.translation()).norm();
  result.rotation_error =
      (t_world_gripper_direct.linear() - t_world_gripper_computed.linear()).norm();

  return result;
}

void UR20DisplayNode::publish_frame_and_label(const Eigen::Isometry3d& t_world_gripper_computed)
{
  visualization_msgs::msg::MarkerArray marker_array;

  const Eigen::Vector3d origin = t_world_gripper_computed.translation();
  const Eigen::Matrix3d rotation = t_world_gripper_computed.linear();
  const double axis_length = 0.20;

  visualization_msgs::msg::Marker axes;
  axes.header.frame_id = m_world_frame;
  axes.header.stamp = now();
  axes.ns = "tf_world_gripper";
  axes.id = 0;
  axes.type = visualization_msgs::msg::Marker::LINE_LIST;
  axes.action = visualization_msgs::msg::Marker::ADD;
  axes.scale.x = 0.01;
  axes.color.a = 1.0;

  auto make_point = [](const Eigen::Vector3d& p) {
    geometry_msgs::msg::Point msg;
    msg.x = p.x();
    msg.y = p.y();
    msg.z = p.z();
    return msg;
  };

  auto make_color = [](float r, float g, float b, float a) {
    std_msgs::msg::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
  };

  const Eigen::Vector3d x_tip = origin + rotation.col(0) * axis_length;
  const Eigen::Vector3d y_tip = origin + rotation.col(1) * axis_length;
  const Eigen::Vector3d z_tip = origin + rotation.col(2) * axis_length;

  axes.points.push_back(make_point(origin));
  axes.points.push_back(make_point(x_tip));
  axes.colors.push_back(make_color(1.0f, 0.0f, 0.0f, 1.0f));
  axes.colors.push_back(make_color(1.0f, 0.0f, 0.0f, 1.0f));

  axes.points.push_back(make_point(origin));
  axes.points.push_back(make_point(y_tip));
  axes.colors.push_back(make_color(0.0f, 1.0f, 0.0f, 1.0f));
  axes.colors.push_back(make_color(0.0f, 1.0f, 0.0f, 1.0f));

  axes.points.push_back(make_point(origin));
  axes.points.push_back(make_point(z_tip));
  axes.colors.push_back(make_color(0.0f, 0.0f, 1.0f, 1.0f));
  axes.colors.push_back(make_color(0.0f, 0.0f, 1.0f, 1.0f));

  visualization_msgs::msg::Marker label;
  label.header.frame_id = m_world_frame;
  label.header.stamp = now();
  label.ns = "tf_world_gripper";
  label.id = 1;
  label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  label.action = visualization_msgs::msg::Marker::ADD;
  label.pose.position = make_point(origin + Eigen::Vector3d(0.0, 0.0, 0.10));
  label.pose.orientation.w = 1.0;
  label.scale.z = 0.07;
  label.color.a = 1.0;
  label.color.r = 1.0;
  label.color.g = 1.0;
  label.color.b = 1.0;
  label.text = "Tf_elbow_gripper";

  marker_array.markers.push_back(std::move(axes));
  marker_array.markers.push_back(std::move(label));
  m_marker_pub->publish(marker_array);
}

void UR20DisplayNode::log_verification(const VerificationResult& result) const
{
  if (result.position_error < 1e-6 && result.rotation_error < 1e-6) {
    RCLCPP_DEBUG(get_logger(), "Transform chain verified. Position error=%.3e, rotation error=%.3e",
                 result.position_error, result.rotation_error);
    return;
  }

  RCLCPP_WARN(get_logger(), "Transform chain mismatch. Position error=%.6f, rotation error=%.6f",
              result.position_error, result.rotation_error);
}

void UR20DisplayNode::timer_callback()
{
  if (!validate_joint_parameters()) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
                          "Joint parameter validation failed. Skipping publish cycle.");
    return;
  }

  publish_joint_state();

  TransformChain chain;
  if (!lookup_transforms(chain)) {
    return;
  }

  Eigen::Isometry3d t_world_gripper_computed = Eigen::Isometry3d::Identity();
  const VerificationResult result = verify_chain(chain, t_world_gripper_computed);
  log_verification(result);
  publish_frame_and_label(t_world_gripper_computed);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UR20DisplayNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
