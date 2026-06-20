#include <chrono>
#include <fmt/format.h>
#include <functional>
#include <optional>
#include <string>
#include <string_view>

#include <Eigen/Geometry>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "linear_algebra_service/srv/least_square_contract.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

/*_________________________________________________________________________________________________*/
class ClientNode : public rclcpp::Node {
  using LeastSquareContract = linear_algebra_service::srv::LeastSquareContract;
  using RequestPtr = std::shared_ptr<LeastSquareContract::Request>;
  using ServiceFuture = rclcpp::Client<LeastSquareContract>::SharedFuture;

public:
  enum class LogLevel { Info, Warn, Error };

  explicit ClientNode(std::string node_name);

  [[nodiscard]] bool init_client(std::string service_name);
  [[nodiscard]] bool init_publisher(std::string topic_name);
  bool send_request(const std::string& config_path);
  void log(std::string_view msg, LogLevel level = LogLevel::Info) const;

private:
  static Eigen::Vector3d to_eigen(const geometry_msgs::msg::Vector3& v);
  static Eigen::Quaterniond to_eigen(const geometry_msgs::msg::Quaternion& q);
  void handle_response(ServiceFuture future);
  std::optional<YAML::Node> load_file(const std::string& config_path) const;
  bool validate_yaml_schema(const YAML::Node& config) const;
  std::optional<RequestPtr> parse_yaml_file(const YAML::Node& config) const;
  void dispatch_request(const RequestPtr& request, const std::string& config_path);

  rclcpp::Client<LeastSquareContract>::SharedPtr m_client;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr m_publisher;
};

/*_________________________________________________________________________________________________*/
ClientNode::ClientNode(std::string node_name) : Node(node_name) {}

/*_________________________________________________________________________________________________*/
void ClientNode::log(std::string_view msg, LogLevel level) const
{
  static constexpr const char* kGreen = "\033[32m";
  static constexpr const char* kReset = "\033[0m";
  const std::string ros_line = fmt::format("{}[CLIENT]{} {}", kGreen, kReset, msg);

  switch (level) {
  case LogLevel::Info:
    RCLCPP_INFO(get_logger(), "%s", ros_line.c_str());
    break;
  case LogLevel::Warn:
    RCLCPP_WARN(get_logger(), "%s", ros_line.c_str());
    break;
  case LogLevel::Error:
    RCLCPP_ERROR(get_logger(), "%s", ros_line.c_str());
    break;
  }
}

/*_________________________________________________________________________________________________*/
bool ClientNode::init_client(std::string service_name)
{
  m_client = create_client<LeastSquareContract>(service_name);

  while (!m_client->wait_for_service(std::chrono::seconds(1))) {
    log(fmt::format("Waiting for service '{}'...", service_name));
  }

  return m_client != nullptr;
}

/*_________________________________________________________________________________________________*/
bool ClientNode::init_publisher(std::string topic_name)
{
  m_publisher = create_publisher<geometry_msgs::msg::Vector3>(topic_name, 10);
  log(fmt::format("Publishing on topic '{}'.", topic_name));
  return m_publisher != nullptr;
}

/*_________________________________________________________________________________________________*/
bool ClientNode::send_request(const std::string& config_path)
{
  bool sent = false;

  std::optional<YAML::Node> config = load_file(config_path);

  if (config) {
    auto request = parse_yaml_file(*config);
    if (request) {
      dispatch_request(*request, config_path);
      sent = true;
    }
  }

  return sent;
}

/*_________________________________________________________________________________________________*/
std::optional<YAML::Node> ClientNode::load_file(const std::string& config_path) const
{
  std::optional<YAML::Node> loaded_config = std::nullopt;

  try {
    loaded_config = YAML::LoadFile(config_path);
  } catch (const YAML::Exception& e) {
    log(fmt::format("Failed to load config '{}': {}", config_path, e.what()), LogLevel::Error);
  }

  return loaded_config;
}

/*_________________________________________________________________________________________________*/
bool ClientNode::validate_yaml_schema(const YAML::Node& config) const
{
  bool valid = true;

  if (!config["a_rows"] || !config["a_rows"].IsSequence()) {
    log("Invalid schema: 'a_rows' must be a sequence of [x, y, z] rows.", LogLevel::Error);
    valid = false;
  }

  if (valid && config["a_rows"].size() < 3) {
    log("Invalid schema: 'a_rows' must contain at least 3 rows.", LogLevel::Error);
    valid = false;
  }

  if (valid) {
    std::size_t row_index = 0;
    for (const auto& row : config["a_rows"]) {
      const bool row_ok = row.IsSequence() && row.size() == 3;
      if (!row_ok) {
        log(fmt::format("Invalid schema: 'a_rows[{}]' must be a sequence with exactly 3 values.",
                        row_index),
            LogLevel::Error);
        valid = false;
        break;
      }
      ++row_index;
    }
  }

  if (!config["b"] || !config["b"].IsSequence()) {
    log("Invalid schema: 'b' must be a sequence.", LogLevel::Error);
    valid = false;
  }

  if (valid && config["b"].size() != config["a_rows"].size()) {
    log("Invalid schema: 'b' size must match number of rows in 'a_rows'.", LogLevel::Error);
    valid = false;
  }

  return valid;
}

/*_________________________________________________________________________________________________*/
std::optional<ClientNode::RequestPtr> ClientNode::parse_yaml_file(const YAML::Node& config) const
{
  std::optional<RequestPtr> parsed_request = std::nullopt;

  const bool schema_valid = validate_yaml_schema(config);

  if (schema_valid) {
    try {
      auto request = std::make_shared<LeastSquareContract::Request>();

      for (const auto& row : config["a_rows"]) {
        geometry_msgs::msg::Point p;
        p.x = row[0].as<double>();
        p.y = row[1].as<double>();
        p.z = row[2].as<double>();
        request->a_rows.push_back(p);
      }

      for (const auto& b_value : config["b"]) {
        request->b.push_back(b_value.as<double>());
      }

      parsed_request = request;
    } catch (const YAML::Exception& e) {
      log(fmt::format("Failed to parse config: {}", e.what()), LogLevel::Error);
    }
  }

  return parsed_request;
}

/*_________________________________________________________________________________________________*/
void ClientNode::dispatch_request(const RequestPtr& request, const std::string& config_path)
{
  log(fmt::format("Sending request with {} row(s) from '{}'.", request->a_rows.size(),
                  config_path));
  m_client->async_send_request(
      request, std::bind(&ClientNode::handle_response, this, std::placeholders::_1));
}

/*_________________________________________________________________________________________________*/
Eigen::Vector3d ClientNode::to_eigen(const geometry_msgs::msg::Vector3& v)
{
  return Eigen::Vector3d(v.x, v.y, v.z);
}

/*_________________________________________________________________________________________________*/
Eigen::Quaterniond ClientNode::to_eigen(const geometry_msgs::msg::Quaternion& q)
{
  return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
}

/*_________________________________________________________________________________________________*/
void ClientNode::handle_response(ServiceFuture future)
{
  auto response = future.get();
  log(fmt::format("Server response: success={}, message='{}'", response->success ? "true" : "false",
                  response->message));

  if (response->success) {
    const Eigen::Vector3d x_prime = to_eigen(response->x_prime);
    const Eigen::Vector3d d_prime = to_eigen(response->d_prime);
    const Eigen::Quaterniond r_prime = to_eigen(response->r_prime).normalized();

    const Eigen::Vector3d x = r_prime.inverse() * (x_prime - d_prime);

    log(fmt::format("Recovered x: [{:.6f}, {:.6f}, {:.6f}]", x.x(), x.y(), x.z()));

    if (m_publisher) {
      geometry_msgs::msg::Vector3 msg;
      msg.x = x.x();
      msg.y = x.y();
      msg.z = x.z();
      m_publisher->publish(msg);
      log(fmt::format("Published x: [{:.6f}, {:.6f}, {:.6f}] to topic.", x.x(), x.y(), x.z()));
    }
  } else {
    log("Skipping reverse transform due to failed server response.", LogLevel::Warn);
  }

  rclcpp::shutdown();
}

/*_________________________________________________________________________________________________*/
int main(int argc, char** argv)
{
  int ret_code = 0;

  rclcpp::init(argc, argv);
  auto client = std::make_shared<ClientNode>("linear_algebra_client");

  if (!client) {
    RCLCPP_ERROR(rclcpp::get_logger("linear_algebra_client"), "Client node is null.");
    ret_code = 1;
  } else if (argc < 2) {
    client->log("Usage: client_node <config.yaml>", ClientNode::LogLevel::Error);
    ret_code = 1;
  } else if (!client->init_client("least_square_service")) {
    client->log("Failed to initialize client.", ClientNode::LogLevel::Error);
    ret_code = 1;
  } else if (!client->init_publisher("least_square_topic")) {
    client->log("Failed to initialize publisher.", ClientNode::LogLevel::Error);
    ret_code = 1;
  } else if (!client->send_request(argv[1])) {
    client->log("Failed to load and send request.", ClientNode::LogLevel::Error);
    ret_code = 1;
  } else {
    rclcpp::spin(client);
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return ret_code;
}

/*_________________________________________________________________________________________________*/
