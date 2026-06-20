#include <string>
#include <string_view>

#include "linear_algebra_service/srv/least_square_contract.hpp"
#include "rclcpp/rclcpp.hpp"

/*_________________________________________________________________________________________________*/
class ServerNode : public rclcpp::Node {
  using LeastSquareContract = linear_algebra_service::srv::LeastSquareContract;
  using RequestPtr = std::shared_ptr<LeastSquareContract::Request>;
  using ResponsePtr = std::shared_ptr<LeastSquareContract::Response>;

public:
  enum class LogLevel { Info, Warn, Error };

  explicit ServerNode(std::string node_name);

  [[nodiscard]] bool init_service(std::string service_name);
  void log(std::string_view msg, LogLevel level = LogLevel::Info) const;

private:
  void handle_request(RequestPtr const request, ResponsePtr response);

  rclcpp::Service<LeastSquareContract>::SharedPtr m_service;
};

/*_________________________________________________________________________________________________*/
ServerNode::ServerNode(std::string node_name) : Node(node_name) {}

/*_________________________________________________________________________________________________*/
void ServerNode::log(std::string_view msg, LogLevel level) const
{
  static constexpr const char* kRed = "\033[31m";
  static constexpr const char* kReset = "\033[0m";
  const std::string line = std::string(kRed) + "[SERVER]" + kReset + " " + std::string(msg);

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
  }
}

/*_________________________________________________________________________________________________*/
bool ServerNode::init_service(std::string service_name)
{
  using namespace std::placeholders;

  if (!m_service) {
    m_service = create_service<LeastSquareContract>(
        service_name, std::bind(&ServerNode::handle_request, this, _1, _2));
  }

  return m_service != nullptr;
}

/*_________________________________________________________________________________________________*/
void ServerNode::handle_request(RequestPtr const request, ResponsePtr response)
{
  log("Received request with " + std::to_string(request->a_rows.size()) + " row(s).");
  response->success = true;
  response->message = "Request processed successfully.";
}

/*_________________________________________________________________________________________________*/
int main(int argc, char** argv)
{
  int ret_code = 0;

  rclcpp::init(argc, argv);
  auto server = std::make_shared<ServerNode>("linear_algebra_server");

  if (!server) {
    RCLCPP_ERROR(rclcpp::get_logger("linear_algebra_server"), "Server node is null.");
    ret_code = 1;
  } else if (!server->init_service("least_square_service")) {
    server->log("Failed to initialize the service.", ServerNode::LogLevel::Error);
    ret_code = 1;
  } else {
    server->log("Service initialized successfully.");
    rclcpp::spin(server);
  }

  rclcpp::shutdown();

  return ret_code;
}

/*_________________________________________________________________________________________________*/