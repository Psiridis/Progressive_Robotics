#include <string>

#include "linear_algebra_service/srv/least_square_contract.hpp"
#include "rclcpp/rclcpp.hpp"

/*_________________________________________________________________________________________________*/
class ServerNode : public rclcpp::Node {
  using LeastSquareContract = linear_algebra_service::srv::LeastSquareContract;
  using RequestPtr = std::shared_ptr<LeastSquareContract::Request>;
  using ResponsePtr = std::shared_ptr<LeastSquareContract::Response>;

public:
  explicit ServerNode(std::string node_name);

  [[nodiscard]] bool init_service(std::string service_name);

private:
  void handle_request(RequestPtr const request, ResponsePtr response);

  rclcpp::Service<LeastSquareContract>::SharedPtr m_service;
};

/*_________________________________________________________________________________________________*/
ServerNode::ServerNode(std::string node_name) : Node(node_name) {}

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
  RCLCPP_INFO(this->get_logger(), "Received request with %zu row(s).", request->a_rows.size());
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
    RCLCPP_ERROR(server->get_logger(), "Failed to initialize the service.");
    ret_code = 1;
  } else {
    RCLCPP_INFO(server->get_logger(), "Service initialized successfully.");
    rclcpp::spin(server);
  }

  rclcpp::shutdown();

  return ret_code;
}

/*_________________________________________________________________________________________________*/