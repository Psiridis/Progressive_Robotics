#include <chrono>
#include <functional>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "linear_algebra_service/srv/least_square_contract.hpp"
#include "rclcpp/rclcpp.hpp"

/*_________________________________________________________________________________________________*/
class ClientNode : public rclcpp::Node {
  using LeastSquareContract = linear_algebra_service::srv::LeastSquareContract;
  using ServiceFuture = rclcpp::Client<LeastSquareContract>::SharedFuture;

public:
  explicit ClientNode(std::string node_name);

  [[nodiscard]] bool init_client(std::string service_name);
  void send_dummy_request();

private:
  void handle_response(ServiceFuture future);

  rclcpp::Client<LeastSquareContract>::SharedPtr m_client;
};

/*_________________________________________________________________________________________________*/
ClientNode::ClientNode(std::string node_name) : Node(node_name) {}

/*_________________________________________________________________________________________________*/
bool ClientNode::init_client(std::string service_name)
{
  m_client = create_client<LeastSquareContract>(service_name);

  while (!m_client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_INFO(get_logger(), "Waiting for service '%s'...", service_name.c_str());
  }

  return m_client != nullptr;
}

/*_________________________________________________________________________________________________*/
void ClientNode::send_dummy_request()
{
  auto request = std::make_shared<LeastSquareContract::Request>();

  geometry_msgs::msg::Point row;
  row.x = 1.0;
  row.y = 0.0;
  row.z = 0.0;
  request->a_rows.push_back(row);

  row.x = 0.0;
  row.y = 1.0;
  row.z = 0.0;
  request->a_rows.push_back(row);

  row.x = 0.0;
  row.y = 0.0;
  row.z = 1.0;
  request->a_rows.push_back(row);

  request->b.x = 0.0;
  request->b.y = 0.0;
  request->b.z = 0.0;

  RCLCPP_INFO(this->get_logger(), "Sending dummy request...");
  m_client->async_send_request(
      request, std::bind(&ClientNode::handle_response, this, std::placeholders::_1));
}

/*_________________________________________________________________________________________________*/
void ClientNode::handle_response(ServiceFuture future)
{
  auto response = future.get();
  RCLCPP_INFO(this->get_logger(), "Server response: success=%s, message='%s'",
              response->success ? "true" : "false", response->message.c_str());
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
  } else if (!client->init_client("least_square_service")) {
    RCLCPP_ERROR(client->get_logger(), "Failed to initialize client.");
    ret_code = 1;
  } else {
    RCLCPP_INFO(client->get_logger(), "Client initialized successfully.");
    client->send_dummy_request();
    rclcpp::spin(client);
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return ret_code;
}

/*_________________________________________________________________________________________________*/
