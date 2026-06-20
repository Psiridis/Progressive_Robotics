#include <cmath>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <optional>
#include <random>
#include <string>
#include <string_view>
#include <thread>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
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
  ~ServerNode() override;

  [[nodiscard]] bool init_service(std::string service_name);
  void init_subscriber(std::string topic_name);
  void log(std::string_view msg, LogLevel level = LogLevel::Info) const;

private:
  static constexpr double kPi = 3.14159265358979323846;

  void on_topic_message(const geometry_msgs::msg::Vector3& msg);
  void start_wait_thread();
  void stop_wait_thread();
  void wait_thread_loop();

  [[nodiscard]] bool validate_request(RequestPtr const& request) const;
  std::optional<Eigen::Vector3d> solve_least_squares(RequestPtr const& request) const;
  void generate_random_transform(Eigen::Quaterniond& rotation, Eigen::Vector3d& displacement);
  static Eigen::Vector3d apply_transform(const Eigen::Vector3d& x,
                                         const Eigen::Quaterniond& rotation,
                                         const Eigen::Vector3d& displacement);
  static geometry_msgs::msg::Vector3 to_vector3(const Eigen::Vector3d& v);
  static geometry_msgs::msg::Quaternion to_quaternion(const Eigen::Quaterniond& q);
  void handle_request(RequestPtr const request, ResponsePtr response);

  rclcpp::Service<LeastSquareContract>::SharedPtr m_service;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr m_subscriber;
  std::mutex m_mutex;
  std::condition_variable m_cv;
  std::thread m_wait_thread;
  geometry_msgs::msg::Vector3 m_last_message;
  bool m_message_ready;
  bool m_stop_requested;
  std::mt19937 m_rng;
  std::normal_distribution<double> m_axis_dist;
  std::uniform_real_distribution<double> m_angle_dist;
  std::uniform_real_distribution<double> m_disp_dist;
};

/*_________________________________________________________________________________________________*/
ServerNode::ServerNode(std::string node_name)
    : Node(node_name), m_rng(std::random_device{}()), m_axis_dist(0.0, 1.0),
      m_angle_dist(-kPi, kPi), m_disp_dist(-1.0, 1.0), m_message_ready(false),
      m_stop_requested(false)
{
  start_wait_thread();
}

/*_________________________________________________________________________________________________*/
ServerNode::~ServerNode()
{
  stop_wait_thread();
}

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
void ServerNode::init_subscriber(std::string topic_name)
{
  m_subscriber = create_subscription<geometry_msgs::msg::Vector3>(
      topic_name, 10, std::bind(&ServerNode::on_topic_message, this, std::placeholders::_1));
  log("Subscribed to topic '" + topic_name + "'.");
}

/*_________________________________________________________________________________________________*/
void ServerNode::on_topic_message(const geometry_msgs::msg::Vector3& msg)
{
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_last_message = msg;
    m_message_ready = true;
  }
  m_cv.notify_one();
}

/*_________________________________________________________________________________________________*/
void ServerNode::start_wait_thread()
{
  m_wait_thread = std::thread(&ServerNode::wait_thread_loop, this);
}

/*_________________________________________________________________________________________________*/
void ServerNode::stop_wait_thread()
{
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_stop_requested = true;
  }
  m_cv.notify_all();

  if (m_wait_thread.joinable()) {
    m_wait_thread.join();
  }
}

/*_________________________________________________________________________________________________*/
void ServerNode::wait_thread_loop()
{
  std::unique_lock<std::mutex> lock(m_mutex);

  while (!m_stop_requested) {
    m_cv.wait(lock, [this]() { return m_message_ready || m_stop_requested; });

    if (m_stop_requested) {
      break;
    }

    const geometry_msgs::msg::Vector3 msg = m_last_message;
    m_message_ready = false;
    lock.unlock();

    log("Received topic message: b = [" + std::to_string(msg.x) + ", " + std::to_string(msg.y) +
        ", " + std::to_string(msg.z) + "]");

    lock.lock();
  }
}

/*_________________________________________________________________________________________________*/
bool ServerNode::validate_request(RequestPtr const& request) const
{
  bool valid = true;

  if (request->a_rows.size() != 3) {
    log("Invalid request: expected exactly 3 rows in a_rows.", LogLevel::Error);
    valid = false;
  }

  return valid;
}

/*_________________________________________________________________________________________________*/
std::optional<Eigen::Vector3d> ServerNode::solve_least_squares(RequestPtr const& request) const
{
  std::optional<Eigen::Vector3d> solution = std::nullopt;

  Eigen::Matrix<double, 3, 3> a;
  for (std::size_t i = 0; i < 3; ++i) {
    a(i, 0) = request->a_rows[i].x;
    a(i, 1) = request->a_rows[i].y;
    a(i, 2) = request->a_rows[i].z;
  }

  const Eigen::Vector3d b(request->b.x, request->b.y, request->b.z);

  if (a.isZero(1e-12)) {
    log("Invalid request: matrix A is all zeros.", LogLevel::Error);
  } else {
    solution = a.colPivHouseholderQr().solve(b);
  }

  return solution;
}

/*_________________________________________________________________________________________________*/
void ServerNode::generate_random_transform(Eigen::Quaterniond& rotation,
                                           Eigen::Vector3d& displacement)
{
  Eigen::Vector3d axis(m_axis_dist(m_rng), m_axis_dist(m_rng), m_axis_dist(m_rng));
  if (axis.norm() < 1e-12) {
    axis = Eigen::Vector3d::UnitX();
  }
  axis.normalize();

  const double angle = m_angle_dist(m_rng);
  rotation = Eigen::AngleAxisd(angle, axis);

  displacement.x() = m_disp_dist(m_rng);
  displacement.y() = m_disp_dist(m_rng);
  displacement.z() = m_disp_dist(m_rng);
}

/*_________________________________________________________________________________________________*/
Eigen::Vector3d ServerNode::apply_transform(const Eigen::Vector3d& x,
                                            const Eigen::Quaterniond& rotation,
                                            const Eigen::Vector3d& displacement)
{
  return rotation * x + displacement;
}

/*_________________________________________________________________________________________________*/
geometry_msgs::msg::Vector3 ServerNode::to_vector3(const Eigen::Vector3d& v)
{
  geometry_msgs::msg::Vector3 msg;
  msg.x = v.x();
  msg.y = v.y();
  msg.z = v.z();
  return msg;
}

/*_________________________________________________________________________________________________*/
geometry_msgs::msg::Quaternion ServerNode::to_quaternion(const Eigen::Quaterniond& q)
{
  geometry_msgs::msg::Quaternion msg;
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  msg.w = q.w();
  return msg;
}

/*_________________________________________________________________________________________________*/
void ServerNode::handle_request(RequestPtr const request, ResponsePtr response)
{
  log("Received request with " + std::to_string(request->a_rows.size()) + " row(s).");
  for (std::size_t i = 0; i < request->a_rows.size(); ++i) {
    const auto& row = request->a_rows[i];
    log("A[" + std::to_string(i) + "] = [" + std::to_string(row.x) + ", " + std::to_string(row.y) +
        ", " + std::to_string(row.z) + "]");
  }
  log("b = [" + std::to_string(request->b.x) + ", " + std::to_string(request->b.y) + ", " +
      std::to_string(request->b.z) + "]");

  response->success = false;
  response->message = "Request failed.";

  if (!validate_request(request)) {
    response->message = "Request validation failed.";
  } else if (const auto solution = solve_least_squares(request); !solution) {
    response->message = "Least-squares computation failed.";
  } else {
    Eigen::Quaterniond rotation;
    Eigen::Vector3d displacement;
    generate_random_transform(rotation, displacement);

    const Eigen::Vector3d x_prime = apply_transform(*solution, rotation, displacement);

    response->x_prime = to_vector3(x_prime);
    response->r_prime = to_quaternion(rotation);
    response->d_prime = to_vector3(displacement);
    response->success = true;
    response->message = "Request processed successfully.";
  }
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
    server->init_subscriber("least_square_topic");
    server->log("Service initialized successfully.");
    rclcpp::spin(server);
  }

  rclcpp::shutdown();

  return ret_code;
}

/*_________________________________________________________________________________________________*/