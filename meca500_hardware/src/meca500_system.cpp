#include "meca500_hardware/meca500_system.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <arpa/inet.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>
#include <poll.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>

namespace meca500_hardware
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("Meca500SystemHardware");

// -------------------------------------------------------
// TCP Helpers
// -------------------------------------------------------

int Meca500SystemHardware::connect_socket(const std::string & ip, int port)
{
  int fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    RCLCPP_ERROR(LOGGER, "Failed to create socket: %s", strerror(errno));
    return -1;
  }

  // Disable Nagle's algorithm for low-latency command sending
  int flag = 1;
  setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

  struct sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  if (inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) <= 0) {
    RCLCPP_ERROR(LOGGER, "Invalid IP address: %s", ip.c_str());
    close(fd);
    return -1;
  }

  // Set a connect timeout via SO_SNDTIMEO
  struct timeval tv{};
  tv.tv_sec = 5;
  setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

  if (connect(fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
    RCLCPP_ERROR(LOGGER, "Failed to connect to %s:%d: %s", ip.c_str(), port, strerror(errno));
    close(fd);
    return -1;
  }

  RCLCPP_INFO(LOGGER, "Connected to %s:%d (fd=%d)", ip.c_str(), port, fd);
  return fd;
}

bool Meca500SystemHardware::send_command(const std::string & cmd)
{
  if (control_fd_ < 0) {
    return false;
  }
  // Meca500 commands are null-terminated ASCII strings
  std::string packet = cmd + '\0';
  ssize_t sent = ::send(control_fd_, packet.c_str(), packet.size(), MSG_NOSIGNAL);
  if (sent < 0) {
    RCLCPP_ERROR(LOGGER, "send_command failed for '%s': %s", cmd.c_str(), strerror(errno));
    return false;
  }
  RCLCPP_DEBUG(LOGGER, "Sent: %s", cmd.c_str());
  return true;
}

std::string Meca500SystemHardware::receive_response(int fd, int timeout_ms)
{
  struct pollfd pfd{};
  pfd.fd = fd;
  pfd.events = POLLIN;

  int ret = poll(&pfd, 1, timeout_ms);
  if (ret <= 0) {
    return {};
  }

  char buf[1024];
  ssize_t n = recv(fd, buf, sizeof(buf) - 1, 0);
  if (n <= 0) {
    return {};
  }
  buf[n] = '\0';
  return std::string(buf, static_cast<size_t>(n));
}

bool Meca500SystemHardware::wait_for_response(int fd, int expected_code, int timeout_ms)
{
  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

  while (std::chrono::steady_clock::now() < deadline) {
    int remaining = static_cast<int>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        deadline - std::chrono::steady_clock::now()).count());
    if (remaining <= 0) break;

    std::string raw = receive_response(fd, remaining);
    if (raw.empty()) continue;

    // The Meca500 may send multiple responses in one packet, split by \0
    std::istringstream stream(raw);
    std::string token;
    while (std::getline(stream, token, '\0')) {
      if (token.empty()) continue;
      int code = 0;
      std::string data;
      if (parse_response(token, code, data)) {
        RCLCPP_INFO(LOGGER, "Response [%d][%s]", code, data.c_str());
        if (code == expected_code) {
          return true;
        }
        // Error responses from Meca500 are in the 1xxx range
        if (code >= 1000 && code < 2000) {
          RCLCPP_WARN(LOGGER, "Robot error [%d]: %s", code, data.c_str());
        }
      }
    }
  }

  RCLCPP_WARN(LOGGER, "Timed out waiting for response code %d", expected_code);
  return false;
}

bool Meca500SystemHardware::parse_response(const std::string & msg, int & code, std::string & data)
{
  // Response format: [code][data]  e.g. [2007][12.3,0.0,-5.1,0.0,90.0,0.0]
  if (msg.size() < 3 || msg[0] != '[') {
    return false;
  }
  auto close1 = msg.find(']', 1);
  if (close1 == std::string::npos) {
    return false;
  }
  try {
    code = std::stoi(msg.substr(1, close1 - 1));
  } catch (...) {
    return false;
  }

  // Check for optional data portion
  if (close1 + 1 < msg.size() && msg[close1 + 1] == '[') {
    auto close2 = msg.find(']', close1 + 2);
    if (close2 != std::string::npos) {
      data = msg.substr(close1 + 2, close2 - close1 - 2);
    }
  }
  return true;
}

bool Meca500SystemHardware::parse_joint_values(const std::string & data, double values[6])
{
  std::istringstream ss(data);
  std::string token;
  std::vector<double> parsed_values;

  while (std::getline(ss, token, ',')) {
    try {
      parsed_values.push_back(std::stod(token));
    } catch (...) {
      return false;
    }
  }

  // Monitoring payload can be either:
  //  - 6 values: [theta1, theta2, theta3, theta4, theta5, theta6]
  //  - 7 values: [timestamp, theta1, theta2, theta3, theta4, theta5, theta6]
  size_t offset = 0;
  if (parsed_values.size() == 7) {
    offset = 1;  // Skip timestamp.
  } else if (parsed_values.size() != 6) {
    return false;
  }

  for (size_t i = 0; i < 6; ++i) {
    values[i] = parsed_values[i + offset];
  }

  return true;
}

void Meca500SystemHardware::close_socket(int & fd)
{
  if (fd >= 0) {
    ::close(fd);
    fd = -1;
  }
}

bool Meca500SystemHardware::reconnect_monitor_socket(int retry_count, int retry_delay_ms)
{
  close_socket(monitor_fd_);

  for (int attempt = 1; attempt <= retry_count; ++attempt) {
    if (!is_active_.load() || is_shutting_down_.load()) {
      return false;
    }

    int fd = connect_socket(robot_ip_, MONITOR_PORT);
    if (fd >= 0) {
      monitor_fd_ = fd;
      RCLCPP_INFO(LOGGER, "Monitoring socket reconnected on attempt %d/%d.", attempt, retry_count);
      return true;
    }

    RCLCPP_WARN(
      LOGGER,
      "Monitoring reconnect attempt %d/%d failed. Retrying in %d ms...",
      attempt,
      retry_count,
      retry_delay_ms);
    std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms));
  }

  return false;
}

// -------------------------------------------------------
// 1. Initialization
// -------------------------------------------------------

hardware_interface::CallbackReturn Meca500SystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read the robot IP from the URDF <param name="robot_ip">
  auto it = info.hardware_parameters.find("robot_ip");
  if (it == info.hardware_parameters.end()) {
    RCLCPP_ERROR(LOGGER, "Missing required hardware parameter 'robot_ip'");
    return hardware_interface::CallbackReturn::ERROR;
  }
  robot_ip_ = it->second;
  RCLCPP_INFO(LOGGER, "Meca500 IP: %s", robot_ip_.c_str());

  // Validate joint count
  if (info.joints.size() != 6) {
    RCLCPP_ERROR(LOGGER, "Expected 6 joints, got %zu", info.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_joint_states_position_.resize(6, 0.0);
  hw_joint_states_velocity_.resize(6, 0.0);
  hw_joint_commands_velocity_.resize(6, 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// -------------------------------------------------------
// 2. Interface Export
// -------------------------------------------------------

std::vector<hardware_interface::StateInterface>
Meca500SystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION,
      &hw_joint_states_position_[i]);

    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
      &hw_joint_states_velocity_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
Meca500SystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
      &hw_joint_commands_velocity_[i]);
  }
  return command_interfaces;
}

// -------------------------------------------------------
// 3. Lifecycle: Activate
// -------------------------------------------------------

hardware_interface::CallbackReturn Meca500SystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(LOGGER, "Activating... connecting to Meca500 at %s", robot_ip_.c_str());
  is_shutting_down_.store(false);
  is_active_.store(false);
  monitor_fault_.store(false);
  monitor_fault_logged_.store(false);

  // --- Connect to Control Port (10000) ---
  control_fd_ = connect_socket(robot_ip_, CONTROL_PORT);
  if (control_fd_ < 0) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // The robot sends a welcome message [3000] on connection
  if (!wait_for_response(control_fd_, 3000, 5000)) {
    RCLCPP_ERROR(LOGGER, "Did not receive welcome message from Meca500");
    close_socket(control_fd_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // --- Connect to Monitoring Port (10001) ---
  monitor_fd_ = connect_socket(robot_ip_, MONITOR_PORT);
  if (monitor_fd_ < 0) {
    close_socket(control_fd_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // --- Activation Sequence ---
  // 1. Reset any existing errors
  send_command("ResetError");
  // Response [2005] = error reset
  wait_for_response(control_fd_, 2005, 3000);

  // 2. Activate the robot motors
  send_command("ActivateRobot");
  // Response [2000] = robots activated
  if (!wait_for_response(control_fd_, 2000, 30000)) {
    RCLCPP_ERROR(LOGGER, "Failed to activate robot");
    close_socket(monitor_fd_);
    close_socket(control_fd_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 3. Home the robot
  send_command("Home");
  // Response [2002] = homing done
  if (!wait_for_response(control_fd_, 2002, 30000)) {
    RCLCPP_ERROR(LOGGER, "Failed to home robot");
    close_socket(monitor_fd_);
    close_socket(control_fd_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 4. Enable real-time monitoring on the monitoring port
  //    This makes the robot stream joint positions and velocities.
  send_command("SetMonitoringInterval(0.001)");
  send_command(
    "SetRealTimeMonitoring(" + std::to_string(MONITOR_RT_JOINT_POSITION_ID) + ", " +
    std::to_string(MONITOR_RT_JOINT_VELOCITY_ID) + ")");

  // 5. Set velocity timeout so the robot stops if we miss sending commands
  send_command("SetVelTimeout(0.05)");

  // 6. Set acceleration to max - 100%
  send_command("SetJointAcc(100)");

  // Zero command velocities
  std::fill(hw_joint_commands_velocity_.begin(), hw_joint_commands_velocity_.end(), 0.0);

  // Start the background monitoring thread
  is_active_.store(true);
  tcp_receive_thread_ = std::thread(&Meca500SystemHardware::receive_data_loop, this);

  RCLCPP_INFO(LOGGER, "Meca500 activated and homed successfully.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// -------------------------------------------------------
// 4. Lifecycle: Deactivate
// -------------------------------------------------------

hardware_interface::CallbackReturn Meca500SystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(LOGGER, "Deactivating... stopping robot.");

  // Signal the monitoring thread to stop
  is_shutting_down_.store(true);
  is_active_.store(false);

  // Closing the monitor socket first ensures recv()/poll() unblock promptly.
  close_socket(monitor_fd_);

  if (tcp_receive_thread_.joinable()) {
    tcp_receive_thread_.join();
  }

  // Send zero velocity to stop all motion
  send_command("MoveJointsVel(0,0,0,0,0,0)");

  // Disable real-time monitoring
  send_command("SetRealTimeMonitoring(0)");

  // Deactivate the robot
  send_command("DeactivateRobot");

  // Close sockets
  close_socket(control_fd_);

  RCLCPP_INFO(LOGGER, "Meca500 deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// -------------------------------------------------------
// 5. Real-Time Read Loop
// -------------------------------------------------------

hardware_interface::return_type Meca500SystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (monitor_fault_.load()) {
    if (!monitor_fault_logged_.exchange(true)) {
      RCLCPP_ERROR(LOGGER, "Monitoring feedback lost, transitioning hardware interface to ERROR.");
    }
    return hardware_interface::return_type::ERROR;
  }

  // The background thread continuously updates position/velocity from
  // the monitoring port. We just copy the latest values under the lock.
  // This keeps read() non-blocking and deterministic.
  // (No lock needed for atomic-friendly patterns, but we use the mutex
  //  since we update 6 doubles at a time.)
  std::lock_guard<std::mutex> lock(state_mutex_);
  // Data is already in hw_joint_states_position_ / hw_joint_states_velocity_
  // (written by receive_data_loop), nothing else to do here.

  return hardware_interface::return_type::OK;
}

// -------------------------------------------------------
// 6. Real-Time Write Loop
// -------------------------------------------------------

hardware_interface::return_type Meca500SystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (monitor_fault_.load()) {
    if (!monitor_fault_logged_.exchange(true)) {
      RCLCPP_ERROR(LOGGER, "Skipping command write because monitoring feedback is unavailable.");
    }
    return hardware_interface::return_type::ERROR;
  }

  // Convert velocity commands from radians/s (ROS) to degrees/s (Meca500)
  char cmd[256];
  std::snprintf(cmd, sizeof(cmd),
    "MoveJointsVel(%.4f,%.4f,%.4f,%.4f,%.4f,%.4f)",
    hw_joint_commands_velocity_[0] * RAD_TO_DEG,
    hw_joint_commands_velocity_[1] * RAD_TO_DEG,
    hw_joint_commands_velocity_[2] * RAD_TO_DEG,
    hw_joint_commands_velocity_[3] * RAD_TO_DEG,
    hw_joint_commands_velocity_[4] * RAD_TO_DEG,
    hw_joint_commands_velocity_[5] * RAD_TO_DEG);

  if (!send_command(cmd)) {
    RCLCPP_ERROR(LOGGER, "Failed to send velocity command, transitioning hardware interface to ERROR.");
    monitor_fault_.store(true);
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

// -------------------------------------------------------
// 7. Background Monitoring Thread
// -------------------------------------------------------

void Meca500SystemHardware::receive_data_loop()
{
  RCLCPP_INFO(LOGGER, "Monitoring thread started.");

  // Buffer for partial TCP reads
  std::string buffer;

  while (is_active_.load()) {
    struct pollfd pfd{};
    pfd.fd = monitor_fd_;
    pfd.events = POLLIN;

    int ret = poll(&pfd, 1, 100);  // 100ms timeout to check is_active_
    if (ret == 0) {
      continue;
    }
    if (ret < 0) {
      if (!is_active_.load() || is_shutting_down_.load()) {
        break;
      }
      RCLCPP_ERROR(LOGGER, "Polling monitor socket failed: %s", strerror(errno));
      if (!reconnect_monitor_socket()) {
        monitor_fault_.store(true);
        break;
      }
      continue;
    }

    if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      if (!is_active_.load() || is_shutting_down_.load()) {
        break;
      }
      RCLCPP_WARN(LOGGER, "Monitoring socket signaled error/hangup (revents=0x%x).", pfd.revents);
      if (!reconnect_monitor_socket()) {
        monitor_fault_.store(true);
        break;
      }
      continue;
    }

    char raw[2048];
    ssize_t n = recv(monitor_fd_, raw, sizeof(raw) - 1, 0);
    if (n <= 0) {
      if (!is_active_.load() || is_shutting_down_.load()) {
        break;
      }
      if (n == 0) {
        RCLCPP_WARN(LOGGER, "Monitoring port disconnected.");
      } else {
        RCLCPP_ERROR(LOGGER, "Monitoring socket receive failed: %s", strerror(errno));
      }
      if (!reconnect_monitor_socket()) {
        monitor_fault_.store(true);
        break;
      }
      continue;
    }

    buffer.append(raw, static_cast<size_t>(n));

    // Process all complete messages (delimited by \0)
    size_t pos = 0;
    while (true) {
      size_t null_pos = buffer.find('\0', pos);
      if (null_pos == std::string::npos) {
        break;
      }

      std::string msg = buffer.substr(pos, null_pos - pos);
      pos = null_pos + 1;

      if (msg.empty()) continue;

      int code = 0;
      std::string data;
      if (!parse_response(msg, code, data)) {
        continue;
      }

      double values[6];

      if (code == MONITOR_RT_JOINT_POSITION_ID && parse_joint_values(data, values)) {
        // Real-time joint positions (degrees from Meca500 → radians for ROS)
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (int i = 0; i < 6; i++) {
          hw_joint_states_position_[i] = values[i] * DEG_TO_RAD;
        }
      } else if (code == MONITOR_RT_JOINT_VELOCITY_ID && parse_joint_values(data, values)) {
        // Real-time joint velocities (degrees/s from Meca500 → radians/s for ROS)
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (int i = 0; i < 6; i++) {
          hw_joint_states_velocity_[i] = values[i] * DEG_TO_RAD;
        }
      }
    }

    // Keep unprocessed partial data
    buffer = buffer.substr(pos);
  }

  RCLCPP_INFO(LOGGER, "Monitoring thread stopped.");
}

}  // namespace meca500_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  meca500_hardware::Meca500SystemHardware, hardware_interface::SystemInterface)