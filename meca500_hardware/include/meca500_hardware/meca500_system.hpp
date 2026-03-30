#ifndef MECA500_HARDWARE__MECA500_SYSTEM_HPP_
#define MECA500_HARDWARE__MECA500_SYSTEM_HPP_

#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace meca500_hardware
{

// Meca500 uses degrees, ROS 2 uses radians
constexpr double DEG_TO_RAD = M_PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / M_PI;

// Meca500 TCP ports
constexpr int CONTROL_PORT = 10000;
constexpr int MONITOR_PORT = 10001;

class Meca500SystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Meca500SystemHardware)

  // ---------------------------------------------------------
  // Lifecycle Methods
  // ---------------------------------------------------------
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // ---------------------------------------------------------
  // Interface Export Methods
  // ---------------------------------------------------------
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // ---------------------------------------------------------
  // Real-Time Loop Methods
  // ---------------------------------------------------------
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // --- Hardware Parameters ---
  std::string robot_ip_;

  // --- Interface Data Storage ---
  std::vector<double> hw_joint_states_position_;
  std::vector<double> hw_joint_states_velocity_;
  std::vector<double> hw_joint_commands_velocity_;

  // --- Asynchronous Communication Setup ---
  std::thread tcp_receive_thread_;
  std::atomic<bool> is_active_{false};
  std::atomic<bool> monitor_fault_{false};
  std::atomic<bool> monitor_fault_logged_{false};
  std::mutex state_mutex_;

  // --- TCP Socket File Descriptors ---
  int control_fd_{-1};
  int monitor_fd_{-1};

  // --- TCP Communication Helpers ---
  int connect_socket(const std::string & ip, int port);
  bool send_command(const std::string & cmd);
  std::string receive_response(int fd, int timeout_ms = 5000);
  bool wait_for_response(int fd, int expected_code, int timeout_ms = 10000);
  void close_socket(int & fd);

  // Background worker function to continuously poll the TCP monitor port
  void receive_data_loop();

  // Parse a Meca500 response message of the form [code][data]
  static bool parse_response(const std::string & msg, int & code, std::string & data);
  static bool parse_joint_values(const std::string & data, double values[6]);
};

}  // namespace meca500_hardware

#endif  // MECA500_HARDWARE__MECA500_SYSTEM_HPP_