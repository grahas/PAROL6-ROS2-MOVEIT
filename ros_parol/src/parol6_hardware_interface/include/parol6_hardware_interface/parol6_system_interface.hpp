#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "parol6_hardware_interface/bridge_client.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace parol6_hardware_interface
{

/// ros2_control SystemInterface for the 6-DOF PAROL6 arm. Reads/writes go
/// to a parol6_bridge daemon over a local TCP socket (see bridge_client.hpp)
/// rather than talking to parol6-server's UDP protocol directly -- the
/// bridge does that using parol6's own Python SDK.
///
/// Targets the "lyrical" generation of the hardware_interface API, where
/// state/command interface storage is owned and auto-exported by the
/// framework (from the URDF-declared interfaces) rather than by the plugin;
/// this class only reads/writes through get_state()/set_state()/
/// get_command() handles cached at on_configure() time. This is NOT
/// source-compatible with Humble's hardware_interface (no
/// HardwareComponentInterfaceParams, no on_export_state_interfaces there) --
/// see README.md.
class Parol6SystemInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Parol6SystemInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static constexpr size_t kNumJoints = 6;

  /// Resolves position_state_handles_/velocity_state_handles_/
  /// position_command_handles_ by name. Only valid to call once the
  /// framework has exported interfaces for this component (on_configure()
  /// or later).
  bool cache_interface_handles();

  // Cached handles into the framework-owned interface storage, resolved by
  // name in on_configure() (after the framework's default
  // on_export_state_interfaces()/on_export_command_interfaces() have run).
  std::array<hardware_interface::StateInterface::SharedPtr, kNumJoints> position_state_handles_;
  std::array<hardware_interface::StateInterface::SharedPtr, kNumJoints> velocity_state_handles_;
  std::array<hardware_interface::CommandInterface::SharedPtr, kNumJoints> position_command_handles_;

  // Last known position, used to seed the command on activate (avoid a jump
  // to a stale/NaN commanded value) and as a read()/write() fallback if the
  // bridge is briefly unreachable.
  std::array<double, kNumJoints> last_position_rad_{};

  std::string bridge_host_{"127.0.0.1"};
  int bridge_port_{6001};
  double connect_timeout_sec_{5.0};

  BridgeClient bridge_;
  bool estop_latched_{false};
  rclcpp::Clock clock_{RCL_STEADY_TIME};
};

}  // namespace parol6_hardware_interface
