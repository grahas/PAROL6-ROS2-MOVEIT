#include "parol6_hardware_interface/parol6_system_interface.hpp"

#include <cmath>
#include <limits>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace parol6_hardware_interface
{

namespace
{
rclcpp::Logger logger() { return rclcpp::get_logger("Parol6SystemInterface"); }

double get_double_param(
  const hardware_interface::HardwareInfo & info, const std::string & name, double fallback)
{
  const auto it = info.hardware_parameters.find(name);
  if (it == info.hardware_parameters.end()) {
    return fallback;
  }
  try {
    return std::stod(it->second);
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      logger(), "hardware param '%s' = '%s' is not a number (%s); using default %f",
      name.c_str(), it->second.c_str(), e.what(), fallback);
    return fallback;
  }
}

std::string get_string_param(
  const hardware_interface::HardwareInfo & info, const std::string & name,
  const std::string & fallback)
{
  const auto it = info.hardware_parameters.find(name);
  return it == info.hardware_parameters.end() ? fallback : it->second;
}
}  // namespace

hardware_interface::CallbackReturn Parol6SystemInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != kNumJoints) {
    RCLCPP_FATAL(
      logger(), "Parol6SystemInterface expects exactly %zu joints, got %zu", kNumJoints,
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        logger(), "Joint '%s' must expose exactly one command interface: position",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(
        logger(), "Joint '%s' must expose two state interfaces: position, velocity",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  bridge_host_ = get_string_param(info_, "bridge_host", "127.0.0.1");
  bridge_port_ = static_cast<int>(get_double_param(info_, "bridge_port", 6001));
  connect_timeout_sec_ = get_double_param(info_, "connect_timeout_sec", 5.0);

  last_position_rad_.fill(std::numeric_limits<double>::quiet_NaN());

  // Interface storage/export is handled by the framework's default
  // on_export_state_interfaces()/on_export_command_interfaces() based on
  // the URDF-declared interfaces -- we just cache handles to it below,
  // once export has happened (on_configure()).

  return hardware_interface::CallbackReturn::SUCCESS;
}

bool Parol6SystemInterface::cache_interface_handles()
{
  try {
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      const std::string & name = info_.joints[i].name;
      position_state_handles_[i] =
        get_state_interface_handle(name + "/" + hardware_interface::HW_IF_POSITION);
      velocity_state_handles_[i] =
        get_state_interface_handle(name + "/" + hardware_interface::HW_IF_VELOCITY);
      position_command_handles_[i] =
        get_command_interface_handle(name + "/" + hardware_interface::HW_IF_POSITION);
    }
  } catch (const std::exception & e) {
    RCLCPP_FATAL(logger(), "Failed to resolve exported interface handles: %s", e.what());
    return false;
  }
  return true;
}

hardware_interface::CallbackReturn Parol6SystemInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!cache_interface_handles()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(
    logger(), "Connecting to parol6_bridge at %s:%d (timeout %.1fs)...", bridge_host_.c_str(),
    bridge_port_, connect_timeout_sec_);

  if (!bridge_.connect(bridge_host_, bridge_port_, connect_timeout_sec_)) {
    RCLCPP_FATAL(
      logger(), "Could not connect to parol6_bridge: %s. Is it running (ros2 run parol6_bridge "
                "parol6_bridge)?",
      bridge_.last_error().c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Seed state so the first read()/write() cycle doesn't see NaN / a large jump.
  BridgeResponse resp{};
  std::array<double, 6> zero{};
  if (bridge_.exchange(kOpPing, zero, resp) && resp.ok) {
    for (size_t i = 0; i < kNumJoints; ++i) {
      last_position_rad_[i] = resp.pos_rad[i];
      set_state(position_state_handles_[i], resp.pos_rad[i], true);
      set_state(velocity_state_handles_[i], resp.vel_rad[i], true);
      set_command(position_command_handles_[i], resp.pos_rad[i], true);
    }
  } else {
    RCLCPP_WARN(logger(), "Initial PING to parol6_bridge did not return a valid state yet");
  }

  RCLCPP_INFO(logger(), "Connected to parol6_bridge");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Parol6SystemInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  BridgeResponse resp{};
  std::array<double, 6> zero{};
  if (!bridge_.exchange(kOpEnable, zero, resp) || !resp.ok) {
    RCLCPP_FATAL(logger(), "Failed to enable robot via parol6_bridge (resume() failed)");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Re-seed the command with current position so activation doesn't jerk
  // the arm toward a stale commanded value.
  for (size_t i = 0; i < kNumJoints; ++i) {
    set_command(position_command_handles_[i], last_position_rad_[i], true);
  }

  estop_latched_ = false;
  RCLCPP_INFO(logger(), "Robot activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Parol6SystemInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  BridgeResponse resp{};
  std::array<double, 6> zero{};
  if (!bridge_.exchange(kOpDisable, zero, resp)) {
    RCLCPP_WARN(logger(), "Failed to send halt() to parol6_bridge during deactivate");
  }
  RCLCPP_INFO(logger(), "Robot deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Parol6SystemInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  BridgeResponse resp{};
  std::array<double, 6> zero{};
  if (!bridge_.exchange(kOpPing, zero, resp) || !resp.ok) {
    RCLCPP_WARN_THROTTLE(
      logger(), clock_, 1000, "parol6_bridge read failed: %s",
      bridge_.last_error().c_str());
    // Keep last known state rather than erroring the whole controller
    // manager out on a transient hiccup.
    return hardware_interface::return_type::OK;
  }

  for (size_t i = 0; i < kNumJoints; ++i) {
    last_position_rad_[i] = resp.pos_rad[i];
    set_state(position_state_handles_[i], resp.pos_rad[i], true);
    set_state(velocity_state_handles_[i], resp.vel_rad[i], true);
  }

  if (resp.estop && !estop_latched_) {
    RCLCPP_ERROR(logger(), "E-stop is engaged -- motion commands will be rejected by the robot");
    estop_latched_ = true;
  } else if (!resp.estop && estop_latched_) {
    RCLCPP_INFO(logger(), "E-stop cleared");
    estop_latched_ = false;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Parol6SystemInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  std::array<double, 6> target{};
  for (size_t i = 0; i < kNumJoints; ++i) {
    double cmd = std::numeric_limits<double>::quiet_NaN();
    get_command(position_command_handles_[i], cmd, true);
    target[i] = std::isfinite(cmd) ? cmd : last_position_rad_[i];
  }

  BridgeResponse resp{};
  if (!bridge_.exchange(kOpUpdate, target, resp) || !resp.ok) {
    RCLCPP_WARN_THROTTLE(
      logger(), clock_, 1000, "parol6_bridge write failed: %s",
      bridge_.last_error().c_str());
    return hardware_interface::return_type::OK;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace parol6_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  parol6_hardware_interface::Parol6SystemInterface, hardware_interface::SystemInterface)
