#pragma once

#include <array>
#include <string>

#include "parol6_hardware_interface/bridge_protocol.hpp"

namespace parol6_hardware_interface
{

/// Blocking TCP client for the local parol6_bridge daemon. One request,
/// one response, per call -- matches the read()/write() cadence of a
/// ros2_control SystemInterface. Linux sockets only.
class BridgeClient
{
public:
  BridgeClient() = default;
  ~BridgeClient();

  BridgeClient(const BridgeClient &) = delete;
  BridgeClient & operator=(const BridgeClient &) = delete;

  /// Connects with retries until timeout_sec elapses. Returns false (and
  /// leaves an error in last_error()) if it never connects.
  bool connect(const std::string & host, int port, double timeout_sec);

  /// Sends one request and blocks for the matching response. Returns false
  /// on any socket error (connection is considered dead afterwards).
  bool exchange(uint8_t op, const std::array<double, 6> & target_pos_rad, BridgeResponse & out);

  void close();
  bool is_connected() const { return fd_ >= 0; }
  const std::string & last_error() const { return last_error_; }

private:
  int fd_{-1};
  std::string last_error_;
};

}  // namespace parol6_hardware_interface
