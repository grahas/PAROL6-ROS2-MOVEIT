#pragma once

// Wire format for the local loopback link to parol6_bridge. Keep this in
// sync with ros2/parol6_bridge/parol6_bridge/protocol.py -- see that file
// for the full rationale/field description.
//
// Both structs are sent/received as raw bytes over a local TCP socket
// (loopback only, so no endianness/architecture concerns in practice).
// #pragma pack keeps them byte-packed to match Python's struct "<...".

#include <array>
#include <cstdint>

namespace parol6_hardware_interface
{

constexpr uint8_t kOpUpdate = 1;
constexpr uint8_t kOpEnable = 2;
constexpr uint8_t kOpDisable = 3;
constexpr uint8_t kOpPing = 4;

#pragma pack(push, 1)
struct BridgeRequest
{
  uint8_t op;
  double target_pos_rad[6];
};

struct BridgeResponse
{
  uint8_t ok;
  double pos_rad[6];
  double vel_rad[6];
  uint8_t hardware_connected;
  uint8_t estop;
};
#pragma pack(pop)

static_assert(sizeof(BridgeRequest) == 49, "BridgeRequest must match protocol.py REQUEST_SIZE");
static_assert(sizeof(BridgeResponse) == 99, "BridgeResponse must match protocol.py RESPONSE_SIZE");

}  // namespace parol6_hardware_interface
