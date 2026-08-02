"""Wire format for the local loopback link between parol6_hardware_interface
(C++, in the ros2_control real-time loop) and this bridge daemon.

This is intentionally NOT the parol6 UDP protocol -- it's a tiny fixed-size
binary framing meant only for a single machine's loopback interface. Keep
this in sync with parol6_hardware_interface/include/parol6_hardware_interface/
bridge_protocol.hpp if you change anything here.

Request (client -> bridge), sent once per read()/write() call:
    uint8   op              OP_UPDATE / OP_ENABLE / OP_DISABLE / OP_PING
    double  target_pos[6]   commanded joint positions, radians (ignored
                             unless op == OP_UPDATE, but always present so
                             the frame size is fixed)

Response (bridge -> client), one per request:
    uint8   ok                  1 if the operation succeeded, 0 otherwise
    double  pos[6]               latest known joint positions, radians
    double  vel[6]               latest known joint velocities, rad/s
    uint8   hardware_connected   1 if parol6-server reports real hardware attached
    uint8   estop                1 if E-stop is pressed / not clear to move
"""

import struct

OP_UPDATE = 1
OP_ENABLE = 2
OP_DISABLE = 3
OP_PING = 4

REQUEST_FMT = "<B6d"
REQUEST_SIZE = struct.calcsize(REQUEST_FMT)

RESPONSE_FMT = "<B6d6dBB"
RESPONSE_SIZE = struct.calcsize(RESPONSE_FMT)


def pack_response(
    ok: bool,
    pos_rad: list,
    vel_rad: list,
    hardware_connected: bool,
    estop: bool,
) -> bytes:
    return struct.pack(
        RESPONSE_FMT,
        1 if ok else 0,
        *pos_rad,
        *vel_rad,
        1 if hardware_connected else 0,
        1 if estop else 0,
    )


def unpack_request(raw: bytes) -> tuple:
    """Returns (op, target_pos_rad: list[6])."""
    fields = struct.unpack(REQUEST_FMT, raw)
    return fields[0], list(fields[1:7])
