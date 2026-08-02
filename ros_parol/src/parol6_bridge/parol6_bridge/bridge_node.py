"""TCP bridge between the parol6_hardware_interface ros2_control plugin and
a running parol6-server instance.

Why this exists: ros2_control hardware interfaces are C++ and run inside a
real-time control loop, but parol6's UDP wire protocol (msgpack framing,
multicast status broadcasts, ack policies, ...) is only implemented in
parol6's own Python SDK. Rather than reimplement ~1900 lines of protocol
code in C++ and keep it in lockstep with parol6 upstream, this small
daemon does the UDP talking with AsyncRobotClient and exposes a minimal,
fixed-size binary protocol over a local TCP loopback socket (see
protocol.py) for the C++ plugin to use instead.

Run standalone for testing:
    parol6_bridge --robot-host 127.0.0.1 --robot-port 5001 --bind-port 6001

Normally launched by parol6_bringup's launch file, one per controller_manager.
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import logging
import math

from parol6 import AsyncRobotClient

from parol6_bridge.protocol import (
    OP_DISABLE,
    OP_ENABLE,
    OP_PING,
    OP_UPDATE,
    REQUEST_SIZE,
    pack_response,
    unpack_request,
)

logger = logging.getLogger("parol6_bridge")


class Bridge:
    """Owns the parol6 client and the latest known joint state."""

    def __init__(self, robot_host: str, robot_port: int, hw_check_interval: float = 1.0):
        self.client = AsyncRobotClient(host=robot_host, port=robot_port)
        self._hw_check_interval = hw_check_interval

        self._pos_rad = [0.0] * 6
        self._vel_rad = [0.0] * 6
        self._estop = False
        self._hardware_connected = False

        self._status_task: asyncio.Task | None = None
        self._ping_task: asyncio.Task | None = None

    async def start(self) -> None:
        if not await self.client.wait_ready(timeout=10.0):
            raise RuntimeError(
                "parol6-server did not respond to PING within 10s -- "
                "is it running and reachable at the configured host/port?"
            )
        self._status_task = asyncio.create_task(self._consume_status())
        self._ping_task = asyncio.create_task(self._poll_hardware_connected())

    async def _consume_status(self) -> None:
        # angles: degrees -> radians. speeds (StatusBuffer) are already rad/s.
        async for status in self.client.stream_status_shared():
            for i in range(6):
                self._pos_rad[i] = math.radians(float(status.angles[i]))
                self._vel_rad[i] = float(status.speeds[i])
            io = status.io
            self._estop = len(io) > 4 and io[4] == 0

    async def _poll_hardware_connected(self) -> None:
        while True:
            try:
                result = await self.client.ping()
                self._hardware_connected = bool(result and result.hardware_connected)
            except Exception:
                logger.debug("ping failed", exc_info=True)
            await asyncio.sleep(self._hw_check_interval)

    async def handle_client(
        self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter
    ) -> None:
        peer = writer.get_extra_info("peername")
        logger.info("hardware interface connected: %s", peer)
        try:
            while True:
                raw = await reader.readexactly(REQUEST_SIZE)
                op, target_pos_rad = unpack_request(raw)
                ok = await self._dispatch(op, target_pos_rad)
                writer.write(
                    pack_response(
                        ok, self._pos_rad, self._vel_rad, self._hardware_connected, self._estop
                    )
                )
                await writer.drain()
        except (asyncio.IncompleteReadError, ConnectionResetError, BrokenPipeError):
            pass
        finally:
            logger.info("hardware interface disconnected: %s", peer)
            with contextlib.suppress(Exception):
                writer.close()

    async def _dispatch(self, op: int, target_pos_rad: list) -> bool:
        try:
            if op == OP_UPDATE:
                angles_deg = [math.degrees(v) for v in target_pos_rad]
                await self.client.servo_j(angles_deg)
                return True
            if op == OP_ENABLE:
                await self.client.resume()
                return True
            if op == OP_DISABLE:
                await self.client.halt()
                return True
            if op == OP_PING:
                return True
            logger.warning("unknown opcode %d", op)
            return False
        except Exception:
            logger.exception("command failed (op=%d)", op)
            return False

    async def close(self) -> None:
        for task in (self._status_task, self._ping_task):
            if task is not None:
                task.cancel()
        await self.client.close()


async def async_main(args: argparse.Namespace) -> None:
    bridge = Bridge(args.robot_host, args.robot_port)
    await bridge.start()

    server = await asyncio.start_server(bridge.handle_client, args.bind_host, args.bind_port)
    addr = ", ".join(str(sock.getsockname()) for sock in server.sockets or [])
    logger.info(
        "parol6_bridge listening on %s, relaying to parol6-server at %s:%d",
        addr,
        args.robot_host,
        args.robot_port,
    )
    try:
        async with server:
            await server.serve_forever()
    finally:
        await bridge.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-host", default="127.0.0.1", help="parol6-server host")
    parser.add_argument("--robot-port", type=int, default=5001, help="parol6-server UDP port")
    parser.add_argument("--bind-host", default="127.0.0.1", help="address to listen on")
    parser.add_argument("--bind-port", type=int, default=6001, help="port to listen on")
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args()

    logging.basicConfig(
        level=args.log_level.upper(), format="%(asctime)s %(levelname)s %(name)s: %(message)s"
    )
    try:
        asyncio.run(async_main(args))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
