"""Bring up MoveIt + ros2_control against a real (or simulated-via-parol6-server)
PAROL6 arm, instead of the mock_components/GenericSystem used by demo.launch.py.

Starts parol6_bridge (TCP<->UDP daemon, see the parol6_bridge package) and then
the usual MoveIt demo stack, with the ros2_control URDF built using
Parol6SystemInterface (parol6_hardware_interface package) instead of the mock
plugin -- see parol6.ros2_control.xacro's use_real_hardware arg.

Prerequisite: parol6-server must already be running and reachable at
robot_host:robot_port (default 127.0.0.1:5001) -- this launch file does not
start it.

Usage:
    ros2 launch parol6_moveit real_robot.launch.py
    ros2 launch parol6_moveit real_robot.launch.py robot_host:=192.168.1.50
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import FindExecutable, LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_host", default_value="127.0.0.1", description="parol6-server host"
        ),
        DeclareLaunchArgument(
            "robot_port", default_value="5001", description="parol6-server UDP port"
        ),
        DeclareLaunchArgument(
            "bridge_host",
            default_value="127.0.0.1",
            description="address parol6_bridge listens on (loopback only, unauthenticated)",
        ),
        DeclareLaunchArgument(
            "bridge_port", default_value="6001", description="port parol6_bridge listens on"
        ),
    ]

    # parol6_bridge is a plain asyncio daemon, not an rclpy node, so it's run
    # via ExecuteProcess rather than launch_ros's Node action.
    bridge_process = ExecuteProcess(
        cmd=[
            FindExecutable(name="parol6_bridge"),
            "--robot-host",
            LaunchConfiguration("robot_host"),
            "--robot-port",
            LaunchConfiguration("robot_port"),
            "--bind-host",
            LaunchConfiguration("bridge_host"),
            "--bind-port",
            LaunchConfiguration("bridge_port"),
        ],
        name="parol6_bridge",
        output="screen",
    )

    moveit_config = (
        MoveItConfigsBuilder("parol6", package_name="parol6_moveit")
        .robot_description(
            mappings={
                "use_real_hardware": "true",
                "bridge_host": LaunchConfiguration("bridge_host"),
                "bridge_port": LaunchConfiguration("bridge_port"),
            }
        )
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )

    demo_launch = generate_demo_launch(moveit_config)

    # Give parol6_bridge a couple seconds to bind and connect to
    # parol6-server before robot_state_publisher/controller_manager try to
    # reach it. Parol6SystemInterface also retries internally
    # (connect_timeout_sec in the xacro), so this is a courtesy, not a hard
    # requirement.
    delayed_demo = TimerAction(period=2.0, actions=list(demo_launch.entities))

    return LaunchDescription(declared_arguments + [bridge_process, delayed_demo])
