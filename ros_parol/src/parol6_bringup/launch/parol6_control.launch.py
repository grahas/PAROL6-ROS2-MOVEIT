"""Bring up the PAROL6 arm under ros2_control, no MoveIt involved.

Starts, in order:
  1. parol6_bridge  (TCP<->UDP daemon talking to parol6-server)
  2. robot_state_publisher (publishes /robot_description + TF)
  3. ros2_control_node (controller_manager), loaded with
     parol6_hardware_interface as the hardware plugin
  4. joint_state_broadcaster and parol6_arm_controller, spawned once
     the controller_manager is up

Assumes a parol6-server is already running and reachable at
robot_host:robot_port (default 127.0.0.1:5001) -- this launch file does
not start it.

For MoveIt-based planning instead of this bare ros2_control setup, see
parol6_moveit's real_robot.launch.py (requires MoveIt2 to be installed).
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_robot_description(context):
    bringup_share = get_package_share_directory("parol6_bringup")
    xacro_path = os.path.join(bringup_share, "urdf", "parol6_with_tools.xacro")

    mappings = {
        "tool": LaunchConfiguration("tool").perform(context),
        "tool_variant": LaunchConfiguration("tool_variant").perform(context),
        "bridge_host": LaunchConfiguration("bridge_host").perform(context),
        "bridge_port": LaunchConfiguration("bridge_port").perform(context),
    }
    doc = xacro.process_file(xacro_path, mappings=mappings)
    return doc.toxml()


def _launch_setup(context, *args, **kwargs):
    robot_description = _build_robot_description(context)

    # parol6_bridge is a plain asyncio daemon, not an rclpy node, so it's run
    # via ExecuteProcess rather than launch_ros's Node action. It's launched
    # with `python3 -m parol6_bridge.bridge_node` (via bridge_python, an
    # explicit interpreter path) rather than the installed console-script
    # entry point, because:
    #   1. the parol6 SDK dependency may only be installed in a specific
    #      venv (not necessarily the one running ROS2 itself), and
    #   2. launch's FindExecutable does a literal $PATH search, which
    #      doesn't know about ROS's ament-index executable resolution (the
    #      mechanism `ros2 run` itself uses) -- so it can't reliably find
    #      colcon-installed console scripts anyway.
    bridge_process = ExecuteProcess(
        cmd=[
            LaunchConfiguration("bridge_python"),
            "-m",
            "parol6_bridge.bridge_node",
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

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    controllers_yaml = os.path.join(
        get_package_share_directory("parol6_bringup"), "config", "parol6_controllers.yaml"
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        # robot_description is intentionally not passed here -- controller_manager
        # subscribes to the '/robot_description' topic (published by
        # robot_state_publisher, below) for that instead.
        #
        # As of this ros2_control generation, controller_manager no longer
        # auto-forwards its own --params-file to spawned controller
        # sub-nodes (see determine_controller_node_options() in
        # controller_manager.cpp: use_global_arguments(false)). Each
        # controller that needs its own parameters (i.e. anything other
        # than joint_state_broadcaster, which tolerates an empty config)
        # must be told where to find them via a `<controller_name>.params_file`
        # parameter declared on controller_manager itself -- hence the
        # second dict below, pointing parol6_arm_controller back at the
        # same file. See README.md for how this was diagnosed.
        parameters=[
            controllers_yaml,
            {"parol6_arm_controller.params_file": controllers_yaml},
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["parol6_arm_controller"],
        output="screen",
    )

    return [
        bridge_process,
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "tool",
            default_value="none",
            description=(
                "End-of-arm tool mounted on L6: none, pneumatic, ssg48, msg, or vacuum. "
                "Must match what's actually bolted onto the robot and what you pass to "
                "RobotClient.select_tool() -- see parol6_with_tools.xacro and README.md."
            ),
        ),
        DeclareLaunchArgument(
            "tool_variant",
            default_value="",
            description=(
                "Tool variant, meaning depends on 'tool': pneumatic -> vertical/horizontal, "
                "ssg48 -> finger/pinch, msg -> 100mm/150mm/200mm. Leave empty for the "
                "per-tool default. Ignored for tool=none/vacuum."
            ),
        ),
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
        DeclareLaunchArgument(
            "bridge_python",
            default_value="python3",
            description=(
                "Python interpreter to run parol6_bridge with -- must have the 'parol6' pip "
                "package (and parol6_bridge itself, e.g. via 'pip install -e parol6_bridge/') "
                "importable. Point this at a venv's python if parol6 isn't installed for the "
                "system interpreter, e.g. /home/you/parol6-server/.venv/bin/python3"
            ),
        ),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=_launch_setup)])
