"""One-shot bringup of the entire PAROL6 real-hardware stack.

Starts, in order:
  1. parol6-server        (owns the USB serial connection to firmware)
  2. real_robot.launch.py (parol6_bridge, robot_state_publisher,
                            ros2_control_node, MoveIt2 move_group -- RViz
                            only if use_rviz:=true, see below)
  3. control_services.launch.py (home/gripper/disable_motors/enable_motors
                            Trigger services + generic CallMethod)
  4. dispenser_setup.launch.py (tray_origin + station TF frames, collision
                            scene -- without this, grid moves fail with
                            "tray_origin ... does not exist")
  5. dispenser_ui          (the web app itself)

use_rviz defaults to false: confirmed on real hardware that with RViz's
MotionPlanning display open, calling ~/home can be followed shortly after
by RViz's own MoveGroupInterface submitting an unprompted "Plan and Execute"
request -- no operator interaction -- driving the arm to an unexpected
pose. The identical stack minus RViz never reproduces this. Root cause is
inside moveit_rviz_plugin (upstream MoveIt2, not this workspace's code),
not chased down further than confirming RViz is the trigger. Pass
use_rviz:=true only when you can watch it happen and are prepared to hit
the E-stop -- and be aware the physical E-stop itself only cancels
whatever command is active *at the moment it's pressed*; it doesn't stop
ros2_control from submitting another one shortly after (a separate,
confirmed gap in parol6-server's E-stop handling, not yet fixed).

This replaces manually opening ~6 terminals for parol6-server,
real_robot.launch.py, control_services.launch.py, dispenser_setup.launch.py,
and dispenser_ui.

parol6-server needs a few seconds to bind its port (JIT warmup + serial
connect) before parol6_bridge can connect to it, so step 2 is delayed by
`server_startup_delay` seconds rather than raced against it.

Deliberately does NOT auto-home: homing drives the arm to its limit
switches, and doing that unattended the instant a launch file starts is a
real way to hit something nobody checked was clear first. Home manually
once this is up:
    ros2 service call /parol6_control_services/home std_srvs/srv/Trigger {}

Usage:
    ros2 launch parol6_moveit full_stack.launch.py
    ros2 launch parol6_moveit full_stack.launch.py serial_port:=/dev/ttyUSB0
    ros2 launch parol6_moveit full_stack.launch.py launch_dispenser_ui:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    robot_host = LaunchConfiguration("robot_host")
    robot_port = LaunchConfiguration("robot_port")

    declared_arguments = [
        DeclareLaunchArgument(
            "server_executable",
            default_value=os.path.expanduser("~/parol6-server/.venv/bin/parol6-server"),
            description="Path to the parol6-server console script (its own venv, not a colcon package)",
        ),
        DeclareLaunchArgument(
            "serial_port", default_value="/dev/ttyACM0", description="Serial port to the control board"
        ),
        DeclareLaunchArgument(
            "server_startup_delay",
            default_value="4.0",
            description="Seconds to wait for parol6-server to bind its port before starting everything that connects to it",
        ),
        DeclareLaunchArgument("robot_host", default_value="127.0.0.1", description="parol6-server host"),
        DeclareLaunchArgument("robot_port", default_value="5001", description="parol6-server UDP port"),
        DeclareLaunchArgument(
            "bridge_python",
            default_value=os.path.expanduser("~/parol6-server/.venv/bin/python3"),
            description="Interpreter with the 'parol6' package importable, for parol6_bridge",
        ),
        DeclareLaunchArgument(
            "use_rviz", default_value="false",
            description=(
                "Launch RViz alongside move_group. Defaults off: confirmed on real "
                "hardware that RViz's MotionPlanning display can submit its own "
                "\"Plan and Execute\" request with no operator interaction shortly "
                "after a home() call, driving the arm to an unexpected pose. Not "
                "reproducible with the same stack minus RViz. Opt in deliberately "
                "(use_rviz:=true) only when you can watch RViz and are prepared "
                "for this."
            ),
        ),
        DeclareLaunchArgument("gripper_io_pin", default_value="0"),
        DeclareLaunchArgument(
            "launch_dispenser_ui", default_value="true", description="Also start the dispenser web app"
        ),
    ]

    parol6_server = ExecuteProcess(
        cmd=[LaunchConfiguration("server_executable"), "--serial", LaunchConfiguration("serial_port")],
        name="parol6-server",
        output="screen",
    )

    real_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("parol6_moveit"), "launch", "real_robot.launch.py")
        ),
        launch_arguments={
            "robot_host": robot_host,
            "robot_port": robot_port,
            "bridge_python": LaunchConfiguration("bridge_python"),
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
    )

    control_services = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("parol6_control_services"),
                "launch",
                "control_services.launch.py",
            )
        ),
        launch_arguments={
            "robot_host": robot_host,
            "robot_port": robot_port,
            "gripper_io_pin": LaunchConfiguration("gripper_io_pin"),
        }.items(),
    )

    dispenser_setup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("parol6_dispenser"),
                "launch",
                "dispenser_setup.launch.py",
            )
        ),
    )

    dispenser_ui = Node(
        package="parol6_dispenser",
        executable="dispenser_ui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("launch_dispenser_ui")),
    )

    delayed = TimerAction(
        period=PythonExpression([LaunchConfiguration("server_startup_delay")]),
        actions=[real_robot, control_services, dispenser_setup, dispenser_ui],
    )

    return LaunchDescription(declared_arguments + [parol6_server, delayed])
