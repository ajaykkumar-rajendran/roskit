"""
RosKit Bridge Launch File

Launch the high-performance Rust ROS2 web bridge.

Usage:
    ros2 launch roskit_bridge_rs roskit_bridge.launch.py
    ros2 launch roskit_bridge_rs roskit_bridge.launch.py port:=9091
    ros2 launch roskit_bridge_rs roskit_bridge.launch.py tls_cert:=/path/to/cert.pem tls_key:=/path/to/key.pem
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='9090',
        description='WebSocket server port'
    )

    host_arg = DeclareLaunchArgument(
        'host',
        default_value='0.0.0.0',
        description='WebSocket server host address'
    )

    tls_cert_arg = DeclareLaunchArgument(
        'tls_cert',
        default_value='',
        description='Path to TLS certificate file (enables WSS)'
    )

    tls_key_arg = DeclareLaunchArgument(
        'tls_key',
        default_value='',
        description='Path to TLS private key file'
    )

    auth_token_arg = DeclareLaunchArgument(
        'auth_token',
        default_value='',
        description='Authentication token (empty = no auth)'
    )

    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Enable verbose logging'
    )

    # Get package share directory for the binary
    pkg_share = FindPackageShare('roskit_bridge_rs')

    # Build command with arguments
    bridge_cmd = [
        PathJoinSubstitution([pkg_share, '..', '..', 'lib', 'roskit_bridge_rs', 'roskit_bridge']),
        '--port', LaunchConfiguration('port'),
        '-H', LaunchConfiguration('host'),
    ]

    # Launch the bridge process
    bridge_process = ExecuteProcess(
        cmd=bridge_cmd,
        name='roskit_bridge',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        # Arguments
        port_arg,
        host_arg,
        tls_cert_arg,
        tls_key_arg,
        auth_token_arg,
        verbose_arg,

        # Log startup info
        LogInfo(msg=['Starting RosKit Bridge on port ', LaunchConfiguration('port')]),

        # Bridge process
        bridge_process,
    ])
