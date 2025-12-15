"""
RosKit Bridge Secure Launch File

Launch the Rust ROS2 web bridge with TLS and authentication enabled.

Usage:
    ros2 launch roskit_bridge_rs roskit_bridge_secure.launch.py \
        tls_cert:=/path/to/cert.pem \
        tls_key:=/path/to/key.pem \
        auth_token:=your_secret_token
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='9443',
        description='WebSocket server port (default 9443 for WSS)'
    )

    host_arg = DeclareLaunchArgument(
        'host',
        default_value='0.0.0.0',
        description='WebSocket server host address'
    )

    tls_cert_arg = DeclareLaunchArgument(
        'tls_cert',
        description='Path to TLS certificate file (required)'
    )

    tls_key_arg = DeclareLaunchArgument(
        'tls_key',
        description='Path to TLS private key file (required)'
    )

    auth_token_arg = DeclareLaunchArgument(
        'auth_token',
        default_value='',
        description='Authentication token (can also use ROSKIT_AUTH_TOKEN env var)'
    )

    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='roskit_bridge_secure',
        description='ROS2 node name for the bridge'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level (trace, debug, info, warn, error)'
    )

    # Get package share directory
    pkg_share = FindPackageShare('roskit_bridge_rs')

    # Build command with TLS arguments
    bridge_cmd = [
        PathJoinSubstitution([pkg_share, '..', '..', 'lib', 'roskit_bridge_rs', 'roskit_bridge']),
        '--port', LaunchConfiguration('port'),
        '--host', LaunchConfiguration('host'),
        '--node-name', LaunchConfiguration('node_name'),
        '--tls-cert', LaunchConfiguration('tls_cert'),
        '--tls-key', LaunchConfiguration('tls_key'),
    ]

    # Launch the bridge process
    bridge_process = ExecuteProcess(
        cmd=bridge_cmd,
        name='roskit_bridge_secure',
        output='screen',
        emulate_tty=True,
        env={
            'RUST_LOG': LaunchConfiguration('log_level'),
            'ROSKIT_AUTH_TOKEN': LaunchConfiguration('auth_token'),
        }
    )

    return LaunchDescription([
        # Arguments
        port_arg,
        host_arg,
        tls_cert_arg,
        tls_key_arg,
        auth_token_arg,
        node_name_arg,
        log_level_arg,

        # Log startup info
        LogInfo(msg=['Starting RosKit Bridge (Secure) on port ', LaunchConfiguration('port')]),
        LogInfo(msg=['TLS Certificate: ', LaunchConfiguration('tls_cert')]),

        # Bridge process
        bridge_process,
    ])
