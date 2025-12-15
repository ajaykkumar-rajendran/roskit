"""
RosKit Bridge (Python) Launch File

Launch the Python ROS2 web bridge.

Usage:
    ros2 launch roskit_bridge roskit_bridge.launch.py
    ros2 launch roskit_bridge roskit_bridge.launch.py port:=9091
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

    # Bridge node
    bridge_node = Node(
        package='roskit_bridge',
        executable='bridge',
        name='roskit_bridge',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'host': LaunchConfiguration('host'),
            'tls_cert': LaunchConfiguration('tls_cert'),
            'tls_key': LaunchConfiguration('tls_key'),
            'auth_token': LaunchConfiguration('auth_token'),
        }]
    )

    return LaunchDescription([
        # Arguments
        port_arg,
        host_arg,
        tls_cert_arg,
        tls_key_arg,
        auth_token_arg,

        # Log startup info
        LogInfo(msg=['Starting RosKit Bridge (Python) on port ', LaunchConfiguration('port')]),

        # Bridge node
        bridge_node,
    ])
