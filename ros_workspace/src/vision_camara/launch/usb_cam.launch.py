"""Lanzamiento del publicador de imagenes de la camara USB."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Configura y lanza el nodo publicador usb_cam."""
    config_path = os.path.join(
        get_package_share_directory('vision_camara'),
        'config',
        'usb_cam.yaml',
    )

    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[config_path],
        ),
    ])