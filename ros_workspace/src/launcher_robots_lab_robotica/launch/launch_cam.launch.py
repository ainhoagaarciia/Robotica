from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cam_path = LaunchConfiguration("cam_path")

    camara = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam",
        output="screen",
        parameters=[
            {
                "video_device": cam_path,
                "image_width": 640,
                "image_height": 480,
                "pixel_format": "yuyv",
                "camera_frame_id": "usb_cam",
                "io_method": "mmap",
            }
        ],
    )

    # Con el espacio de nombres (por ejemplo cam1) el topic es /cam1/usb_cam/image_raw
    visor = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        name="visor_imagen",
        output="screen",
        arguments=["usb_cam/image_raw"],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("cam_path", description="Dispositivo de la cámara, por ejemplo /dev/video0"),
            camara,
            visor,
        ]
    )
