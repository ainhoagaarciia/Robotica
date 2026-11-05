from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    compartido = FindPackageShare("launcher_robots_lab_robotica")

    def camara(espacio, dispositivo):
        return GroupAction(
            [
                PushRosNamespace(espacio),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([compartido, "launch", "launch_cam.launch.py"])
                    ),
                    launch_arguments={"cam_path": dispositivo}.items(),
                ),
            ]
        )

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([compartido, "launch", "robot_205.launch.py"]))
    )

    return LaunchDescription([camara("cam1", "/dev/video1"), camara("cam2", "/dev/video5"), robot])
