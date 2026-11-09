from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ip", default_value="", description="IP del robot al que está conectada la pinza"),
            DeclareLaunchArgument("simulate", default_value="false", description="Simular la pinza (sin robot)"),
            DeclareLaunchArgument("prefix", default_value="", description="Prefijo de las articulaciones de la pinza"),
            DeclareLaunchArgument("namespace", default_value="", description="Espacio de nombres del nodo"),
            Node(
                package="rg2_driver",
                executable="rg2_control_node",
                name="rg2_control_node",
                namespace=LaunchConfiguration("namespace"),
                output="screen",
                parameters=[
                    {
                        "prefix": ParameterValue(LaunchConfiguration("prefix"), value_type=str),
                        "robot_ip": ParameterValue(LaunchConfiguration("robot_ip"), value_type=str),
                        "simulate": ParameterValue(LaunchConfiguration("simulate"), value_type=bool),
                    }
                ],
            ),
        ]
    )
