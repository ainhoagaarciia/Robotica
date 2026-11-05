"""Lógica común de los launch de los robots del laboratorio (reales y simulados)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launcher_robots_lab_robotica.robots import ROBOTS, TOOL_TCP_PORT

PAQUETE = "launcher_robots_lab_robotica"


def generar_lanzamiento(id_robot, simulacion):
    """Devuelve la LaunchDescription del robot `id_robot` ("203" ... "206").

    Si `simulacion` es True se usa el hardware simulado (mock) de ros2_control y la
    pinza simulada; si no, se conecta al robot y a la pinza reales.
    """
    cfg = ROBOTS[id_robot]
    compartido = FindPackageShare(PAQUETE)

    argumentos = [
        DeclareLaunchArgument(
            "robot_ip",
            default_value=cfg["robot_ip"],
            description="IP del robot (no se usa en simulación).",
        ),
        DeclareLaunchArgument("launch_rviz", default_value="true", description="¿Lanzar RViz?"),
    ]

    # Se aísla en un GroupAction para que sus argumentos (p. ej. launch_rviz=false) no se filtren al resto.
    ur_control_incluido = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py"])
        ),
        launch_arguments={
            "ur_type": "ur3e",
            "robot_ip": LaunchConfiguration("robot_ip"),
            # description_file y kinematics_params_file los lee ur_rsp.launch.py,
            # incluido desde ur_control.launch.py
            "description_file": PathJoinSubstitution(
                [compartido, "urdf", f"ur3e_rg2_{id_robot}.urdf.xacro"]
            ),
            "kinematics_params_file": PathJoinSubstitution(
                [compartido, "calibration", f"calibration_{id_robot}.yaml"]
            ),
            "reverse_port": cfg["reverse_port"],
            "script_sender_port": cfg["script_sender_port"],
            "trajectory_port": cfg["trajectory_port"],
            "tool_tcp_port": TOOL_TCP_PORT,
            "use_mock_hardware": "true" if simulacion else "false",
            "launch_rviz": "false",
            "launch_dashboard_client": "false" if simulacion else "true",
        }.items(),
    )

    ur_control = GroupAction([ur_control_incluido], scoped=True)

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([compartido, "launch", "moveit.launch.py"])),
        launch_arguments={"launch_rviz": LaunchConfiguration("launch_rviz")}.items(),
    )

    rg2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("rg2_driver"), "launch", "rg2_driver.launch.py"])
        ),
        launch_arguments={
            "robot_ip": LaunchConfiguration("robot_ip"),
            "simulate": "true" if simulacion else "false",
        }.items(),
    )

    return LaunchDescription(argumentos + [ur_control, moveit, rg2])
