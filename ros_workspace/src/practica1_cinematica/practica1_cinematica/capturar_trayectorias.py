#!/usr/bin/env python3

"""Nodo ROS 2 para ejecutar trayectorias y registrar estados articulares."""
import csv
import math
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, TypedDict
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes, RobotTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
JOINT_STATES_TOPIC = '/joint_states'
MOVE_GROUP_ACTION = 'move_action'
EXECUTE_TRAJECTORY_ACTION = 'execute_trajectory'
PLANNING_GROUP = 'robot'
MESSAGE_QUEUE_SIZE = 10
ACTION_SERVER_TIMEOUT_SEC = 30.0
STATE_WAIT_TIMEOUT_SEC = 20.0
STATE_POLL_INTERVAL_SEC = 0.05
FUTURE_POLL_INTERVAL_SEC = 0.01
CAPTURE_SETTLE_TIME_SEC = 0.2
TRAJECTORY_SETTLE_TIME_SEC = 0.1
VELOCITY_SCALE = 0.1
ACCELERATION_SCALE = 0.1
JOINT_GOAL_TOLERANCE_RAD = 0.01
INITIAL_JOINT_TARGET_RAD = [0.0, -math.pi / 2, -math.pi / 2, -math.pi / 2, math.pi / 2, 0.0]

class MuestraArticular(TypedDict):
    """Fila de datos capturada para una articulacion y una marca temporal."""
    trayectoria: str
    tiempo_s: float
    fuente_tiempo: str
    articulacion: str
    posicion_rad: float
    velocidad_rad_s: float | None

class CapturadorTrayectorias(Node):

    def __init__(self) -> None:
        """Prepara la captura Joint con MoveIt."""
        super().__init__('capturador_trayectorias')
        self._lock = threading.Lock()
        self._estado: dict[str, float] = {}
        self._capturando = False
        self._trayectoria = ''
        self._muestras: list[MuestraArticular] = []
        self.create_subscription(JointState, JOINT_STATES_TOPIC, self._al_recibir_estado, MESSAGE_QUEUE_SIZE)
        self._move_group = ActionClient(self, MoveGroup, MOVE_GROUP_ACTION)
        self._execute = ActionClient(self, ExecuteTrajectory, EXECUTE_TRAJECTORY_ACTION)
        self.get_logger().info('Esperando los servicios y acciones de MoveIt...')
        if not self._move_group.wait_for_server(timeout_sec=ACTION_SERVER_TIMEOUT_SEC):
            raise RuntimeError('No se encontró la acción move_action. ¿Está activa la simulación?')
        if not self._execute.wait_for_server(timeout_sec=ACTION_SERVER_TIMEOUT_SEC):
            raise RuntimeError('No se encontró la acción execute_trajectory.')

    def esperar_estado_inicial(self) -> None:
        """Espera hasta recibir el estado de las articulaciones del brazo."""
        limite = time.monotonic() + STATE_WAIT_TIMEOUT_SEC
        while time.monotonic() < limite:
            with self._lock:
                if all((nombre in self._estado for nombre in JOINTS)):
                    return
            time.sleep(STATE_POLL_INTERVAL_SEC)
        raise RuntimeError('No llegaron todas las articulaciones por /joint_states.')

    def _al_recibir_estado(self, msg: JointState) -> None:
        """Registra posición y velocidad mientras se captura la trayectoria."""
        tiempo_ns = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        fuente_tiempo = 'joint_states.header.stamp'
        if tiempo_ns == 0:
            tiempo_ns = self.get_clock().now().nanoseconds
            fuente_tiempo = 'reloj_ros'
        tiempo_s = tiempo_ns / 1000000000
        with self._lock:
            for indice, nombre in enumerate(msg.name):
                if nombre in JOINTS and indice < len(msg.position):
                    self._estado[nombre] = msg.position[indice]
            if not self._capturando:
                return
            for indice, nombre in enumerate(msg.name):
                if nombre not in JOINTS or indice >= len(msg.position):
                    continue
                velocidad = msg.velocity[indice] if indice < len(msg.velocity) else None
                self._muestras.append({'trayectoria': self._trayectoria, 'tiempo_s': tiempo_s, 'fuente_tiempo': fuente_tiempo, 'articulacion': nombre, 'posicion_rad': msg.position[indice], 'velocidad_rad_s': velocidad})

    def _iniciar_captura(self, nombre: str) -> None:
        """Prepara las muestras de la siguiente trayectoria."""
        with self._lock:
            self._trayectoria = nombre
            self._muestras = []
            self._capturando = True

    def _terminar_captura(self) -> list[MuestraArticular]:
        """Desactiva la captura y devuelve una copia del buffer recopilado."""
        with self._lock:
            self._capturando = False
            return list(self._muestras)

    @staticmethod
    def _esperar(future: Any) -> Any:
        """Espera una operacion ROS sin bloquear el executor de callbacks."""
        while not future.done():
            time.sleep(FUTURE_POLL_INTERVAL_SEC)
        return future.result()

    def _planificar_articulaciones(self, objetivo: list[float]) -> RobotTrajectory:
        """Planifica un objetivo articular sin ejecutarlo."""
        if len(objetivo) != len(JOINTS):
            raise ValueError(f'Se esperaban {len(JOINTS)} valores articulares; se recibieron {len(objetivo)}.')
        restricciones = Constraints()
        for nombre, valor in zip(JOINTS, objetivo):
            restricciones.joint_constraints.append(JointConstraint(joint_name=nombre, position=float(valor), tolerance_above=JOINT_GOAL_TOLERANCE_RAD, tolerance_below=JOINT_GOAL_TOLERANCE_RAD, weight=1.0))
        goal = MoveGroup.Goal()
        goal.request.group_name = PLANNING_GROUP
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = VELOCITY_SCALE
        goal.request.max_acceleration_scaling_factor = ACCELERATION_SCALE
        goal.request.goal_constraints.append(restricciones)
        goal.request.start_state.is_diff = True
        goal.planning_options.plan_only = True
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        handle = self._esperar(self._move_group.send_goal_async(goal))
        if not handle.accepted:
            raise RuntimeError('MoveIt rechazó la planificación articular.')
        resultado = self._esperar(handle.get_result_async()).result
        if resultado.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f'Falló la planificación articular (código {resultado.error_code.val}).')
        return resultado.planned_trajectory

    def _ejecutar_y_guardar(self, nombre: str, trayectoria: RobotTrajectory) -> None:
        """Ejecuta una trayectoria y guarda las muestras en un CSV."""
        self._iniciar_captura(nombre)
        try:
            goal = ExecuteTrajectory.Goal()
            goal.trajectory = trayectoria
            handle = self._esperar(self._execute.send_goal_async(goal))
            if not handle.accepted:
                raise RuntimeError(f'MoveIt rechazó la trayectoria «{nombre}».')
            resultado = self._esperar(handle.get_result_async()).result
            if resultado.error_code.val != MoveItErrorCodes.SUCCESS:
                raise RuntimeError(f'Falló la ejecución de la trayectoria «{nombre}»: código MoveIt {resultado.error_code.val}.')
            time.sleep(TRAJECTORY_SETTLE_TIME_SEC)
        finally:
            time.sleep(CAPTURE_SETTLE_TIME_SEC)
            muestras = self._terminar_captura()
            self._guardar_csv(nombre, muestras)
        self.get_logger().info(f'Trayectoria «{nombre}» completada.')

    @staticmethod
    def _guardar_csv(nombre: str, muestras: list[MuestraArticular]) -> None:
        """Escribe un CSV independiente para la trayectoria."""
        carpeta = Path.home() / 'ros_workspace' / 'datos_trayectorias'
        carpeta.mkdir(parents=True, exist_ok=True)
        marca_tiempo = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        ruta = carpeta / f'{marca_tiempo}_{nombre}.csv'
        columnas = ['trayectoria', 'tiempo_s', 'fuente_tiempo', 'articulacion', 'posicion_rad', 'velocidad_rad_s']
        with ruta.open('w', newline='', encoding='utf-8') as archivo:
            escritor = csv.DictWriter(archivo, fieldnames=columnas)
            escritor.writeheader()
            escritor.writerows(muestras)
        print(f'Datos guardados: {ruta} ({len(muestras)} filas)')

    def ejecutar_practica(self) -> None:
        """Planifica y captura Joint."""
        self.get_logger().info('Planificando movimiento articular...')
        objetivo = list(INITIAL_JOINT_TARGET_RAD)
        plan_articular = self._planificar_articulaciones(objetivo)
        self._ejecutar_y_guardar('joint', plan_articular)

def main(args: list[str] | None=None) -> None:
    """Inicializa ROS 2, ejecuta la practica y libera sus recursos."""
    rclpy.init(args=args)
    node: CapturadorTrayectorias | None = None
    executor = SingleThreadedExecutor()
    hilo: threading.Thread | None = None
    try:
        node = CapturadorTrayectorias()
        executor.add_node(node)
        hilo = threading.Thread(target=executor.spin, daemon=True)
        hilo.start()
        node.esperar_estado_inicial()
        node.ejecutar_practica()
    except Exception as error:
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(f'Error al iniciar el capturador: {error}')
        raise
    finally:
        executor.shutdown(timeout_sec=5.0)
        if hilo is not None:
            hilo.join(timeout=5.0)
        if node is not None:
            executor.remove_node(node)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
if __name__ == '__main__':
    main()