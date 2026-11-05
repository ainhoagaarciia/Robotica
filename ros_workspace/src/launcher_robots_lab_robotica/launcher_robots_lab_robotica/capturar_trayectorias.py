#!/usr/bin/env python3
"""Nodo ROS 2 para ejecutar trayectorias y registrar estados articulares."""

import copy
import csv
import math
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, TypedDict

from geometry_msgs.msg import Pose

from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
    RobotTrajectory,
)
from moveit_msgs.srv import GetCartesianPath

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time

from robotica_msgs.msg import JointMotion

from sensor_msgs.msg import JointState

from tf2_ros import Buffer, TransformListener


JOINTS = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]
REFERENCE_FRAME = 'world'
END_EFFECTOR_LINK = 'wrist_3_link'
JOINT_STATES_TOPIC = '/joint_states'
JOINT_MOTION_TOPIC = '/joint_motion'
MOVE_GROUP_ACTION = 'move_action'
EXECUTE_TRAJECTORY_ACTION = 'execute_trajectory'
CARTESIAN_PATH_SERVICE = 'compute_cartesian_path'
PLANNING_GROUP = 'robot'

MESSAGE_QUEUE_SIZE = 10
ACTION_SERVER_TIMEOUT_SEC = 30.0
STATE_WAIT_TIMEOUT_SEC = 20.0
STATE_POLL_INTERVAL_SEC = 0.05
TRANSFORM_TIMEOUT_SEC = 5.0
FUTURE_POLL_INTERVAL_SEC = 0.01
CAPTURE_SETTLE_TIME_SEC = 0.2
TRAJECTORY_SETTLE_TIME_SEC = 0.1

VELOCITY_SCALE = 0.1
ACCELERATION_SCALE = 0.1
JOINT_GOAL_TOLERANCE_RAD = 0.01
CARTESIAN_MAX_STEP_M = 0.001
CARTESIAN_MIN_FRACTION = 0.999
LINEAR_OFFSET_M = 0.01
CIRCLE_RADIUS_M = 0.01
CIRCLE_POINT_COUNT = 32
MOVEMENT_VELOCITY_THRESHOLD_RAD_S = 0.001
MOVEMENT_POSITION_THRESHOLD_RAD = 1e-5

INITIAL_JOINT_TARGET_RAD = [
    0.0,
    -math.pi / 2,
    -math.pi / 2,
    -math.pi / 2,
    math.pi / 2,
    0.0,
]


class MuestraArticular(TypedDict):
    """Fila de datos capturada para una articulacion y una marca temporal."""

    trayectoria: str
    tiempo_s: float
    fuente_tiempo: str
    articulacion: str
    posicion_rad: float
    velocidad_rad_s: float | None
    aceleracion_rad_s2: float | None


RutaCartesiana = tuple[str, list[Pose]]


class CapturadorTrayectorias(Node):
    """Ejecuta trayectorias MoveIt y guarda estados articulares en CSV."""

    def __init__(self) -> None:
        """Inicializa la suscripcion, los clientes de MoveIt y TF2."""
        super().__init__('capturador_trayectorias')
        self._lock = threading.Lock()
        self._estado: dict[str, float] = {}
        self._capturando = False
        self._trayectoria = ''
        self._muestras: list[MuestraArticular] = []
        self._velocidad_anterior: dict[str, tuple[float, float]] = {}
        self._estado_anterior: dict[str, tuple[float, float]] = {}

        self.create_subscription(
            JointState,
            JOINT_STATES_TOPIC,
            self._al_recibir_estado,
            MESSAGE_QUEUE_SIZE,
        )
        self._publicador_movimiento = self.create_publisher(
            JointMotion,
            JOINT_MOTION_TOPIC,
            MESSAGE_QUEUE_SIZE,
        )
        self._move_group = ActionClient(self, MoveGroup, MOVE_GROUP_ACTION)
        self._execute = ActionClient(
            self,
            ExecuteTrajectory,
            EXECUTE_TRAJECTORY_ACTION,
        )
        self._cartesian = self.create_client(
            GetCartesianPath,
            CARTESIAN_PATH_SERVICE,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.get_logger().info(
            'Esperando los servicios y acciones de MoveIt...'
        )
        if not self._move_group.wait_for_server(
            timeout_sec=ACTION_SERVER_TIMEOUT_SEC
        ):
            raise RuntimeError(
                'No se encontró la acción move_action. '
                '¿Está activa la simulación?'
            )
        if not self._execute.wait_for_server(
            timeout_sec=ACTION_SERVER_TIMEOUT_SEC
        ):
            raise RuntimeError('No se encontró la acción execute_trajectory.')
        if not self._cartesian.wait_for_service(
            timeout_sec=ACTION_SERVER_TIMEOUT_SEC
        ):
            raise RuntimeError(
                'No se encontró el servicio compute_cartesian_path.'
            )

    def esperar_estado_inicial(self) -> None:
        """Espera hasta recibir el estado de las articulaciones del brazo."""
        limite = time.monotonic() + STATE_WAIT_TIMEOUT_SEC
        while time.monotonic() < limite:
            with self._lock:
                if all(nombre in self._estado for nombre in JOINTS):
                    return
            time.sleep(STATE_POLL_INTERVAL_SEC)
        raise RuntimeError(
            'No llegaron todas las articulaciones por /joint_states.'
        )

    def _al_recibir_estado(self, msg: JointState) -> None:
        """Publica y guarda muestras solo mientras detecta movimiento."""
        tiempo_ns = (
            msg.header.stamp.sec * 1_000_000_000
            + msg.header.stamp.nanosec
        )
        fuente_tiempo = 'joint_states.header.stamp'
        sello_tiempo = copy.deepcopy(msg.header)
        if tiempo_ns == 0:
            sello_tiempo.stamp = self.get_clock().now().to_msg()
            tiempo_ns = (
                sello_tiempo.stamp.sec * 1_000_000_000
                + sello_tiempo.stamp.nanosec
            )
            fuente_tiempo = 'reloj_ros'
        tiempo_s = tiempo_ns / 1_000_000_000
        articulaciones: list[
            tuple[str, float, float | None, float | None]
        ] = []
        muestras_movimiento: list[JointMotion] = []

        with self._lock:
            hay_movimiento = False
            # El estado previo permite detectar movimiento entre mensajes.
            for indice, nombre in enumerate(msg.name):
                if (
                    nombre not in JOINTS
                    or indice >= len(msg.position)
                ):
                    continue
                posicion = msg.position[indice]
                self._estado[nombre] = posicion

                anterior = self._estado_anterior.get(nombre)
                velocidad_estimada: float | None = None
                aceleracion: float | None = None
                if anterior is not None:
                    tiempo_anterior, posicion_anterior = anterior
                    dt = tiempo_s - tiempo_anterior
                    if dt > 0:
                        desplazamiento = posicion - posicion_anterior
                        if (
                            abs(desplazamiento)
                            > MOVEMENT_POSITION_THRESHOLD_RAD
                        ):
                            velocidad_estimada = desplazamiento / dt

                velocidad_mensaje = (
                    msg.velocity[indice]
                    if indice < len(msg.velocity)
                    else None
                )
                velocidad = velocidad_mensaje
                if (
                    velocidad is None
                    or (
                        abs(velocidad)
                        <= MOVEMENT_VELOCITY_THRESHOLD_RAD_S
                        and velocidad_estimada is not None
                    )
                ):
                    velocidad = velocidad_estimada

                articulacion_en_movimiento = (
                    velocidad is not None
                    and abs(velocidad) > MOVEMENT_VELOCITY_THRESHOLD_RAD_S
                )
                hay_movimiento = hay_movimiento or articulacion_en_movimiento

                velocidad_previa = self._velocidad_anterior.get(nombre)
                if (
                    articulacion_en_movimiento
                    and velocidad is not None
                    and velocidad_previa is not None
                ):
                    tiempo_velocidad_anterior, valor_velocidad_anterior = (
                        velocidad_previa
                    )
                    dt_velocidad = tiempo_s - tiempo_velocidad_anterior
                    if dt_velocidad > 0:
                        aceleracion = (
                            velocidad - valor_velocidad_anterior
                        ) / dt_velocidad

                self._estado_anterior[nombre] = (tiempo_s, posicion)
                self._velocidad_anterior[nombre] = (
                    tiempo_s,
                    velocidad if velocidad is not None else 0.0,
                )
                articulaciones.append(
                    (nombre, posicion, velocidad, aceleracion)
                )

            if not self._capturando or not hay_movimiento:
                return

            for nombre, posicion, velocidad, aceleracion in articulaciones:
                if velocidad is None:
                    velocidad = 0.0
                self._muestras.append(
                    {
                        'trayectoria': self._trayectoria,
                        'tiempo_s': tiempo_s,
                        'fuente_tiempo': fuente_tiempo,
                        'articulacion': nombre,
                        'posicion_rad': posicion,
                        'velocidad_rad_s': velocidad,
                        'aceleracion_rad_s2': aceleracion,
                    }
                )
                muestra = JointMotion()
                muestra.header = copy.deepcopy(sello_tiempo)
                muestra.trajectory = self._trayectoria
                muestra.joint_name = nombre
                muestra.position_rad = posicion
                muestra.velocity_rad_s = velocidad
                muestra.acceleration_rad_s2 = (
                    aceleracion if aceleracion is not None else 0.0
                )
                muestra.acceleration_valid = aceleracion is not None
                muestras_movimiento.append(muestra)

        for muestra in muestras_movimiento:
            self._publicador_movimiento.publish(muestra)

    def _iniciar_captura(self, nombre: str) -> None:
        """Reinicia el buffer y activa la captura para una trayectoria."""
        with self._lock:
            self._trayectoria = nombre
            self._muestras = []
            self._velocidad_anterior = {}
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

    def _posiciones_actuales(self) -> list[float]:
        """Devuelve las posiciones actuales en el orden definido por JOINTS."""
        with self._lock:
            return [self._estado[nombre] for nombre in JOINTS]

    def _planificar_articulaciones(
        self,
        objetivo: list[float],
    ) -> RobotTrajectory:
        """Planifica un objetivo articular sin ejecutarlo."""
        if len(objetivo) != len(JOINTS):
            raise ValueError(
                f'Se esperaban {len(JOINTS)} valores articulares; '
                f'se recibieron {len(objetivo)}.'
            )

        restricciones = Constraints()
        for nombre, valor in zip(JOINTS, objetivo):
            restricciones.joint_constraints.append(
                JointConstraint(
                    joint_name=nombre,
                    position=float(valor),
                    tolerance_above=JOINT_GOAL_TOLERANCE_RAD,
                    tolerance_below=JOINT_GOAL_TOLERANCE_RAD,
                    weight=1.0,
                )
            )

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
            raise RuntimeError(
                'Falló la planificación articular '
                f'(código {resultado.error_code.val}).'
            )
        return resultado.planned_trajectory

    def _pose_actual(self) -> Pose:
        """Obtiene en el frame de referencia la pose actual del efector."""
        transformacion = self._tf_buffer.lookup_transform(
            REFERENCE_FRAME,
            END_EFFECTOR_LINK,
            Time(),
            timeout=Duration(seconds=TRANSFORM_TIMEOUT_SEC),
        )
        pose = Pose()
        pose.position.x = transformacion.transform.translation.x
        pose.position.y = transformacion.transform.translation.y
        pose.position.z = transformacion.transform.translation.z
        pose.orientation = transformacion.transform.rotation
        return pose

    def _solicitar_cartesiana(
        self,
        poses: list[Pose],
    ) -> GetCartesianPath.Response:
        """Solicita una ruta cartesiana desde el estado actual del robot."""
        peticion = GetCartesianPath.Request()
        peticion.header.frame_id = REFERENCE_FRAME
        peticion.start_state.joint_state.header.stamp = (
            self.get_clock().now().to_msg()
        )
        with self._lock:
            peticion.start_state.joint_state.name = list(JOINTS)
            peticion.start_state.joint_state.position = [
                self._estado[nombre] for nombre in JOINTS
            ]
        peticion.start_state.is_diff = True
        peticion.group_name = PLANNING_GROUP
        peticion.link_name = END_EFFECTOR_LINK
        peticion.waypoints = [self._pose_actual(), *poses]
        peticion.max_step = CARTESIAN_MAX_STEP_M
        peticion.jump_threshold = 0.0
        peticion.avoid_collisions = True
        peticion.max_velocity_scaling_factor = VELOCITY_SCALE
        peticion.max_acceleration_scaling_factor = ACCELERATION_SCALE

        respuesta = self._esperar(self._cartesian.call_async(peticion))
        return respuesta

    def _planificar_cartesiana(
        self,
        candidatas: list[RutaCartesiana],
    ) -> RobotTrajectory:
        """Selecciona la primera ruta cartesiana que MoveIt completa."""
        resultados: list[str] = []
        for descripcion, poses in candidatas:
            respuesta = self._solicitar_cartesiana(poses)
            if respuesta.fraction >= CARTESIAN_MIN_FRACTION:
                self.get_logger().info(
                    f'Recorrido cartesiano planificado: {descripcion}.'
                )
                return respuesta.solution
            resultados.append(
                f'{descripcion}: {respuesta.fraction:.1%} '
                f'(código {respuesta.error_code.val})'
            )

        detalle = '; '.join(resultados)
        raise RuntimeError(
            'MoveIt no pudo planificar ninguno de los recorridos cartesianos '
            f'probados: {detalle}.'
        )

    def _ejecutar_y_guardar(
        self,
        nombre: str,
        trayectoria: RobotTrajectory,
    ) -> None:
        """Ejecuta una trayectoria y guarda las muestras en un CSV."""
        self._iniciar_captura(nombre)
        try:
            goal = ExecuteTrajectory.Goal()
            goal.trajectory = trayectoria
            handle = self._esperar(self._execute.send_goal_async(goal))
            if not handle.accepted:
                raise RuntimeError(
                    f'MoveIt rechazó la trayectoria «{nombre}».'
                )
            resultado = self._esperar(handle.get_result_async()).result
            if resultado.error_code.val != MoveItErrorCodes.SUCCESS:
                raise RuntimeError(
                    f'Falló la ejecución de la trayectoria «{nombre}»: '
                    f'código MoveIt {resultado.error_code.val}.'
                )
            time.sleep(TRAJECTORY_SETTLE_TIME_SEC)
        finally:
            time.sleep(CAPTURE_SETTLE_TIME_SEC)
            muestras = self._terminar_captura()
            self._guardar_csv(nombre, muestras)

        self.get_logger().info(f'Trayectoria «{nombre}» completada.')

    @staticmethod
    def _guardar_csv(
        nombre: str,
        muestras: list[MuestraArticular],
    ) -> None:
        """Escribe solo las muestras de movimiento en un CSV independiente."""
        carpeta = Path.home() / 'ros_workspace' / 'datos_trayectorias'
        carpeta.mkdir(parents=True, exist_ok=True)
        marca_tiempo = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        ruta = carpeta / f'{marca_tiempo}_{nombre}.csv'
        columnas = [
            'trayectoria',
            'tiempo_s',
            'fuente_tiempo',
            'articulacion',
            'posicion_rad',
            'velocidad_rad_s',
            'aceleracion_rad_s2',
        ]
        with ruta.open('w', newline='', encoding='utf-8') as archivo:
            escritor = csv.DictWriter(archivo, fieldnames=columnas)
            escritor.writeheader()
            escritor.writerows(muestras)
        print(f'Datos guardados: {ruta} ({len(muestras)} filas)')

    def ejecutar_practica(self) -> None:
        """Ejecuta y captura movimientos articulares, lineales y circulares."""
        self.get_logger().info('Planificando movimiento articular...')
        objetivo = list(INITIAL_JOINT_TARGET_RAD)
        plan_articular = self._planificar_articulaciones(objetivo)
        self._ejecutar_y_guardar('joint', plan_articular)

        pose = self._pose_actual()
        lineales: list[RutaCartesiana] = []
        for eje in ('x', 'y', 'z'):
            for sentido in (-1.0, 1.0):
                destino = copy.deepcopy(pose)
                setattr(
                    destino.position,
                    eje,
                    getattr(pose.position, eje) + sentido * LINEAR_OFFSET_M,
                )
                descripcion = f'{eje} {sentido * LINEAR_OFFSET_M:.3f} m'
                lineales.append((descripcion, [destino]))
        plan_lineal = self._planificar_cartesiana(lineales)
        self._ejecutar_y_guardar('lineal', plan_lineal)

        pose = self._pose_actual()
        radio = CIRCLE_RADIUS_M
        circulares: list[RutaCartesiana] = []
        for eje_a, eje_b, descripcion in (
            ('x', 'y', 'plano XY'),
            ('x', 'z', 'plano XZ'),
            ('y', 'z', 'plano YZ'),
        ):
            centro_a = getattr(pose.position, eje_a) + radio
            puntos = []
            for indice in range(1, CIRCLE_POINT_COUNT + 1):
                angulo = (
                    math.pi
                    + 2.0 * math.pi * indice / CIRCLE_POINT_COUNT
                )
                punto = copy.deepcopy(pose)
                setattr(
                    punto.position,
                    eje_a,
                    centro_a + radio * math.cos(angulo),
                )
                setattr(
                    punto.position,
                    eje_b,
                    getattr(pose.position, eje_b) + radio * math.sin(angulo),
                )
                puntos.append(punto)
            circulares.append((descripcion, puntos))
        plan_circular = self._planificar_cartesiana(circulares)
        self._ejecutar_y_guardar('circular', plan_circular)


def main(args: list[str] | None = None) -> None:
    """Inicializa ROS 2, ejecuta la practica y libera sus recursos."""
    rclpy.init(args=args)
    node: CapturadorTrayectorias | None = None
    executor = MultiThreadedExecutor()
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
        executor.shutdown(timeout_sec=5.0, wait_for_threads=True)
        if hilo is not None:
            hilo.join(timeout=5.0)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
