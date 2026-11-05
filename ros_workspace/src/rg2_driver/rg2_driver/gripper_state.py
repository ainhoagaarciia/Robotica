"""Acceso a la pinza OnRobot RG2 (real o simulada)."""

from math import asin
from typing import Tuple
from xmlrpc.client import ServerProxy

ANCHURA_MAXIMA_MM = 100.0
NOMBRES_ARTICULACIONES = [
    "finger_joint",
    "left_inner_knuckle_joint",
    "left_inner_finger_joint",
    "right_outer_knuckle_joint",
    "right_inner_knuckle_joint",
    "right_inner_finger_joint",
]


def anchura_a_rad(anchura: float) -> float:
    """Convierte la anchura de la pinza (mm) en el ángulo de la articulación finger_joint."""
    return 0.855211333 - asin((7.720897365417915 + anchura) / 110.6835961023352)


def posiciones_articulaciones(anchura: float) -> list:
    rads = anchura_a_rad(anchura)
    return [rads, -rads, rads, -rads, -rads, rads]


class PinzaReal:
    """Pinza conectada al robot UR: lectura por RTDE y órdenes por XML-RPC."""

    def __init__(self, robot_ip: str) -> None:
        import rtde_receive

        self.robot_ip = robot_ip
        self.rtde_receive_ = rtde_receive.RTDEReceiveInterface(robot_ip, use_upper_range_registers=True)
        self.servidor = ServerProxy(f"http://{robot_ip}:41414")

    def leer_estado(self) -> Tuple[float, bool, bool]:
        """Devuelve (anchura en mm, ocupada, agarre detectado)."""
        anchura = self.rtde_receive_.getOutputDoubleRegister(36)
        ocupada = self.rtde_receive_.getOutputIntRegister(36)
        agarre = self.rtde_receive_.getOutputIntRegister(37)
        return anchura, bool(ocupada), bool(agarre)

    def mover(self, anchura: float, fuerza: float) -> None:
        if anchura < 0.0 or anchura > ANCHURA_MAXIMA_MM:
            return
        self.servidor.rg_grip(0, float(anchura), float(fuerza))


class PinzaSimulada:
    """Pinza simulada para trabajar sin robot real."""

    VELOCIDAD_MM_S = 150.0

    def __init__(self) -> None:
        self.anchura = ANCHURA_MAXIMA_MM
        self.objetivo = ANCHURA_MAXIMA_MM
        self.ultima_actualizacion = None

    def _actualizar(self, ahora: float) -> None:
        if self.ultima_actualizacion is not None:
            paso = self.VELOCIDAD_MM_S * (ahora - self.ultima_actualizacion)
            diferencia = self.objetivo - self.anchura
            self.anchura += max(-paso, min(paso, diferencia))
        self.ultima_actualizacion = ahora

    def leer_estado(self, ahora: float = None) -> Tuple[float, bool, bool]:
        from time import monotonic

        self._actualizar(monotonic() if ahora is None else ahora)
        return self.anchura, abs(self.objetivo - self.anchura) > 0.1, False

    def mover(self, anchura: float, fuerza: float) -> None:
        if anchura < 0.0 or anchura > ANCHURA_MAXIMA_MM:
            return
        self.leer_estado()
        self.objetivo = float(anchura)
