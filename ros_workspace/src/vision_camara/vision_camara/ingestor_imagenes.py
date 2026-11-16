"""Nodo de ingesta de imágenes publicadas por la cámara USB."""

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image

NOMBRE_NODO = 'ingestor_imagenes'
TOPIC_IMAGEN_POR_DEFECTO = '/image_raw'
# Cada cuántas imágenes recibidas se informa por el log, para comprobar que
# llegan datos sin saturar la terminal (la cámara publica a 30 fps).
INTERVALO_LOG_IMAGENES = 30
# Diferencia media mínima (niveles de gris, 0-255) con la última imagen
# aceptada para considerar que una imagen nueva es distinta. Por debajo de
# este valor la diferencia se atribuye al ruido del sensor.
UMBRAL_DIFERENCIA_POR_DEFECTO = 8.0
# Lado de la miniatura usada para comparar: reducir la imagen acelera la
# comparación y promedia el ruido de píxeles individuales.
LADO_MINIATURA_PX = 64


class IngestorImagenes(Node):
    """Se suscribe al topic de la cámara y recibe sus imágenes."""

    def __init__(self) -> None:
        """Declara el topic de entrada como parámetro y crea la suscripción."""
        super().__init__(NOMBRE_NODO)
        # El topic es un parámetro para poder cambiarlo desde la línea de
        # comandos (--ros-args -p topic_imagen:=...) sin tocar el código.
        self.declare_parameter('topic_imagen', TOPIC_IMAGEN_POR_DEFECTO)
        topic_imagen: str = self.get_parameter('topic_imagen').value
        self.declare_parameter(
            'umbral_diferencia', UMBRAL_DIFERENCIA_POR_DEFECTO
        )
        self._umbral_diferencia: float = (
            self.get_parameter('umbral_diferencia').value
        )

        self._bridge = CvBridge()
        self._imagenes_recibidas: int = 0
        self._imagenes_aceptadas: int = 0
        # Miniatura de la última imagen aceptada; None hasta recibir la
        # primera, que siempre se acepta.
        self._ultima_miniatura: np.ndarray | None = None
        # qos_profile_sensor_data (best effort) es el perfil recomendado para
        # sensores: si se pierde una imagen no se reenvía, se usa la siguiente.
        self.create_subscription(
            Image,
            topic_imagen,
            self._al_recibir_imagen,
            qos_profile_sensor_data,
        )
        self.get_logger().info(f'Suscrito a {topic_imagen}.')

    def _al_recibir_imagen(self, msg: Image) -> None:
        """Convierte la imagen y la acepta solo si es distinta a la anterior."""
        self._imagenes_recibidas += 1
        if self._imagenes_recibidas % INTERVALO_LOG_IMAGENES == 0:
            self.get_logger().info(
                f'Imágenes recibidas: {self._imagenes_recibidas} '
                f'({msg.width}x{msg.height}, {msg.encoding})'
            )

        imagen = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        miniatura = self._crear_miniatura(imagen)
        if not self._es_distinta(miniatura):
            return

        self._ultima_miniatura = miniatura
        self._imagenes_aceptadas += 1
        self.get_logger().info(
            f'Imagen distinta aceptada ({self._imagenes_aceptadas}).'
        )

    @staticmethod
    def _crear_miniatura(imagen: np.ndarray) -> np.ndarray:
        """Reduce la imagen a una miniatura en gris para compararla."""
        gris = cv2.cvtColor(imagen, cv2.COLOR_BGR2GRAY)
        # INTER_AREA promedia los píxeles de cada bloque, lo que suaviza el
        # ruido del sensor antes de comparar.
        return cv2.resize(
            gris,
            (LADO_MINIATURA_PX, LADO_MINIATURA_PX),
            interpolation=cv2.INTER_AREA,
        )

    def _es_distinta(self, miniatura: np.ndarray) -> bool:
        """Indica si la miniatura difiere lo suficiente de la última aceptada.

        Se compara con la última imagen aceptada (no con la inmediatamente
        anterior) para que un cambio muy lento de la escena no acabe
        colando imágenes casi idénticas entre sí.
        """
        if self._ultima_miniatura is None:
            return True
        diferencia_media = float(
            np.mean(cv2.absdiff(miniatura, self._ultima_miniatura))
        )
        return diferencia_media > self._umbral_diferencia


def main(args: list[str] | None = None) -> None:
    """Inicializa ROS 2 y mantiene activo el nodo hasta Ctrl+C."""
    rclpy.init(args=args)
    nodo = IngestorImagenes()
    try:
        rclpy.spin(nodo)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C o parada externa (p. ej. desde ros2 launch): salida normal.
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
