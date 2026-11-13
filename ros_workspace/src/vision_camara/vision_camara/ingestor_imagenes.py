"""Nodo de ingesta de imágenes publicadas por la cámara USB."""

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


class IngestorImagenes(Node):
    """Se suscribe al topic de la cámara y recibe sus imágenes."""

    def __init__(self) -> None:
        """Declara el topic de entrada como parámetro y crea la suscripción."""
        super().__init__(NOMBRE_NODO)
        # El topic es un parámetro para poder cambiarlo desde la línea de
        # comandos (--ros-args -p topic_imagen:=...) sin tocar el código.
        self.declare_parameter('topic_imagen', TOPIC_IMAGEN_POR_DEFECTO)
        topic_imagen: str = self.get_parameter('topic_imagen').value

        self._imagenes_recibidas: int = 0
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
        """Cuenta las imágenes recibidas e informa periódicamente."""
        self._imagenes_recibidas += 1
        if self._imagenes_recibidas % INTERVALO_LOG_IMAGENES == 0:
            self.get_logger().info(
                f'Imágenes recibidas: {self._imagenes_recibidas} '
                f'({msg.width}x{msg.height}, {msg.encoding})'
            )


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
