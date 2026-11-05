#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Publicador(Node):
    def __init__(self):
        super().__init__("publicador_basico")
        self.publicador = self.create_publisher(String, "primer_topic", 10)
        self.create_timer(0.1, self.publicar)  # 10 Hz

    def publicar(self):
        mensaje = String()
        mensaje.data = f"Hola Mundo {self.get_clock().now().nanoseconds / 1e9}"
        self.publicador.publish(mensaje)


if __name__ == '__main__':
    rclpy.init()
    nodo = Publicador()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    nodo.destroy_node()
    rclpy.try_shutdown()
