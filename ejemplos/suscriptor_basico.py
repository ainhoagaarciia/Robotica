#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Suscriptor(Node):
    def __init__(self):
        super().__init__("suscriptor_basico")
        self.create_subscription(String, "primer_topic", self.callback, 10)

    def callback(self, mensaje: String):
        print(f"He recibido: {mensaje.data}")


if __name__ == '__main__':
    rclpy.init()
    nodo = Suscriptor()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    nodo.destroy_node()
    rclpy.try_shutdown()
