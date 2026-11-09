#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class CapturadorTrayectorias(Node):
    #Recibe los estados articulares publicados en /joint_states

    def __init__(self) -> None:
        super().__init__('capturador_trayectorias')

        self._mensajes_recibidos = 0

        self.create_subscription(
            JointState,
            '/joint_states',
            self._al_recibir_estado,
            10,
        )

        self.get_logger().info('Suscrito a /joint_states.')

    def _al_recibir_estado(self, msg: JointState) -> None:
        #Muestra las posiciones articulares cada 100 mensajes

        self._mensajes_recibidos += 1

        if self._mensajes_recibidos % 100 == 0:
            posiciones = dict(zip(msg.name, msg.position))
            self.get_logger().info(
                f'Estado articular: {posiciones}'
            )


def main(args: list[str] | None = None) -> None:
    #Inicializa el nodo y mantiene activa la suscripción

    rclpy.init(args=args)
    nodo = CapturadorTrayectorias()

    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()