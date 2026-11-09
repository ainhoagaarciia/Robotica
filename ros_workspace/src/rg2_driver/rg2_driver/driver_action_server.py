#!/usr/bin/env python3
"""Nodo del driver de la pinza RG2: servidor de acción GripperCommand y publicación del estado."""

from time import sleep, time

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from rg2_driver.gripper_state import (
    NOMBRES_ARTICULACIONES,
    PinzaReal,
    PinzaSimulada,
    posiciones_articulaciones,
)

FRECUENCIA_HZ = 60.0
TIMEOUT_S = 10.0


class RG2ActionServer(Node):

    def __init__(self) -> None:
        super().__init__("rg2_control_node")
        self.declare_parameter("robot_ip", "")
        self.declare_parameter("simulate", False)
        # Prefijo de las articulaciones (p. ej. "203_") cuando hay varios robots en el mismo URDF
        self.declare_parameter("prefix", "")
        self.nombres_articulaciones = [
            self.get_parameter("prefix").value + nombre for nombre in NOMBRES_ARTICULACIONES
        ]

        simular = self.get_parameter("simulate").value
        if simular:
            self.pinza = PinzaSimulada()
            self.get_logger().info("Pinza RG2 simulada")
        else:
            robot_ip = self.get_parameter("robot_ip").value
            self.pinza = PinzaReal(robot_ip)
            self.get_logger().info(f"Pinza RG2 conectada al robot {robot_ip}")

        grupo = ReentrantCallbackGroup()
        # El estado de la pinza se publica en /joint_states junto al de los joints del UR
        self.state_publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.grip_publisher = self.create_publisher(Bool, "rg2/grip_detected", 10)
        self.timer = self.create_timer(1.0 / FRECUENCIA_HZ, self.__publicar_estado, callback_group=grupo)

        self.gripper_action_server = ActionServer(
            self,
            GripperCommand,
            "rg2_action_server",
            execute_callback=self.__execute_callback,
            goal_callback=lambda goal: GoalResponse.ACCEPT,
            cancel_callback=lambda goal: CancelResponse.ACCEPT,
            callback_group=grupo,
        )

    def __publicar_estado(self) -> None:
        anchura, _, agarre = self.pinza.leer_estado()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.nombres_articulaciones
        msg.position = posiciones_articulaciones(anchura)
        self.state_publisher.publish(msg)
        self.grip_publisher.publish(Bool(data=agarre))

    def __execute_callback(self, goal_handle):
        goal = goal_handle.request
        self.pinza.mover(anchura=goal.command.position, fuerza=goal.command.max_effort)
        feedback = GripperCommand.Feedback()

        # Bucle que acaba cuando acabe el grip
        inicio = time()
        sleep(0.1)

        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return GripperCommand.Result()

            posicion, ocupada, atascada = self.pinza.leer_estado()
            feedback.position = float(posicion)
            feedback.stalled = bool(atascada)
            feedback.effort = goal.command.max_effort
            feedback.reached_goal = not ocupada
            goal_handle.publish_feedback(feedback)

            if feedback.reached_goal or time() - inicio > TIMEOUT_S:
                break
            sleep(1.0 / FRECUENCIA_HZ)

        result = GripperCommand.Result()
        result.effort = goal.command.max_effort
        result.stalled = bool(feedback.stalled)
        result.position = feedback.position

        if abs(goal.command.position - feedback.position) < 2.0 or feedback.stalled:
            result.reached_goal = True
            goal_handle.succeed()
            self.get_logger().info("Objetivo alcanzado")
        else:
            result.reached_goal = False
            goal_handle.abort()
            self.get_logger().warn("Objetivo NO alcanzado")
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    nodo = RG2ActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(nodo)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
