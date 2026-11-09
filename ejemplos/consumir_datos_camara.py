#!/usr/bin/python3

import threading
import time
from copy import deepcopy

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class NodoCamara(Node):
    def __init__(self):
        super().__init__("nodo_camara")
        self.bridge = CvBridge()
        self.ultima_imagen = None
        self.create_subscription(Image, "/usb_cam/image_raw", self.callback_imagen, 10)

    def callback_imagen(self, msg: Image):
        self.ultima_imagen = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")


if __name__ == '__main__':
    rclpy.init()
    nodo = NodoCamara()
    # Los callbacks se atienden en un hilo aparte
    threading.Thread(target=rclpy.spin, args=(nodo,), daemon=True).start()

    while rclpy.ok():
        if nodo.ultima_imagen is not None:
            imagen = deepcopy(nodo.ultima_imagen)
            ...  # Procesar la imagen aquí
        time.sleep(1)

    nodo.destroy_node()
    rclpy.try_shutdown()
