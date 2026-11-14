"""Carga de los parametros de calibracion de la camara."""

from pathlib import Path

import numpy as np
import yaml


class CargadorCalibracion:
    """Lee la matriz intrinseca y la distorsion desde un archivo YAML."""

    def __init__(self, ruta_calibracion: str | Path) -> None:
        """Guarda la ruta y carga los parametros de calibracion."""
        self._ruta_calibracion = Path(ruta_calibracion)

        self.matriz_camara: np.ndarray
        self.coeficientes_distorsion: np.ndarray

        self._cargar_calibracion()

    def _cargar_calibracion(self) -> None:
        """Lee el YAML y convierte los parametros a matrices NumPy."""
        with self._ruta_calibracion.open(
            'r', encoding='utf-8'
        ) as archivo:
            datos = yaml.safe_load(archivo)

        matriz_k = datos['camera_matrix']['data']
        distorsion = datos['distortion_coefficients']['data']

        self.matriz_camara = np.array(
            matriz_k, dtype=np.float64
        ).reshape(3, 3)

        self.coeficientes_distorsion = np.array(
            distorsion, dtype=np.float64
        )
