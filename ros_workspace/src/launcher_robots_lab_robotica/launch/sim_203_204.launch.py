from launcher_robots_lab_robotica.lanzador_doble import generar_lanzamiento_doble


def generate_launch_description():
    return generar_lanzamiento_doble(("203", "204"), simulacion=True)
