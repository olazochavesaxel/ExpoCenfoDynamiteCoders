# strategy.py
#
# Estrategia "greedy" sugerida en solucion_simple.md del repo oficial:
# mientras queden cubos sin entregar, cada rover va por el de menor costo
# (distancia hasta el cubo + distancia del cubo a su zona), evitando pisarse
# con el companero gracias a coordination.py.

import time
import config
import geometry
import motion


def cubos_pendientes(msg):
    """Lista de (cubo, depot) para los cubos que aun no estan entregados."""
    depots_por_color = {d["color"]: d for d in msg["depots"]}
    pendientes = []

    for cubo in msg["cubes"]:
        depot = depots_por_color.get(cubo["color"])
        if depot is None:
            continue

        entregado, _ = geometry.cubo_en_su_zona(
            cubo, depot, msg["depot_size"], msg["grid"], msg["cube_side"]
        )
        if not entregado:
            pendientes.append((cubo, depot))

    return pendientes


def elegir_cubo(msg, robot_id, peer_color_elegido):
    """
    De los cubos pendientes, elige el de menor costo, penalizando (no
    prohibiendo) el color que ya eligio el companero, para reducir choques
    sin bloquearse si es realmente el mejor cubo para nosotros.
    """
    rover = motion.buscar_rover(msg, robot_id)
    if rover is None:
        return None, None

    mejor = None  # (costo, cubo, depot)

    for cubo, depot in cubos_pendientes(msg):
        penalizacion = 3.0 if cubo["color"] == peer_color_elegido else 0.0
        costo = geometry.costo_cubo(rover, cubo, depot) + penalizacion

        if mejor is None or costo < mejor[0]:
            mejor = (costo, cubo, depot)

    if mejor is None:
        return None, None

    return mejor[1], mejor[2]


def punto_de_empuje(cubo, depot, retroceso_celdas=2.0):
    """
    Punto desde donde empezar a empujar: sobre la linea cubo->depot, del
    lado contrario al depot, para terminar empujando el cubo HACIA su zona.
    """
    dx = depot["col"] - cubo["col"]
    dy = depot["row"] - cubo["row"]
    dist = max(0.0001, geometry.distancia(cubo, depot))
    ux, uy = dx / dist, dy / dist

    return {
        "col": cubo["col"] - ux * retroceso_celdas,
        "row": cubo["row"] - uy * retroceso_celdas,
    }


def empujar_cubo(vision_client, robot_id, color, depot, timeout=20):
    """
    Empuja el cubo 'color' en linea hacia su depot, corrigiendo rumbo en
    cada mensaje de telemetria, hasta que quede entregado o se acabe el
    tiempo. Devuelve True si quedo entregado.
    """
    inicio = time.monotonic()

    while time.monotonic() - inicio < timeout:
        msg = vision_client.poll()
        if msg is None:
            time.sleep(0.02)
            continue

        cubo = None
        for c in msg.get("cubes", []):
            if c["color"] == color:
                cubo = c
                break
        if cubo is None:
            time.sleep(0.02)
            continue

        entregado, _ = geometry.cubo_en_su_zona(
            cubo, depot, msg["depot_size"], msg["grid"], msg["cube_side"]
        )
        if entregado:
            motion.detener()
            print("Cubo", color, "entregado")
            return True

        rover = motion.buscar_rover(msg, robot_id)
        if rover is None:
            time.sleep(0.02)
            continue

        rumbo = geometry.heading_hacia(cubo, depot)
        error = geometry.diferencia_angular(rumbo, rover["theta"])
        correccion = max(-0.25, min(0.25, error / 45.0))

        motion.ib.motor_1.throttle = max(-1, min(1, config.MOTOR_SPEED_SLOW - correccion))
        motion.ib.motor_2.throttle = max(-1, min(1, config.MOTOR_SPEED_SLOW + correccion))

        time.sleep(0.02)

    motion.detener()
    print("Se agoto el tiempo empujando el cubo", color)
    return False


def entregar_cubo(vision_client, robot_id, cubo, depot):
    """Secuencia completa para un cubo: ir detras de el, apuntar, empujar."""

    msg = vision_client.last_message
    rover = motion.buscar_rover(msg, robot_id) if msg else None
    if rover is None:
        return False

    punto_atras = punto_de_empuje(cubo, depot)

    print("Yendo al punto de empuje del cubo", cubo["color"])
    rumbo_ida = geometry.heading_hacia(rover, punto_atras)
    motion.girar_hacia(vision_client, robot_id, rumbo_ida)
    motion.avanzar_hacia(vision_client, robot_id, punto_atras)

    print("Girando hacia cubo -> deposito")
    rumbo_empuje = geometry.heading_hacia(cubo, depot)
    motion.girar_hacia(vision_client, robot_id, rumbo_empuje)

    print("Empujando cubo", cubo["color"])
    return empujar_cubo(vision_client, robot_id, cubo["color"], depot)


def ciclo_principal(vision_client, coordinator, robot_id):
    """
    Bucle principal: mientras queden cubos por entregar, elegir el mejor
    disponible (coordinando con el companero) y entregarlo. Termina cuando
    no quedan cubos o el sistema de vision marca FINISHED.
    """

    while True:
        coordinator.recibir_pendientes()

        msg = vision_client.poll()
        if msg is None:
            if not vision_client.connected:
                vision_client.ensure_connected()
            time.sleep(0.02)
            continue

        if msg["phase"] == "FINISHED":
            print("Ronda terminada (FINISHED)")
            motion.detener()
            return

        if not cubos_pendientes(msg):
            print("No quedan cubos por entregar")
            motion.detener()
            return

        peer_color = coordinator.peer_eligio()
        cubo, depot = elegir_cubo(msg, robot_id, peer_color)

        if cubo is None:
            time.sleep(0.05)
            continue

        rover = motion.buscar_rover(msg, robot_id)
        if rover is not None:
            coordinator.anunciar(cubo["color"], rover)

        entregar_cubo(vision_client, robot_id, cubo, depot)
