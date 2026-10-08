# motion.py
#
# Control de movimiento del rover usando la telemetria de vision como
# retroalimentacion de posicion/orientacion (col, row, theta a ~20 Hz),
# en vez de solo estimar todo con el giroscopio a ciegas como hacian los
# ejemplos move_heading.py / turn_angle.py del repo oficial (esos sirven
# para probar motores en la mesa sin camara; aca SI tenemos camara, y es
# una fuente de posicion mucho mejor).
#
# OJO CON EL SIGNO: cual motor hay que mover para "girar hacia theta
# creciente" es un supuesto (ver comentario en girar_hacia) que hay que
# confirmar viendo al robot moverse frente al sistema de vision. Si gira
# siempre al reves de lo pedido, invertir el signo de 'direccion' abajo.

import time
from ideaboard import IdeaBoard

import config
import geometry

ib = IdeaBoard()


def detener():
    ib.motor_1.throttle = 0
    ib.motor_2.throttle = 0


def _limitar(v, lo=-1.0, hi=1.0):
    return max(lo, min(hi, v))


def buscar_rover(msg, robot_id):
    for r in msg.get("rovers", []):
        if r["id"] == robot_id:
            return r
    return None


def girar_hacia(vision_client, robot_id, angulo_objetivo,
                tolerancia=config.ANGLE_TOLERANCE_DEG, timeout=6):
    """
    Gira en el lugar hasta apuntar aproximadamente a 'angulo_objetivo'
    (grados, misma convencion que theta), usando el theta que reporta la
    vision en cada mensaje nuevo. Devuelve True si llego, False si se
    acabo el tiempo.
    """
    inicio = time.monotonic()

    while time.monotonic() - inicio < timeout:
        msg = vision_client.poll()
        if msg is None:
            time.sleep(0.02)
            continue

        rover = buscar_rover(msg, robot_id)
        if rover is None:
            time.sleep(0.02)
            continue

        error = geometry.diferencia_angular(angulo_objetivo, rover["theta"])

        if abs(error) <= tolerancia:
            detener()
            return True

        velocidad = (
            config.MOTOR_SPEED_TURN if abs(error) > 25
            else config.MOTOR_SPEED_TURN * 0.6
        )
        direccion = 1 if error > 0 else -1

        # error > 0: el objetivo esta en sentido antihorario (theta debe
        # crecer). Igual que left() en test_motores.py: motor_1 hacia
        # atras, motor_2 hacia adelante. VERIFICAR EN CANCHA (ver arriba).
        ib.motor_1.throttle = -velocidad * direccion
        ib.motor_2.throttle = velocidad * direccion

        time.sleep(0.02)

    detener()
    return False


def avanzar_hacia(vision_client, robot_id, destino,
                   tolerancia_celdas=config.DISTANCE_ARRIVED_CELLS,
                   velocidad=config.MOTOR_SPEED_CRUISE, timeout=15,
                   corregir_rumbo=True):
    """
    Avanza en linea recta (con correccion continua de rumbo) hasta quedar a
    'tolerancia_celdas' o menos de 'destino' (dict con 'col'/'row').

    Devuelve (llego: bool, ultimo_rover_visto_o_None).
    """
    inicio = time.monotonic()
    ultimo_rover = None

    while time.monotonic() - inicio < timeout:
        msg = vision_client.poll()
        if msg is None:
            time.sleep(0.02)
            continue

        rover = buscar_rover(msg, robot_id)
        if rover is None:
            time.sleep(0.02)
            continue

        ultimo_rover = rover

        if geometry.distancia(rover, destino) <= tolerancia_celdas:
            detener()
            return True, rover

        error = 0
        if corregir_rumbo:
            objetivo_heading = geometry.heading_hacia(rover, destino)
            error = geometry.diferencia_angular(objetivo_heading, rover["theta"])

        # Correccion suave: mas rapido el motor "de afuera" de la curva,
        # mas lento el "de adentro", sin invertir ninguno (a diferencia de
        # girar_hacia, que si puede invertir uno para girar en el lugar).
        correccion = _limitar(error / 45.0, -0.35, 0.35)

        ib.motor_1.throttle = _limitar(velocidad - correccion)
        ib.motor_2.throttle = _limitar(velocidad + correccion)

        time.sleep(0.02)

    detener()
    return False, ultimo_rover
