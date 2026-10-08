# geometry.py
#
# Matematica pura del contrato de telemetria. Nada de hardware aqui, para
# poder probarlo en la computadora si hace falta (con python3 normal).

import math


def cubo_en_su_zona(cubo, depot, depot_size, grid, cube_side):
    """
    Copiado tal cual de la seccion "Cuando un cubo esta en su zona" del
    CONTRATO.md oficial (es la misma cuenta que usa el sistema de vision).

    Devuelve (adentro: bool, cuanto_falta: float en celdas).
    """
    distancias = {
        "arriba": depot["row"],
        "abajo": grid["rows"] - depot["row"],
        "izquierda": depot["col"],
        "derecha": grid["cols"] - depot["col"],
    }
    lado = min(distancias, key=lambda l: distancias[l])

    if lado in ("arriba", "abajo"):
        semi_col, semi_row = depot_size["length"] / 2, depot_size["depth"] / 2
    else:
        semi_col, semi_row = depot_size["depth"] / 2, depot_size["length"] / 2

    margen = cube_side * math.sqrt(2) / 2

    exceso_col = max(0.0, abs(cubo["col"] - depot["col"]) - (semi_col - margen))
    exceso_row = max(0.0, abs(cubo["row"] - depot["row"]) - (semi_row - margen))
    falta = math.hypot(exceso_col, exceso_row)
    return falta == 0.0, falta


def distancia(a, b):
    """Distancia euclidiana en celdas entre dos puntos con 'col' y 'row'."""
    return math.hypot(a["col"] - b["col"], a["row"] - b["row"])


def costo_cubo(rover, cubo, depot):
    """
    Costo aproximado de ir por este cubo: distancia del rover al cubo, mas
    distancia del cubo a su zona de acopio. Variante sugerida en
    solucion_simple.md del repo oficial (mejor que solo "el cubo mas
    cercano", porque tambien pesa que quede lejos de su zona).
    """
    return distancia(rover, cubo) + distancia(cubo, depot)


def heading_hacia(origen, destino):
    """
    Angulo en grados [0,360) al que habria que apuntar para ir de origen a
    destino, en la misma convencion que 'theta' del contrato: 0 = hacia
    col creciente (derecha), sentido antihorario visto desde arriba.

    OJO: 'row' crece hacia ABAJO en la imagen (seccion 4 del contrato), por
    eso se invierte el eje aqui para que el sentido de giro coincida con
    theta. Esto es un supuesto que HAY QUE VERIFICAR en la cancha real
    (ver README de firmware/rover): si el rover gira siempre al reves de
    lo esperado, el arreglo es invertir el signo de dy en esta funcion.
    """
    dx = destino["col"] - origen["col"]
    dy = -(destino["row"] - origen["row"])
    angulo = math.degrees(math.atan2(dy, dx))
    if angulo < 0:
        angulo += 360
    return angulo


def diferencia_angular(objetivo, actual):
    """Diferencia angular mas corta (objetivo - actual), en [-180, 180]."""
    return (objetivo - actual + 180) % 360 - 180
