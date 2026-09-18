# Diagnostico general del CenfoBot - Expo Cenfo 2026 / DynamiteCoders
#
# Corre, en orden: luz indicadora (NeoPixel), sensores infrarrojos,
# sensor ultrasonico y una secuencia corta de motores. Pensado para
# confirmar que el armado y el cableado quedaron bien conectados antes
# de programar logica real del reto.
#
# ATENCION: la seccion de motores SI mueve el robot. Apoyalo sobre algo
# (una caja, un soporte) para que las ruedas no toquen el piso, o dejale
# espacio libre alrededor antes de correr esto.
#
# Requiere que en la carpeta lib/ de la unidad CIRCUITPY esten instaladas
# las librerias: adafruit_motor, neopixel, simpleio, adafruit_pixelbuf y
# hcsr04. Si el robot vino configurado de fabrica, ya deberian estar.
#
# Basado en las funciones reales de ideaboard.py y en los ejemplos
# test_motores.py / code_4IR.py / code_ultrasonic.py del repo oficial:
# https://github.com/Universidad-Cenfotec/Vision-Rover-Challenge

import time
import board
from ideaboard import IdeaBoard
from hcsr04 import HCSR04

ib = IdeaBoard()


def separador(titulo):
    print()
    print("=" * 40)
    print(titulo)
    print("=" * 40)


def prueba_luz():
    separador("1. LUZ INDICADORA (NeoPixel)")
    secuencia = [((255, 0, 0), "rojo"), ((0, 255, 0), "verde"),
                 ((0, 0, 255), "azul"), ((0, 0, 0), "apagado")]
    for color, nombre in secuencia:
        print("  -> {}".format(nombre))
        ib.pixel = color
        time.sleep(0.6)
    print("Si viste los tres colores encendidos, el NeoPixel esta bien.")


def prueba_infrarrojos(ciclos=6):
    separador("2. SENSORES INFRARROJOS")
    sensores = [
        ("IR1 adelante-izq", ib.AnalogIn(board.IO36)),
        ("IR2 adelante-der", ib.AnalogIn(board.IO39)),
        ("IR3 atras-izq", ib.AnalogIn(board.IO34)),
        ("IR4 atras-der", ib.AnalogIn(board.IO35)),
    ]
    print("Tapa cada sensor con la mano y confirma que su numero cambia.")
    for _ in range(ciclos):
        for nombre, sen in sensores:
            print("  {}: {}".format(nombre, sen.value))
        print("  ---")
        time.sleep(0.5)


def prueba_ultrasonico(ciclos=6):
    separador("3. SENSOR ULTRASONICO")
    sonar = HCSR04(board.IO25, board.IO26)  # TRIG=IO25, ECHO=IO26
    print("Acerca y aleja la mano del sensor.")
    for _ in range(ciclos):
        try:
            print("  distancia: {:.1f} cm".format(sonar.dist_cm()))
        except RuntimeError:
            print("  (sin eco todavia)")
        time.sleep(0.5)


def prueba_motores():
    separador("4. MOTORES")
    print("Atencion: el robot se va a mover. Confirma que esta en un")
    print("lugar seguro. Arrancando en 3 segundos...")
    time.sleep(3)

    def mover(nombre, m1, m2, t=0.8, velocidad=0.5):
        print("  -> {}".format(nombre))
        ib.motor_1.throttle = m1 * velocidad
        ib.motor_2.throttle = m2 * velocidad
        time.sleep(t)
        ib.motor_1.throttle = 0
        ib.motor_2.throttle = 0
        time.sleep(0.4)

    mover("adelante", 1, 1)
    mover("atras", -1, -1)
    mover("giro izquierda", -1, 1)
    mover("giro derecha", 1, -1)

    print("Si el robot hizo los 4 movimientos en ese orden, los motores")
    print("y las ruedas estan bien conectados. Si alguno giro al reves,")
    print("cambia los dos cables de ese motor (ver armado/README.md del")
    print("repo oficial del reto).")


# ==========================
# SECUENCIA PRINCIPAL
# ==========================
print("Diagnostico CenfoBot - iniciando en 2 segundos...")
time.sleep(2)

prueba_luz()
prueba_infrarrojos()
prueba_ultrasonico()
prueba_motores()

separador("DIAGNOSTICO COMPLETO")
print("Revisa arriba si algo imprimio un error en vez de un valor normal.")
