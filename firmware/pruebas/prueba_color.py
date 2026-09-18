# Prueba OPCIONAL del sensor de color - adaptado de color_detect.py del
# repo oficial del reto, casi sin cambios.
#
# ADVERTENCIA IMPORTANTE:
# Este ejemplo asume un sensor de color hecho con un fotodiodo (analogio en
# board.IO39) iluminado por un NeoPixel externo en board.IO4, no un sensor
# de color digital I2C. Ademas, board.IO39 es el MISMO pin que usa el
# sensor infrarrojo IR2 en firmware/code.py - no corras ambos scripts a la
# vez en el mismo robot.
#
# Antes de usar esto, confirmen con el diagrama de conexiones del repo
# oficial (conexiones/README.md) que sensor de color trae realmente su
# CenfoBot y en que pines esta conectado. Si el sensor de su kit es un
# chip I2C (se conecta con 4 cables: VCC, GND, SDA, SCL - normalmente por
# el cable Qwiic), este script no les va a servir tal cual y hay que
# adaptarlo.
#
# Para usar: copiar este archivo junto a ideaboard.py a la raiz de la
# unidad CIRCUITPY, renombrandolo a code.py.

import board
import neopixel
import analogio
import keypad
from time import sleep
from ideaboard import IdeaBoard

# Calibracion de referencia del ejemplo original - hay que recalibrar con
# su propio sensor y su propia iluminacion del entorno.
MIN_LUZ = 2819
MAX_LUZ = 62238

ib = IdeaBoard()
ib.brightness = 0.2

pixel = neopixel.NeoPixel(board.IO4, 1, brightness=1, auto_write=True)
luz = analogio.AnalogIn(board.IO39)
boton = keypad.Keys((board.IO0,), value_when_pressed=False, pull=True)


def medir_color(color):
    pixel[0] = color
    sleep(0.2)
    total = 0
    for _ in range(20):
        total += luz.value
        sleep(0.005)
    pixel[0] = (0, 0, 0)
    return total / 20


print("Listo. Presiona el boton BOOT de la IdeaBoard para medir.")

while True:
    evento = boton.events.get()
    if evento and evento.released:
        rojo_raw = medir_color((255, 0, 0))
        verde_raw = medir_color((0, 255, 0))
        azul_raw = medir_color((0, 0, 255))
        print("RAW: <{}, {}, {}>".format(int(rojo_raw), int(verde_raw), int(azul_raw)))

        if rojo_raw <= verde_raw and rojo_raw <= azul_raw:
            print("Color detectado: ROJO")
            ib.pixel = (255, 0, 0)
        elif verde_raw <= rojo_raw and verde_raw <= azul_raw:
            print("Color detectado: VERDE")
            ib.pixel = (0, 255, 0)
        else:
            print("Color detectado: AZUL")
            ib.pixel = (0, 0, 255)
        sleep(0.5)
